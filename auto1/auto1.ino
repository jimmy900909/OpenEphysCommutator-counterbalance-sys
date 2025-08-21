#include "HX711.h"

// ---------------- Pins ----------------
#define DOUT 4
#define CLK  5

#define AIN1 6     // DRV8833 IN1 (PWM)
#define AIN2 9     // DRV8833 IN2 (PWM)

// Encoder pins
#define ENCODER_A 2
#define ENCODER_B 3

// ---------------- HX711 ----------------
HX711 scale;
float calibration_factor = 4180;  // g
const float HX_RATE_HZ = 80.0f;   // 你的 HX711 取樣率
unsigned long last_hx_ms = 0;
bool hx_updated = false;
bool have_prev_T = false;

// ---------------- Encoder → v_line ----------------
const float COUNTS_PER_METER = 11357.0f;
volatile long encoderCount = 0;
long lastCount = 0;
unsigned long lastVtMs = 0;
float v_line = 0.0f;         // m/s（放線 < 0，收線 > 0）
float leash_len_m = 0.0f;

// ---------------- Motor / PWM ----------------
const int remotorSpeed = 150;  // retract PWM
int pwm_state = 0;
const int PWM_STEP_OUT = 25;   // loosen ramp（防止過度放）
const int PWM_STEP_IN  = 4;    // retract ramp

// 放線 PWM（基礎 + 比例）
const int   BASE_PAYOUT_PWM   = 180;
const int   MAX_PAYOUT_PWM    = 255;
const float KP_PWM_PER_G      = 20.0f;

// ---------------- EMG（突發強拉） ----------------
const int threshold_emg = 50;         // g
bool emg_boost = false;
unsigned long emg_until_ms = 0;
const unsigned long EMG_BOOST_MS = 70;

const int BURST_PWM = 130;
const unsigned long BURST_MS = 200;
unsigned long burst_until_ms = 0;

// ★ 新增：Burst 後禁止回收視窗
const unsigned long POST_BURST_NO_RETRACT_MS = 300;
unsigned long no_retract_until_ms = 0;

// ---------------- 張力平滑 ----------------
float T_fast_g = 0.0f;                // 快 EMA（進入放線）
float T_slow_g = 0.0f;                // 慢 EMA（退出/回收）
const float T_ALPHA_FAST = 0.40f;
const float T_ALPHA_SLOW = 0.19f;

// ---------------- 門檻/許可 ----------------
float lower_limit = 17.8f;
float upper_limit = 22.8f;

float V_OUT_MIN = 0.002f;
float T_MARGIN_G = 0.7f;
unsigned long ALLOW_PAYOUT_CONFIRM_MS = 2;
bool allow_payout_latch = false;
unsigned long allow_payout_since_ms = 0;

// 鎖軸快速通道
const float V_EPS_LOCKED   = 0.003f;
const float T_LOCK_MARGING = 3.0f;
const float DTDT_LOCK_GPS  = 55.0f;
const unsigned long LOCK_CONFIRM_MS = 12;
bool locked_latch = false;
unsigned long locked_since_ms = 0;

// ---- dT/dt（改：僅新樣本時計算 + EMA 平滑）----
float dTdt_raw = 0.0f;
float dTdt_gps = 0.0f;         // 平滑後
float T_prev_for_dt = 0.0f;
const float DTDTA = 0.25f;     // dT/dt 的 EMA 係數（0.2~0.35）

// Active-Coast：接近上限時的微放
const int ASSIST_PWM = 90;

// ---------------- Anti-chatter / Grace ----------------
bool inRetract = false;          // 是否處於回收狀態
unsigned long retract_start_ms;  // 回收開始時間
long          retract_start_cnt; // 回收開始時的編碼器

const unsigned long MIN_RETRACT_MS  = 50;
const long          MIN_RETRACT_CNT = 10;

const unsigned long LOOSEN_GRACE_MS = 120; // 放線後寬鬆期：禁止立刻回收
unsigned long loosen_grace_until_ms = 0;

const float LOWER_HYST_G = 0.8f;  // 低門檻遲滯

// ---------------- Loop 節拍 ----------------
const unsigned long LOOP_MS = 1;
unsigned long last_loop_ms = 0;

// ---------------- helpers ----------------
inline void motorCoast() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void driveMotor(int pwmSigned) {
  if (pwmSigned == 0) { motorCoast(); return; }
  if (pwmSigned > 0) {           // RETRACT
    int pwm = constrain(pwmSigned, 0, 255);
    digitalWrite(AIN1, LOW);
    analogWrite(AIN2, pwm);
  } else {                       // LOOSEN
    int pwm = constrain(-pwmSigned, 0, 255);
    analogWrite(AIN1, pwm);
    digitalWrite(AIN2, LOW);
  }
}

int slewPWM(int target) {
  if (target == pwm_state) return pwm_state;
  if ((target > 0 && pwm_state >= 0) || (target < 0 && pwm_state <= 0)) {
    int step = (target > 0) ? PWM_STEP_IN : PWM_STEP_OUT;
    if (target > pwm_state) pwm_state = min(pwm_state + step, target);
    else                    pwm_state = max(pwm_state - step, target);
  } else {
    int step = (pwm_state > 0) ? PWM_STEP_IN : PWM_STEP_OUT;
    if (pwm_state > 0) pwm_state = max(0, pwm_state - step);
    else               pwm_state = min(0, pwm_state + step);
  }
  return pwm_state;
}

// ---------------- setup ----------------
void setup() {
  Serial.begin(9600);
  delay(300);

  scale.begin(DOUT, CLK);
  delay(500);
  if (!scale.is_ready()) { Serial.println("HX711 not found."); while (1); }
  scale.set_scale(calibration_factor);
  scale.tare();

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  motorCoast();

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  lastVtMs = millis();
  last_loop_ms = millis();
  last_hx_ms = millis();
  T_prev_for_dt = 0.0f;

  Serial.println("Ready ver2.0b (dT/dt fix + post-burst no-retract).");
}

// ---------------- loop ----------------
void loop() {
  if (millis() - last_loop_ms < LOOP_MS) return;
  last_loop_ms = millis();

  // ---- 張力：雙 EMA（僅在有新樣本時更新）----
  hx_updated = false;
  if (scale.is_ready()) {
    float raw = scale.get_units(1);
    T_fast_g = T_fast_g + T_ALPHA_FAST * (raw - T_fast_g);
    T_slow_g = T_slow_g + T_ALPHA_SLOW * (raw - T_slow_g);
    hx_updated = true;
  }
  float T_fast = T_fast_g;
  float T_slow = T_slow_g;

  // ---- v_line ----
  unsigned long now = millis();
  unsigned long dt_ms = now - lastVtMs; if (dt_ms == 0) dt_ms = 1;
  lastVtMs = now;

  long cnt; noInterrupts(); cnt = encoderCount; interrupts();
  long dcount = cnt - lastCount; lastCount = cnt;

  float dt = dt_ms * 1e-3f;
  v_line = (COUNTS_PER_METER > 1e-6f) ? ((float)dcount / COUNTS_PER_METER) / dt : 0.0f;
  leash_len_m += v_line * dt;

  // ---- dT/dt：只在新樣本時計算 raw，再用 EMA 平滑 ----
  if (hx_updated) {
    float dtT_s = max(0.001f, (now - last_hx_ms) * 1e-3f);
    if (!have_prev_T) { dTdt_raw = 0.0f; have_prev_T = true; }
    else               dTdt_raw = (T_fast - T_prev_for_dt) / dtT_s;
    T_prev_for_dt = T_fast;
    last_hx_ms = now;
  }
  dTdt_gps = DTDTA * dTdt_raw + (1.0f - DTDTA) * dTdt_gps; // EMA

  // ---- EMG ----
  if (T_fast > threshold_emg) { emg_boost = true; emg_until_ms = now + EMG_BOOST_MS; }
  if (emg_boost && now > emg_until_ms) emg_boost = false;

  // ---- 立即觸發（強拉/快速上升） → 放線 + burst + 禁回收窗 ----
  bool sudden_pull = (T_fast > upper_limit + 0.8f) || (dTdt_gps > 80.0f);
  bool not_retracting = (v_line <= 0.0f);   // ★ 線沒有被主動收進去
  bool instant_payout = sudden_pull && not_retracting;

  if (instant_payout) {
      allow_payout_latch   = true;
      allow_payout_since_ms = now;
      burst_until_ms        = now + BURST_MS;
      loosen_grace_until_ms = now + LOOSEN_GRACE_MS;
      no_retract_until_ms   = now + POST_BURST_NO_RETRACT_MS;
      inRetract = false; 
      Serial.println("[BURST] Triggered by sudden pull!");
  }

  // ---- 放線許可：方向 + 張力 + 連續 ----
  bool line_out = (v_line < -V_OUT_MIN);
  bool high_T   = (T_fast > (upper_limit + T_MARGIN_G));

  if (line_out && high_T) {
    if (!allow_payout_latch) { allow_payout_latch = true; allow_payout_since_ms = now; }
  } else {
    allow_payout_latch = false;
  }
  bool allow_payout_dir = allow_payout_latch && (now - allow_payout_since_ms >= ALLOW_PAYOUT_CONFIRM_MS);

  // 鎖軸快速通道
  bool spool_static = (fabs(v_line) < V_EPS_LOCKED);
  bool locked_trig  = spool_static && (T_fast > upper_limit + T_LOCK_MARGING) && (dTdt_gps > DTDT_LOCK_GPS);
  if (locked_trig) {
    if (!locked_latch) { locked_latch = true; locked_since_ms = now; }
  } else {
    locked_latch = false;
  }
  bool allow_payout_locked = locked_latch && (now - locked_since_ms >= LOCK_CONFIRM_MS);
  bool allow_payout = allow_payout_dir || allow_payout_locked;

  // ---- 主邏輯：目標 PWM ----
  int targetPWM = 0; // >0 回收；<0 放線；=0 停

  if (T_fast > upper_limit) {
    // 放線
    if (emg_boost) {
      targetPWM = -255;
      loosen_grace_until_ms = now + LOOSEN_GRACE_MS;
      no_retract_until_ms   = now + POST_BURST_NO_RETRACT_MS;
      inRetract = false;
    } else if (allow_payout) {
      float over_g = T_fast - upper_limit;
      int pwm = (int)(BASE_PAYOUT_PWM + KP_PWM_PER_G * over_g);
      pwm = constrain(pwm, BASE_PAYOUT_PWM, MAX_PAYOUT_PWM);
      targetPWM = -pwm;
      loosen_grace_until_ms = now + LOOSEN_GRACE_MS; // 每次有效放線都延長寬鬆期
      no_retract_until_ms   = max(no_retract_until_ms, now + POST_BURST_NO_RETRACT_MS);
      inRetract = false; // 放線時確保不在回收狀態
    } else {
      if (now < burst_until_ms) {
        targetPWM = -BURST_PWM;
        loosen_grace_until_ms = now + LOOSEN_GRACE_MS;
        no_retract_until_ms   = max(no_retract_until_ms, now + POST_BURST_NO_RETRACT_MS);
        inRetract = false;
      } else {
        targetPWM = 0; // 等待
      }
    }

  } else if (T_slow < (lower_limit - LOWER_HYST_G)
             && now >= loosen_grace_until_ms
             && now >= no_retract_until_ms)              // ★ 禁回收窗生效
  {
    // 允許回收
    if (!inRetract) { inRetract = true; retract_start_ms = now; retract_start_cnt = cnt; }
    targetPWM = +remotorSpeed;

  } else if (T_fast < lower_limit) {
    targetPWM = 0;

  } else {
    // 區間內：若先前在回收，仍需受「禁回收窗/寬鬆期」約束
    if (inRetract) {
      bool time_ok = (now - retract_start_ms) >= MIN_RETRACT_MS;
      bool cnt_ok  = (labs(cnt - retract_start_cnt) >= MIN_RETRACT_CNT);
      bool window_ok = (now >= loosen_grace_until_ms) && (now >= no_retract_until_ms); // ★ 新增
      targetPWM = (time_ok && cnt_ok && window_ok) ? 0 : +remotorSpeed;
      if (sudden_pull) {
        targetPWM = -BURST_PWM;        // 立刻強制放線
        burst_until_ms = now + BURST_MS;
        loosen_grace_until_ms = now + LOOSEN_GRACE_MS;
        no_retract_until_ms   = now + POST_BURST_NO_RETRACT_MS;
        inRetract = false;
        Serial.println("[BURST] Forced interrupt during retract!");
      }
      if (time_ok && cnt_ok && window_ok) inRetract = false;
      // 若進入放線寬鬆或禁回收窗，強制終止回收
      if (!window_ok) { targetPWM = 0; inRetract = false; }
    } else {
      targetPWM = 0;
    }
  }

  // ---- Active-Coast：只在接近上限 0.5 g 內才給輕助力 ----
  bool nearly_upper = (T_fast >= (upper_limit - 0.6f));
  if (targetPWM == 0 && nearly_upper && spool_static) {
    targetPWM = -ASSIST_PWM;
  }

  // ---- 脈衝期間直通，其餘走 ramp ----
  bool burst_active = (now < burst_until_ms);
  int steppedPWM = burst_active ? targetPWM : slewPWM(targetPWM);

  // ---- 輸出 ----
  driveMotor(steppedPWM);

  // ---- Debug ----
  if ((millis() % 100) < LOOP_MS) {
    Serial.print("T_fast(g)="); Serial.print(T_fast, 2);
    Serial.print("  T_slow(g)="); Serial.print(T_slow, 2);
    Serial.print("  v_line(m/s)="); Serial.print(v_line, 3);
    Serial.print("  mode=");
    if (steppedPWM > 0) Serial.print("RETRACT");
    else if (steppedPWM < 0) Serial.print("LOOSEN");
    else Serial.print("STOP");
    Serial.print("  allow="); Serial.print(allow_payout ? "Y" : "N");
    Serial.print("  dTdt_raw="); Serial.print(dTdt_raw, 0);
    Serial.print("  dTdt_f="); Serial.print(dTdt_gps, 0);
    Serial.print("  grace(ms)="); Serial.print(max(0, (int)(loosen_grace_until_ms - now)));
    Serial.print("  noR(ms)="); Serial.print(max(0, (int)(no_retract_until_ms - now))); // ★ 新增
    Serial.print("  burst="); Serial.print(burst_active ? "Y":"N");
    //Serial.print("  hxUpd="); Serial.print(hx_updated ? "Y":"N");
    Serial.println();
  }
}

// ---------------- ISR ----------------
void encoderISR() {
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);
  if (a == b) encoderCount++;
  else        encoderCount--;
}
