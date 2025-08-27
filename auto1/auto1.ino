#include "HX711.h"

// ---------------- Pins ----------------
// HX711
#define DOUT 4
#define CLK  5
// Motor
#define AIN1 6
#define AIN2 9
// Encoder pins
#define ENCODER_A 2
#define ENCODER_B 3

// ---------------- HX711 ----------------
HX711 scale;
float calibration_factor = 4180;  // g
const float HX_RATE_HZ = 80.0f;   // soldering modified
unsigned long last_hx_ms = 0;
bool hx_updated = false;
bool have_prev_T = false;

// ---------------- Encoder → v_line ----------------
const float COUNTS_PER_METER = 11357.0f;
volatile long encoderCount = 0;
long lastCount = 0;
unsigned long lastVtMs = 0;
float v_line = 0.0f;         // motor loosening < 0, retracting > 0
float leash_len_m = 0.0f;

// ---------------- Motor / PWM ----------------
const int remotorSpeed = 50;   // retract PWM
int pwm_state = 0;
const int PWM_STEP_OUT = 70;   // loosen ramp
const int PWM_STEP_IN  = 4;    // retract ramp

// 放線 PWM（基礎 + 比例）
const int   BASE_PAYOUT_PWM   = 190;
const int   MAX_PAYOUT_PWM    = 255;
const float KP_PWM_PER_G      = 20.0f;

// ---------------- Burst（強制放線） ----------------
const int BURST_PWM = 140;                 // 全速放線 PWM
const unsigned long BURST_MS = 30;         // 鎖定時間
unsigned long burst_until_ms = 0;

// Burst 後禁止回收視窗
const unsigned long POST_BURST_NO_RETRACT_MS = 600;
unsigned long no_retract_until_ms = 0;

// ---------------- 張力平滑 ----------------
float T_fast_g = 0.0f;                // 快 EMA（進入放線）
float T_slow_g = 0.0f;                // 慢 EMA（退出/回收）
const float T_ALPHA_FAST = 0.50f;
const float T_ALPHA_SLOW = 0.25f;

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

// ---- dT/dt（僅新樣本時計）+ EMA ----
float dTdt_raw = 0.0f;
float dTdt_gps = 0.0f;
float T_prev_for_dt = 0.0f;
const float DTDTA = 0.25f;

// Active-Coast：接近上限時的微放
const int ASSIST_PWM = 60;

// ---------------- Anti-chatter / Grace ----------------
bool inRetract = false;
unsigned long retract_start_ms;
long          retract_start_cnt;

const unsigned long MIN_RETRACT_MS  = 50;
const long          MIN_RETRACT_CNT = 10;

const unsigned long LOOSEN_GRACE_MS = 300;
unsigned long loosen_grace_until_ms = 0;

const float LOWER_HYST_G = 0.8f;

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

// --- 死區補償 ---
int applyDeadzone(int pwmSigned) {
  const int DEADZONE = 40;   // 依你測到的死區設定
  if (pwmSigned == 0) return 0;

  if (pwmSigned > 0) {
    return max(pwmSigned, DEADZONE);
  } else {
    return min(pwmSigned, -DEADZONE);
  }
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

  Serial.println("Ready ver2.xc (deadzone compensated, global burst-lock).");
}

// ---------------- loop ----------------
void loop() {
  if (millis() - last_loop_ms < LOOP_MS) return;
  last_loop_ms = millis();

  // ---- 張力：雙 EMA（僅新樣本時更新）----
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

  // ---- dT/dt（僅新樣本時計 raw，再 EMA）----
  if (hx_updated) {
    float dtT_s = max(0.001f, (now - last_hx_ms) * 1e-3f);
    if (!have_prev_T) { dTdt_raw = 0.0f; have_prev_T = true; }
    else               dTdt_raw = (T_fast - T_prev_for_dt) / dtT_s;
    T_prev_for_dt = T_fast;
    last_hx_ms = now;
  }
  dTdt_gps = DTDTA * dTdt_raw + (1.0f - DTDTA) * dTdt_gps;

  // ---- 突發強拉判斷（非回收狀態才算）----
  bool sudden_pull    = (T_fast > upper_limit + 5.0f) && (dTdt_gps > 80.0f);
  bool not_retracting = (v_line <= 0.0f);  // 沒有在主動收線
  bool instant_payout = sudden_pull && not_retracting;

  // ---- 觸發 burst：延長並全域鎖定 ----
  if (instant_payout) {
    unsigned long new_until = now + BURST_MS;
    if (new_until > burst_until_ms) burst_until_ms = new_until;

    // 伴隨的防抖策略
    loosen_grace_until_ms = max(loosen_grace_until_ms, now + LOOSEN_GRACE_MS);
    no_retract_until_ms   = max(no_retract_until_ms,   now + POST_BURST_NO_RETRACT_MS);
    inRetract = false;
    allow_payout_latch = true;
    allow_payout_since_ms = now;
  }

  // ---------------- 全域 burst 鎖定層 ----------------
  if (now < burst_until_ms) {
    pwm_state = -BURST_PWM;
    driveMotor(pwm_state);
    return; // 確保不中斷
  }

  // ---- 放線許可 ----
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

  // ---- 主邏輯 ----
  int targetPWM = 0;

  if (T_fast > upper_limit) {
    if (allow_payout) {
      float over_g = T_fast - upper_limit;
      int pwm = (int)(BASE_PAYOUT_PWM + KP_PWM_PER_G * over_g);
      pwm = constrain(pwm, BASE_PAYOUT_PWM, MAX_PAYOUT_PWM);
      targetPWM = -pwm;
      loosen_grace_until_ms = now + LOOSEN_GRACE_MS;
      no_retract_until_ms   = max(no_retract_until_ms, now + POST_BURST_NO_RETRACT_MS);
      inRetract = false;
    } else {
      targetPWM = 0;
    }

  } else if (T_slow < (lower_limit - LOWER_HYST_G)
             && now >= loosen_grace_until_ms
             && now >= no_retract_until_ms)
  {
    if (!inRetract) { inRetract = true; retract_start_ms = now; retract_start_cnt = cnt; }
    targetPWM = +remotorSpeed;

  } else if (T_fast < lower_limit) {
    targetPWM = 0;

  } else {
    if (inRetract) {
      bool time_ok = (now - retract_start_ms) >= MIN_RETRACT_MS;
      bool cnt_ok  = (labs(cnt - retract_start_cnt) >= MIN_RETRACT_CNT);
      bool window_ok = (now >= loosen_grace_until_ms) && (now >= no_retract_until_ms);
      targetPWM = (time_ok && cnt_ok && window_ok) ? 0 : +remotorSpeed;
      if (time_ok && cnt_ok && window_ok) inRetract = false;
      if (!window_ok) { targetPWM = 0; inRetract = false; }
    } else {
      targetPWM = 0;
    }
  }

  // ---- Active-Coast ----
  bool nearly_upper = (T_fast >= (upper_limit - 0.5f));
  if (targetPWM == 0 && nearly_upper && spool_static) {
    targetPWM = -ASSIST_PWM;
  }

  if (now < loosen_grace_until_ms || now < no_retract_until_ms) {
    if (targetPWM > 0) { targetPWM = 0; }
  }

  // ---- 正常輸出 ----
  int steppedPWM = slewPWM(targetPWM);
  steppedPWM = applyDeadzone(steppedPWM); // ★加上死區補償
  driveMotor(steppedPWM);

  // ---- Debug ----
  if ((millis() % 100) < LOOP_MS) {
    Serial.print("T_fast(g)="); Serial.print(T_fast, 2);
    Serial.print("  T_slow(g)="); Serial.print(T_slow, 2);
    Serial.print("  v_line(m/s)="); Serial.print(v_line, 3);
    Serial.print("  PWM="); Serial.print(steppedPWM);
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
