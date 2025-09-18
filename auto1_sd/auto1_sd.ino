#include "HX711.h"
#include <SPI.h>
#include <SD.h>

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
// SD module
const int SD_CS = 10;   // CS pin of SD card

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
float v_line = 0.0f;         
float leash_len_m = 0.0f;

// ---------------- Motor / PWM ----------------
const int remotorSpeed = 65;   // retract PWM (upper bound)
int pwm_state = 0;
const int PWM_STEP_OUT = 120;   
const int PWM_STEP_IN  = 30;   
// retracting PWM
const int   BASE_RETRACT_PWM     = 45;     // base PWM
const int   MAX_RETRACT_PWM      = 70;     // Maximum PWM
const int   MIN_RETRACT_PWM      = 40;     // Minimum
const float KP_RETRACT_PWM_PER_G = 3.0f;   // gain per gram lower than lowerlimit

// loosening PWM
const int   BASE_PAYOUT_PWM   = 70;
const int   MAX_PAYOUT_PWM    = 190;
const float KP_PWM_PER_G      = 4.0f; // gain per gram after surpass upperlimit

// ---------------- Burst（instantpayout for strong drag） ----------------
const int BURST_PWM = 230;                     
const unsigned long BURST_MS = 150;            
unsigned long burst_until_ms = 0;

// ---------------- Landing impact give-way ----------------
const float IMPACT_DTDT_GPS = 90.0f;            
const float IMPACT_NEAR_UPPER_G = 0.8f;         
const unsigned long IMPACT_COAST_MS = 120;      
const int IMPACT_PAYOUT_PWM = 70;               
unsigned long impact_until_ms = 0;              

// No retracting after burst
const unsigned long POST_BURST_NO_RETRACT_MS = 600;     
unsigned long no_retract_until_ms = 0;

// ---------------- Smoothen tension ----------------
float T_fast_g = 0.0f;                
float T_slow_g = 0.0f;                
const float T_ALPHA_FAST = 0.8f;      // ★ bigger to be more sensitive 
const float T_ALPHA_SLOW = 0.25f;

// ---------------- threshold ----------------
float lower_limit = 11.7f;
float upper_limit = 19.5f;

float V_OUT_MIN = 0.004f;             
float T_MARGIN_G = 0.7f;
unsigned long ALLOW_PAYOUT_CONFIRM_MS = 5;  
bool allow_payout_latch = false;
unsigned long allow_payout_since_ms = 0;

// 鎖軸快速通道
const float V_EPS_LOCKED   = 0.003f;
const float T_LOCK_MARGING = 3.0f;
const float DTDT_LOCK_GPS  = 55.0f;
const unsigned long LOCK_CONFIRM_MS = 12;
bool locked_latch = false;
unsigned long locked_since_ms = 0;

// ---- dT/dt ----
float dTdt_raw = 0.0f;
float dTdt_gps = 0.0f;
float T_prev_for_dt = 0.0f;
const float DTDTA = 0.25f;

// Active-Coast
const int ASSIST_PWM = 140;               

// ---------------- Anti-chatter / Grace ----------------
bool inRetract = false;
unsigned long retract_start_ms;
long          retract_start_cnt;

const unsigned long MIN_RETRACT_MS  = 50;
const long          MIN_RETRACT_CNT = 10;

const unsigned long LOOSEN_GRACE_MS = 800;
unsigned long loosen_grace_until_ms = 0;

const float LOWER_HYST_G = 0.8f;

// ---------------- Loop time ----------------
const unsigned long LOOP_MS = 1;
unsigned long last_loop_ms = 0;

// ================= SD Logging =================
File logFile;
bool sd_ok = false;
unsigned long last_log_ms = 0;
const unsigned long LOG_INTERVAL_MS = 100; 
uint32_t log_lines_since_flush = 0;
const uint32_t FLUSH_EVERY = 20;           

// File check LOG00.CSV ~ LOG99.CSV
bool openLogFile() {
  char name[12] = "LOG00.CSV";
  for (int i = 0; i < 100; i++) {
    name[3] = '0' + (i/10);
    name[4] = '0' + (i%10);
    if (!SD.exists(name)) {
      logFile = SD.open(name, FILE_WRITE);
      if (logFile) {
        logFile.println(F("ms,T_fast_g,T_slow_g,v_line_mps,leash_m,pwm_state,enc_count,dTdt_gps"));
        logFile.flush();
        return true;
      } else {
        return false;
      }
    }
  }
  return false; 
}

void logLineCSV(unsigned long ms,
                float T_fast, float T_slow,
                float vline, float leash,
                int pwm, long enc, float dTdt) {
  if (!sd_ok || !logFile) return;
  logFile.print(ms);
  logFile.print(',');
  logFile.print(T_fast, 4);
  logFile.print(',');
  logFile.print(T_slow, 4);
  logFile.print(',');
  logFile.print(vline, 6);
  logFile.print(',');
  logFile.print(leash, 4);
  logFile.print(',');
  logFile.print(pwm);
  logFile.print(',');
  logFile.print(enc);
  logFile.print(',');
  logFile.println(dTdt, 2);

  if (++log_lines_since_flush >= FLUSH_EVERY) {
    log_lines_since_flush = 0;
    logFile.flush(); 
  }
}

// ---------------- helpers ----------------
inline void motorCoast() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void driveMotor(int pwmSigned) {
  if (pwmSigned == 0) { motorCoast(); return; }
  if (pwmSigned > 0) {           
    int pwm = constrain(pwmSigned, 0, 255);
    digitalWrite(AIN1, LOW);
    analogWrite(AIN2, pwm);
  } else {                       
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

int map_constrain_float(float x, float in_min, float in_max, int out_min, int out_max) {
  if (in_max - in_min == 0) return out_min;
  float t = (x - in_min) / (in_max - in_min);
  if (t < 0) t = 0; if (t > 1) t = 1;
  float y = out_min + t * (out_max - out_min);
  return (int)y;
}

int applyDeadzone(int pwmSigned) {
  const int DEADZONE = 60;   
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

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH); 
  if (SD.begin(SD_CS)) {
    sd_ok = openLogFile();
    if (sd_ok) Serial.println(F("SD ready, logging started."));
    else       Serial.println(F("SD present but open file failed."));
  } else {
    Serial.println(F("SD init failed. Check wiring/level shifting."));
  }

  scale.begin(DOUT, CLK);
  delay(1500);
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

  Serial.println("Ready ver5.instant-payout (no ramp, raw trigger).");
}

// ---------------- loop ----------------
void loop() {
  if (millis() - last_loop_ms < LOOP_MS) return;
  last_loop_ms = millis();

  // ---- 張力 ----
  hx_updated = false;
  float raw = 0.0f;
  if (scale.is_ready()) {
    raw = scale.get_units(1);
    T_fast_g = T_fast_g + T_ALPHA_FAST * (raw - T_fast_g);
    T_slow_g = T_slow_g + T_ALPHA_SLOW * (raw - T_slow_g);
    hx_updated = true;
  }
  float T_fast = -1*T_fast_g;
  float T_slow = -1*T_slow_g;

  unsigned long now = millis();
  unsigned long dt_ms = now - lastVtMs; if (dt_ms == 0) dt_ms = 1;
  lastVtMs = now;

  long cnt; noInterrupts(); cnt = encoderCount; interrupts();
  long dcount = cnt - lastCount; lastCount = cnt;

  float dt = dt_ms * 1e-3f;
  v_line = (COUNTS_PER_METER > 1e-6f) ? ((float)dcount / COUNTS_PER_METER) / dt : 0.0f;
  leash_len_m += v_line * dt;

  if (hx_updated) {
    float dtT_s = max(0.001f, (now - last_hx_ms) * 1e-3f);
    if (!have_prev_T) { dTdt_raw = 0.0f; have_prev_T = true; }
    else               dTdt_raw = (T_fast - T_prev_for_dt) / dtT_s;
    T_prev_for_dt = T_fast;
    last_hx_ms = now;
  }
  dTdt_gps = DTDTA * dTdt_raw + (1.0f - DTDTA) * dTdt_gps;

  // ---- 瞬間放線判斷 ----
  bool sudden_pull = ((raw > upper_limit + 15.0f) || (T_fast > upper_limit + 15.0f)) && (dTdt_gps > 70.0f);
  bool not_retracting = (v_line <= 0.0f);
  bool instant_payout = sudden_pull && not_retracting;

  if (instant_payout) {
    unsigned long new_until = now + BURST_MS;
    if (new_until > burst_until_ms) burst_until_ms = new_until;
    loosen_grace_until_ms = max(loosen_grace_until_ms, now + LOOSEN_GRACE_MS);
    no_retract_until_ms   = max(no_retract_until_ms,   now + POST_BURST_NO_RETRACT_MS);
    inRetract = false;
    Serial.println("00000");
    allow_payout_latch = true;
    allow_payout_since_ms = now;
  }

  // ---- 即時 burst ----
  if (now < burst_until_ms) {
    int steppedPWM = slewPWM(-BURST_PWM);
    steppedPWM = applyDeadzone(steppedPWM);
    driveMotor(steppedPWM);
    return;
  }

  // ---- 落地衝擊讓步 ----
  bool impact_detect = inRetract && ((dTdt_gps > IMPACT_DTDT_GPS) || (T_fast > upper_limit - IMPACT_NEAR_UPPER_G));
  if (impact_detect) {
    unsigned long new_until = now + IMPACT_COAST_MS;
    if (new_until > impact_until_ms) impact_until_ms = new_until;
  }
  if (now < impact_until_ms) {
    int targetPWM = 0;
    if (T_fast > upper_limit - 0.3f) {
      targetPWM = -IMPACT_PAYOUT_PWM; 
    }
    int steppedPWM = slewPWM(targetPWM);
    steppedPWM = applyDeadzone(steppedPWM);
    driveMotor(steppedPWM);
    inRetract = false;
    return;
  }

  // ---- 放線許可 ----
  bool line_out = (v_line < -V_OUT_MIN);
  bool high_T   = (T_fast > (upper_limit + T_MARGIN_G));
  if (high_T) {
    if (!allow_payout_latch) { allow_payout_latch = true; allow_payout_since_ms = now; }
  } else {
    allow_payout_latch = false;
  }
  bool allow_payout_dir = allow_payout_latch && (now - allow_payout_since_ms >= ALLOW_PAYOUT_CONFIRM_MS);

  bool spool_static = (fabs(v_line) < 0.003f);
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
             && now >= no_retract_until_ms) {
    if (!inRetract) { inRetract = true; retract_start_ms = now; retract_start_cnt = cnt; }
  float under_g = (lower_limit - T_slow);
  int pwm = (int)(BASE_RETRACT_PWM + KP_RETRACT_PWM_PER_G * under_g);
  pwm = constrain(pwm, MIN_RETRACT_PWM, MAX_RETRACT_PWM);
  targetPWM = +pwm;
  } else if (T_fast < lower_limit) {
    targetPWM = 0;
  } else {
    if (inRetract) {
      if (T_fast > upper_limit - 0.8f) { targetPWM = 0; inRetract = false; }
      else {
        bool time_ok = (now - retract_start_ms) >= MIN_RETRACT_MS;
        bool cnt_ok  = (labs(cnt - retract_start_cnt) >= MIN_RETRACT_CNT);
        bool window_ok = (now >= loosen_grace_until_ms) && (now >= no_retract_until_ms);
        if (time_ok && cnt_ok && window_ok) { targetPWM = 0; inRetract = false; }
        else {
          int cap = map_constrain_float(T_fast, lower_limit - 1.0f, upper_limit - 0.5f, remotorSpeed, 10);
          cap = constrain(cap, 10, remotorSpeed);
          targetPWM = +cap;
        }
        if (!window_ok) { targetPWM = 0; inRetract = false; }
      }
    } else {
      targetPWM = 0;
    }
  }

  bool nearly_upper = (T_fast >= (upper_limit - 0.3f));
  if (targetPWM == 0 && nearly_upper && spool_static) {
    targetPWM = -ASSIST_PWM;
  }

  if (now < loosen_grace_until_ms || now < no_retract_until_ms) {
    if (targetPWM > 0) { targetPWM = 0; }
  }

  int steppedPWM = slewPWM(targetPWM);
  steppedPWM = applyDeadzone(steppedPWM);
  driveMotor(steppedPWM);

  if ((millis() % 100) < LOOP_MS) {
    Serial.print("T_fast(g)="); Serial.print(T_fast, 2);
    Serial.print("  T_slow(g)="); Serial.print(T_slow, 2);
    Serial.print("  v_line(m/s)="); Serial.print(v_line, 3);
    Serial.print("  PWM="); Serial.print(steppedPWM);
    Serial.println();
  }

  if (sd_ok && (now - last_log_ms >= LOG_INTERVAL_MS)) {
    last_log_ms = now;
    long enc_now; noInterrupts(); enc_now = encoderCount; interrupts();
    logLineCSV(now, T_fast, T_slow, v_line, leash_len_m, steppedPWM, enc_now, dTdt_gps);
  }
}

// ---------------- ISR ----------------
void encoderISR() {
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);
  if (a == b) encoderCount++;
  else        encoderCount--;
}
