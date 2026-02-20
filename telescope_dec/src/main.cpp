/*
  Nano v3 + TMC2209 + TMCStepper
  Wiring (typical 1-wire UART):
    
    GND common, VM+motor power as usual.

  STEP/DIR/EN to your driver module pins.
*/

#include <Arduino.h>
#include <AltSoftSerial.h>
#include <TMCStepper.h>
#include <stdio.h>

// ---------- Pins ----------
static const uint8_t STEP_PIN = 10;
static const uint8_t DIR_PIN  = 3;
static const uint8_t EN_PIN   = 4;   // Enable pin to driver
static const bool    EN_ACTIVE_LOW = true;

static const uint8_t TMC_RX_PIN = 8;
static const uint8_t TMC_TX_PIN = 9;

// ---------- TMC config ----------
static const uint32_t TMC_BAUD = 9600;
// Most SilentStepStick-like modules use 0.11 ohm; check your board to be correct.
static const float R_SENSE = 0.11f;
// Address depends on MS1/MS2 (CFG pins) strapping; often 0b00 if both low.
static const uint8_t DRIVER_ADDRESS = 0b00;

AltSoftSerial TMCSerial(TMC_RX_PIN, TMC_TX_PIN);
TMC2209Stepper driver(&TMCSerial, R_SENSE, DRIVER_ADDRESS);

// ---------- Motion state (v2) ----------
struct RunnerV2 {
  bool enabled = false;
  bool dir = false;
  bool running = false;
  bool stopRequested = false;
  bool hasTarget = false;
  long target = 0;
  float speedSps = 500.0f;
  float actualSpeedSps = 0.0f;
  float desiredSpeedSps = 0.0f;
  float accelStepsPerUs = 0.001f;
  uint32_t stepIntervalUs = 2000;
  uint32_t pulseWidthUs   = 3;
  uint32_t nextStepUs     = 0;
  bool stepHigh = false;
  uint32_t stepHighUntilUs = 0;
  uint32_t lastStepUs = 0;
  uint32_t lastUpdateUs = 0;
  uint32_t lastStepperUs = 0;
  float stepAcc = 0.0f;
} runV2;

static bool v2Initialized = false;

struct Profiler {
  float serial = 0.0;
  float serialLastProcessed = 0.0;

  float stepper = 0.0;
  float updateMotionState = 0.0;
  float stepperHigh = 0.0;
  float stepperLow = 0.0;
} profiler;

// ---------- Step position counter ----------
static const long STEP_POSITIVE = 1;
static const long STEP_NEGATIVE = -1;

static long stepPosition = 0;

static inline void setPosition(long value) {
  stepPosition = value;
}

static inline long getPosition() {
  return stepPosition;
}

static const char* const PHASE_IDLE_V2 = "idle";
static const char* const PHASE_HOLD_V2 = "hold";
static const char* const PHASE_ACCEL_V2 = "acceleration";
static const char* const PHASE_RUN_V2 = "running";
static const char* const PHASE_DECEL_V2 = "deceleration";

// ---------- V2 formatting ----------
static const uint8_t HEX_WIDTH = 8;
static const uint8_t HEX_PREFIX_LEN = 2;
static const uint8_t HEX_BUF_LEN = HEX_PREFIX_LEN + HEX_WIDTH + 1;

// ---------- Simple line parser ----------
static char lineBufV2[256];
static uint8_t lineLenV2 = 0;

// ---------- Fast TX ring buffer ----------
static char outBufV2[256];
static uint8_t outWriteV2 = 0;
static uint8_t outReadV2 = 0;

static inline bool outHasDataV2() {
  return outWriteV2 != outReadV2;
}

static inline bool outPushV2(char c) {
  uint8_t next = (uint8_t)(outWriteV2 + 1);
  if (next == outReadV2) return false;
  outBufV2[outWriteV2] = c;
  outWriteV2 = next;
  return true;
}

static inline bool outPopV2(char* c) {
  if (!outHasDataV2()) return false;
  *c = outBufV2[outReadV2];
  outReadV2 = (uint8_t)(outReadV2 + 1);
  return true;
}

static inline void outAppendCharV2(char c) {
  outPushV2(c);
}

static inline void outAppendStrV2(const char* s) {
  if (!s) return;
  while (*s) {
    if (!outPushV2(*s++)) return;
  }
}

static inline void outAppendNumLongV2(long v) {
  char tmp[24];
  int n = snprintf(tmp, sizeof(tmp), "%ld", v);
  if (n > 0) {
    for (int i = 0; i < n; i++) {
      if (!outPushV2(tmp[i])) break;
    }
  }
}

static inline void outAppendNumU32V2(uint32_t v) {
  char tmp[24];
  int n = snprintf(tmp, sizeof(tmp), "%lu", (unsigned long)v);
  if (n > 0) {
    for (int i = 0; i < n; i++) {
      if (!outPushV2(tmp[i])) break;
    }
  }
}

static inline void outAppendNumFloatV2(float v, uint8_t decimals) {
  // dtostrf is faster than repeated Serial.print on AVR
  char tmp[32];
  dtostrf(v, 0, decimals, tmp);
  // dtostrf may pad leading spaces; trim them
  char* p = tmp;
  while (*p == ' ') p++;
  outAppendStrV2(p);
}

static inline void outAppendHexU32V2(uint32_t v) {
  char tmp[HEX_BUF_LEN];
  snprintf(tmp, sizeof(tmp), "0x%0*lX", HEX_WIDTH, (unsigned long)v);
  outAppendStrV2(tmp);
}

static inline void outFlushLineV2() {
  outAppendCharV2('\n');
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(0);
  while (!Serial) {}

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  runV2.dir = false;
  digitalWrite(DIR_PIN, LOW);
  runV2.enabled = false;
  digitalWrite(EN_PIN, EN_ACTIVE_LOW ? HIGH : LOW);
  runV2.lastStepUs = micros();
  runV2.lastUpdateUs = micros();
  runV2.nextStepUs = 0;

  TMCSerial.begin(TMC_BAUD);

  // while (!TMCSerial) {};

  // Basic driver init (safe-ish defaults; tune later)
  driver.begin();
  driver.pdn_disable(true);          // use UART
  driver.I_scale_analog(false);      // use internal current reference
  driver.mstep_reg_select(true);     // microsteps via registers (UART)
  driver.toff(4);                    // enable driver
  driver.blank_time(24);
  driver.rms_current(600);           // RMS mA; adjust for your motor
  driver.microsteps(16);
  driver.en_spreadCycle(false);      // stealth by default
  driver.pwm_autoscale(true);

  // Clear latched flags
  driver.GSTAT(0x7);

  v2Initialized = true;
  Serial.println(F("ready"));
}

static inline void setEnableV2(bool on) {
  runV2.enabled = on;
  digitalWrite(EN_PIN, (EN_ACTIVE_LOW ? !on : on) ? HIGH : LOW);
}

static inline void setDirV2(bool dir) {
  runV2.dir = dir;
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);
}

static bool parseLongV2(const char* s, long* value) {
  if (!s || !*s) return false;
  char* end = nullptr;
  long parsed = strtol(s, &end, 10);
  if (end == s || *end != 0) return false;
  *value = parsed;
  return true;
}

static bool parseU32V2(const char* s, uint32_t* value) {
  if (!s || !*s) return false;
  char* end = nullptr;
  unsigned long parsed = strtoul(s, &end, 10);
  if (end == s || *end != 0) return false;
  *value = (uint32_t)parsed;
  return true;
}

static void respondStartV2(bool ok) {
  outAppendCharV2(ok ? '1' : '0');
  outAppendCharV2(';');
}

static void respondEndV2() {
  outFlushLineV2();
}

static void respondKeyValueLongV2(const char* key, long value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendNumLongV2(value);
  outAppendCharV2(';');
}

static void respondKeyValueU32V2(const char* key, uint32_t value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendNumU32V2(value);
  outAppendCharV2(';');
}

static void respondKeyValueBoolV2(const char* key, bool value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendCharV2(value ? '1' : '0');
  outAppendCharV2(';');
}

static void respondKeyValueFloatV2(const char* key, float value, uint8_t decimals) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendNumFloatV2(value, decimals);
  outAppendCharV2(';');
}

static void respondKeyValueStrV2(const char* key, const char* value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendStrV2(value);
  outAppendCharV2(';');
}

static void respondKeyValueHexV2(const char* key, uint32_t value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendHexU32V2(value);
  outAppendCharV2(';');
}

static void respondErrorV2(const char* msg) {
  respondStartV2(false);
  if (msg && *msg) {
    respondKeyValueStrV2("error", msg);
  }
  respondEndV2();
}

static char* stripParamPrefixV2(char* token) {
  if (token && token[0] == ':') return token + 1;
  return token;
}

static bool parseNameValueTokenV2(char* token, char** name, char** value) {
  if (!token || !name || !value) return false;
  char* trimmed = stripParamPrefixV2(token);
  char* eq = strchr(trimmed, '=');
  if (!eq) return false;
  *eq = 0;
  *name = trimmed;
  *value = eq + 1;
  if (!**name || !**value) return false;
  return true;
}

static const uint16_t MICROSTEPS_ALLOWED_V2[] = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };
static const uint32_t TPWMTHRS_MAX_V2 = 0xFFFFF;

static bool isMicrostepsAllowedV2(uint16_t value) {
  for (uint8_t i = 0; i < (sizeof(MICROSTEPS_ALLOWED_V2) / sizeof(MICROSTEPS_ALLOWED_V2[0])); i++) {
    if (MICROSTEPS_ALLOWED_V2[i] == value) return true;
  }
  return false;
}

// --- Microsteps/CHOPCONF helpers ---
static inline uint16_t microstepsFromChopconfV2(uint32_t chopconf) {
  // CHOPCONF.MRES bits [24..27], encoding: 0->256, 1->128, 2->64, ..., 8->1
  const uint8_t mres = (uint8_t)((chopconf >> 24) & 0x0F);
  if (mres >= 8) return 1;
  return (uint16_t)(256U >> mres);
}

static inline bool intpolFromChopconfV2(uint32_t chopconf) {
  // CHOPCONF.INTPOL bit 28
  return ((chopconf >> 28) & 0x01) != 0;
}

static uint8_t calcCurrentScaleFromMaV2(uint16_t mA, bool highSense) {
  const float vfs = highSense ? 0.180f : 0.325f;
  float cs = 32.0f * 1.41421f * mA / 1000.0f * (R_SENSE + 0.02f) / vfs - 1.0f;
  if (cs < 0.0f) cs = 0.0f;
  if (cs > 31.0f) cs = 31.0f;
  return (uint8_t)(cs + 0.5f);
}

static uint8_t calcRunCurrentScaleFromMaV2(uint16_t mA) {
  float cs = 32.0f * 1.41421f * mA / 1000.0f * (R_SENSE + 0.02f) / 0.325f - 1.0f;
  if (cs < 16.0f) {
    driver.vsense(true);
    cs = 32.0f * 1.41421f * mA / 1000.0f * (R_SENSE + 0.02f) / 0.180f - 1.0f;
  } else {
    driver.vsense(false);
  }
  if (cs < 0.0f) cs = 0.0f;
  if (cs > 31.0f) cs = 31.0f;
  return (uint8_t)(cs + 0.5f);
}

static uint8_t calcHoldCurrentScaleFromMaV2(uint16_t mA) {
  return calcCurrentScaleFromMaV2(mA, driver.vsense());
}

static bool appendParamValueV2(const char* name, bool emit) {
  if (!strcmp(name, "ihold")) {
    if (emit) respondKeyValueU32V2("ihold", driver.cs2rms(driver.ihold()));
    return true;
  }
  if (!strcmp(name, "irun")) {
    if (emit) respondKeyValueU32V2("irun", driver.cs2rms(driver.irun()));
    return true;
  }
  if (!strcmp(name, "tpowerdown")) {
    if (emit) respondKeyValueU32V2("tpowerdown", driver.TPOWERDOWN());
    return true;
  }
  if (!strcmp(name, "tpwmthrs")) {
    if (emit) respondKeyValueU32V2("tpwmthrs", driver.TPWMTHRS());
    return true;
  }
  if (!strcmp(name, "sgthrs")) {
    if (emit) respondKeyValueU32V2("sgthrs", driver.SGTHRS());
    return true;
  }
  if (!strcmp(name, "microsteps")) {
    if (emit) {
      const uint32_t chop = driver.CHOPCONF();
      respondKeyValueU32V2("microsteps", microstepsFromChopconfV2(chop));
    }
    return true;
  }
  if (!strcmp(name, "intpol")) {
    if (emit) {
      const uint32_t chop = driver.CHOPCONF();
      respondKeyValueBoolV2("intpol", intpolFromChopconfV2(chop));
    }
    return true;
  }
  if (!strcmp(name, "chopconf")) {
    if (emit) respondKeyValueHexV2("chopconf", driver.CHOPCONF());
    return true;
  }
  if (!strcmp(name, "stealth")) {
    if (emit) respondKeyValueBoolV2("stealth", !driver.en_spreadCycle());
    return true;
  }
  return false;
}

static bool applySetParamV2(const char* name, const char* value, const char** errorKey) {
  if (!name || !value) {
    if (errorKey) *errorKey = "bad_param";
    return false;
  }
  if (!strcmp(name, "ihold")) {
    long mA = 0;
    if (!parseLongV2(value, &mA)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (mA < 0 || mA > 2000) { if (errorKey) *errorKey = "range"; return false; }
    driver.ihold(calcHoldCurrentScaleFromMaV2((uint16_t)mA));
    return true;
  }
  if (!strcmp(name, "irun")) {
    long mA = 0;
    if (!parseLongV2(value, &mA)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (mA < 0 || mA > 2000) { if (errorKey) *errorKey = "range"; return false; }
    driver.irun(calcRunCurrentScaleFromMaV2((uint16_t)mA));
    return true;
  }
  if (!strcmp(name, "tpowerdown")) {
    long ticks = 0;
    if (!parseLongV2(value, &ticks)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (ticks < 0 || ticks > 255) { if (errorKey) *errorKey = "range"; return false; }
    driver.TPOWERDOWN((uint8_t)ticks);
    return true;
  }
  if (!strcmp(name, "tpwmthrs")) {
    uint32_t thrs = 0;
    if (!parseU32V2(value, &thrs)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (thrs > TPWMTHRS_MAX_V2) { if (errorKey) *errorKey = "range"; return false; }
    driver.TPWMTHRS(thrs);
    return true;
  }
  if (!strcmp(name, "sgthrs")) {
    long v = 0;
    if (!parseLongV2(value, &v)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (v < 0 || v > 255) { if (errorKey) *errorKey = "range"; return false; }
    driver.SGTHRS((uint8_t)v);
    return true;
  }
  if (!strcmp(name, "microsteps")) {
    long v = 0;
    if (!parseLongV2(value, &v)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (v < 1 || v > 256) { if (errorKey) *errorKey = "range"; return false; }
    if (!isMicrostepsAllowedV2((uint16_t)v)) { if (errorKey) *errorKey = "invalid_microsteps"; return false; }
    driver.microsteps((uint16_t)v);
    return true;
  }
  if (!strcmp(name, "intpol")) {
    long v = 0;
    if (!parseLongV2(value, &v)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (v != 0 && v != 1) { if (errorKey) *errorKey = "invalid_bool"; return false; }
    driver.intpol(v != 0);
    return true;
  }
  if (!strcmp(name, "stealth")) {
    long v = 0;
    if (!parseLongV2(value, &v)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (v != 0 && v != 1) { if (errorKey) *errorKey = "invalid_bool"; return false; }
    const bool stealth = (v != 0);
    driver.en_spreadCycle(!stealth);
    driver.pwm_autoscale(true);
    return true;
  }
  if (errorKey) *errorKey = "unknown_param";
  return false;
}

static bool setSpeedSpsV2(long sps, const char** errorKey) {
  if (sps < 0) { if (errorKey) *errorKey = "range"; return false; }
  if (sps > 40000) { if (errorKey) *errorKey = "range"; return false; }
  runV2.speedSps = (float)sps;
  return true;
}

static bool setAccelStepsPerUsV2(long accel, const char** errorKey) {
  if (accel < 0) { if (errorKey) *errorKey = "range"; return false; }
  if (accel > 100000) { if (errorKey) *errorKey = "range"; return false; }
  runV2.accelStepsPerUs = (float)accel / 1000000.0f;
  return true;
}

static void setDeltaV2(long delta) {
  runV2.target += delta;
  runV2.hasTarget = true;
}

static void completeTargetV2() {
  runV2.hasTarget = false;
  runV2.running = false;
  runV2.stopRequested = false;
  runV2.actualSpeedSps = 0.0f;
  runV2.desiredSpeedSps = 0.0f;
  runV2.nextStepUs = 0;
  runV2.lastStepperUs = 0;
  runV2.stepAcc = 0.0f;
}

static const char* getPhaseV2() {
  if (!runV2.enabled) return PHASE_IDLE_V2;
  if (runV2.actualSpeedSps <= 0.0f) return PHASE_HOLD_V2;
  if (runV2.desiredSpeedSps <= 0.0f) return PHASE_DECEL_V2;
  if (runV2.actualSpeedSps < runV2.desiredSpeedSps) return PHASE_ACCEL_V2;
  if (runV2.actualSpeedSps > runV2.desiredSpeedSps) return PHASE_DECEL_V2;
  return PHASE_RUN_V2;
}

static void updateMotionStateV2() {
  const uint32_t nowUs = micros();

  if (runV2.lastUpdateUs == 0) {
    runV2.lastUpdateUs = nowUs;
    return;
  }
  const uint32_t dtUs = nowUs - runV2.lastUpdateUs;
  if (dtUs == 0) return;
  runV2.lastUpdateUs = nowUs;

  if (!runV2.enabled) {
    runV2.actualSpeedSps = 0.0f;
    runV2.desiredSpeedSps = 0.0f;
    runV2.running = false;
    runV2.stopRequested = false;
    return;
  }

  if (runV2.hasTarget) {
    const long delta = runV2.target - getPosition();
    if (delta == 0) {
      completeTargetV2();
      return;
    }
    setDirV2(delta < 0);
  }

  float desired = 0.0f;
  if (runV2.stopRequested) {
    desired = 0.0f;
  } else if (runV2.running) {
    desired = runV2.speedSps;
  }

  if (runV2.hasTarget && desired > 0.0f && runV2.accelStepsPerUs > 0.0f) {
    const long delta = runV2.target - getPosition();
    const float accelSps2 = runV2.accelStepsPerUs * 1000000.0f;
    const float stoppingDistance = (runV2.actualSpeedSps * runV2.actualSpeedSps) / (2.0f * accelSps2);
    if ((float)labs(delta) <= stoppingDistance) {
      desired = 0.0f;
    }
  }

  runV2.desiredSpeedSps = desired;
  if (runV2.accelStepsPerUs <= 0.0f) {
    runV2.actualSpeedSps = desired;
  } else {
    const float deltaSpeed = runV2.accelStepsPerUs * (float)dtUs;
    if (runV2.actualSpeedSps < desired) {
      runV2.actualSpeedSps += deltaSpeed;
      if (runV2.actualSpeedSps > desired) runV2.actualSpeedSps = desired;
    } else if (runV2.actualSpeedSps > desired) {
      runV2.actualSpeedSps -= deltaSpeed;
      if (runV2.actualSpeedSps < desired) runV2.actualSpeedSps = desired;
    }
  }

  if (runV2.stopRequested && runV2.actualSpeedSps <= 0.0f) {
    runV2.stopRequested = false;
    runV2.running = false;
    runV2.actualSpeedSps = 0.0f;
    runV2.desiredSpeedSps = 0.0f;
  }

  profiler.updateMotionState += micros() - nowUs;
  profiler.updateMotionState /= 2.;

}

static void handleLineV2(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  if (!*s) return;

  char* cmd = strtok(s, " \t");
  if (!cmd) return;

  if (!strcmp(cmd, "status")) {
    respondStartV2(true);
    respondKeyValueBoolV2("initialised", v2Initialized);
    respondKeyValueBoolV2("enabled", runV2.enabled);
    respondKeyValueLongV2("position", getPosition());
    respondKeyValueStrV2("phase", getPhaseV2());
    respondKeyValueLongV2("target", runV2.target);
    respondKeyValueBoolV2("target_set", runV2.hasTarget);
    respondKeyValueFloatV2("speed", runV2.speedSps, 2);
    respondKeyValueFloatV2("actual_speed", runV2.actualSpeedSps, 2);
    respondKeyValueFloatV2("accel_per_s", runV2.accelStepsPerUs * 1000000., 2);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "full_status") || !strcmp(cmd, "driver_status")) {
    respondStartV2(true);
    respondKeyValueHexV2("gconf", driver.GCONF());
    respondKeyValueHexV2("chopconf", driver.CHOPCONF());
    respondKeyValueHexV2("drv_status", driver.DRV_STATUS());
    respondKeyValueU32V2("sg_result", driver.SG_RESULT());
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "get")) {
    char* name = strtok(nullptr, " \t");
    if (!name) { respondErrorV2("missing_param"); return; }
    name = stripParamPrefixV2(name);
    if (!appendParamValueV2(name, false)) { respondErrorV2("unknown_param"); return; }
    respondStartV2(true);
    appendParamValueV2(name, true);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "set")) {
    char* token = strtok(nullptr, " \t");
    char* name = nullptr;
    char* value = nullptr;
    while (token) {
      if (name) { respondErrorV2("single_param"); return; }
      if (!parseNameValueTokenV2(token, &name, &value)) { respondErrorV2("bad_param"); return; }
      token = strtok(nullptr, " \t");
    }
    if (!name) { respondErrorV2("missing_param"); return; }
    const char* errorKey = nullptr;
    if (!applySetParamV2(name, value, &errorKey)) { respondErrorV2(errorKey ? errorKey : "set_failed"); return; }
    respondStartV2(true);
    appendParamValueV2(name, true);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "position")) {
    char* a = strtok(nullptr, " \t");
    long value = 0;
    if (!a || !parseLongV2(a, &value)) { respondErrorV2("bad_value"); return; }
    setPosition(value);
    respondStartV2(true);
    respondKeyValueLongV2("position", getPosition());
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "enabled")) {
    char* a = strtok(nullptr, " \t");
    long value = 0;
    if (!a || !parseLongV2(a, &value)) { respondErrorV2("bad_value"); return; }
    const bool enable = (value != 0);
    setEnableV2(enable);
    if (!enable) {
      runV2.running = false;
      runV2.stopRequested = false;
      runV2.actualSpeedSps = 0.0f;
      runV2.desiredSpeedSps = 0.0f;
      runV2.nextStepUs = 0;
      runV2.lastStepperUs = 0;
      runV2.stepAcc = 0.0f;
    }
    respondStartV2(true);
    respondKeyValueBoolV2("enabled", runV2.enabled);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "direction")) {
    char* a = strtok(nullptr, " \t");
    long value = 0;
    if (!a || !parseLongV2(a, &value)) { respondErrorV2("bad_value"); return; }
    setDirV2(value != 0);
    respondStartV2(true);
    respondKeyValueBoolV2("direction", runV2.dir);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "speed")) {
    char* a = strtok(nullptr, " \t");
    long value = 0;
    if (!a || !parseLongV2(a, &value)) { respondErrorV2("bad_value"); return; }
    const char* errorKey = nullptr;
    if (!setSpeedSpsV2(value, &errorKey)) { respondErrorV2(errorKey ? errorKey : "range"); return; }
    respondStartV2(true);
    respondKeyValueFloatV2("speed", runV2.speedSps, 2);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "acceleration")) {
    char* a = strtok(nullptr, " \t");
    long value = 0;
    if (!a || !parseLongV2(a, &value)) { respondErrorV2("bad_value"); return; }
    const char* errorKey = nullptr;
    if (!setAccelStepsPerUsV2(value, &errorKey)) { respondErrorV2(errorKey ? errorKey : "range"); return; }
    respondStartV2(true);
    respondKeyValueFloatV2("accel_per_s", runV2.accelStepsPerUs * 1000000.0f, 2);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "delta")) {
    char* a = strtok(nullptr, " \t");
    long value = 0;
    if (!a || !parseLongV2(a, &value)) { respondErrorV2("bad_value"); return; }
    setDeltaV2(value);
    if (runV2.target == getPosition()) {
      completeTargetV2();
    }
    respondStartV2(true);
    respondKeyValueLongV2("delta", value);
    respondKeyValueLongV2("target", runV2.target);
    respondKeyValueBoolV2("target_set", runV2.hasTarget);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "run")) {
    runV2.running = true;
    runV2.stopRequested = false;
    if (!runV2.enabled) setEnableV2(true);
    runV2.lastUpdateUs = micros();
    runV2.lastStepperUs = 0;
    runV2.stepAcc = 0.0f;
    respondStartV2(true);
    respondKeyValueBoolV2("running", true);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "stop")) {
    runV2.stopRequested = true;
    runV2.running = true;
    runV2.lastUpdateUs = micros();
    // keep accumulated fractional steps; only reset time base
    runV2.lastStepperUs = 0;
    respondStartV2(true);
    respondKeyValueBoolV2("stopping", true);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "profile")) {
    respondStartV2(true);
    respondKeyValueFloatV2("serial", profiler.serial, 2);
    respondKeyValueFloatV2("serialLastProcessed", profiler.serialLastProcessed, 2);
    respondKeyValueFloatV2("stepper", profiler.stepper, 2);
    respondKeyValueFloatV2("stepperHigh", profiler.stepperHigh, 2);
    respondKeyValueFloatV2("stepperLow", profiler.stepperLow, 2);
    respondKeyValueFloatV2("updateMotionState", profiler.updateMotionState, 2);
    respondEndV2();
    return;
  }

  respondErrorV2("unknown_cmd");
}

void serviceSerialv2() {
  // Non-blocking block read + scan for '\n'. Avoids per-char Stream parsing overhead,
  // but still correctly detects line endings.
  static uint8_t tmp[32];

  while (Serial.available()) {
    const int avail = Serial.available();
    if (avail <= 0) break;

    size_t toRead = (size_t)avail;
    if (toRead > sizeof(tmp)) toRead = sizeof(tmp);

    size_t n = Serial.readBytes(tmp, toRead);
    for (size_t i = 0; i < n; i++) {
      char c = (char)tmp[i];
      if (c == '\r') continue;

      if (c == '\n') {
        lineBufV2[lineLenV2] = 0;
        const uint32_t start = micros();
        handleLineV2(lineBufV2);
        profiler.serialLastProcessed = micros() - start;
        lineLenV2 = 0;
        continue;
      }

      if (lineLenV2 < sizeof(lineBufV2) - 1) {
        lineBufV2[lineLenV2++] = c;
      } else {
        // Overflow without newline: drop and resync.
        lineLenV2 = 0;
      }
    }

  }

  if (Serial.availableForWrite() > 0) {
    char c = 0;
    if (outPopV2(&c)) Serial.write((uint8_t)c);
  }
}

void serviceStepperv2() {
  const uint32_t nowUs = micros();

  // Finish pulse (STEP LOW)
  if (runV2.stepHigh) {
    if ((int32_t)(nowUs - runV2.stepHighUntilUs) >= 0) {
      digitalWrite(STEP_PIN, LOW);
      runV2.stepHigh = false;
    }
    return;
  }

  // Update motion profile (desired/actual speed)
  updateMotionStateV2();

  if (!runV2.enabled || runV2.actualSpeedSps <= 0.0f) {
    // When stopped/disabled, reset phase accumulator and time base.
    runV2.stepAcc = 0.0f;
    runV2.lastStepperUs = nowUs;
    return;
  }

  // Time base for accumulator
  if (runV2.lastStepperUs == 0) {
    runV2.lastStepperUs = nowUs;
    return;
  }

  const uint32_t dtUs = nowUs - runV2.lastStepperUs;
  runV2.lastStepperUs = nowUs;
  if (dtUs == 0) return;

  // Accumulate fractional steps: acc += speed[sps] * dt[s]
  runV2.stepAcc += runV2.actualSpeedSps * ((float)dtUs / 1000000.0f);

  // Clamp accumulator to avoid runaway after long stalls
  if (runV2.stepAcc > 8.0f) runV2.stepAcc = 8.0f;

  // Emit at most ONE step per call because we need a HIGH->LOW pulse across calls
  if (runV2.stepAcc < 1.0f) return;

  const uint32_t tUs = micros();

  digitalWrite(STEP_PIN, HIGH);
  runV2.stepHigh = true;
  runV2.stepHighUntilUs = tUs + runV2.pulseWidthUs;
  runV2.lastStepUs = tUs;

  // Consume one whole step from accumulator
  runV2.stepAcc -= 1.0f;

  // Update position
  stepPosition += runV2.dir ? STEP_NEGATIVE : STEP_POSITIVE;

  // Target completion check (same logic as before)
  if (runV2.hasTarget) {
    const long pos = getPosition();
    if ((runV2.dir && pos <= runV2.target) || (!runV2.dir && pos >= runV2.target)) {
      setPosition(runV2.target);
      completeTargetV2();
    }
  }
}

void loop() {
  long start = micros();
  serviceSerialv2();
  profiler.serial += micros() - start;
  profiler.serial /= 2.;

  start = micros();
  serviceStepperv2();
  profiler.stepper += micros() - start;
  profiler.stepper /= 2.;
}
