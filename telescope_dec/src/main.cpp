/*
  Nano v3 + TMC2209 + TMCStepper
  Commands in Serial Monitor (115200, newline):
    help
    info              - dump registers/status
    fastinfo          - compact status (fast)
    enable 0|1
    dir 0|1
    run <sps>         - continuous, steps per second; negative => reverse
    move <steps> <sps>- relative move; steps can be negative; sps optional (default last)
    stop
    current <mA>      - RMS current (library calc depends on R_SENSE)
    microsteps <n>    - 1/2/4/8/16/32/64/128/256
    stealth 0|1       - 1 = stealthChop, 0 = spreadCycle (en_spreadCycle = !stealth)
    sgthrs <0..255>

  Wiring (typical 1-wire UART):
    
    GND common, VM+motor power as usual.

  STEP/DIR/EN to your driver module pins.
*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include <stdio.h>

// ---------- Pins ----------
static const uint8_t STEP_PIN = 2;
static const uint8_t DIR_PIN  = 3;
static const uint8_t EN_PIN   = 4;   // Enable pin to driver
static const bool    EN_ACTIVE_LOW = true;

static const uint8_t TMC_RX_PIN = 8;
static const uint8_t TMC_TX_PIN = 9;

// ---------- TMC config ----------
static const uint32_t TMC_BAUD = 115200;
// Most SilentStepStick-like modules use 0.11 ohm; check your board to be correct.
static const float R_SENSE = 0.11f;
// Address depends on MS1/MS2 (CFG pins) strapping; often 0b00 if both low.
static const uint8_t DRIVER_ADDRESS = 0b00;

SoftwareSerial TMCSerial(TMC_RX_PIN, TMC_TX_PIN);
TMC2209Stepper driver(&TMCSerial, R_SENSE, DRIVER_ADDRESS);

// ---------- Simple step generator (non-blocking pulses) ----------
struct Runner {
  bool enabled = false;
  bool continuous = false;
  bool dir = false;
  long remaining = 0;              // steps left (for move)
  uint32_t stepIntervalUs = 2000;  // default 500 sps
  uint32_t pulseWidthUs   = 3;
  uint32_t nextStepUs     = 0;
  bool stepHigh = false;
  uint32_t stepHighUntilUs = 0;
  int lastSps = 500;
  uint32_t lastStepUs = 0;
} run;

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

static const char* const COMMAND_POS = "pos";
static const char* const COMMAND_SET_POS = "setpos";
static const char* const COMMAND_FAST_INFO = "fastinfo";

static inline void setEnable(bool on) {
  run.enabled = on;
  digitalWrite(EN_PIN, (EN_ACTIVE_LOW ? !on : on) ? HIGH : LOW);
}

static inline void setDir(bool dir) {
  run.dir = dir;
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);
}

static inline void setSpeedSps(int sps_abs) {
  if (sps_abs < 1) sps_abs = 1;
  if (sps_abs > 40000) sps_abs = 40000;                 // sanity cap
  run.lastSps = sps_abs;
  run.stepIntervalUs = (uint32_t)(1000000UL / (uint32_t)sps_abs);
  if (run.stepIntervalUs < (run.pulseWidthUs + 4)) run.stepIntervalUs = run.pulseWidthUs + 4;
}

static const char* const MODE_IDLE = "idle";
static const char* const MODE_MOVE = "move";
static const char* const MODE_RUN = "run";

static inline const char* getRunMode() {
  if (run.continuous) return MODE_RUN;
  if (run.remaining > 0) return MODE_MOVE;
  return MODE_IDLE;
}

static void serviceStepper() {
  // TODO: prompt change digital write to faster direct pin set
  const uint32_t now = micros();

  if (run.stepHigh) {
    if ((int32_t)(now - run.stepHighUntilUs) >= 0) {
      digitalWrite(STEP_PIN, LOW);
      run.stepHigh = false;
      if (!run.continuous && run.remaining == 0) {
        // finished move
      }
    }
    return;
  }

  const bool shouldStep = run.enabled && (run.continuous || run.remaining > 0);
  if (!shouldStep) return;

  if ((int32_t)(now - run.nextStepUs) >= 0) {
    digitalWrite(STEP_PIN, HIGH);
    run.stepHigh = true;
    run.stepHighUntilUs = now + run.pulseWidthUs;
    run.nextStepUs = now + run.stepIntervalUs;

    run.lastStepUs = now;
    stepPosition += run.dir ? STEP_NEGATIVE : STEP_POSITIVE;

    if (!run.continuous && run.remaining > 0) {
      run.remaining--;
    }
  }
}

// ---------- Printing helpers ----------
static const uint8_t HEX_WIDTH = 8;
static const uint8_t HEX_PREFIX_LEN = 2;
static const uint8_t HEX_BUF_LEN = HEX_PREFIX_LEN + HEX_WIDTH + 1;

static const char* const DUMP_SUBSYS_TMC = "tmc";
static const char* const DUMP_SUBSYS_IOIN = "ioin";
static const char* const DUMP_SUBSYS_STATUS = "status";
static const char* const DUMP_SUBSYS_FAST = "fast";

static const char* const DUMP_KEY_IFCNT = "ifcnt";
static const char* const DUMP_KEY_IFCNT_DELTA = "ifcnt_delta";
static const char* const DUMP_KEY_GCONF = "gconf";
static const char* const DUMP_KEY_GSTAT = "gstat";
static const char* const DUMP_KEY_IHOLD_IRUN = "ihold_irun";
static const char* const DUMP_KEY_TPOWERDOWN = "tpowerdown";
static const char* const DUMP_KEY_TPWMTHRS = "tpwmthrs";
static const char* const DUMP_KEY_TCOOLTHRS = "tcoolthrs";
static const char* const DUMP_KEY_SGTHRS = "sgthrs";
static const char* const DUMP_KEY_CHOPCONF = "chopconf";
static const char* const DUMP_KEY_PWMCONF = "pwmconf";
static const char* const DUMP_KEY_VACTUAL = "vactual";
static const char* const DUMP_KEY_TSTEP = "tstep";
static const char* const DUMP_KEY_MSCNT = "mscnt";
static const char* const DUMP_KEY_MSCURACT = "mscuract";
static const char* const DUMP_KEY_DRV_STATUS = "drv_status";
static const char* const DUMP_KEY_SG_RESULT = "sg_result";
static const char* const DUMP_KEY_ENABLED = "enabled";
static const char* const DUMP_KEY_MODE = "mode";
static const char* const DUMP_KEY_REMAINING_STEPS = "remaining_steps";
static const char* const DUMP_KEY_CMD_SPS = "cmd_sps";
static const char* const DUMP_KEY_LAST_STEP_AGE_US = "last_step_age_us";

static const char* const DUMP_KEY_RAW = "raw";
static const char* const DUMP_KEY_VERSION = "version";
static const char* const DUMP_KEY_ENN = "enn";
static const char* const DUMP_KEY_MS1 = "ms1";
static const char* const DUMP_KEY_MS2 = "ms2";
static const char* const DUMP_KEY_DIAG = "diag";
static const char* const DUMP_KEY_PDN_UART = "pdn_uart";
static const char* const DUMP_KEY_STEP = "step";
static const char* const DUMP_KEY_DIR = "dir";
static const char* const DUMP_KEY_SPREAD_EN = "spread_en";

static const char* const DUMP_KEY_OT = "ot";
static const char* const DUMP_KEY_OTPW = "otpw";
static const char* const DUMP_KEY_S2GA = "s2ga";
static const char* const DUMP_KEY_S2GB = "s2gb";
static const char* const DUMP_KEY_OLA = "ola";
static const char* const DUMP_KEY_OLB = "olb";
static const char* const DUMP_KEY_T120 = "t120";
static const char* const DUMP_KEY_T143 = "t143";
static const char* const DUMP_KEY_T150 = "t150";
static const char* const DUMP_KEY_T157 = "t157";
static const char* const DUMP_KEY_STST = "stst";
static const char* const DUMP_KEY_STEALTH = "stealth";
static const char* const DUMP_KEY_CS_ACTUAL = "cs_actual";
static const char* const DUMP_KEY_STALL_GUARD = "stall_guard";

static const uint32_t IFCNT_MAX = 0xFF;

static void printKeyPrefix(const char* subsystem, const char* key) {
  Serial.print(subsystem);
  Serial.print('.');
  Serial.print(key);
  Serial.print(F(" = "));
}

static void printKeyValueU32(const char* subsystem, const char* key, uint32_t v) {
  printKeyPrefix(subsystem, key);
  Serial.println(v);
}

static void printKeyValueBool(const char* subsystem, const char* key, bool v) {
  printKeyPrefix(subsystem, key);
  Serial.println(v ? 1 : 0);
}

static void printKeyHex32(const char* subsystem, const char* key, uint32_t v) {
  char buf[HEX_BUF_LEN];
  snprintf(buf, sizeof(buf), "0x%0*lX", HEX_WIDTH, (unsigned long)v);
  printKeyPrefix(subsystem, key);
  Serial.println(buf);
}

static void dumpInfo() {
  // ---------- Связь / идентификация ----------
  // IFCNT: счётчик успешных UART-пакетов, которые принял драйвер.
  // Растёт при каждом корректном обмене по UART. Если не растёт — проблемы с линией UART/адресом/скоростью.
  printKeyValueU32(DUMP_SUBSYS_TMC, DUMP_KEY_IFCNT, driver.IFCNT());

  // IOIN: входной регистр состояния ног/страпов и версионных битов.
  // Полезен, чтобы понять: что реально видит драйвер на своих входах (STEP/DIR/EN), включён ли PDN_UART и т.п.
  printKeyHex32(DUMP_SUBSYS_IOIN, DUMP_KEY_RAW, driver.IOIN());

  // IOIN.version: версия/идентификатор кристалла (по документации Trinamic).
  printKeyHex32(DUMP_SUBSYS_IOIN, DUMP_KEY_VERSION, driver.version());

  // Ниже — отдельные биты из IOIN, прочитанные хелперами библиотеки:
  // enn      : состояние входа ENN (enable, активный LOW на большинстве модулей)
  // ms1/ms2  : состояния страпов MS1/MS2 (на TMC2209 часто задают адрес UART/режим)
  // diag     : состояние вывода DIAG (обычно используется для StallGuard/ошибок)
  // pdn_uart : состояние PDN_UART (пин, совмещающий power-down и UART)
  // step/dir : логические уровни на входах STEP и DIR, которые видит драйвер
  // spread_en: состояние входа SPREAD (выбор spreadCycle/stealthChop, зависит от конфигурации)
  printKeyValueBool(DUMP_SUBSYS_IOIN, DUMP_KEY_ENN, driver.enn());
  printKeyValueBool(DUMP_SUBSYS_IOIN, DUMP_KEY_MS1, driver.ms1());
  printKeyValueBool(DUMP_SUBSYS_IOIN, DUMP_KEY_MS2, driver.ms2());
  printKeyValueBool(DUMP_SUBSYS_IOIN, DUMP_KEY_DIAG, driver.diag());
  printKeyValueBool(DUMP_SUBSYS_IOIN, DUMP_KEY_PDN_UART, driver.pdn_uart());
  printKeyValueBool(DUMP_SUBSYS_IOIN, DUMP_KEY_STEP, driver.step());
  printKeyValueBool(DUMP_SUBSYS_IOIN, DUMP_KEY_DIR, driver.dir());
  printKeyValueBool(DUMP_SUBSYS_IOIN, DUMP_KEY_SPREAD_EN, driver.spread_en());

  // ---------- Ключевые конфиги ----------
  // GCONF: общий конфиг драйвера (режимы UART, инверсии, выбор stealth/spread и пр.).
  printKeyHex32(DUMP_SUBSYS_TMC, DUMP_KEY_GCONF, driver.GCONF());

  // GSTAT: латч-статусы/события (например reset, driver error). Обычно чистится записью 1 в соответствующие биты.
  printKeyHex32(DUMP_SUBSYS_TMC, DUMP_KEY_GSTAT, driver.GSTAT());

  // IHOLD_IRUN: ток удержания (IHOLD), ток движения (IRUN) и время плавного перехода (IHOLDDELAY).
  // Влияет на момент/нагрев/шум.
  printKeyHex32(DUMP_SUBSYS_TMC, DUMP_KEY_IHOLD_IRUN, driver.IHOLD_IRUN());

  // TPOWERDOWN: задержка (в тактах) до перехода на IHOLD после остановки (энергосбережение/нагрев).
  printKeyValueU32(DUMP_SUBSYS_TMC, DUMP_KEY_TPOWERDOWN, driver.TPOWERDOWN());

  // TPWMTHRS: порог скорости, ниже которого активен stealthChop (PWM) (если он включён).
  // Выше порога обычно переключаются на spreadCycle (в зависимости от настроек).
  printKeyHex32(DUMP_SUBSYS_TMC, DUMP_KEY_TPWMTHRS, driver.TPWMTHRS());

  // TCOOLTHRS: порог для функций coolStep/StallGuard (условно: с какой скорости начинаем оценивать нагрузку).
  printKeyHex32(DUMP_SUBSYS_TMC, DUMP_KEY_TCOOLTHRS, driver.TCOOLTHRS());

  // SGTHRS: чувствительность StallGuard (0..255). Чем больше — тем чувствительнее/раньше срабатывает (в общем случае).
  // Точное поведение зависит от механики/тока/скорости.
  const uint32_t sg_thrs = driver.SGTHRS();
  printKeyValueU32(DUMP_SUBSYS_TMC, DUMP_KEY_SGTHRS, sg_thrs);

  // CHOPCONF: конфиг чопера (toff, blank time, hysteresis, microstep interp и пр.).
  // Критично для стабильности, шума и качества шага.
  printKeyHex32(DUMP_SUBSYS_TMC, DUMP_KEY_CHOPCONF, driver.CHOPCONF());

  // PWMCONF: параметры PWM для stealthChop (амплитуда, градиент, авто-режимы и т.п.).
  printKeyHex32(DUMP_SUBSYS_TMC, DUMP_KEY_PWMCONF, driver.PWMCONF());

  // ---------- Текущее движение / статус ----------
  // VACTUAL: текущая скорость в "внутренних единицах" драйвера (актуально при управлении через внутренний motion controller).
  // В нашем проекте шаги генерируются внешним STEP, поэтому обычно будет 0.
  printKeyHex32(DUMP_SUBSYS_TMC, DUMP_KEY_VACTUAL, driver.VACTUAL());

  // TSTEP: измеренный период между микрошагами (приблизительная оценка скорости). Может быть 0/макс при стоянии.
  printKeyValueU32(DUMP_SUBSYS_TMC, DUMP_KEY_TSTEP, driver.TSTEP());

  // MSCNT: счётчик позиции внутри микрошагового синуса (0..1023 для 256 микрошагов).
  // Полезно для диагностики, что микрошаги реально бегут.
  printKeyValueU32(DUMP_SUBSYS_TMC, DUMP_KEY_MSCNT, driver.MSCNT());

  // MSCURACT: фактические токи фаз (A/B) в текущем микрошаге (в кодировке драйвера).
  printKeyHex32(DUMP_SUBSYS_TMC, DUMP_KEY_MSCURACT, driver.MSCURACT());

  // DRV_STATUS: большой регистр статуса (ошибки, перегрев, stallguard, режимы, токи и пр.).
  // По нему библиотека и выдаёт флаги ниже.
  printKeyHex32(DUMP_SUBSYS_TMC, DUMP_KEY_DRV_STATUS, driver.DRV_STATUS());

  // SG_RESULT: значение StallGuard (оценка нагрузки/скольжения). Обычно выше = легче крутится.
  // Имеет смысл только в диапазоне скоростей, где StallGuard активен.
  const uint32_t sg_result = driver.SG_RESULT();
  printKeyValueU32(DUMP_SUBSYS_TMC, DUMP_KEY_SG_RESULT, sg_result);

  // ---------- Декодированные флаги безопасности/диагностики (из DRV_STATUS) ----------
  // ot      : overtemperature shutdown — перегрев, драйвер отключил выход.
  // otpw    : overtemperature prewarning — приближение к перегреву.
  // s2ga/b  : short to ground A/B — КЗ фазы A или B на землю.
  // ola/olb : open load A/B — обрыв фазы A или B (или ток слишком мал для детекта).
  // t120..t157: пороги температуры (примерная ступенчатая индикация нагрева кристалла).
  // stst    : standstill — драйвер считает, что стоит (для некоторых режимов/регистров).
  // stealth : фактический режим stealthChop активен сейчас.
  // cs_actual: фактическое значение тока (current scale) в данный момент.
  printKeyValueBool(DUMP_SUBSYS_STATUS, DUMP_KEY_OT, driver.ot());
  printKeyValueBool(DUMP_SUBSYS_STATUS, DUMP_KEY_OTPW, driver.otpw());
  printKeyValueBool(DUMP_SUBSYS_STATUS, DUMP_KEY_S2GA, driver.s2ga());
  printKeyValueBool(DUMP_SUBSYS_STATUS, DUMP_KEY_S2GB, driver.s2gb());
  printKeyValueBool(DUMP_SUBSYS_STATUS, DUMP_KEY_OLA, driver.ola());
  printKeyValueBool(DUMP_SUBSYS_STATUS, DUMP_KEY_OLB, driver.olb());
  printKeyValueBool(DUMP_SUBSYS_STATUS, DUMP_KEY_T120, driver.t120());
  printKeyValueBool(DUMP_SUBSYS_STATUS, DUMP_KEY_T143, driver.t143());
  printKeyValueBool(DUMP_SUBSYS_STATUS, DUMP_KEY_T150, driver.t150());
  printKeyValueBool(DUMP_SUBSYS_STATUS, DUMP_KEY_T157, driver.t157());
  printKeyValueBool(DUMP_SUBSYS_STATUS, DUMP_KEY_STST, driver.stst());
  printKeyValueBool(DUMP_SUBSYS_STATUS, DUMP_KEY_STEALTH, driver.stealth());
  printKeyValueBool(DUMP_SUBSYS_STATUS, DUMP_KEY_STALL_GUARD, driver.stallguard());
  printKeyValueU32(DUMP_SUBSYS_STATUS, DUMP_KEY_CS_ACTUAL, driver.cs_actual());
}

static void fastDumpInfo() {
  const uint32_t ifcnt = driver.IFCNT();
  static bool has_prev_ifcnt = false;
  static uint32_t prev_ifcnt = 0;

  printKeyValueU32(DUMP_SUBSYS_FAST, DUMP_KEY_IFCNT, ifcnt);
  if (has_prev_ifcnt) {
    uint32_t delta = 0;
    if (ifcnt >= prev_ifcnt) {
      delta = ifcnt - prev_ifcnt;
    } else {
      delta = (IFCNT_MAX + 1u - prev_ifcnt) + ifcnt;
    }
    printKeyValueU32(DUMP_SUBSYS_FAST, DUMP_KEY_IFCNT_DELTA, delta);
  }
  prev_ifcnt = ifcnt;
  has_prev_ifcnt = true;

  const bool enn_state = driver.enn();
  const bool enabled = EN_ACTIVE_LOW ? !enn_state : enn_state;
  printKeyValueBool(DUMP_SUBSYS_FAST, DUMP_KEY_ENABLED, enabled);

  const char* const mode = getRunMode();
  printKeyPrefix(DUMP_SUBSYS_FAST, DUMP_KEY_MODE);
  Serial.println(mode);

  printKeyValueBool(DUMP_SUBSYS_FAST, DUMP_KEY_STST, driver.stst());
  printKeyValueU32(DUMP_SUBSYS_FAST, DUMP_KEY_TSTEP, driver.TSTEP());

  if (!strcmp(mode, MODE_MOVE)) {
    printKeyValueU32(DUMP_SUBSYS_FAST, DUMP_KEY_REMAINING_STEPS, (uint32_t)run.remaining);
  }

  printKeyValueU32(DUMP_SUBSYS_FAST, DUMP_KEY_CMD_SPS, (uint32_t)run.lastSps);

  const uint32_t now = micros();
  const uint32_t last_step_age_us = now - run.lastStepUs;
  printKeyValueU32(DUMP_SUBSYS_FAST, DUMP_KEY_LAST_STEP_AGE_US, last_step_age_us);

  printKeyValueBool(DUMP_SUBSYS_FAST, DUMP_KEY_OTPW, driver.otpw());
  printKeyValueBool(DUMP_SUBSYS_FAST, DUMP_KEY_OT, driver.ot());
  printKeyValueBool(DUMP_SUBSYS_FAST, DUMP_KEY_S2GA, driver.s2ga());
  printKeyValueBool(DUMP_SUBSYS_FAST, DUMP_KEY_S2GB, driver.s2gb());
  printKeyValueBool(DUMP_SUBSYS_FAST, DUMP_KEY_OLA, driver.ola());
  printKeyValueBool(DUMP_SUBSYS_FAST, DUMP_KEY_OLB, driver.olb());

  printKeyValueU32(DUMP_SUBSYS_FAST, DUMP_KEY_SG_RESULT, driver.SG_RESULT());
  printKeyValueU32(DUMP_SUBSYS_FAST, DUMP_KEY_CS_ACTUAL, driver.cs_actual());
}

static void printHelp() {
  Serial.println(F(
    "help | info | fastinfo | enable 0|1 | dir 0|1 | run <sps> | move <steps> <sps> | stop\n"
    "current <mA> | microsteps <n> | stealth 0|1 | sgthrs <0..255>\n"
    "pos | setpos <value>\n"
  ));
}

// ---------- Simple line parser ----------
static char lineBuf[96];
static uint8_t lineLen = 0;

static void handleLine(char *s) {
  while (*s == ' ' || *s == '\t') s++;
  if (!*s) return;

  char *cmd = strtok(s, " \t");
  if (!cmd) return;

  if (!strcmp(cmd, "help")) { printHelp(); return; }
  if (!strcmp(cmd, "info") || !strcmp(cmd, "dump")) { dumpInfo(); return; }
  if (!strcmp(cmd, COMMAND_FAST_INFO)) { fastDumpInfo(); return; }
  if (!strcmp(cmd, COMMAND_POS)) {
    Serial.print(F("pos="));
    Serial.println(getPosition());
    return;
  }
  if (!strcmp(cmd, COMMAND_SET_POS)) {
    char *a = strtok(nullptr, " \t");
    if (!a) { Serial.println(F("setpos needs <value>")); return; }
    setPosition(atol(a));
    Serial.print(F("pos="));
    Serial.println(getPosition());
    return;
  }

  if (!strcmp(cmd, "enable")) {
    char *a = strtok(nullptr, " \t");
    setEnable(a && atoi(a) != 0);
    Serial.print(F("enable=")); Serial.println(run.enabled);
    return;
  }

  if (!strcmp(cmd, "dir")) {
    char *a = strtok(nullptr, " \t");
    setDir(a && atoi(a) != 0);
    Serial.print(F("dir=")); Serial.println(run.dir);
    return;
  }

  if (!strcmp(cmd, "stop")) {
    run.continuous = false;
    run.remaining = 0;
    Serial.println(F("stopped"));
    return;
  }

  if (!strcmp(cmd, "run")) {
    char *a = strtok(nullptr, " \t");
    int sps = a ? atoi(a) : run.lastSps;
    bool dir = (sps < 0);
    if (sps < 0) sps = -sps;
    setDir(dir);
    setSpeedSps(sps);
    run.continuous = true;
    run.remaining = 0;
    setEnable(true);
    Serial.print(F("run sps=")); Serial.print(sps);
    Serial.print(F(" dir=")); Serial.println(dir);
    return;
  }

  if (!strcmp(cmd, "move")) {
    char *a = strtok(nullptr, " \t");
    char *b = strtok(nullptr, " \t");
    if (!a) { Serial.println(F("move needs <steps>")); return; }
    long steps = atol(a);
    int sps = b ? atoi(b) : run.lastSps;
    bool dir = (steps < 0);
    if (steps < 0) steps = -steps;
    if (sps < 0) sps = -sps;

    setDir(dir);
    setSpeedSps(sps);
    run.continuous = false;
    run.remaining = steps;
    setEnable(true);

    Serial.print(F("move steps=")); Serial.print(steps);
    Serial.print(F(" sps=")); Serial.print(sps);
    Serial.print(F(" dir=")); Serial.println(dir);
    return;
  }

  if (!strcmp(cmd, "current")) {
    char *a = strtok(nullptr, " \t");
    if (!a) { Serial.println(F("current needs <mA>")); return; }
    uint16_t mA = (uint16_t)constrain(atoi(a), 50, 2000);
    driver.rms_current(mA);
    Serial.print(F("rms_current(mA)=")); Serial.println(mA);
    return;
  }

  if (!strcmp(cmd, "microsteps")) {
    char *a = strtok(nullptr, " \t");
    if (!a) { Serial.println(F("microsteps needs <n>")); return; }
    uint16_t ms = (uint16_t)atoi(a);
    driver.microsteps(ms);
    Serial.print(F("microsteps=")); Serial.println(ms);
    return;
  }

  if (!strcmp(cmd, "stealth")) {
    char *a = strtok(nullptr, " \t");
    if (!a) { Serial.println(F("stealth needs 0|1")); return; }
    bool stealth = atoi(a) != 0;
    driver.en_spreadCycle(!stealth);
    driver.pwm_autoscale(true);
    Serial.print(F("stealth=")); Serial.println(stealth);
    return;
  }

  if (!strcmp(cmd, "sgthrs")) {
    char *a = strtok(nullptr, " \t");
    if (!a) { Serial.println(F("sgthrs needs 0..255")); return; }
    uint8_t v = (uint8_t)constrain(atoi(a), 0, 255);
    driver.SGTHRS(v);
    Serial.print(F("SGTHRS=")); Serial.println(v);
    return;
  }

  Serial.print(F("unknown: ")); Serial.println(cmd);
}

static void serviceSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = 0;
      handleLine(lineBuf);
      lineLen = 0;
      continue;
    }
    if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  setDir(false);
  setEnable(false);
  run.lastStepUs = micros();

  TMCSerial.begin(TMC_BAUD);

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

  Serial.println(F("Ready. Type 'help'."));
  dumpInfo();
}

void loop() {
  serviceSerial();
  serviceStepper();

  // Optional: notify when a move finished (edge)
  static long lastRemaining = -1;
  if (!run.continuous && lastRemaining != 0 && run.remaining == 0) {
    Serial.println(F("move done"));
  }
  lastRemaining = run.remaining;
}
