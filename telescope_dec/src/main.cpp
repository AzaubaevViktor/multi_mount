/*
  Nano v3 + TMC2209 + TMCStepper
  Commands in Serial Monitor (115200, newline):
    help
    info              - dump registers/status
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
} run;

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

static void serviceStepper() {
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

    if (!run.continuous && run.remaining > 0) {
      run.remaining--;
    }
  }
}

// ---------- Printing helpers ----------
class DumpFormatConstants {
 public:
  static constexpr uint8_t HEX_WIDTH = 8;
  static constexpr uint8_t HEX_PREFIX_LEN = 2;
  static constexpr uint8_t HEX_BUF_LEN = HEX_PREFIX_LEN + HEX_WIDTH + 1;
};

namespace DumpKeys {
static const __FlashStringHelper* const SUBSYS_TMC = F("tmc");
static const __FlashStringHelper* const SUBSYS_IOIN = F("ioin");
static const __FlashStringHelper* const SUBSYS_STATUS = F("status");

static const __FlashStringHelper* const KEY_IFCNT = F("ifcnt");
static const __FlashStringHelper* const KEY_GCONF = F("gconf");
static const __FlashStringHelper* const KEY_GSTAT = F("gstat");
static const __FlashStringHelper* const KEY_IHOLD_IRUN = F("ihold_irun");
static const __FlashStringHelper* const KEY_TPOWERDOWN = F("tpowerdown");
static const __FlashStringHelper* const KEY_TPWMTHRS = F("tpwmthrs");
static const __FlashStringHelper* const KEY_TCOOLTHRS = F("tcoolthrs");
static const __FlashStringHelper* const KEY_SGTHRS = F("sgthrs");
static const __FlashStringHelper* const KEY_CHOPCONF = F("chopconf");
static const __FlashStringHelper* const KEY_PWMCONF = F("pwmconf");
static const __FlashStringHelper* const KEY_VACTUAL = F("vactual");
static const __FlashStringHelper* const KEY_TSTEP = F("tstep");
static const __FlashStringHelper* const KEY_MSCNT = F("mscnt");
static const __FlashStringHelper* const KEY_MSCURACT = F("mscuract");
static const __FlashStringHelper* const KEY_DRV_STATUS = F("drv_status");
static const __FlashStringHelper* const KEY_SG_RESULT = F("sg_result");

static const __FlashStringHelper* const KEY_RAW = F("raw");
static const __FlashStringHelper* const KEY_VERSION = F("version");
static const __FlashStringHelper* const KEY_ENN = F("enn");
static const __FlashStringHelper* const KEY_MS1 = F("ms1");
static const __FlashStringHelper* const KEY_MS2 = F("ms2");
static const __FlashStringHelper* const KEY_DIAG = F("diag");
static const __FlashStringHelper* const KEY_PDN_UART = F("pdn_uart");
static const __FlashStringHelper* const KEY_STEP = F("step");
static const __FlashStringHelper* const KEY_DIR = F("dir");
static const __FlashStringHelper* const KEY_SPREAD_EN = F("spread_en");

static const __FlashStringHelper* const KEY_OT = F("ot");
static const __FlashStringHelper* const KEY_OTPW = F("otpw");
static const __FlashStringHelper* const KEY_S2GA = F("s2ga");
static const __FlashStringHelper* const KEY_S2GB = F("s2gb");
static const __FlashStringHelper* const KEY_OLA = F("ola");
static const __FlashStringHelper* const KEY_OLB = F("olb");
static const __FlashStringHelper* const KEY_T120 = F("t120");
static const __FlashStringHelper* const KEY_T143 = F("t143");
static const __FlashStringHelper* const KEY_T150 = F("t150");
static const __FlashStringHelper* const KEY_T157 = F("t157");
static const __FlashStringHelper* const KEY_STST = F("stst");
static const __FlashStringHelper* const KEY_STEALTH = F("stealth");
static const __FlashStringHelper* const KEY_CS_ACTUAL = F("cs_actual");
}  // namespace DumpKeys

static void printKeyPrefix(const __FlashStringHelper* subsystem, const __FlashStringHelper* key) {
  Serial.print(subsystem);
  Serial.print('.');
  Serial.print(key);
  Serial.print(F(" = "));
}

static void printKeyValueU32(const __FlashStringHelper* subsystem, const __FlashStringHelper* key, uint32_t v) {
  printKeyPrefix(subsystem, key);
  Serial.println(v);
}

static void printKeyValueBool(const __FlashStringHelper* subsystem, const __FlashStringHelper* key, bool v) {
  printKeyPrefix(subsystem, key);
  Serial.println(v ? 1 : 0);
}

static void printKeyHex32(const __FlashStringHelper* subsystem, const __FlashStringHelper* key, uint32_t v) {
  char buf[DumpFormatConstants::HEX_BUF_LEN];
  snprintf(buf, sizeof(buf), "0x%0*lX", DumpFormatConstants::HEX_WIDTH, (unsigned long)v);
  printKeyPrefix(subsystem, key);
  Serial.println(buf);
}

static void dumpInfo() {
  // ---------- Связь / идентификация ----------
  // IFCNT: счётчик успешных UART-пакетов, которые принял драйвер.
  // Растёт при каждом корректном обмене по UART. Если не растёт — проблемы с линией UART/адресом/скоростью.
  printKeyValueU32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_IFCNT, driver.IFCNT());

  // IOIN: входной регистр состояния ног/страпов и версионных битов.
  // Полезен, чтобы понять: что реально видит драйвер на своих входах (STEP/DIR/EN), включён ли PDN_UART и т.п.
  printKeyHex32(DumpKeys::SUBSYS_IOIN, DumpKeys::KEY_RAW, driver.IOIN());

  // IOIN.version: версия/идентификатор кристалла (по документации Trinamic).
  printKeyHex32(DumpKeys::SUBSYS_IOIN, DumpKeys::KEY_VERSION, driver.version());

  // Ниже — отдельные биты из IOIN, прочитанные хелперами библиотеки:
  // enn      : состояние входа ENN (enable, активный LOW на большинстве модулей)
  // ms1/ms2  : состояния страпов MS1/MS2 (на TMC2209 часто задают адрес UART/режим)
  // diag     : состояние вывода DIAG (обычно используется для StallGuard/ошибок)
  // pdn_uart : состояние PDN_UART (пин, совмещающий power-down и UART)
  // step/dir : логические уровни на входах STEP и DIR, которые видит драйвер
  // spread_en: состояние входа SPREAD (выбор spreadCycle/stealthChop, зависит от конфигурации)
  printKeyValueBool(DumpKeys::SUBSYS_IOIN, DumpKeys::KEY_ENN, driver.enn());
  printKeyValueBool(DumpKeys::SUBSYS_IOIN, DumpKeys::KEY_MS1, driver.ms1());
  printKeyValueBool(DumpKeys::SUBSYS_IOIN, DumpKeys::KEY_MS2, driver.ms2());
  printKeyValueBool(DumpKeys::SUBSYS_IOIN, DumpKeys::KEY_DIAG, driver.diag());
  printKeyValueBool(DumpKeys::SUBSYS_IOIN, DumpKeys::KEY_PDN_UART, driver.pdn_uart());
  printKeyValueBool(DumpKeys::SUBSYS_IOIN, DumpKeys::KEY_STEP, driver.step());
  printKeyValueBool(DumpKeys::SUBSYS_IOIN, DumpKeys::KEY_DIR, driver.dir());
  printKeyValueBool(DumpKeys::SUBSYS_IOIN, DumpKeys::KEY_SPREAD_EN, driver.spread_en());

  // ---------- Ключевые конфиги ----------
  // GCONF: общий конфиг драйвера (режимы UART, инверсии, выбор stealth/spread и пр.).
  printKeyHex32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_GCONF, driver.GCONF());

  // GSTAT: латч-статусы/события (например reset, driver error). Обычно чистится записью 1 в соответствующие биты.
  printKeyHex32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_GSTAT, driver.GSTAT());

  // IHOLD_IRUN: ток удержания (IHOLD), ток движения (IRUN) и время плавного перехода (IHOLDDELAY).
  // Влияет на момент/нагрев/шум.
  printKeyHex32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_IHOLD_IRUN, driver.IHOLD_IRUN());

  // TPOWERDOWN: задержка (в тактах) до перехода на IHOLD после остановки (энергосбережение/нагрев).
  printKeyValueU32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_TPOWERDOWN, driver.TPOWERDOWN());

  // TPWMTHRS: порог скорости, ниже которого активен stealthChop (PWM) (если он включён).
  // Выше порога обычно переключаются на spreadCycle (в зависимости от настроек).
  printKeyHex32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_TPWMTHRS, driver.TPWMTHRS());

  // TCOOLTHRS: порог для функций coolStep/StallGuard (условно: с какой скорости начинаем оценивать нагрузку).
  printKeyHex32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_TCOOLTHRS, driver.TCOOLTHRS());

  // SGTHRS: чувствительность StallGuard (0..255). Чем больше — тем чувствительнее/раньше срабатывает (в общем случае).
  // Точное поведение зависит от механики/тока/скорости.
  printKeyValueU32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_SGTHRS, driver.SGTHRS());

  // CHOPCONF: конфиг чопера (toff, blank time, hysteresis, microstep interp и пр.).
  // Критично для стабильности, шума и качества шага.
  printKeyHex32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_CHOPCONF, driver.CHOPCONF());

  // PWMCONF: параметры PWM для stealthChop (амплитуда, градиент, авто-режимы и т.п.).
  printKeyHex32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_PWMCONF, driver.PWMCONF());

  // ---------- Текущее движение / статус ----------
  // VACTUAL: текущая скорость в "внутренних единицах" драйвера (актуально при управлении через внутренний motion controller).
  // В нашем проекте шаги генерируются внешним STEP, поэтому обычно будет 0.
  printKeyHex32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_VACTUAL, driver.VACTUAL());

  // TSTEP: измеренный период между микрошагами (приблизительная оценка скорости). Может быть 0/макс при стоянии.
  printKeyValueU32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_TSTEP, driver.TSTEP());

  // MSCNT: счётчик позиции внутри микрошагового синуса (0..1023 для 256 микрошагов).
  // Полезно для диагностики, что микрошаги реально бегут.
  printKeyValueU32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_MSCNT, driver.MSCNT());

  // MSCURACT: фактические токи фаз (A/B) в текущем микрошаге (в кодировке драйвера).
  printKeyHex32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_MSCURACT, driver.MSCURACT());

  // DRV_STATUS: большой регистр статуса (ошибки, перегрев, stallguard, режимы, токи и пр.).
  // По нему библиотека и выдаёт флаги ниже.
  printKeyHex32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_DRV_STATUS, driver.DRV_STATUS());

  // SG_RESULT: значение StallGuard (оценка нагрузки/скольжения). Обычно выше = легче крутится.
  // Имеет смысл только в диапазоне скоростей, где StallGuard активен.
  printKeyValueU32(DumpKeys::SUBSYS_TMC, DumpKeys::KEY_SG_RESULT, driver.SG_RESULT());

  // ---------- Декодированные флаги безопасности/диагностики (из DRV_STATUS) ----------
  // ot      : overtemperature shutdown — перегрев, драйвер отключил выход.
  // otpw    : overtemperature prewarning — приближение к перегреву.
  // s2ga/b  : short to ground A/B — КЗ фазы A или B на землю.
  // ola/olb : open load A/B — обрыв фазы A или B (или ток слишком мал для детекта).
  // t120..t157: пороги температуры (примерная ступенчатая индикация нагрева кристалла).
  // stst    : standstill — драйвер считает, что стоит (для некоторых режимов/регистров).
  // stealth : фактический режим stealthChop активен сейчас.
  // cs_actual: фактическое значение тока (current scale) в данный момент.
  printKeyValueBool(DumpKeys::SUBSYS_STATUS, DumpKeys::KEY_OT, driver.ot());
  printKeyValueBool(DumpKeys::SUBSYS_STATUS, DumpKeys::KEY_OTPW, driver.otpw());
  printKeyValueBool(DumpKeys::SUBSYS_STATUS, DumpKeys::KEY_S2GA, driver.s2ga());
  printKeyValueBool(DumpKeys::SUBSYS_STATUS, DumpKeys::KEY_S2GB, driver.s2gb());
  printKeyValueBool(DumpKeys::SUBSYS_STATUS, DumpKeys::KEY_OLA, driver.ola());
  printKeyValueBool(DumpKeys::SUBSYS_STATUS, DumpKeys::KEY_OLB, driver.olb());
  printKeyValueBool(DumpKeys::SUBSYS_STATUS, DumpKeys::KEY_T120, driver.t120());
  printKeyValueBool(DumpKeys::SUBSYS_STATUS, DumpKeys::KEY_T143, driver.t143());
  printKeyValueBool(DumpKeys::SUBSYS_STATUS, DumpKeys::KEY_T150, driver.t150());
  printKeyValueBool(DumpKeys::SUBSYS_STATUS, DumpKeys::KEY_T157, driver.t157());
  printKeyValueBool(DumpKeys::SUBSYS_STATUS, DumpKeys::KEY_STST, driver.stst());
  printKeyValueBool(DumpKeys::SUBSYS_STATUS, DumpKeys::KEY_STEALTH, driver.stealth());
  printKeyValueU32(DumpKeys::SUBSYS_STATUS, DumpKeys::KEY_CS_ACTUAL, driver.cs_actual());
}

static void printHelp() {
  Serial.println(F(
    "help | info | enable 0|1 | dir 0|1 | run <sps> | move <steps> <sps> | stop\n"
    "current <mA> | microsteps <n> | stealth 0|1 | sgthrs <0..255>\n"
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
