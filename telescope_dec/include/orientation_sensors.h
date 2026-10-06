#pragma once
#include "frame_v3.h"

// Only raw vectors live here. Calibration and sky coordinates belong to Python.
// Both modules must be rigidly mounted with matching chip X/Y/Z directions.
template<class Bus> struct OrientationSensors {
  Bus bus;
  RawSensorV3 raw = {};
  uint32_t next = 0, gravityAt = 0, magneticAt = 0;
  uint8_t mpu = 0, qmc = 0, mpuAddress = 0x68, device = 0;
  bool pending = false, gravityValid = false, magneticValid = false;

  RawSensorV3 snapshot(uint32_t now) const {
    RawSensorV3 result = raw;
    result.flags = (mpu || qmc) ? 1 : 0;
    if (gravityValid) {
      result.flags |= 2;
      uint32_t age = now - gravityAt;
      const uint32_t magAge = now - magneticAt;
      if (magneticValid && magAge <= 200) {
        result.flags |= 4;
        if (magAge > age) age = magAge;
      }
      result.ageMs = age > 65535 ? 65535 : uint16_t(age);
    }
    return result;
  }

  void tick(uint32_t now) {
    bus.tick(now);
    if (bus.phase) return;

    if (pending) {
      pending = false;
      uint8_t& stage = device ? qmc : mpu;
      if (!bus.ok) {
        stage = 0;
        if (device) magneticValid = false;
        else { gravityValid = false; mpuAddress ^= 1; }
      } else if (!stage) {
        const uint8_t identity = device ? 0xFF : 0x68;
        if (bus.data[0] == identity) stage = 1;
        else if (!device) mpuAddress ^= 1;
      } else if ((!device && stage < 5) || (device && stage < 3)) {
        stage++;
      } else if (!device && stage == 5) {
        // INT_STATUS precedes the accelerometer burst. Read it first: reading
        // data alone is not proof of a fresh measurement.
        if (bus.data[0] & 1) stage = 6;
      } else if (device && stage == 3) {
        // QMC clears DRDY when any data register is read. Status MUST precede
        // the burst; OVL samples must still be drained to release protection.
        if (bus.data[0] & 2) magneticValid = false;
        if (bus.data[0] & 1) stage = (bus.data[0] & 2) ? 5 : 4;
      } else {
        bool valid = !device || stage == 4;
        for (uint8_t i = 0; i < 3; i++) {
          const uint8_t offset = 2 * i;
          const int16_t value = device
              ? int16_t(uint16_t(bus.data[offset + 1]) << 8 | bus.data[offset])
              : int16_t(uint16_t(bus.data[offset]) << 8 | bus.data[offset + 1]);
          if (value == -32768 || value == 32767) valid = false;
          if (device) raw.magnetic[i] = int32_t(value) * 5 / 6; // ±2G: .01 µT
          else raw.gravity[i] = -int32_t(value) * 9807 / 16384; // specific force -> DOWN
        }
        if (device) {
          magneticValid = valid;
          if (valid) magneticAt = now;
          stage = 3;
        } else {
          gravityValid = valid;
          if (valid) { gravityAt = now; raw.sequence++; }
          stage = 5;
        }
      }
      device ^= 1;
      next = now + 20;
    }

    if (int32_t(now - next) < 0) return;
    const uint8_t stage = device ? qmc : mpu;
    uint8_t reg = 0, count = 0, value = 0;
    if (device) {
      if (stage == 0) { reg = 0x0D; count = 1; }
      else if (stage == 1) { reg = 0x0B; value = 1; }
      else if (stage == 2) { reg = 0x09; value = 0x05; } // 50Hz, ±2G, OSR512
      else if (stage == 3) { reg = 0x06; count = 1; }
      else { reg = 0; count = 6; }
    } else {
      if (stage == 0) { reg = 0x75; count = 1; }
      else if (stage == 1) { reg = 0x6B; value = 1; } // wake, gyro PLL
      else if (stage == 2) { reg = 0x1A; value = 5; } // 10Hz accel low-pass
      else if (stage == 3) { reg = 0x1C; value = 0; } // ±2g
      else if (stage == 4) { reg = 0x38; value = 1; } // data-ready status
      else if (stage == 5) { reg = 0x3A; count = 1; }
      else { reg = 0x3B; count = 6; }
    }
    bus.start(device ? 0x0D : mpuAddress, reg, count, value, now);
    pending = true;
  }
};
