#pragma once
#include <stdint.h>

// One bounded, asynchronous register transaction. Port is the AVR TWI register
// adapter (or the same adapter driven by a host test). No Wire buffers or ISR.
template<class Port> struct SensorI2c {
  uint8_t data[6] = {};
  uint8_t phase = 0, address = 0, reg = 0, count = 0, index = 0, value = 0;
  uint32_t started = 0;
  bool ok = false;

  void start(uint8_t addr, uint8_t r, uint8_t n, uint8_t v, uint32_t now) {
    address = addr; reg = r; count = n; value = v; index = 0;
    started = now; ok = false; phase = 1;
    Port::start();
  }

  void tick(uint32_t now) {
    if (!phase) return;
    if (uint32_t(now - started) >= 20) {
      Port::reset(); phase = 0; ok = false;
      return;
    }
    if (phase == 8) {
      if (!Port::stopping()) phase = 0;
      return;
    }
    if (!Port::ready()) return;

    const uint8_t status = Port::status();
    if (phase == 1 && status == 0x08) {
      Port::send(address << 1); phase = 2;
    } else if (phase == 2 && status == 0x18) {
      Port::send(reg); phase = 3;
    } else if (phase == 3 && status == 0x28) {
      if (count) { Port::start(); phase = 4; }
      else { Port::send(value); phase = 7; }
    } else if (phase == 4 && status == 0x10) {
      Port::send((address << 1) | 1); phase = 5;
    } else if (phase == 5 && status == 0x40) {
      Port::receive(count > 1); phase = 6;
    } else if (phase == 6 && status == (index + 1 < count ? 0x50 : 0x58)) {
      data[index++] = Port::data();
      if (index < count) Port::receive(index + 1 < count);
      else { ok = true; Port::stop(); phase = 8; }
    } else if (phase == 7 && status == 0x28) {
      ok = true; Port::stop(); phase = 8;
    } else {
      // NACK, arbitration loss and bus errors discard the whole transaction.
      Port::stop(); phase = 8; ok = false;
    }
  }
};
