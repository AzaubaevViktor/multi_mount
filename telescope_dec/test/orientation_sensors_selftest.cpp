// Exercises the production TWI and sensor state machines together, with fake
// register events. No Arduino or connected hardware is needed.
#include <cassert>
#include <cstdio>
#include <cstring>
#include "../include/sensor_i2c.h"
#include "../include/orientation_sensors.h"

struct FakePort {
  static uint8_t state, byte, pointer, addr, mpuAddress;
  static bool active, stuck, stopStuck, mpuPresent, qmcPresent, fresh, qmcFresh, overflow;
  static unsigned resets, writes;
  static uint8_t mpu[128], qmc[128];
  static void reset() { active = false; stopStuck = false; resets++; }
  static void start() { state = active ? 0x10 : 0x08; active = true; }
  static void stop() { active = false; }
  static bool stopping() { return stopStuck; }
  static bool ready() { return !stuck; }
  static uint8_t status() { return state; }
  static uint8_t data() { return byte; }
  static void send(uint8_t value) {
    if (state == 0x08 || state == 0x10) {
      addr = value >> 1;
      const bool present = (addr == mpuAddress && mpuPresent) || (addr == 0x0D && qmcPresent);
      state = present ? ((value & 1) ? 0x40 : 0x18) : ((value & 1) ? 0x48 : 0x20);
    } else if (state == 0x18) {
      pointer = value; state = 0x28;
    } else {
      (addr == 0x0D ? qmc : mpu)[pointer] = value; writes++; state = 0x28;
    }
  }
  static void receive(bool ack) {
    if (pointer == 0x3A && addr != 0x0D) byte = fresh ? 1 : 0;
    else if (pointer == 6 && addr == 0x0D) byte = ((fresh && qmcFresh) ? 1 : 0) | (overflow ? 2 : 0);
    else byte = (addr == 0x0D ? qmc : mpu)[pointer];
    pointer++; state = ack ? 0x50 : 0x58;
  }
};
uint8_t FakePort::state, FakePort::byte, FakePort::pointer, FakePort::addr;
uint8_t FakePort::mpuAddress = 0x68;
bool FakePort::active, FakePort::stuck, FakePort::stopStuck;
bool FakePort::mpuPresent, FakePort::qmcPresent, FakePort::overflow;
bool FakePort::fresh = true;
bool FakePort::qmcFresh = true;
unsigned FakePort::resets, FakePort::writes;
uint8_t FakePort::mpu[128], FakePort::qmc[128];
using Bus = SensorI2c<FakePort>;
using Sensors = OrientationSensors<Bus>;

static void advance(Sensors& sensors, uint32_t& now, unsigned ms) {
  while (ms--) sensors.tick(now++);
}

int main() {
  FakePort::mpu[0x75] = 0x68;
  FakePort::qmc[0x0D] = 0xFF;
  FakePort::mpu[0x3F] = 0x40; // +1g Z, big endian -> gravity -9807
  FakePort::qmc[0] = 0x10; FakePort::qmc[1] = 0x0E; // 3600 -> 3000 (.01µT)
  FakePort::qmc[2] = 0xF0; FakePort::qmc[3] = 0xF1; // -3600 -> -3000
  Sensors sensors;
  uint32_t now = 0;
  advance(sensors, now, 1000);
  assert(sensors.snapshot(now).flags == 0 && sensors.raw.sequence == 0);
  std::puts("ok absent sensors: no measurements, repeated NACK does not hang");

  FakePort::qmcPresent = true;
  advance(sensors, now, 500);
  assert(sensors.snapshot(now).flags == 1);
  std::puts("ok QMC only: present, no gravity or sky solution");

  FakePort::mpuPresent = true;
  advance(sensors, now, 1000);
  auto sample = sensors.snapshot(now);
  assert(sample.flags == 7 && sample.gravity[2] == -9807);
  assert(sample.magnetic[0] == 3000 && sample.magnetic[1] == -3000);
  assert(sample.sequence > 0 && sample.ageMs < 200);
  assert(FakePort::mpu[0x6B] == 1 && FakePort::mpu[0x1A] == 5);
  assert(FakePort::mpu[0x1C] == 0 && FakePort::mpu[0x38] == 1);
  assert(FakePort::qmc[0x09] == 5 && FakePort::qmc[0x0B] == 1);
  std::puts("ok hot plug: configure both, signed conversion, opposite gravity sign");

  FakePort::overflow = true;
  advance(sensors, now, 300);
  assert(sensors.snapshot(now).flags == 3);
  FakePort::overflow = false;
  advance(sensors, now, 300);
  assert(sensors.snapshot(now).flags == 7);
  FakePort::qmc[1] = 0x7F; FakePort::qmc[0] = 0xFF;
  advance(sensors, now, 300);
  assert(sensors.snapshot(now).flags == 3);
  FakePort::qmc[1] = 0x0E; FakePort::qmc[0] = 0x10;
  std::puts("ok magnetometer overflow and clipped ADC: heading withheld, recovery");

  FakePort::qmcFresh = false;
  advance(sensors, now, 500);
  assert(sensors.snapshot(now).flags == 3 && sensors.snapshot(now).ageMs < 200);
  FakePort::qmcFresh = true;
  advance(sensors, now, 300);
  assert(sensors.snapshot(now).flags == 7);
  std::puts("ok frozen magnetometer: new gravity cannot refresh old magnetic data");

  FakePort::mpu[0x3F] = 0x80;
  advance(sensors, now, 300);
  assert(sensors.snapshot(now).flags == 1);
  FakePort::mpu[0x3F] = 0x40;
  advance(sensors, now, 300);
  assert(sensors.snapshot(now).flags == 7);
  std::puts("ok clipped accelerometer: no gravity or sky solution until recovery");

  FakePort::qmcPresent = false;
  advance(sensors, now, 300);
  assert(sensors.snapshot(now).flags == 3);
  std::puts("ok MPU only: gravity remains available after unplugging magnetometer");

  FakePort::fresh = false;
  advance(sensors, now, 300);
  const auto seq = sensors.raw.sequence;
  const auto age = sensors.snapshot(now).ageMs;
  advance(sensors, now, 70000);
  assert(sensors.raw.sequence == seq && sensors.snapshot(now).ageMs == 65535 && age > 200);
  std::puts("ok no data-ready: no fabricated fresh samples, age saturates");

  FakePort::fresh = true; FakePort::mpuPresent = false;
  advance(sensors, now, 300);
  assert(sensors.snapshot(now).flags == 0);
  FakePort::mpuAddress = 0x69; FakePort::mpuPresent = true;
  advance(sensors, now, 1000);
  assert(sensors.snapshot(now).flags == 3 && sensors.mpuAddress == 0x69);
  std::puts("ok MPU AD0 high: address 0x69 auto detected");

  FakePort::stuck = true;
  advance(sensors, now, 400);
  assert(FakePort::resets > 0 && sensors.snapshot(now).flags == 0);
  FakePort::stuck = false; FakePort::qmcPresent = true;
  advance(sensors, now, 1000);
  assert(sensors.snapshot(now).flags == 7);
  std::puts("ok stalled TWI: bounded timeout, discard, recovery");

  Bus bus;
  FakePort::stopStuck = true;
  bus.start(0x69, 0x75, 1, 0, now);
  for (unsigned i = 0; i < 25; i++) bus.tick(now++);
  assert(!bus.phase && !bus.ok && !FakePort::stopStuck);
  std::puts("ok stuck STOP also times out");

  bus.start(0x69, 0x3B, 6, 0, now);
  for (unsigned i = 0; i < 6; i++) bus.tick(now++);
  FakePort::state = 0; // bus error halfway through a burst
  bus.tick(now++); bus.tick(now++);
  assert(!bus.phase && !bus.ok);
  std::puts("ok truncated burst / bus error: never accepted as a sample");

  // All unsigned elapsed-time calculations must survive millis() rollover.
  Sensors rollover;
  now = 0xFFFFFF00UL;
  rollover.next = now;
  advance(rollover, now, 1500);
  assert(rollover.snapshot(now).flags == 7 && rollover.snapshot(now).ageMs < 200);
  std::puts("ok millis rollover: acquisition and ages continue");

  FakePort::mpu[0x75] = 0x70; FakePort::qmc[0x0D] = 0;
  Sensors wrong;
  now = 0;
  advance(wrong, now, 1000);
  assert(wrong.snapshot(now).flags == 0);
  std::puts("ok incompatible chip identities are rejected");
}
