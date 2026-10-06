#pragma once
// Framed DEC protocol v3 — the half that is pure computation.
//
// Wire format and the reasoning behind every field: src/tmc2209/protocol.py.
//
//     '#' HEX(LEN) HEX(OP) HEX(SEQ) HEX(PAYLOAD[LEN]) HEX(CRC16) '\n'
//
// This header holds the codec and nothing else: no pins, no motion state, no
// Arduino. That is deliberate — the board cannot be flashed and probed from a
// unit test, so the only way to prove that the firmware builds the same bytes
// the driver expects is to compile this file for the host and compare it with
// the golden frames (telescope_dec/test/frame_v3_selftest.cpp, the same literals
// as src/tests/units/test_dec_framed_protocol.py).
//
// The frame *dispatch* stays in main.cpp: it touches the motion state and the TX
// ring, which are the firmware's business, not the protocol's.

#include <stdint.h>

static const uint8_t FRAME_PROTOCOL_VERSION_V3 = 3;
static const uint8_t FRAME_FIRMWARE_BUILD_V3 = 4;

// LEN + OP + SEQ, then the payload, then two bytes of CRC.
static const uint8_t FRAME_HEADER_V3 = 3;
static const uint8_t FRAME_CRC_V3 = 2;
static const uint8_t FRAME_OVERHEAD_V3 = FRAME_HEADER_V3 + FRAME_CRC_V3;

enum FrameOpV3 : uint8_t {
  OP_ERROR_V3 = 0x00,
  OP_HELLO_V3 = 0x01,
  OP_STATUS_V3 = 0x02,
  OP_POSITION_V3 = 0x10,
  OP_SPEED_V3 = 0x11,
  OP_ACCELERATION_V3 = 0x12,
  OP_DIRECTION_V3 = 0x13,
  OP_DELTA_V3 = 0x14,
  OP_MODE_V3 = 0x15,
  OP_ENABLED_V3 = 0x16,
  OP_MICROSTEPS_V3 = 0x17,
  OP_RUN_V3 = 0x20,
  OP_STOP_V3 = 0x21,
  OP_SENSOR_V3 = 0x30
};

// Same numbers as ErrorCode in protocol.py; 1..9 are the line protocol's own errors.
enum FrameErrorV3 : uint8_t {
  FRAME_OK_V3 = 0,
  ERR_UNKNOWN_CMD_V3 = 1,
  ERR_BAD_VALUE_V3 = 2,
  ERR_RANGE_V3 = 3,
  ERR_INVALID_MICROSTEPS_V3 = 8,
  ERR_TX_OVERFLOW_V3 = 10,
  ERR_BAD_CRC_V3 = 11,
  ERR_BAD_FRAME_V3 = 12,
  ERR_DRIVER_FAULT_V3 = 13
};

static const uint8_t STATUS_PAYLOAD_LEN_V3 = 38;

// Status flag bits (payload byte 0).
static const uint8_t FRAME_FLAG_INITIALISED_V3 = 0x01;
static const uint8_t FRAME_FLAG_ENABLED_V3 = 0x02;
static const uint8_t FRAME_FLAG_FREE_RIDE_V3 = 0x04;
static const uint8_t FRAME_FLAG_TARGET_SET_V3 = 0x08;

// CRC-16/CCITT-FALSE, bitwise: a table would cost 512 bytes of flash for payloads
// that never exceed 38 bytes. The check value over "123456789" is 0x29B1, and that
// constant is what pins this implementation to the host's.
static inline uint16_t crc16UpdateV3(uint16_t crc, uint8_t value) {
  crc ^= (uint16_t)value << 8;
  for (uint8_t bit = 0; bit < 8; bit++) {
    crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// Where the assembled characters go. The firmware hands in its TX ring, the self
// test a plain buffer; neither is this header's concern.
typedef void (*FrameSinkV3)(char);

struct FrameWriterV3 {
  FrameSinkV3 sink;
  uint16_t crc;

  void hex(uint8_t value) const {
    const uint8_t hi = (uint8_t)(value >> 4);
    const uint8_t lo = (uint8_t)(value & 0x0F);
    sink((char)(hi < 10 ? ('0' + hi) : ('A' + hi - 10)));
    sink((char)(lo < 10 ? ('0' + lo) : ('A' + lo - 10)));
  }

  void byte(uint8_t value) {
    crc = crc16UpdateV3(crc, value);
    hex(value);
  }

  void begin(uint8_t op, uint8_t seq, uint8_t payloadLen) {
    crc = 0xFFFF;
    sink('#');
    byte(payloadLen);
    byte(op);
    byte(seq);
  }

  void u16(uint16_t value) {
    byte((uint8_t)(value >> 8));
    byte((uint8_t)(value & 0xFF));
  }

  void u32(uint32_t value) {
    byte((uint8_t)(value >> 24));
    byte((uint8_t)(value >> 16));
    byte((uint8_t)(value >> 8));
    byte((uint8_t)(value & 0xFF));
  }

  void i32(int32_t value) { u32((uint32_t)value); }

  void end() const {
    hex((uint8_t)(crc >> 8));
    hex((uint8_t)(crc & 0xFF));
    sink('\n');
  }
};

// Fixed point with two decimals, the same resolution the line protocol printed
// with dtostrf(v, 0, 2, ...): the host divides by 100 and gets the same string.
static inline uint32_t frameCentiV3(float value) {
  if (value <= 0.0f) return 0;
  return (uint32_t)(value * 100.0f + 0.5f);
}

struct RawSensorV3 {
  uint8_t flags;  // bit 0: present, bit 1: gravity valid, bit 2: magnetic valid
  uint32_t sequence;
  uint16_t ageMs;
  int16_t gravity[3];  // 0.001 m/s², sensor frame, points down
  int16_t magnetic[3];  // 0.01 µT, same frame
};

static inline void frameWriteSensorV3(FrameWriterV3& writer, uint8_t seq, const RawSensorV3& sensor) {
  writer.begin(OP_SENSOR_V3, seq, 19);
  writer.byte(sensor.flags);
  writer.u32(sensor.sequence);
  writer.u16(sensor.ageMs);
  for (uint8_t i = 0; i < 3; i++) writer.u16((uint16_t)sensor.gravity[i]);
  for (uint8_t i = 0; i < 3; i++) writer.u16((uint16_t)sensor.magnetic[i]);
}

// The 38-byte status snapshot, in one place because it is the one payload with a
// layout worth getting wrong. The caller
// passes plain numbers, so the same code runs on the board and in the self test.
static inline void frameWriteStatusV3(FrameWriterV3& writer, uint8_t seq, uint8_t flags, uint8_t phase,
                                      int32_t position, int32_t target, float speedSps, float actualSps,
                                      float accelSps2, float powerV, uint16_t txOverflow,
                                      uint8_t driverFlags, uint8_t safetyState,
                                      uint16_t activeMicrosteps, uint16_t fineMicrosteps,
                                      uint32_t speedLimitSps, uint16_t safetyEvents) {
  writer.begin(OP_STATUS_V3, seq, STATUS_PAYLOAD_LEN_V3);
  writer.byte(flags);
  // The phase code is the wire value: MOTION_PHASE_*_V2 in main.cpp and PHASE_NAMES
  // in protocol.py are the same list in the same order, and must stay that way.
  writer.byte(phase);
  writer.i32(position);
  writer.i32(target);
  writer.u32(frameCentiV3(speedSps));
  writer.u32(frameCentiV3(actualSps));
  writer.u32(frameCentiV3(accelSps2));
  const uint32_t powerCenti = frameCentiV3(powerV);
  writer.u16((uint16_t)(powerCenti > 0xFFFFUL ? 0xFFFFUL : powerCenti));
  writer.u16(txOverflow);
  writer.byte(driverFlags);
  writer.byte(safetyState);
  writer.u16(activeMicrosteps);
  writer.u16(fineMicrosteps);
  writer.u32(speedLimitSps);
  writer.u16(safetyEvents);
}

struct FrameV3 {
  uint8_t error;
  uint8_t op;
  uint8_t seq;
  uint8_t len;
  const uint8_t* payload;
};

static inline bool frameHexNibbleV3(char c, uint8_t* value) {
  if (c >= '0' && c <= '9') { *value = (uint8_t)(c - '0'); return true; }
  if (c >= 'A' && c <= 'F') { *value = (uint8_t)(c - 'A' + 10); return true; }
  if (c >= 'a' && c <= 'f') { *value = (uint8_t)(c - 'a' + 10); return true; }
  return false;
}

// Parse one received line *in place*. `line` is modified: the hex is decoded over
// itself, which is safe because two characters collapse into one byte, and costs
// no second buffer on a board with 208 free bytes of RAM.
//
// `len` is the number of bytes received, and it is a *length* rather than a NUL
// terminator on purpose. The line arrives from a wire that can deliver any of the
// 256 byte values, and an embedded 0x00 used to end the scan early: a frame with a
// NUL in front of its marker was not recognised as a frame at all and the board
// answered nothing (measured live: 0 replies out of 10, against 10 out of 10 for
// the same frame without the NUL). Length-driven parsing sees the whole line, so
// the NUL becomes what it should always have been — a non-hex character, refused
// with `bad_frame` like any other damage.
//
// On failure `error` says which check refused the frame and `seq` carries the
// sequence number as received — possibly the damaged byte itself, but answering
// with it is still right: a host that did not send that number rejects the answer.
static inline FrameV3 frameParseV3(char* line, uint8_t len) {
  FrameV3 frame = {ERR_BAD_FRAME_V3, 0, 0, 0, 0};

  // Resynchronise on the newest marker. The 64-byte RX FIFO overflows while the
  // board is busy and leaves half a command in the line buffer; the next write
  // then arrives glued to that stump. The newest frame is the one somebody is
  // still waiting for, so the stump is dropped rather than parsed.
  uint8_t start = 0;
  for (uint8_t i = 0; i < len; i++) {
    if (line[i] == '#') start = (uint8_t)(i + 1);
  }

  char* body = line + start;
  const uint8_t bodyLen = (uint8_t)(len - start);

  uint8_t* raw = (uint8_t*)body;
  uint8_t count = 0;
  for (uint8_t i = 0; i + 1 < bodyLen; i += 2) {
    uint8_t hi = 0;
    uint8_t lo = 0;
    if (!frameHexNibbleV3(body[i], &hi) || !frameHexNibbleV3(body[i + 1], &lo)) {
      // A non-hex character means bytes were lost or altered on the way in —
      // which is the point of sending hex at all.
      return frame;
    }
    raw[count++] = (uint8_t)((hi << 4) | lo);
  }
  // An odd character count is the other half of that check: a lost byte shifts
  // every following nibble, and the leftover character is what gives it away.
  if (bodyLen & 1) return frame;

  if (count < FRAME_OVERHEAD_V3) return frame;

  frame.len = raw[0];
  frame.op = raw[1];
  frame.seq = raw[2];
  frame.payload = raw + FRAME_HEADER_V3;

  // A declared length that does not match what arrived is a *structural* failure,
  // reported apart from a CRC mismatch — and it is checked first, because the
  // dispatcher reads `len` payload bytes and a frame claiming more than it
  // carries would send it past the end of the buffer.
  if ((uint16_t)count != (uint16_t)frame.len + FRAME_OVERHEAD_V3) {
    frame.error = ERR_BAD_FRAME_V3;
    return frame;
  }

  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i + FRAME_CRC_V3 < count; i++) crc = crc16UpdateV3(crc, raw[i]);
  const uint16_t received = (uint16_t)(((uint16_t)raw[count - 2] << 8) | raw[count - 1]);
  if (crc != received) {
    frame.error = ERR_BAD_CRC_V3;
    return frame;
  }

  frame.error = FRAME_OK_V3;
  return frame;
}

static inline int32_t framePayloadI32V3(const uint8_t* payload) {
  return (int32_t)(((uint32_t)payload[0] << 24) | ((uint32_t)payload[1] << 16) |
                   ((uint32_t)payload[2] << 8) | (uint32_t)payload[3]);
}

static inline uint16_t framePayloadU16V3(const uint8_t* payload) {
  return (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);
}
