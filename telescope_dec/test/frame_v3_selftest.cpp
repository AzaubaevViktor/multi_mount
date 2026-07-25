// Host self-test for the firmware's half of the framed DEC protocol.
//
//     c++ -std=c++11 -O1 -Wall -Wextra -o /tmp/frame_v3_selftest \
//         telescope_dec/test/frame_v3_selftest.cpp && /tmp/frame_v3_selftest
//
// Why this exists. The board cannot be flashed and probed from the unit suite —
// it is not always attached, and reflashing is a separate, deliberate step. So
// "the firmware compiles" would otherwise be the only thing anybody could say
// about a protocol implementation that has to agree with the driver byte for
// byte. include/frame_v3.h has no Arduino in it precisely so that the *same*
// code can be compiled here and compared against the golden frames from
// src/tests/units/test_dec_framed_protocol.py.
//
// Not wired into PlatformIO's test runner on purpose: that would pull in the
// `native` platform (a download) for four asserts. It is a plain program.

#include <cstdio>
#include <cstring>
#include <string>

#include "../include/frame_v3.h"

static std::string emitted;
static void collect(char c) { emitted.push_back(c); }
static FrameWriterV3 writer = {collect, 0xFFFF};

static int failures = 0;

static void check(bool condition, const char* what) {
  if (!condition) {
    failures++;
    std::printf("FAIL %s\n", what);
  } else {
    std::printf("ok   %s\n", what);
  }
}

static void checkEqual(const std::string& actual, const std::string& expected, const char* what) {
  if (actual != expected) {
    failures++;
    std::printf("FAIL %s\n     expected %s\n     actual   %s\n", what, expected.c_str(), actual.c_str());
  } else {
    std::printf("ok   %s = %s\n", what, actual.c_str());
  }
}

static std::string frame(uint8_t op, uint8_t seq, const uint8_t* payload, uint8_t len) {
  emitted.clear();
  writer.begin(op, seq, len);
  for (uint8_t i = 0; i < len; i++) writer.byte(payload[i]);
  writer.end();
  return emitted;
}

int main() {
  // 1. CRC-16/CCITT-FALSE against its published check value. Without this the two
  //    implementations could agree on the same wrong checksum forever.
  uint16_t crc = 0xFFFF;
  const char* check_input = "123456789";
  for (const char* p = check_input; *p; p++) crc = crc16UpdateV3(crc, (uint8_t)*p);
  check(crc == 0x29B1, "crc16(\"123456789\") == 0x29B1");

  // 2. The golden frames, character for character the same literals the driver
  //    tests assert on.
  checkEqual(frame(OP_STATUS_V3, 1, 0, 0), "#000201BADF\n", "status request");
  checkEqual(frame(OP_HELLO_V3, 1, 0, 0), "#000101EF8C\n", "hello request");

  const uint8_t speed[4] = {0x00, 0x00, 0x03, 0xE8};
  checkEqual(frame(OP_SPEED_V3, 7, speed, 4), "#041107000003E8218D\n", "speed=1000 request");

  // 3. The whole status payload, through the function the firmware itself calls.
  emitted.clear();
  frameWriteStatusV3(writer, 0x2A,
                     (uint8_t)(FRAME_FLAG_INITIALISED_V3 | FRAME_FLAG_ENABLED_V3 | FRAME_FLAG_TARGET_SET_V3),
                     3, 123456, 200000, 1000.0f, 980.0f, 1000.0f, 12.34f, 7);
  writer.end();
  checkEqual(emitted, "#1A022A0B030001E24000030D40000186A000017ED0000186A004D200078986\n", "status reply");
  check(emitted.size() == 64, "a status reply is 64 bytes (147 in the line protocol)");

  // 4. The receive path: a good frame, and every way of damaging it.
  char good[] = "#041107000003E8218D";
  FrameV3 parsed = frameParseV3(good, sizeof(good) - 1);
  check(parsed.error == FRAME_OK_V3 && parsed.op == OP_SPEED_V3 && parsed.seq == 7 && parsed.len == 4,
        "a well formed frame parses");
  check(framePayloadI32V3(parsed.payload) == 1000, "its payload reads back as 1000");

  char corrupted[] = "#041107000003E9218D";  // one hex digit changed
  check(frameParseV3(corrupted, sizeof(corrupted) - 1).error == ERR_BAD_CRC_V3, "an altered byte fails the CRC");

  char shortened[] = "#041107000003E8218";  // one character lost
  check(frameParseV3(shortened, sizeof(shortened) - 1).error == ERR_BAD_FRAME_V3, "an odd hex count is refused");

  // Declares nine payload bytes, carries four — and its CRC is *correct*, which is
  // the only way to prove the length is checked in its own right. Without that
  // check the dispatcher would go on to read nine bytes out of a four-byte payload.
  char mislabelled[] = "#091107000003E8CB87";
  check(frameParseV3(mislabelled, sizeof(mislabelled) - 1).error == ERR_BAD_FRAME_V3, "a wrong declared length is refused on its own");

  char junk[] = "#04110ZZZ0003E8218D";
  check(frameParseV3(junk, sizeof(junk) - 1).error == ERR_BAD_FRAME_V3, "a non-hex character is refused");

  // The RX FIFO cut a command in half and the next one arrived glued to the stump.
  char glued[] = "#0411#041107000003E8218D";
  FrameV3 resynced = frameParseV3(glued, sizeof(glued) - 1);
  check(resynced.error == FRAME_OK_V3 && resynced.seq == 7, "the board resynchronises on the newest marker");

  char empty[] = "#";
  check(frameParseV3(empty, sizeof(empty) - 1).error == ERR_BAD_FRAME_V3, "an empty frame body is refused");

  // A NUL in front of the marker. On the board this used to be invisible: the
  // dialect was chosen with strchr() over a C string, so the scan stopped at the
  // NUL, no marker was found, and the frame went to the line parser which answered
  // nothing at all (0 replies out of 10, measured live). Parsing by length instead
  // sees the whole line, and the frame behind the NUL is answered normally.
  char nulPrefixed[] = "\0#041107000003E8218D";
  FrameV3 behindNul = frameParseV3(nulPrefixed, sizeof(nulPrefixed) - 1);
  check(behindNul.error == FRAME_OK_V3 && behindNul.seq == 7,
        "a frame behind a NUL is still parsed");

  // The mirror case: a NUL *inside* the hex is damage like any other, and the
  // length-driven scan must refuse it rather than stop early and call it a frame.
  char nulInside[] = "#04110700\0003E8218D";
  check(frameParseV3(nulInside, sizeof(nulInside) - 1).error == ERR_BAD_FRAME_V3,
        "a NUL inside the payload is refused as damage");

  // 5. Fixed point: the host divides by 100 and must get the string the line
  //    protocol used to print, so the rounding has to match dtostrf's, not C's cast.
  check(frameCentiV3(12.34f) == 1234, "12.34 V -> 1234 centivolts");
  // 1.05f * 100.0f is 104.99999 in single precision, so a plain cast would report
  // 1.04 V for a supply that reads 1.05 — one count of drift on every such value.
  check(frameCentiV3(1.05f) == 105, "1.05 -> 105, i.e. rounded and not truncated");
  check(frameCentiV3(-1.0f) == 0, "a negative value clamps to zero");

  std::printf(failures ? "\n%d FAILURE(S)\n" : "\nall checks passed\n", failures);
  return failures ? 1 : 0;
}
