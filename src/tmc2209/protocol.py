"""Wire protocol of the DEC controller: the framed dialect (v3) and the legacy line one (v2).

Why a second dialect exists
---------------------------
The line protocol documented in ``docs/protocol/DEC_PROTOCOL.md`` carries neither a
length nor a checksum, and the board's own failure modes attack exactly those two
gaps: the 255-byte TX ring cuts a reply on an arbitrary byte boundary and the next
reply is glued onto the stump (§6.1), the 64-byte RX FIFO swallows whole commands
(§6.2), and a stray ``\\r`` is deleted *silently* in any position (§5.5). A digit
lost inside ``position=12345`` therefore produces ``position=1234`` — a frame that
passes every check the host can make and moves the axis to the wrong place.

The framed dialect closes that by construction. Every frame states its own length
and ends with a CRC over its whole body, so a byte that is lost, added or altered
is detected three times over: the hex nibble count goes odd, the declared length
stops matching, and the CRC fails.

Frame layout
------------
One line of printable ASCII, terminated by ``\\n``::

    '#' HEX(LEN) HEX(OP) HEX(SEQ) HEX(PAYLOAD[LEN]) HEX(CRC16) '\\n'

``#``
    Frame start marker and the resynchronisation anchor. A receiver skips whatever
    precedes the last usable ``#`` instead of folding the garbage into the frame,
    which is what turns the TX-ring glue from silent corruption into a resync.
``LEN``
    Payload length in bytes, 0..64. Excludes OP, SEQ and the CRC. This is the
    field the old protocol lacked: a truncated frame is short, and short is now a
    provable statement rather than "maybe the reply just had fewer fields".
``OP``
    Opcode; in a reply it is the request's opcode echoed back, or ``0x00`` = error.
    Echoing it means a reply can never be attributed to the wrong command.
``SEQ``
    Request counter, echoed verbatim. A reply that was still sitting in the ring
    from a previous, timed-out command is rejected instead of being taken for the
    answer to the current one.
``CRC16``
    CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over LEN, OP, SEQ and PAYLOAD.
    Two bytes, computed bitwise on both sides — no table, because the firmware has
    211 bytes of RAM to spare.

Payloads are big-endian, which keeps the hex dump readable in the byte recorder
traces the whole protocol documentation is built on.

Why hex and not raw binary
--------------------------
``SerialLine`` is shared with the RA driver: it encodes payloads as ASCII, reads
up to a byte terminator and decodes with ``errors="ignore"``. Raw binary would
need byte stuffing (a payload byte 0x0A ends the line), would be silently mangled
by that decode, and would mean changing a module three other drivers sit on.
Hex costs a factor of two on the wire and buys a frame that survives the existing
transport untouched. The extended ``status`` reply is 88 bytes, so two complete
snapshots still fit in the 255-byte TX ring.
"""

import dataclasses
from enum import IntEnum

FRAME_START = "#"
FRAME_TERMINATOR = "\n"
# LEN + OP + SEQ, the part of the frame that is always present.
HEADER_SIZE = 3
CRC_SIZE = 2
# A command carries at most one 32-bit argument; the status reply is the largest
# frame in either direction. The ceiling keeps a frame inside the board's 64-byte
# hardware RX FIFO and lets a receiver reject an absurd length before allocating.
MAX_PAYLOAD = 64

PROTOCOL_VERSION = 3


class Op(IntEnum):
    """Opcodes. Values are frozen: they travel on the wire and into recorded traces."""

    ERROR = 0x00
    HELLO = 0x01
    STATUS = 0x02
    POSITION = 0x10
    SPEED = 0x11
    ACCELERATION = 0x12
    DIRECTION = 0x13
    DELTA = 0x14
    MODE = 0x15
    ENABLED = 0x16
    MICROSTEPS = 0x17
    RUN = 0x20
    STOP = 0x21
    SENSOR = 0x30


class ErrorCode(IntEnum):
    """Error payload codes. 1..9 are the firmware's existing line-protocol errors."""

    UNKNOWN_CMD = 1
    BAD_VALUE = 2
    RANGE = 3
    MISSING_PARAM = 4
    UNKNOWN_PARAM = 5
    BAD_PARAM = 6
    SINGLE_PARAM = 7
    INVALID_MICROSTEPS = 8
    INVALID_BOOL = 9
    TX_OVERFLOW = 10
    BAD_CRC = 11
    BAD_FRAME = 12
    DRIVER_FAULT = 13


# The names the line protocol used, so that a caller (and every log line) sees the same
# error string whichever dialect it is talking.
ERROR_NAMES: dict[int, str] = {
    ErrorCode.UNKNOWN_CMD: "unknown_cmd",
    ErrorCode.BAD_VALUE: "bad_value",
    ErrorCode.RANGE: "range",
    ErrorCode.MISSING_PARAM: "missing_param",
    ErrorCode.UNKNOWN_PARAM: "unknown_param",
    ErrorCode.BAD_PARAM: "bad_param",
    ErrorCode.SINGLE_PARAM: "single_param",
    ErrorCode.INVALID_MICROSTEPS: "invalid_microsteps",
    ErrorCode.INVALID_BOOL: "invalid_bool",
    ErrorCode.TX_OVERFLOW: "tx_overflow",
    ErrorCode.BAD_CRC: "bad_crc",
    ErrorCode.BAD_FRAME: "bad_frame",
    ErrorCode.DRIVER_FAULT: "driver_fault",
}

PHASE_NAMES: tuple[str, ...] = ("idle", "hold", "acceleration", "running", "deceleration")
MODE_NAMES: tuple[str, ...] = ("target", "free_ride")
SAFETY_NAMES: tuple[str, ...] = ("normal", "derated", "stopping", "shutdown")

# Status flag bits (payload byte 0).
FLAG_INITIALISED = 0x01
FLAG_ENABLED = 0x02
FLAG_FREE_RIDE = 0x04
FLAG_TARGET_SET = 0x08

LEGACY_STATUS_PAYLOAD_SIZE = 26
STATUS_PAYLOAD_SIZE = 38

RESPONSE_DELIMITER = ";"
KEY_VALUE_SEPARATOR = "="


class TMC2209MotorError(Exception):
    pass


class TMC2209MotorProtocolError(TMC2209MotorError):
    pass


class TMC2209MotorCommandError(TMC2209MotorError):
    pass


class TMC2209MotorTimeoutError(TMC2209MotorProtocolError):
    """Nothing came back at all: the controller is silent or the port is gone."""


class TMC2209MotorTruncatedResponseError(TMC2209MotorProtocolError):
    """A response arrived cut off - the controller dropped bytes or the read timed out mid-line."""


class TMC2209MotorConcatenatedResponseError(TMC2209MotorProtocolError):
    """Two responses arrived glued together, the first one having lost its terminator."""


class TMC2209MotorIntegrityError(TMC2209MotorProtocolError):
    """The frame arrived complete but its CRC does not match: a byte was altered or moved."""


class TMC2209MotorStaleResponseError(TMC2209MotorProtocolError):
    """The frame is valid but answers another request (wrong sequence number or opcode)."""


class TMC2209MotorEchoMismatchError(TMC2209MotorProtocolError):
    """The controller acknowledged a value different from the one that was sent."""


class TMC2209MotorLegacyResponseError(TMC2209MotorProtocolError):
    """A v2 line-protocol reply arrived where a frame was expected: the board runs old firmware."""


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.

    Bitwise on purpose. A table costs 512 bytes of AVR flash for a payload that is
    never longer than 38 bytes, and both sides must agree byte for byte, so the two
    implementations are kept trivially comparable.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclasses.dataclass(frozen=True)
class Frame:
    op: int
    seq: int
    payload: bytes


def encode_frame(op: int, seq: int, payload: bytes = b"") -> str:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload of {len(payload)} bytes exceeds the {MAX_PAYLOAD}-byte frame limit")
    body = bytes((len(payload), int(op) & 0xFF, seq & 0xFF)) + payload
    return f"{FRAME_START}{(body + crc16(body).to_bytes(2, 'big')).hex().upper()}{FRAME_TERMINATOR}"


def decode_frame(text: str) -> Frame:
    """Decode one candidate frame body (the hex between ``#`` and the terminator)."""
    hex_body = text.strip()
    if not hex_body:
        raise TMC2209MotorTruncatedResponseError("empty frame body after the `#` marker")
    if len(hex_body) % 2:
        # A lost byte inside a hex stream shifts every following nibble; the odd
        # count catches it before the CRC even runs.
        raise TMC2209MotorTruncatedResponseError(f"frame has an odd number of hex digits ({len(hex_body)}): {text!r}")
    try:
        body = bytes.fromhex(hex_body)
    except ValueError as error:
        raise TMC2209MotorIntegrityError(f"frame contains a non-hex character: {text!r}") from error

    if len(body) < HEADER_SIZE + CRC_SIZE:
        raise TMC2209MotorTruncatedResponseError(f"frame of {len(body)} bytes is shorter than an empty one: {text!r}")

    declared = body[0]
    expected = HEADER_SIZE + declared + CRC_SIZE
    if len(body) != expected:
        raise TMC2209MotorTruncatedResponseError(
            f"frame declares {declared} payload bytes ({expected} total) but carries {len(body)}: {text!r}"
        )

    received_crc = int.from_bytes(body[-CRC_SIZE:], "big")
    computed_crc = crc16(body[:-CRC_SIZE])
    if received_crc != computed_crc:
        raise TMC2209MotorIntegrityError(
            f"frame CRC mismatch: got 0x{received_crc:04X}, computed 0x{computed_crc:04X} over {text!r}"
        )

    return Frame(op=body[1], seq=body[2], payload=body[HEADER_SIZE:-CRC_SIZE])


def decode_response(raw: str, op: int, seq: int) -> Frame:
    """Pick the frame answering ``op``/``seq`` out of whatever the read returned.

    A single read can hand back more than one frame's worth of bytes: the TX ring
    cuts a reply without its terminator and the next one is appended to the stump,
    and a reply that timed out earlier is still in the buffer. Candidates are
    therefore tried newest first, and a frame that decodes but answers something
    else is reported as stale rather than accepted.
    """
    if not raw.strip():
        raise TMC2209MotorTimeoutError("no response: controller sent nothing before the read timed out")
    if FRAME_START not in raw:
        cleaned = raw.strip()
        if cleaned[:2] in {"0;", "1;"}:
            raise TMC2209MotorLegacyResponseError(f"controller answered in the v2 line protocol: {raw!r}")
        raise TMC2209MotorProtocolError(f"no frame marker `{FRAME_START}` in the response: {raw!r}")

    candidates = [chunk for chunk in raw.split(FRAME_START)[1:] if chunk.strip()]
    failure: TMC2209MotorProtocolError = TMC2209MotorTruncatedResponseError(f"no frame body in the response: {raw!r}")
    stale: Frame | None = None
    for chunk in reversed(candidates):
        try:
            frame = decode_frame(chunk)
        except TMC2209MotorProtocolError as error:
            failure = error
            continue
        if frame.seq == (seq & 0xFF) and frame.op in (int(op), int(Op.ERROR)):
            return frame
        stale = frame

    if stale is not None:
        raise TMC2209MotorStaleResponseError(
            f"response answers op=0x{stale.op:02X} seq={stale.seq}, expected op=0x{int(op):02X} seq={seq & 0xFF}"
        )
    raise failure


# ---------------------------------------------------------------------------
# Requests: the driver keeps speaking in the line protocol's command names, and
# the mapping to opcodes lives here so that both dialects stay one lookup apart.
# ---------------------------------------------------------------------------


def build_request(command: str, args: list[str] | None = None) -> tuple[Op, bytes]:
    values = args or []
    if command == "status":
        return Op.STATUS, b""
    if command == "hello":
        return Op.HELLO, b""
    if command == "sensor":
        return Op.SENSOR, b""
    if command == "run":
        return Op.RUN, b""
    if command == "stop":
        return Op.STOP, b""
    if command in ("position", "speed", "acceleration", "delta"):
        op = {
            "position": Op.POSITION,
            "speed": Op.SPEED,
            "acceleration": Op.ACCELERATION,
            "delta": Op.DELTA,
        }[command]
        return op, int(values[0]).to_bytes(4, "big", signed=True)
    if command in ("direction", "enabled"):
        op = Op.DIRECTION if command == "direction" else Op.ENABLED
        return op, bytes((1 if int(values[0]) != 0 else 0,))
    if command == "mode":
        if values[0] not in MODE_NAMES:
            raise ValueError(f"unknown motion mode: {values[0]!r}")
        return Op.MODE, bytes((MODE_NAMES.index(values[0]),))
    if command == "set":
        name, separator, value = values[0].partition(KEY_VALUE_SEPARATOR)
        if name != "microsteps" or not separator:
            raise ValueError(f"the framed protocol carries no parameter {values[0]!r}")
        return Op.MICROSTEPS, int(value).to_bytes(2, "big")
    if command == "get":
        if values[0] != "microsteps":
            raise ValueError(f"the framed protocol carries no parameter {values[0]!r}")
        return Op.MICROSTEPS, b""
    raise ValueError(f"no framed opcode for command {command!r}")


# ---------------------------------------------------------------------------
# Replies: decoded into the same key-value shape the line protocol produced, so
# that everything above the codec (status parsing, echo checks, the hardware
# tests) is written once and works in either dialect.
# ---------------------------------------------------------------------------


def _expect(payload: bytes, size: int, op: int) -> None:
    if len(payload) != size:
        raise TMC2209MotorProtocolError(f"reply to op=0x{op:02X} carries {len(payload)} payload bytes, expected {size}")


def _i32(payload: bytes, offset: int) -> int:
    return int.from_bytes(payload[offset : offset + 4], "big", signed=True)


def _u32(payload: bytes, offset: int) -> int:
    return int.from_bytes(payload[offset : offset + 4], "big")


def _u16(payload: bytes, offset: int) -> int:
    return int.from_bytes(payload[offset : offset + 2], "big")


def _centi(value: int) -> str:
    return f"{value / 100:.2f}"


def response_values(frame: Frame) -> dict[str, str]:
    payload = frame.payload
    if frame.op == Op.ERROR:
        _expect(payload, 1, frame.op)
        return {"error": ERROR_NAMES.get(payload[0], f"error_{payload[0]}")}
    if frame.op == Op.HELLO:
        _expect(payload, 2, frame.op)
        return {"protocol": str(payload[0]), "firmware": str(payload[1])}
    if frame.op == Op.SENSOR:
        _expect(payload, 19, frame.op)
        flags = payload[0]
        if flags not in (0, 1, 3, 7):
            raise TMC2209MotorProtocolError("inconsistent sensor flags")
        values = {"sensor_flags": str(flags)}
        if flags & 2:
            values.update({"sample": str(_u32(payload, 1)), "age_ms": str(_u16(payload, 5))})
            for index, key in enumerate(("gx", "gy", "gz", "mx", "my", "mz")):
                if index < 3 or flags & 4:
                    values[key] = str(int.from_bytes(payload[7 + index * 2 : 9 + index * 2], "big", signed=True))
        return values
    if frame.op == Op.STATUS:
        if len(payload) not in (LEGACY_STATUS_PAYLOAD_SIZE, STATUS_PAYLOAD_SIZE):
            raise TMC2209MotorProtocolError(
                f"reply to op=0x{frame.op:02X} carries {len(payload)} payload bytes, "
                f"expected {LEGACY_STATUS_PAYLOAD_SIZE} or {STATUS_PAYLOAD_SIZE}"
            )
        flags = payload[0]
        phase = payload[1]
        if phase >= len(PHASE_NAMES):
            raise TMC2209MotorProtocolError(f"status reports an unknown phase code {phase}")
        values = {
            "initialised": "1" if flags & FLAG_INITIALISED else "0",
            "enabled": "1" if flags & FLAG_ENABLED else "0",
            "mode": MODE_NAMES[1] if flags & FLAG_FREE_RIDE else MODE_NAMES[0],
            "position": str(_i32(payload, 2)),
            "phase": PHASE_NAMES[phase],
            "target": str(_i32(payload, 6)),
            "target_set": "1" if flags & FLAG_TARGET_SET else "0",
            "speed": _centi(_u32(payload, 10)),
            "actual_speed": _centi(_u32(payload, 14)),
            "accel_per_s": _centi(_u32(payload, 18)),
            "power_v": _centi(_u16(payload, 22)),
            "tx_overflow": str(_u16(payload, 24)),
        }
        if len(payload) == STATUS_PAYLOAD_SIZE:
            safety = payload[27]
            if safety >= len(SAFETY_NAMES):
                raise TMC2209MotorProtocolError(f"status reports an unknown safety code {safety}")
            values.update(
                {
                    "drv_flags": str(payload[26]),
                    "safety": SAFETY_NAMES[safety],
                    "mres": str(_u16(payload, 28)),
                    "fine_mres": str(_u16(payload, 30)),
                    "limit": str(_u32(payload, 32)),
                    "events": str(_u16(payload, 36)),
                }
            )
        return values
    if frame.op == Op.POSITION:
        _expect(payload, 4, frame.op)
        return {"position": str(_i32(payload, 0))}
    if frame.op == Op.SPEED:
        _expect(payload, 4, frame.op)
        return {"speed": _centi(_u32(payload, 0))}
    if frame.op == Op.ACCELERATION:
        _expect(payload, 4, frame.op)
        return {"accel_per_s": _centi(_u32(payload, 0))}
    if frame.op == Op.DIRECTION:
        _expect(payload, 1, frame.op)
        return {"direction": str(payload[0])}
    if frame.op == Op.DELTA:
        _expect(payload, 9, frame.op)
        return {"delta": str(_i32(payload, 0)), "target": str(_i32(payload, 4)), "target_set": str(payload[8])}
    if frame.op == Op.MODE:
        _expect(payload, 1, frame.op)
        if payload[0] >= len(MODE_NAMES):
            raise TMC2209MotorProtocolError(f"reply reports an unknown mode code {payload[0]}")
        return {"mode": MODE_NAMES[payload[0]]}
    if frame.op == Op.ENABLED:
        _expect(payload, 1, frame.op)
        return {"enabled": str(payload[0])}
    if frame.op == Op.MICROSTEPS:
        _expect(payload, 2, frame.op)
        return {"microsteps": str(_u16(payload, 0))}
    if frame.op == Op.RUN:
        _expect(payload, 1, frame.op)
        return {"running": str(payload[0])}
    if frame.op == Op.STOP:
        _expect(payload, 1, frame.op)
        return {"stopping": str(payload[0])}
    raise TMC2209MotorProtocolError(f"reply carries an unknown opcode 0x{frame.op:02X}")


def encode_status_payload(
    *,
    initialised: bool,
    enabled: bool,
    free_ride: bool,
    target_set: bool,
    phase: str,
    position: int,
    target: int,
    speed_sps: float,
    actual_sps: float,
    accel_sps2: float,
    power_v: float,
    tx_overflow: int,
    driver_flags: int = 0,
    safety: str = "normal",
    active_microsteps: int = 16,
    fine_microsteps: int = 16,
    speed_limit_sps: int = 40000,
    safety_events: int = 0,
) -> bytes:
    """Pack the 38-byte status snapshot the controller sends.

    Lives next to :func:`response_values` on purpose: the layout is stated once and
    both directions are read off the same lines. The firmware is the third
    implementation of these 38 bytes and is pinned to them by a golden frame in
    ``src/tests/units/test_dec_framed_protocol.py``.
    """
    flags = 0
    if initialised:
        flags |= FLAG_INITIALISED
    if enabled:
        flags |= FLAG_ENABLED
    if free_ride:
        flags |= FLAG_FREE_RIDE
    if target_set:
        flags |= FLAG_TARGET_SET
    return (
        bytes((flags, PHASE_NAMES.index(phase)))
        + int(position).to_bytes(4, "big", signed=True)
        + int(target).to_bytes(4, "big", signed=True)
        + to_centi(speed_sps).to_bytes(4, "big")
        + to_centi(actual_sps).to_bytes(4, "big")
        + to_centi(accel_sps2).to_bytes(4, "big")
        + min(to_centi(power_v), 0xFFFF).to_bytes(2, "big")
        + min(int(tx_overflow), 0xFFFF).to_bytes(2, "big")
        + bytes((int(driver_flags) & 0xFF, SAFETY_NAMES.index(safety)))
        + int(active_microsteps).to_bytes(2, "big")
        + int(fine_microsteps).to_bytes(2, "big")
        + int(speed_limit_sps).to_bytes(4, "big")
        + min(int(safety_events), 0xFFFF).to_bytes(2, "big")
    )


def encode_sensor_payload(flags: int, sequence: int = 0, age_ms: int = 0, vectors: tuple[int, ...] = (0,) * 6) -> bytes:
    """Pack raw gravity (0.001 m/s²) and magnetic (0.01 µT) components."""
    if flags not in (0, 1, 3, 7) or len(vectors) != 6:
        raise ValueError("inconsistent sensor packet")
    return (
        bytes((flags,))
        + sequence.to_bytes(4, "big")
        + min(age_ms, 65535).to_bytes(2, "big")
        + b"".join(value.to_bytes(2, "big", signed=True) for value in vectors)
    )


def to_centi(value: float) -> int:
    # Round half up, matching `(uint32_t)(v * 100.0f + 0.5f)` in the firmware. All
    # four centi-fields (speed, actual speed, acceleration, supply voltage) are
    # non-negative, so the two expressions agree exactly.
    return max(0, int(value * 100 + 0.5))


@dataclasses.dataclass(frozen=True)
class Response:
    ok: bool
    values: dict[str, str]
    error: str | None

    @classmethod
    def from_frame(cls, frame: Frame) -> "Response":
        values = response_values(frame)
        ok = frame.op != Op.ERROR
        return cls(ok=ok, values=values, error=values.get("error"))

    @classmethod
    def from_line(cls, line: str) -> "Response":
        """Parse a v2 line-protocol reply: ``1;key=value;...`` / ``0;error=code;``.

        Kept for the board that is still in the field. Every check it can make is
        structural — without a length or a checksum, a value that lost a digit is
        indistinguishable from a value that never had it (DEC_PROTOCOL.md §6.1).
        """
        cleaned = line.strip()
        if not cleaned:
            raise TMC2209MotorTimeoutError("no response: controller sent nothing before the read timed out")
        tokens = [token for token in cleaned.split(RESPONSE_DELIMITER) if token]
        if not tokens or tokens[0] not in {"0", "1"}:
            raise TMC2209MotorProtocolError(f"unrecognised response, no `0;`/`1;` status prefix: {line!r}")
        # A bare status prefix inside the line means the previous response lost its
        # terminator and the next one was appended to it.
        if any(token in {"0", "1"} for token in tokens[1:]):
            raise TMC2209MotorConcatenatedResponseError(f"two responses glued into one line: {line!r}")
        if not cleaned.endswith(RESPONSE_DELIMITER):
            raise TMC2209MotorTruncatedResponseError(f"response cut off before its terminator: {line!r}")
        values: dict[str, str] = {}
        for token in tokens[1:]:
            if KEY_VALUE_SEPARATOR not in token:
                raise TMC2209MotorTruncatedResponseError(f"response cut off inside token {token!r}: {line!r}")
            key, value = token.split(KEY_VALUE_SEPARATOR, 1)
            if not key or value == "":
                raise TMC2209MotorProtocolError(f"invalid key-value token: {token!r}")
            values[key] = value
        ok = tokens[0] == "1"
        return cls(ok=ok, values=values, error=values.get("error"))
