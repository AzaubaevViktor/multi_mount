from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


FRAME_PREFIX = b":"
FRAME_TERMINATOR = b"\r"
SUCCESS_PREFIX = b"="
ERROR_PREFIX = b"!"
AXIS_RA = "1"


class SkyWatcherCommand(str, Enum):
    INITIALIZE = "F"
    INQUIRE_GRID_PER_REVOLUTION = "a"
    INQUIRE_TIMER_FREQUENCY = "b"
    INQUIRE_STATUS = "f"
    INQUIRE_HIGHSPEED_RATIO = "g"
    INQUIRE_POSITION = "j"
    SET_STEP_PERIOD = "I"
    SET_GOTO_TARGET_INCREMENT = "H"
    SET_BREAK_POINT_INCREMENT = "M"
    SET_AXIS_POSITION = "E"
    SET_MOTION_MODE = "G"
    START_MOTION = "J"
    STOP_MOTION = "K"


@dataclass(frozen=True, slots=True)
class SkyWatcherResponse:
    ok: bool
    payload: str
    raw: str


@dataclass(frozen=True, slots=True)
class SkyWatcherStatusBits:
    raw: int
    running: bool
    high_speed: bool
    direction_positive: bool
    target_mode: bool


def encode_u24(value: int) -> str:
    if not 0 <= value < (1 << 24):
        raise ValueError(f"Value does not fit into unsigned 24-bit field: {value}")
    return f"{value:06X}"


def encode_s24(value: int) -> str:
    if not -(1 << 23) <= value < (1 << 23):
        raise ValueError(f"Value does not fit into signed 24-bit field: {value}")
    if value < 0:
        value = (1 << 24) + value
    return f"{value:06X}"


def decode_u24(payload: str) -> int:
    return int(payload, 16)


def decode_s24(payload: str) -> int:
    value = int(payload, 16)
    if value >= (1 << 23):
        value -= 1 << 24
    return value


def build_command(command: SkyWatcherCommand, axis: str = AXIS_RA, payload: str = "") -> bytes:
    return f":{command.value}{axis}{payload}\r".encode("ascii")


def parse_response(data: bytes | str) -> SkyWatcherResponse:
    text = data.decode("ascii") if isinstance(data, bytes) else data
    if not text.endswith("\r"):
        raise ValueError(f"Invalid SkyWatcher response terminator: {text!r}")
    prefix = text[:1]
    payload = text[1:-1]
    if prefix == "=":
        return SkyWatcherResponse(ok=True, payload=payload, raw=text)
    if prefix == "!":
        return SkyWatcherResponse(ok=False, payload=payload, raw=text)
    raise ValueError(f"Invalid SkyWatcher response prefix: {text!r}")


def parse_status_bits(payload: str) -> SkyWatcherStatusBits:
    raw = int(payload, 16)
    return SkyWatcherStatusBits(
        raw=raw,
        running=bool(raw & 0x01),
        high_speed=bool(raw & 0x02),
        direction_positive=bool(raw & 0x04),
        target_mode=bool(raw & 0x08),
    )
