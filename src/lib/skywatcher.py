from __future__ import annotations
import dataclasses
import logging
from enum import IntEnum
from typing import Optional

from serial_prims import SerialLineDevice


@dataclasses.dataclass
class SkyWatcherAxisInfo:
    cpr: int = 0
    timer_freq: int = 0
    last_pos: int = 0
    updated_monotonic: float = 0.0
    last_status: Optional["SkyWatcherStatus"] = None


class SkyWatcherAxis(IntEnum):
    RA = 0
    DEC = 1

    @classmethod
    def from_channel(cls, value: int | str) -> "SkyWatcherAxis":
        if isinstance(value, str):
            value = value.strip()
        if value in (1, "1"):
            return cls.RA
        if value in (2, "2"):
            return cls.DEC
        raise ValueError(f"invalid axis channel {value!r}, expected 1 or 2")

    def to_bytes(self) -> bytes:
        if self.value not in (0, 1):
            raise ValueError(f"invalid axis {self.value!r}, expected 0 or 1")
        return str(self.value + 1).encode("ascii")

class SkyWatcherDirection(IntEnum):
    BACKWARD = 0
    FORWARD = 1


class SkyWatcherSlewMode(IntEnum):
    SLEW = 0
    GOTO = 1


class SkyWatcherSpeedMode(IntEnum):
    LOWSPEED = 0
    HIGHSPEED = 1


@dataclasses.dataclass(frozen=True)
class SkyWatcherStatus:
    raw: int
    running: bool
    initialized: bool
    slew_mode: SkyWatcherSlewMode
    direction: SkyWatcherDirection
    speed_mode: SkyWatcherSpeedMode

    @classmethod
    def from_bytes(cls, data: bytes) -> "SkyWatcherStatus":
        b1 = data[0] if len(data) > 0 else 0
        b2 = data[1] if len(data) > 1 else 0
        b3 = data[2] if len(data) > 2 else 0
        raw = b2 | (b1 << 8) | (b3 << 16)
        running = bool(b2 & 0x01)
        initialized = bool(b3 & 0x01)
        slew_mode = SkyWatcherSlewMode.SLEW if (b1 & 0x01) else SkyWatcherSlewMode.GOTO
        direction = SkyWatcherDirection.BACKWARD if (b1 & 0x02) else SkyWatcherDirection.FORWARD
        speed_mode = SkyWatcherSpeedMode.HIGHSPEED if (b1 & 0x04) else SkyWatcherSpeedMode.LOWSPEED
        return cls(
            raw=raw,
            running=running,
            initialized=initialized,
            slew_mode=slew_mode,
            direction=direction,
            speed_mode=speed_mode,
        )


@dataclasses.dataclass(frozen=True)
class SkyWatcherMotionMode:
    slew_mode: SkyWatcherSlewMode
    direction: SkyWatcherDirection
    speed_mode: SkyWatcherSpeedMode

    def to_command(self) -> str:
        if self.slew_mode == SkyWatcherSlewMode.SLEW and self.speed_mode == SkyWatcherSpeedMode.LOWSPEED:
            mode = "1"
        elif self.slew_mode == SkyWatcherSlewMode.SLEW and self.speed_mode == SkyWatcherSpeedMode.HIGHSPEED:
            mode = "3"
        elif self.slew_mode == SkyWatcherSlewMode.GOTO and self.speed_mode == SkyWatcherSpeedMode.LOWSPEED:
            mode = "2"
        else:
            mode = "0"
        direction = "1" if self.direction == SkyWatcherDirection.BACKWARD else "0"
        return f"{mode}{direction}"


class SkyWatcherMC:
    """SkyWatcher motor controller protocol wrapper."""

    _LEADING = b":"
    _TRAILING = b"\r"

    def __init__(self, dev: SerialLineDevice, logger: Optional[logging.Logger] = None) -> None:
        self.dev = dev
        self.log = logger or logging.getLogger("skywatcher.mc")

    def inquire_timer_freq(self, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> int:
        data = self._transact("b", axis)
        return self._revu24_to_int(data)

    def inquire_cpr(self, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> int:
        data = self._transact("a", axis)
        return self._revu24_to_int(data)

    def inquire_position(self, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> int:
        data = self._transact("j", axis)
        return self._revu24_to_int(data)

    def inquire_status(self, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> SkyWatcherStatus:
        data = self._transact("f", axis)
        return SkyWatcherStatus.from_bytes(data)

    def set_step_period(self, axis: SkyWatcherAxis, period: int) -> None:
        arg = self._int_to_revu24(period)
        self._transact("I", axis, arg)

    def set_goto_target(self, axis: SkyWatcherAxis, target: int) -> None:
        arg = self._int_to_revu24(target)
        self._transact("S", axis, arg)

    def set_motion_mode(self, axis: SkyWatcherAxis, mode: SkyWatcherMotionMode) -> None:
        self._transact("G", axis, mode.to_command())

    def start_motion(self, axis: SkyWatcherAxis) -> None:
        self._transact("J", axis)

    def stop_motion(self, axis: SkyWatcherAxis) -> None:
        self._transact("K", axis)

    def instant_stop(self, axis: SkyWatcherAxis) -> None:
        self._transact("L", axis)

    def _transact(self, cmd: str, axis: SkyWatcherAxis, arg: Optional[str] = None) -> bytes:
        axis_char = self._normalize_axis(axis)
        if arg is None:
            payload = self._LEADING + cmd.encode("ascii") + axis_char + self._TRAILING
        else:
            payload = self._LEADING + cmd.encode("ascii") + axis_char + arg.encode("ascii") + self._TRAILING
        resp = self.dev.transact(payload, terminator=self._TRAILING)
        if not resp:
            raise RuntimeError(f"empty response for cmd={cmd} axis={axis}")
        if resp.endswith(self._TRAILING):
            resp = resp[:-1]
        if not resp:
            raise RuntimeError(f"empty response for cmd={cmd} axis={axis}")
        if resp[:1] == b"!":
            raise RuntimeError(f"skywatcher command error: {resp!r}")
        if resp[:1] != b"=":
            raise RuntimeError(f"invalid response: {resp!r}")
        return resp[1:]

    def _normalize_axis(self, axis: SkyWatcherAxis) -> bytes:
        if not isinstance(axis, SkyWatcherAxis):
            raise TypeError(f"axis must be SkyWatcherAxis, got {type(axis)!r}")
        return axis.to_bytes()

    def _revu24_to_int(self, data: bytes) -> int:
        if len(data) < 6:
            raise ValueError(f"revu24 data too short: {data!r}")
        def hex_val(b: int) -> int:
            if 48 <= b <= 57:
                return b - 48
            if 65 <= b <= 70:
                return b - 55
            if 97 <= b <= 102:
                return b - 87
            raise ValueError(f"invalid hex digit: {b!r}")
        res = hex_val(data[4])
        res = (res << 4) | hex_val(data[5])
        res = (res << 4) | hex_val(data[2])
        res = (res << 4) | hex_val(data[3])
        res = (res << 4) | hex_val(data[0])
        res = (res << 4) | hex_val(data[1])
        return res

    def _int_to_revu24(self, value: int) -> str:
        n = int(value) & 0xFFFFFF
        hexa = "0123456789ABCDEF"
        return "".join(
            [
                hexa[(n & 0xF0) >> 4],
                hexa[(n & 0x0F)],
                hexa[(n & 0xF000) >> 12],
                hexa[(n & 0x0F00) >> 8],
                hexa[(n & 0xF00000) >> 20],
                hexa[(n & 0x0F0000) >> 16],
            ]
        )
