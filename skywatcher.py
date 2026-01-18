from __future__ import annotations
import dataclasses
import logging
from typing import Optional

from serial_prims import SerialLineDevice


def encode_hex_le(value: int, nbytes: int) -> str:
    if value < 0:
        raise ValueError("encode_hex_le expects unsigned")
    b = value.to_bytes(nbytes, byteorder="little", signed=False)
    return b.hex().upper()


def decode_hex_le(hexstr: str, signed: bool = False) -> int:
    bs = bytes.fromhex(hexstr)
    val = int.from_bytes(bs, byteorder="little", signed=False)
    if signed:
        bits = 8 * len(bs)
        signbit = 1 << (bits - 1)
        if val & signbit:
            val = val - (1 << bits)
    return val


@dataclasses.dataclass
class SkyWatcherAxisInfo:
    cpr: int = 0
    timer_freq: int = 0
    last_pos: int = 0
    last_status: int = 0
    updated_monotonic: float = 0.0


class SkyWatcherMC:
    def __init__(self, dev: SerialLineDevice, status_interval: float = 0.0):
        self.dev = dev
        self.log = logging.getLogger("ra.swmc")
        self.status_interval = float(status_interval or 0.0)
        self._status_task = None

    def _cmd(self, header: str, channel: Optional[str], data_hex: str = "") -> str:
        if channel is None:
            wire = f":{header}{data_hex}\r".encode("ascii")
        else:
            wire = f":{header}{channel}{data_hex}\r".encode("ascii")
        self.log.debug("SWMC TX %r", wire)
        raw = self.dev.transact(wire, terminator=b"\r")
        self.log.debug("SWMC RX %r", raw)
        if not raw or len(raw) < 2:
            raise RuntimeError(f"bad response: {raw!r}")
        if raw[0:1] == b"!":
            err = raw[1:-1].decode("ascii", errors="replace")
            raise RuntimeError(f"SWMC error: {err!r} for cmd {wire!r}")
        if raw[0:1] != b"=":
            raise RuntimeError(f"bad response start: {raw!r}")
        return raw[1:-1].decode("ascii", errors="replace")

    async def start_status_loop(self, ch: str) -> None:
        if self.status_interval <= 0:
            return
        if self._status_task is not None:
            return
        import asyncio

        async def _loop():
            self.log.info("SkyWatcher status loop started (ch=%s interval=%.3fs)", ch, self.status_interval)
            try:
                while True:
                    try:
                        st = await asyncio.to_thread(self.inquire_status, ch)
                        self.log.debug("SkyWatcher status ch=%s -> %r", ch, st)
                    except Exception:
                        self.log.debug("SkyWatcher status inquiry failed", exc_info=True)
                    await asyncio.sleep(self.status_interval)
            except asyncio.CancelledError:
                self.log.info("SkyWatcher status loop cancelled")

        self._status_task = asyncio.create_task(_loop())

    async def stop_status_loop(self) -> None:
        if self._status_task:
            self._status_task.cancel()
            try:
                await self._status_task
            except Exception:
                pass
            self._status_task = None

    def inquire_cpr(self, ch: str) -> int:
        hexdata = self._cmd("a", ch)
        return decode_hex_le(hexdata, signed=False)

    def inquire_timer_freq(self) -> int:
        hexdata = self._cmd("b", None, data_hex="1")
        return decode_hex_le(hexdata, signed=False)

    def inquire_position(self, ch: str) -> int:
        hexdata = self._cmd("j", ch)
        return decode_hex_le(hexdata, signed=True)

    def inquire_status(self, ch: str) -> int:
        hexdata = self._cmd("f", ch)
        if len(hexdata) not in (2, 4):
            self.log.debug("status length unexpected: %r", hexdata)
        return decode_hex_le(hexdata, signed=False)

    def set_motion_mode(self, ch: str, *, tracking: bool, ccw: bool, fast: bool = False, medium: bool = False) -> None:
        db1 = 0
        db1 |= (1 if tracking else 0) << 0
        db1 |= (1 if fast else 0) << 1
        db1 |= (1 if medium else 0) << 2
        db2 = 0
        db2 |= (1 if ccw else 0) << 0
        mode = f"{db1:X}{db2:X}00"
        _ = self._cmd("G", ch, data_hex=mode)

    def set_goto_target(self, ch: str, target_pos: int) -> None:
        if target_pos < 0:
            target_pos &= (1 << 24) - 1
        hexdata = encode_hex_le(target_pos, 3)
        _ = self._cmd("S", ch, data_hex=hexdata)

    def set_step_period(self, ch: str, preset: int) -> None:
        hexdata = encode_hex_le(preset, 3)
        _ = self._cmd("I", ch, data_hex=hexdata)

    def start_motion(self, ch: str) -> None:
        _ = self._cmd("J", ch)

    def stop_motion(self, ch: str) -> None:
        _ = self._cmd("K", ch)

    def instant_stop(self, ch: str) -> None:
        _ = self._cmd("L", ch)
