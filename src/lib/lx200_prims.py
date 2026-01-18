from __future__ import annotations
import asyncio
import datetime as dt
import logging
from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional, Any, Tuple

from coords import parse_dec_dms, parse_ra_hms, fmt_ra, fmt_dec, fmt_lx200_lat, fmt_lx200_lon

class LX200Cmd(Enum):
    GET_RA = "GR"
    GET_DEC = "GD"
    GET_DATE = "GC"
    GET_TIME = "GL"
    GET_LAT = "Gt"
    GET_LON = "Gg"
    SET_RA = "Sr"
    SET_DEC = "Sd"
    START_GOTO = "MS"
    SYNC = "CM"
    ABORT = "Q"
    SLEW_RATE_RG = "RG"
    SLEW_RATE_RC = "RC"
    SLEW_RATE_RM = "RM"
    SLEW_RATE_RS = "RS"
    MOVE_N_START = "Mn"
    MOVE_S_START = "Ms"
    MOVE_E_START = "Me"
    MOVE_W_START = "Mw"
    MOVE_N_STOP = "Qn"
    MOVE_S_STOP = "Qs"
    MOVE_E_STOP = "Qe"
    MOVE_W_STOP = "Qw"
    SET_ACCEL = "XAC"
    SET_VMAX = "XVM"


class LX200BackendABC(ABC):
    """Abstract LX200 backend interface used by `LX200Server`.

    Implementations may be synchronous or asynchronous; `LX200Server` will
    accept either return values or awaitables.
    """

    # attributes expected by server
    @property
    @abstractmethod
    def dec(self) -> Any:
        ...

    @property
    @abstractmethod
    def site(self) -> Any:
        ...

    # DEC configuration properties
    @property
    @abstractmethod
    def dec_accel(self) -> float:
        ...

    @dec_accel.setter
    @abstractmethod
    def dec_accel(self, v: float) -> None:
        ...

    @property
    @abstractmethod
    def dec_vmax(self) -> float:
        ...

    @dec_vmax.setter
    @abstractmethod
    def dec_vmax(self, v: float) -> None:
        ...

    # State/query methods
    @abstractmethod
    def get_ra_dec(self) -> Tuple[float, float]:
        ...

    @abstractmethod
    def set_target_ra(self, ra_h: float) -> Any:
        ...

    @abstractmethod
    def set_target_dec(self, dec_deg: float) -> Any:
        ...

    @abstractmethod
    def sync_to_target(self) -> Any:
        ...

    @abstractmethod
    def goto_target(self) -> Any:
        ...

    @abstractmethod
    def abort(self) -> Any:
        ...

    @abstractmethod
    def move(self, axis: str, direction: str, start: bool, rate_deg_s: float) -> Any:
        ...


class LX200Client(LX200BackendABC):
    """Client-side backend implementation that sends LX200 commands over a protocol.

    `protocol` must expose `transact(request: bytes, terminator: bytes) -> bytes`.
    """

    def __init__(self, protocol: Any, logger_name: str = "lx200.client"):
        self.protocol = protocol
        self.log = logging.getLogger(logger_name)
        self._dec_accel = 0.0
        self._dec_vmax = 0.0

    def _format_cmd(self, cmd: str, expect_hash: bool = True) -> tuple[bytes, bytes]:
        if not cmd.startswith(":"):
            cmd = ":" + cmd
        if expect_hash:
            if not cmd.endswith("#"):
                cmd = cmd + "#"
            term = b"#"
        else:
            if not cmd.endswith("\n"):
                cmd = cmd + "\n"
            term = b"\n"
        return cmd.encode("ascii"), term

    def _transact(self, request: bytes, terminator: bytes) -> bytes:
        return self.protocol.transact(request, terminator=terminator)

    def cmd(self, cmd: str, expect_hash: bool = True) -> str:
        req, term = self._format_cmd(cmd, expect_hash=expect_hash)
        self.log.debug("TX %r (term=%r)", req, term)
        raw = self._transact(req, terminator=term)
        self.log.debug("RX %r", raw)
        if expect_hash:
            if not raw.endswith(b"#"):
                raise RuntimeError(f"bad lx200 response {raw!r}")
            return raw[:-1].decode("ascii", errors="replace")
        return raw.decode("ascii", errors="replace")

    # properties
    @property
    def dec(self):
        return self

    @property
    def site(self):
        return None

    @property
    def dec_accel(self) -> float:
        return self._dec_accel

    @dec_accel.setter
    def dec_accel(self, v: float) -> None:
        self._dec_accel = v
        # push to device if supported
        try:
            self.set_accel(v)
        except Exception:
            pass

    @property
    def dec_vmax(self) -> float:
        return self._dec_vmax

    @dec_vmax.setter
    def dec_vmax(self, v: float) -> None:
        self._dec_vmax = v
        try:
            self.set_max_rate(v)
        except Exception:
            pass

    # Implement backend methods by translating to LX200 commands
    def get_ra_dec(self) -> Tuple[float, float]:
        ra = self.get_ra()
        dec = self.get_dec()
        return (ra if ra is not None else 0.0, dec)

    def get_ra(self) -> Optional[float]:
        try:
            resp = self.cmd(LX200Cmd.GET_RA.value)
            return parse_ra_hms(resp)
        except Exception:
            return None

    def get_dec(self) -> float:
        resp = self.cmd(LX200Cmd.GET_DEC.value)
        return parse_dec_dms(resp)

    def set_target_ra(self, ra_h: float) -> bool:
        return self.cmd(f"{LX200Cmd.SET_RA.value} {fmt_ra(ra_h)[:-1]}") in ("1", "0", "")

    def set_target_dec(self, dec_deg: float) -> bool:
        return self.cmd(f"{LX200Cmd.SET_DEC.value} {fmt_dec(dec_deg)[:-1]}") in ("1", "0", "")

    def sync_to_target(self):
        return self.cmd(LX200Cmd.SYNC.value)

    def goto_target(self):
        return self.cmd(LX200Cmd.START_GOTO.value)

    def abort(self):
        try:
            _ = self.cmd(LX200Cmd.ABORT.value)
        except Exception:
            pass

    def move(self, axis: str, direction: str, start: bool, rate_deg_s: float):
        # translate to simple LX200 move commands (no rate parameter supported)
        if axis == "dec":
            if direction == "N":
                return self.cmd(LX200Cmd.MOVE_N_START.value if start else LX200Cmd.MOVE_N_STOP.value)
            if direction == "S":
                return self.cmd(LX200Cmd.MOVE_S_START.value if start else LX200Cmd.MOVE_S_STOP.value)
        else:
            if direction == "E":
                return self.cmd(LX200Cmd.MOVE_E_START.value if start else LX200Cmd.MOVE_E_STOP.value)
            if direction == "W":
                return self.cmd(LX200Cmd.MOVE_W_START.value if start else LX200Cmd.MOVE_W_STOP.value)

    def set_accel(self, accel_deg_s2: float) -> bool:
        try:
            _ = self.cmd(f"{LX200Cmd.SET_ACCEL.value}{accel_deg_s2:+08.3f}")
            return True
        except Exception as e:
            self.log.debug("DEC accel extension not supported: %s", e)
            return False

    def set_max_rate(self, rate_deg_s: float) -> bool:
        try:
            _ = self.cmd(f"{LX200Cmd.SET_VMAX.value}{rate_deg_s:07.3f}")
            return True
        except Exception as e:
            self.log.debug("DEC vmax extension not supported: %s", e)
            return False


class LX200Server:
    """Server-side dispatcher: parse incoming LX200 commands and call backend mount APIs.

    `backend` is expected to implement methods used by the dispatcher (e.g. `get_ra_dec`,
    `set_target_ra`, `set_target_dec`, `goto_target`, `sync_to_target`, `abort`, `move`).
    """

    def __init__(self, backend: LX200BackendABC, logger_name: str = "lx200.server"):
        self.backend = backend
        self.log = logging.getLogger(logger_name)
        self.slew_rate = "C"
        self.rates = {"C": 4.0, "M": 1.0, "S": 0.2, "G": 0.02}

    async def _maybe_await(self, maybe_coro: Any) -> Any:
        if asyncio.iscoroutine(maybe_coro):
            return await maybe_coro
        return maybe_coro

    async def dispatch(self, cmd: str) -> Optional[str]:
        cmd = cmd.strip()
        if not (cmd.startswith(":") and cmd.endswith("#")):
            return None
        body = cmd[1:-1]
        if body == LX200Cmd.GET_RA.value:
            ra, _ = self.backend.get_ra_dec()
            return fmt_ra(ra)
        if body == LX200Cmd.GET_DEC.value:
            _, dec = self.backend.get_ra_dec()
            return fmt_dec(dec)
        if body == LX200Cmd.GET_DATE.value:
            return f"{self.backend.site.local_date_str()}#" if hasattr(self.backend.site, "local_date_str") else "#"
        if body == LX200Cmd.GET_TIME.value:
            return f"{self.backend.site.local_time_str()}#" if hasattr(self.backend.site, "local_time_str") else "#"
        if body == LX200Cmd.GET_LAT.value:
            return fmt_lx200_lat(self.backend.site.lat_deg)
        if body == LX200Cmd.GET_LON.value:
            return fmt_lx200_lon(self.backend.site.lon_deg_east)
        if body.startswith(f"{LX200Cmd.SET_RA.value} "):
            try:
                ra = parse_ra_hms(body[3:])
                ok = await self._maybe_await(self.backend.set_target_ra(ra))
                return ("1#" if ok else "0#")
            except Exception:
                return "0#"
        if body.startswith(f"{LX200Cmd.SET_DEC.value} "):
            try:
                dec = parse_dec_dms(body[3:])
                ok = await self._maybe_await(self.backend.set_target_dec(dec))
                return ("1#" if ok else "0#")
            except Exception:
                return "0#"
        if body == LX200Cmd.START_GOTO.value:
            res = await self._maybe_await(self.backend.goto_target())
            return f"{res}#" if isinstance(res, str) else "1#"
        if body == LX200Cmd.SYNC.value:
            res = await self._maybe_await(self.backend.sync_to_target())
            return f"{res}#" if isinstance(res, str) else "1#"
        if body == LX200Cmd.ABORT.value:
            await self._maybe_await(self.backend.abort())
            return "#"
        if body in (LX200Cmd.SLEW_RATE_RG.value, LX200Cmd.SLEW_RATE_RC.value,
                    LX200Cmd.SLEW_RATE_RM.value, LX200Cmd.SLEW_RATE_RS.value):
            self.slew_rate = body[1]
            return "1#"
        if body in (
            LX200Cmd.MOVE_N_START.value,
            LX200Cmd.MOVE_S_START.value,
            LX200Cmd.MOVE_E_START.value,
            LX200Cmd.MOVE_W_START.value,
        ):
            rate = self.rates.get(self.slew_rate, 0.2)
            if body in (LX200Cmd.MOVE_N_START.value, LX200Cmd.MOVE_S_START.value):
                direction = "N" if body == LX200Cmd.MOVE_N_START.value else "S"
                await self._maybe_await(self.backend.move("dec", direction, True, rate))
                return "1#"
            else:
                direction = "E" if body == LX200Cmd.MOVE_E_START.value else "W"
                await self._maybe_await(self.backend.move("ra", direction, True, rate))
                return "1#"
        if body in (LX200Cmd.MOVE_N_STOP.value, LX200Cmd.MOVE_S_STOP.value,
                    LX200Cmd.MOVE_E_STOP.value, LX200Cmd.MOVE_W_STOP.value):
            rate = self.rates.get(self.slew_rate, 0.2)
            if body in (LX200Cmd.MOVE_N_STOP.value, LX200Cmd.MOVE_S_STOP.value):
                direction = "N" if body == LX200Cmd.MOVE_N_STOP.value else "S"
                await self._maybe_await(self.backend.move("dec", direction, False, rate))
                return "1#"
            else:
                direction = "E" if body == LX200Cmd.MOVE_E_STOP.value else "W"
                await self._maybe_await(self.backend.move("ra", direction, False, rate))
                return "1#"
        if body.startswith(LX200Cmd.SET_ACCEL.value):
            try:
                accel = float(body[3:])
                self.backend.dec_accel = accel
                await asyncio.to_thread(self.backend.dec.set_accel, accel)
                return "1#"
            except Exception:
                return "0#"
        if body.startswith(LX200Cmd.SET_VMAX.value):
            try:
                vmax = float(body[3:])
                self.backend.dec_vmax = vmax
                await asyncio.to_thread(self.backend.dec.set_max_rate, vmax)
                return "1#"
            except Exception:
                return "0#"
        return "#"


class LX200Backend(LX200BackendABC):
    """Adapter exposing LX200-backend methods backed by a `FrankenMount`-like object.

    This class provides the attributes and methods the `LX200Server` dispatcher expects
    so a mount implementation can be used directly as the LX200 backend.
    """

    def __init__(self, mount: Any):
        self._mount = mount
        # expose commonly accessed attributes via properties
        self._dec = mount.dec
        self._site = mount.site

    @property
    def dec(self) -> Any:
        return self._dec

    @property
    def site(self) -> Any:
        return self._site

    # Properties forwarded to allow the server to configure DEC limits
    @property
    def dec_accel(self) -> float:
        return getattr(self._mount, "dec_accel", 0.0)

    @dec_accel.setter
    def dec_accel(self, v: float) -> None:
        setattr(self._mount, "dec_accel", v)

    @property
    def dec_vmax(self) -> float:
        return getattr(self._mount, "dec_vmax", 0.0)

    @dec_vmax.setter
    def dec_vmax(self, v: float) -> None:
        setattr(self._mount, "dec_vmax", v)

    # Basic state queries
    def get_ra_dec(self):
        return self._mount.get_ra_dec()

    # Target setters (can be sync)
    def set_target_ra(self, ra_h: float):
        return self._mount.set_target_ra(ra_h)

    def set_target_dec(self, dec_deg: float):
        return self._mount.set_target_dec(dec_deg)

    # Async operations
    async def sync_to_target(self):
        return await self._mount.sync_to_target()

    async def goto_target(self):
        return await self._mount.goto_target()

    async def abort(self):
        return await self._mount.abort()

    async def move(self, axis: str, direction: str, start: bool, rate_deg_s: float):
        return await self._mount.move(axis, direction, start, rate_deg_s)

    # Local time/date formatting used by LX200 responses
    def local_date_str(self) -> str:
        utc = self.site.now_utc()
        offset = dt.timedelta(hours=self.site.utc_offset_hours)
        local = utc.astimezone(dt.timezone(offset))
        return f"{local.month:02d}/{local.day:02d}/{local.year%100:02d}"

    def local_time_str(self) -> str:
        utc = self.site.now_utc()
        offset = dt.timedelta(hours=self.site.utc_offset_hours)
        local = utc.astimezone(dt.timezone(offset))
        return f"{local.hour:02d}:{local.minute:02d}:{local.second:02d}"
