from __future__ import annotations

from typing import Any, Tuple
import datetime as dt

from lx200_prims import LX200BackendABC


class DummyDEC(LX200BackendABC):
    """Lightweight DEC backend used when no Arduino is attached.

    Implements both the high-level LX200 backend interface and the
    device-specific methods callers expect (set_accel, set_max_rate,
    get_dec, move_ns/move_we, goto, abort, ...).
    """

    def __init__(self, site: Any = None) -> None:
        self._site = site
        self._dec_accel = 0.0
        self._dec_vmax = 0.0

    # minimal attributes expected by server/adapter
    @property
    def dec(self) -> Any:
        return self

    @property
    def site(self) -> Any:
        return self._site

    # DEC configuration
    @property
    def dec_accel(self) -> float:
        return self._dec_accel

    @dec_accel.setter
    def dec_accel(self, v: float) -> None:
        self._dec_accel = float(v)

    @property
    def dec_vmax(self) -> float:
        return self._dec_vmax

    @dec_vmax.setter
    def dec_vmax(self, v: float) -> None:
        self._dec_vmax = float(v)

    # high-level backend API
    def get_ra_dec(self) -> Tuple[float, float]:
        return 0.0, 0.0

    def set_target_ra(self, ra_h: float) -> bool:
        return True

    def set_target_dec(self, dec_deg: float) -> bool:
        return True

    async def sync_to_target(self):
        return "1#"

    async def goto_target(self):
        return "0#"

    async def abort(self):
        return None

    async def move(self, axis: str, direction: str, start: bool, rate_deg_s: float):
        return None

    # device-style helpers used by mount
    def set_accel(self, v: float) -> bool:
        self._dec_accel = float(v)
        return True

    def set_max_rate(self, v: float) -> bool:
        self._dec_vmax = float(v)
        return True

    def get_dec(self) -> float:
        return 0.0

    def goto(self) -> str:
        return "0#"

    def abort(self) -> None:
        return None

    def move_ns(self, north: bool, start: bool) -> None:
        return None

    def move_we(self, east: bool, start: bool) -> None:
        return None

    # optional helpers for LX200Server formatting
    def local_date_str(self) -> str:
        if self._site is None:
            now = dt.datetime.utcnow()
            return f"{now.month:02d}/{now.day:02d}/{now.year%100:02d}"
        return self._site.local_date_str() if hasattr(self._site, "local_date_str") else ""

    def local_time_str(self) -> str:
        if self._site is None:
            now = dt.datetime.utcnow()
            return f"{now.hour:02d}:{now.minute:02d}:{now.second:02d}"
        return self._site.local_time_str() if hasattr(self._site, "local_time_str") else ""
