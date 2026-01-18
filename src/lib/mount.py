from __future__ import annotations
import asyncio
import dataclasses
import datetime as dt
import logging
import time
from typing import Optional, Tuple, Any

from coords import lst_hours, fmt_ra, fmt_dec, clamp, wrap_hours
from skywatcher import SkyWatcherMC, SkyWatcherAxisInfo


@dataclasses.dataclass
class SiteTime:
    lat_deg: float
    lon_deg_east: float
    utc_offset_hours: float = 0.0
    local_datetime: Optional[dt.datetime] = None

    def now_utc(self) -> dt.datetime:
        if self.local_datetime is None:
            return dt.datetime.now(dt.timezone.utc)
        offset = dt.timedelta(hours=self.utc_offset_hours)
        tz = dt.timezone(offset)
        local = self.local_datetime.replace(tzinfo=tz)
        return local.astimezone(dt.timezone.utc)


@dataclasses.dataclass
class MountState:
    ra_hours: float = 0.0
    dec_deg: float = 0.0
    target_ra_hours: Optional[float] = None
    target_dec_deg: Optional[float] = None
    tracking: bool = True
    ra_cpr: int = 0
    ra_sign: int = +1
    ra_ha0_ticks: int = 0
    last_ra_ticks: int = 0
    last_dec_deg: float = 0.0


class FrankenMount:
    def __init__(
        self,
        *,
        ra: SkyWatcherMC,
        ra_ch: str,
        dec: Any,
        site: SiteTime,
        ra_ccw: bool,
        ra_sign: int,
        dec_accel_deg_s2: float,
        dec_vmax_deg_s: float,
        guide_rate_arcsec_s: float = 7.5,
        status_interval: float = 5.0,
    ):
        self.log = logging.getLogger("mount")
        self.ra = ra
        self.ra_ch = ra_ch
        self.dec = dec
        self.site = site
        self.ra_ccw = ra_ccw
        self.ra_sign = ra_sign
        self.axis = SkyWatcherAxisInfo()
        self.state = MountState()
        self._poll_task: Optional[asyncio.Task] = None
        self._goto_lock = asyncio.Lock()
        self._tracking_task: Optional[asyncio.Task] = None
        self.status_interval = float(status_interval or 0.0)
        self._status_task: Optional[asyncio.Task] = None

        self.dec_accel = dec_accel_deg_s2
        self.dec_vmax = dec_vmax_deg_s
        self.guide_rate_arcsec_s = guide_rate_arcsec_s

    def _sync_call(self, method: str, *args, **kwargs):
        fn = getattr(self.dec, method, None)
        if fn is None:
            return None
        return fn(*args, **kwargs)

    def _sync_call_first(self, methods: list[str], *args, **kwargs):
        for m in methods:
            fn = getattr(self.dec, m, None)
            if fn is not None:
                return fn(*args, **kwargs)
        return None

    async def start(self) -> None:
        await self._init_backends()
        self._poll_task = asyncio.create_task(self._poll_loop(), name="poll_loop")
        if self.status_interval and self.status_interval > 0:
            self._status_task = asyncio.create_task(self._status_loop(), name="status_loop")

    async def close(self) -> None:
        if self._poll_task:
            self._poll_task.cancel()
        if self._status_task:
            self._status_task.cancel()

    async def _init_backends(self) -> None:
        self.log.info("Init backends: reading RA CPR and timer freq, configuring DEC")
        cpr = await asyncio.to_thread(self.ra.inquire_cpr, self.ra_ch)
        tf = await asyncio.to_thread(self.ra.inquire_timer_freq)
        self.axis.cpr = cpr
        self.axis.timer_freq = tf
        self.state.ra_cpr = cpr
        self.log.info("RA: CPR=%d, TMR_Freq=%d Hz", cpr, tf)
        await asyncio.to_thread(self._sync_call, "set_accel", self.dec_accel)
        await asyncio.to_thread(self._sync_call, "set_max_rate", self.dec_vmax)
        ticks = await asyncio.to_thread(self.ra.inquire_position, self.ra_ch)
        self.state.last_ra_ticks = ticks
        now_utc = self.site.now_utc()
        lst = lst_hours(now_utc, self.site.lon_deg_east)
        ha = lst
        self.state.ra_ha0_ticks = ticks - self._ha_hours_to_ticks(ha)
        self.log.info("Initial HA0 ticks set (needs :CM sync for real sky).")

    def _ticks_to_ha_hours(self, ticks: int) -> float:
        if self.state.ra_cpr <= 0:
            return 0.0
        deg = (ticks * 360.0 / self.state.ra_cpr) * self.state.ra_sign
        return wrap_hours(deg / 15.0)

    def _ha_hours_to_ticks(self, ha_hours: float) -> int:
        if self.state.ra_cpr <= 0:
            return 0
        import math
        deg = wrap_hours(ha_hours * 15.0)
        ticks = (deg / 360.0) * self.state.ra_cpr
        return int(round(ticks)) * self.state.ra_sign

    def _compute_radec_from_axes(self, ra_ticks: int, dec_deg: float, t_utc) -> Tuple[float, float]:
        ha = self._ticks_to_ha_hours(ra_ticks - self.state.ra_ha0_ticks)
        lst = lst_hours(t_utc, self.site.lon_deg_east)
        ra = wrap_hours(lst - ha)
        return ra, dec_deg

    async def _poll_loop(self) -> None:
        self.log.info("Poll loop started.")
        while True:
            try:
                t_utc = self.site.now_utc()
                ra_ticks = await asyncio.to_thread(self.ra.inquire_position, self.ra_ch)
                self.axis.last_pos = ra_ticks
                self.axis.updated_monotonic = time.monotonic()
                self.state.last_ra_ticks = ra_ticks
                dec_deg = await asyncio.to_thread(self._sync_call, "get_dec")
                if dec_deg is None:
                    dec_deg = 0.0
                self.state.last_dec_deg = dec_deg
                ra_h, dec_d = self._compute_radec_from_axes(ra_ticks, dec_deg, t_utc)
                self.state.ra_hours = ra_h
                self.state.dec_deg = dec_d
                # update RA status if available
                try:
                    status = await asyncio.to_thread(self.ra.inquire_status, self.ra_ch)
                    self.axis.last_status = status
                except Exception:
                    pass
                await asyncio.sleep(0.2)
            except asyncio.CancelledError:
                return
            except Exception as e:
                self.log.warning("Poll error: %s", e, exc_info=True)
                await asyncio.sleep(1.0)

    def get_ra_dec(self) -> Tuple[float, float]:
        return self.state.ra_hours, self.state.dec_deg

    def set_target_ra(self, ra_hours: float) -> str:
        self.state.target_ra_hours = wrap_hours(ra_hours)
        return "1#"

    def set_target_dec(self, dec_deg: float) -> str:
        self.state.target_dec_deg = clamp(dec_deg, -90.0, 90.0)
        return "1#"

    async def sync_to_target(self) -> str:
        if self.state.target_ra_hours is None or self.state.target_dec_deg is None:
            return "0#"
        t_utc = self.site.now_utc()
        lst = lst_hours(t_utc, self.site.lon_deg_east)
        ha_target = wrap_hours(lst - self.state.target_ra_hours)
        ticks = self.state.last_ra_ticks
        self.state.ra_ha0_ticks = ticks - self._ha_hours_to_ticks(ha_target)
        self.log.info("SYNC: set HA0 ticks=%d using target RA=%s DEC=%.3f at LST=%.6fh",
                      self.state.ra_ha0_ticks, fmt_ra(self.state.target_ra_hours), self.state.target_dec_deg, lst)
        return "1#"

    async def abort(self) -> str:
        self.log.info("ABORT requested.")
        try:
            await asyncio.to_thread(self.ra.instant_stop, self.ra_ch)
        except Exception as e:
            self.log.warning("RA abort failed: %s", e)
        try:
            await asyncio.to_thread(self._sync_call_first, ["abort", "stop"]) 
        except Exception as e:
            self.log.warning("DEC abort failed: %s", e)
        return "1#"

    async def goto_target(self) -> str:
        if self.state.target_ra_hours is None or self.state.target_dec_deg is None:
            return "0#"
        async with self._goto_lock:
            ra_t = self.state.target_ra_hours
            dec_t = self.state.target_dec_deg
            t_utc = self.site.now_utc()
            lst = lst_hours(t_utc, self.site.lon_deg_east)
            ha_target = wrap_hours(lst - ra_t)
            ra_ticks_target = self.state.ra_ha0_ticks + self._ha_hours_to_ticks(ha_target)
            self.log.info("GOTO: target RA=%s DEC=%.3f -> HA=%.6fh -> RA ticks target=%d",
                          fmt_ra(ra_t), dec_t, ha_target, ra_ticks_target)
            try:
                await asyncio.to_thread(self._sync_call, "set_target_dec", dec_t)
                try:
                    await asyncio.to_thread(self._sync_call_first, ["set_target_ra", "set_ra"], ra_t)
                except Exception:
                    pass
                await asyncio.to_thread(self._sync_call_first, ["goto", "goto_target"]) 
            except Exception as e:
                self.log.warning("DEC goto failed: %s", e, exc_info=True)
            try:
                await asyncio.to_thread(self.ra.instant_stop, self.ra_ch)
                await asyncio.sleep(0.1)
                await asyncio.to_thread(self.ra.set_motion_mode, self.ra_ch, tracking=False, ccw=self.ra_ccw)
                await asyncio.to_thread(self.ra.set_goto_target, self.ra_ch, ra_ticks_target)
                await asyncio.to_thread(self.ra.start_motion, self.ra_ch)
            except Exception as e:
                self.log.warning("RA goto failed: %s", e, exc_info=True)
            await self._wait_slew_finish(ra_ticks_target, dec_t, timeout_s=180.0)
            await self.enable_tracking(True)
            return "0#"

    async def _wait_slew_finish(self, ra_target_ticks: int, dec_target_deg: float, timeout_s: float) -> None:
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            ra_ticks = self.state.last_ra_ticks
            dec_deg = self.state.last_dec_deg
            if self.state.ra_cpr > 0:
                deg_err = abs((ra_ticks - ra_target_ticks) * 360.0 / self.state.ra_cpr)
            else:
                deg_err = 999
            dec_err = abs(dec_deg - dec_target_deg)
            if deg_err < 0.05 and dec_err < 0.05:
                self.log.info("Slew complete: RA err=%.3f deg, DEC err=%.3f deg", deg_err, dec_err)
                return
            await asyncio.sleep(0.3)
        self.log.warning("Slew timeout after %.1fs", timeout_s)

    async def enable_tracking(self, enabled: bool) -> None:
        if self.axis.cpr <= 0 or self.axis.timer_freq <= 0:
            self.log.warning("Tracking not configured (missing CPR/TMR_Freq).")
            return
        speed_deg_s = 360.0 / 86164.0905
        counts_per_s = speed_deg_s * self.axis.cpr / 360.0
        preset = int(round(self.axis.timer_freq / counts_per_s))
        preset = int(clamp(preset, 1, (1 << 24) - 1))
        self.log.info("Tracking %s: sidereal preset=%d (TMR=%d, CPR=%d)",
                      "ON" if enabled else "OFF", preset, self.axis.timer_freq, self.axis.cpr)
        try:
            await asyncio.to_thread(self.ra.instant_stop, self.ra_ch)
            await asyncio.sleep(0.1)
            if enabled:
                # try setting the preferred preset, fall back to smaller presets if device rejects
                tried = set()
                presets = [preset, max(1, preset // 10), max(1, preset // 100), 1]
                ok = False
                for p in presets:
                    if p in tried:
                        continue
                    tried.add(p)
                    try:
                        await asyncio.to_thread(self.ra.set_step_period, self.ra_ch, p)
                        await asyncio.to_thread(self.ra.set_motion_mode, self.ra_ch, tracking=True, ccw=self.ra_ccw)
                        await asyncio.to_thread(self.ra.start_motion, self.ra_ch)
                        ok = True
                        self.log.info("Tracking ON using preset=%d", p)
                        break
                    except Exception as e:
                        self.log.debug("Tracking attempt with preset=%d failed: %s", p, e)
                if not ok:
                    self.log.warning("All tracking preset attempts failed; device rejected tracking command(s)")
            else:
                await asyncio.to_thread(self.ra.stop_motion, self.ra_ch)
        except Exception as e:
            self.log.warning("Tracking command failed: %s", e, exc_info=True)

    async def move(self, axis: str, direction: str, start: bool, rate_deg_s: float) -> str:
        if axis == "dec":
            # prefer device-specific move methods, fallback to generic 'move'
            if direction == "N":
                fn = getattr(self.dec, "move_ns", None) or getattr(self.dec, "move", None)
                if fn:
                    await asyncio.to_thread(fn, True, start)
            elif direction == "S":
                fn = getattr(self.dec, "move_ns", None) or getattr(self.dec, "move", None)
                if fn:
                    await asyncio.to_thread(fn, False, start)
            elif direction == "E":
                fn = getattr(self.dec, "move_we", None) or getattr(self.dec, "move", None)
                if fn:
                    await asyncio.to_thread(fn, True, start)
            elif direction == "W":
                fn = getattr(self.dec, "move_we", None) or getattr(self.dec, "move", None)
                if fn:
                    await asyncio.to_thread(fn, False, start)
            return "1#"
        if self.axis.cpr <= 0 or self.axis.timer_freq <= 0:
            return "0#"
        rate_deg_s = abs(rate_deg_s)
        if rate_deg_s < 1e-6:
            return "0#"
        counts_per_s = rate_deg_s * self.axis.cpr / 360.0
        preset = int(round(self.axis.timer_freq / counts_per_s))
        preset = int(clamp(preset, 1, (1 << 24) - 1))
        ccw = self.ra_ccw
        if direction == "E":
            ccw = not self.ra_ccw
        try:
            if start:
                await asyncio.to_thread(self.ra.instant_stop, self.ra_ch)
                await asyncio.sleep(0.05)
                await asyncio.to_thread(self.ra.set_motion_mode, self.ra_ch, tracking=True, ccw=ccw)
                await asyncio.to_thread(self.ra.set_step_period, self.ra_ch, preset)
                await asyncio.to_thread(self.ra.start_motion, self.ra_ch)
            else:
                await asyncio.to_thread(self.ra.stop_motion, self.ra_ch)
        except Exception as e:
            self.log.warning("RA manual move failed: %s", e, exc_info=True)
            return "0#"
        return "1#"

    async def get_ra_status(self) -> Optional[int]:
        """Query RA motor status (returns integer status or None)."""
        try:
            status = await asyncio.to_thread(self.ra.inquire_status, self.ra_ch)
            self.axis.last_status = status
            return status
        except Exception:
            return None

    def is_tracking_enabled(self) -> Optional[bool]:
        """Return tracked state if known (bit0==1), else None."""
        s = self.axis.last_status
        if s is None:
            return None
        return bool(s & 0x1)

    async def _status_loop(self) -> None:
        self.log.info("Status loop started (interval=%.3fs)", self.status_interval)
        try:
            while True:
                try:
                    status = await asyncio.to_thread(self.ra.inquire_status, self.ra_ch)
                    self.axis.last_status = status
                    self.log.debug("RA status update: %r", status)
                except asyncio.CancelledError:
                    raise
                except Exception:
                    self.log.debug("RA status poll failed", exc_info=True)
                await asyncio.sleep(self.status_interval)
        except asyncio.CancelledError:
            self.log.info("Status loop cancelled")
            return
