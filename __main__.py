from __future__ import annotations

import argparse
import asyncio
import logging
import os
import sys

from serial_prims import SerialLineDevice
from skywatcher import SkyWatcherMC
from arduino_dec import LX200SerialClient
from mount import FrankenMount, SiteTime
from lx200_interface import LX200TCPServer
from coords import parse_ra_hms, parse_dec_dms, fmt_ra, fmt_dec


def setup_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s.%(msecs)03d %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


async def console_cli(mount: FrankenMount) -> None:
    print("Test console started. Type 'help' for commands.")
    while True:
        try:
            line = await asyncio.to_thread(input, "console> ")
        except (EOFError, KeyboardInterrupt):
            print("Exiting console.")
            os._exit(0)
        cmd = line.strip()
        if not cmd:
            continue
        if cmd in ("help", "h"):
            print("pos, stop, setra HH:MM:SS, setdec +DD*MM[:SS], goto, sidereal on|off, exit")
            continue
        if cmd == "pos":
            ra, dec = mount.get_ra_dec()
            print(f"RA={fmt_ra(ra)[:-1]} DEC={fmt_dec(dec)[:-1]}")
            continue
        if cmd == "stop":
            await mount.abort()
            print("Abort sent.")
            continue
        if cmd.startswith("setra "):
            try:
                ra = parse_ra_hms(cmd[6:])
                mount.set_target_ra(ra)
                print(f"Target RA set to {fmt_ra(ra)[:-1]}")
            except Exception as e:
                print(f"Bad RA: {e}")
            continue
        if cmd.startswith("setdec "):
            try:
                dec = parse_dec_dms(cmd[7:])
                mount.set_target_dec(dec)
                print(f"Target DEC set to {fmt_dec(dec)[:-1]}")
            except Exception as e:
                print(f"Bad DEC: {e}")
            continue
        if cmd == "goto":
            print("Starting GOTO...")
            res = await mount.goto_target()
            print(f"GOTO result: {res}")
            continue
        if cmd.startswith("sidereal "):
            a = cmd.split()
            if len(a) >= 2 and a[1] in ("on", "off"):
                await mount.enable_tracking(a[1] == "on")
                print(f"Sidereal tracking {'enabled' if a[1]=='on' else 'disabled'}.")
            else:
                print("Usage: sidereal on|off")
            continue
        if cmd in ("exit", "quit"):
            print("Exiting process.")
            os._exit(0)
        print("Unknown command. Type 'help'.")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", default="127.0.0.1:10001", help="host:port for LX200 TCP server")
    ap.add_argument("--log-level", default="INFO")

    ap.add_argument("--ra-port", required=True)
    ap.add_argument("--ra-baud", type=int, default=9600)
    ap.add_argument("--ra-timeout", type=float, default=0.5)
    ap.add_argument("--ra-channel", default="1", choices=["1", "2"]) 
    ap.add_argument("--ra-ccw", action="store_true")
    ap.add_argument("--ra-sign", type=int, default=+1, choices=[-1, +1])

    ap.add_argument("--dec-port", required=True)
    ap.add_argument("--dec-baud", type=int, default=115200)
    ap.add_argument("--dec-timeout", type=float, default=0.5)

    ap.add_argument("--site-lat", type=float, default=0.0)
    ap.add_argument("--site-lon", type=float, default=0.0)
    ap.add_argument("--utc-offset", type=float, default=0.0)
    ap.add_argument("--dec-accel", type=float, default=5.0)
    ap.add_argument("--dec-vmax", type=float, default=4.0)
    ap.add_argument("--test-mode", action="store_true", help="Run interactive test console for mount control")

    args = ap.parse_args()
    setup_logging(args.log_level)

    host, port_s = args.listen.split(":")
    port = int(port_s)

    ra_dev = SerialLineDevice(args.ra_port, args.ra_baud, args.ra_timeout, name="serial.ra")
    dec_dev = SerialLineDevice(args.dec_port, args.dec_baud, args.dec_timeout, name="serial.dec")

    ra = SkyWatcherMC(ra_dev)
    dec = LX200SerialClient(dec_dev)
    site = SiteTime(lat_deg=args.site_lat, lon_deg_east=args.site_lon, utc_offset_hours=args.utc_offset)

    mount = FrankenMount(
        ra=ra,
        ra_ch=args.ra_channel,
        dec=dec,
        site=site,
        ra_ccw=args.ra_ccw,
        ra_sign=args.ra_sign,
        dec_accel_deg_s2=args.dec_accel,
        dec_vmax_deg_s=args.dec_vmax,
    )

    server = LX200TCPServer(mount)

    async def runner():
        await mount.start()
        if args.test_mode:
            asyncio.create_task(console_cli(mount))
        srv = await asyncio.start_server(server.handle_client, host, port)
        addrs = ", ".join(str(sock.getsockname()) for sock in srv.sockets or [])
        logging.getLogger("main").info("LX200 TCP listening on %s", addrs)
        async with srv:
            await srv.serve_forever()

    try:
        asyncio.run(runner())
    finally:
        try:
            ra_dev.close()
        except Exception:
            pass
        try:
            dec_dev.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()

    def get_dec(self) -> float:
        resp = self.cmd(":GD#")
        return parse_dec_dms(resp)

    def get_ra(self) -> Optional[float]:
        # optional: if your Arduino returns it
        try:
            resp = self.cmd(":GR#")
            return parse_ra_hms(resp)
        except Exception:
            return None

    def set_target_ra(self, ra_h: float) -> bool:
        return self.cmd(f":Sr {fmt_ra(ra_h)[:-1]}#") in ("1", "0", "")

    def set_target_dec(self, dec_deg: float) -> bool:
        return self.cmd(f":Sd {fmt_dec(dec_deg)[:-1]}#") in ("1", "0", "")

    def goto(self) -> str:
        # many controllers return "0" for success; some return empty string.
        return self.cmd(":MS#")

    def abort(self) -> None:
        try:
            _ = self.cmd(":Q#")
        except Exception:
            pass

    def move_ns(self, north: bool, start: bool) -> None:
        if start:
            _ = self.cmd(":Mn#" if north else ":Ms#")
        else:
            _ = self.cmd(":Qn#" if north else ":Qs#")

    def move_we(self, east: bool, start: bool) -> None:
        if start:
            _ = self.cmd(":Me#" if east else ":Mw#")
        else:
            _ = self.cmd(":Qe#" if east else ":Qw#")

    def set_accel(self, accel_deg_s2: float) -> None:
        # Non-standard extension. Implement on Arduino side if you want runtime tuning.
        # Example encoding: :XAC+001.23# (degrees/sec^2)
        try:
            _ = self.cmd(f":XAC{accel_deg_s2:+08.3f}#")
        except Exception as e:
            self.log.debug("DEC accel extension not supported: %s", e)

    def set_max_rate(self, rate_deg_s: float) -> None:
        # Non-standard extension.
        try:
            _ = self.cmd(f":XVM{rate_deg_s:07.3f}#")
        except Exception as e:
            self.log.debug("DEC vmax extension not supported: %s", e)


# -----------------------------
# Coordinator / Mount Model
# -----------------------------

@dataclasses.dataclass
class SiteTime:
    lat_deg: float
    lon_deg_east: float
    utc_offset_hours: float = 0.0  # used for parsing :SL/:SC (local time/date)
    # last set local date/time; if None => use system clock
    local_datetime: Optional[dt.datetime] = None  # naive local

    def now_utc(self) -> dt.datetime:
        if self.local_datetime is None:
            return dt.datetime.now(dt.timezone.utc)
        # interpret local_datetime with utc_offset_hours
        offset = dt.timedelta(hours=self.utc_offset_hours)
        tz = dt.timezone(offset)
        local = self.local_datetime.replace(tzinfo=tz)
        return local.astimezone(dt.timezone.utc)

@dataclasses.dataclass
class MountState:
    # Sky coordinates reported outward
    ra_hours: float = 0.0
    dec_deg: float = 0.0
    # targets set by :Sr/:Sd
    target_ra_hours: Optional[float] = None
    target_dec_deg: Optional[float] = None
    # tracking
    tracking: bool = True
    # sync/alignment: hour-angle zero point, stored as ticks offset
    ra_cpr: int = 0
    ra_sign: int = +1         # +1: ticks increase -> HA increases; -1 flips
    ra_ha0_ticks: int = 0     # ticks corresponding to HA=0 at current epoch
    last_ra_ticks: int = 0
    last_dec_deg: float = 0.0

class FrankenMount:
    def __init__(
        self,
        *,
        ra: SkyWatcherMC,
        ra_ch: str,
        dec: LX200SerialClient,
        site: SiteTime,
        ra_ccw: bool,
        ra_sign: int,
        dec_accel_deg_s2: float,
        dec_vmax_deg_s: float,
        guide_rate_arcsec_s: float = 7.5,  # typical guiding correction magnitude (arcsec/s)
    ):
        self.log = logging.getLogger("mount")
        self.ra = ra
        self.ra_ch = ra_ch
        self.dec = dec
        self.site = site
        self.ra_ccw = ra_ccw
        self.state = MountState(ra_sign=ra_sign)
        self.axis = SkyWatcherAxisInfo()
        self._goto_lock = asyncio.Lock()
        self._poll_task: Optional[asyncio.Task] = None
        self._tracking_task: Optional[asyncio.Task] = None

        self.dec_accel = dec_accel_deg_s2
        self.dec_vmax = dec_vmax_deg_s
        self.guide_rate_arcsec_s = guide_rate_arcsec_s

    async def start(self) -> None:
        # init backends: read RA CPR and timer freq; set DEC accel/vmax
        await self._init_backends()
        self._poll_task = asyncio.create_task(self._poll_loop(), name="poll_loop")

    async def close(self) -> None:
        if self._poll_task:
            self._poll_task.cancel()

    async def _init_backends(self) -> None:
        self.log.info("Init: reading RA CPR and timer freq, configuring DEC accel/vmax (if supported).")

        cpr = await asyncio.to_thread(self.ra.inquire_cpr, self.ra_ch)
        tf = await asyncio.to_thread(self.ra.inquire_timer_freq)

        self.axis.cpr = cpr
        self.axis.timer_freq = tf
        self.state.ra_cpr = cpr

        self.log.info("RA: CPR=%d, TMR_Freq=%d Hz", cpr, tf)

        # Best-effort DEC tuning extensions
        await asyncio.to_thread(self.dec.set_accel, self.dec_accel)
        await asyncio.to_thread(self.dec.set_max_rate, self.dec_vmax)

        # initial poll and sync: we don't know alignment, so we start with HA0 based on current ticks and current LST and ra=0
        ticks = await asyncio.to_thread(self.ra.inquire_position, self.ra_ch)
        self.state.last_ra_ticks = ticks
        # default: assume current pointing RA=0 at current LST -> HA = LST, so HA0 ticks chosen so that HA = ticks_to_ha(ticks) => LST
        now_utc = self.site.now_utc()
        lst = lst_hours(now_utc, self.site.lon_deg_east)
        ha = lst  # RA=0
        self.state.ra_ha0_ticks = ticks - self._ha_hours_to_ticks(ha)
        self.log.info("Initial HA0 ticks set (needs :CM sync for real sky).")

    def _ticks_to_ha_hours(self, ticks: int) -> float:
        if self.state.ra_cpr <= 0:
            return 0.0
        # scale ticks -> degrees -> hours
        deg = (ticks * 360.0 / self.state.ra_cpr) * self.state.ra_sign
        return wrap_hours(deg / 15.0)

    def _ha_hours_to_ticks(self, ha_hours: float) -> int:
        if self.state.ra_cpr <= 0:
            return 0
        deg = wrap_deg(ha_hours * 15.0)
        ticks = (deg / 360.0) * self.state.ra_cpr
        return int(round(ticks)) * self.state.ra_sign

    def _compute_radec_from_axes(self, ra_ticks: int, dec_deg: float, t_utc: dt.datetime) -> Tuple[float, float]:
        # HA = ticks_to_ha( ticks - ha0 )
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

                # DEC from Arduino
                dec_deg = await asyncio.to_thread(self.dec.get_dec)
                self.state.last_dec_deg = dec_deg

                ra_h, dec_d = self._compute_radec_from_axes(ra_ticks, dec_deg, t_utc)
                self.state.ra_hours = ra_h
                self.state.dec_deg = dec_d

                await asyncio.sleep(0.2)
            except asyncio.CancelledError:
                return
            except Exception as e:
                self.log.warning("Poll error: %s", e, exc_info=True)
                await asyncio.sleep(1.0)

    # --- outward LX200 behavior ---

    def get_ra_dec(self) -> Tuple[float, float]:
        ra, dec = self.state.ra_hours, self.state.dec_deg
        self.log.debug("get_ra_dec -> RA=%.6f h, DEC=%.6f deg", ra, dec)
        return ra, dec

    def set_target_ra(self, ra_hours: float) -> str:
        self.state.target_ra_hours = wrap_hours(ra_hours)
        self.log.info("set_target_ra: target_ra_hours set to %s", fmt_ra(self.state.target_ra_hours))
        self.log.debug("set_target_ra returning '1#'")
        return "1#"

    def set_target_dec(self, dec_deg: float) -> str:
        self.state.target_dec_deg = clamp(dec_deg, -90.0, 90.0)
        self.log.info("set_target_dec: target_dec_deg set to %.6f", self.state.target_dec_deg)
        self.log.debug("set_target_dec returning '1#'")
        return "1#"

    async def sync_to_target(self) -> str:
        """
        Sync semantics:
        - INDI typically does: set target RA/DEC, then :CM#.
        - We treat it as "current axes correspond to target sky coords".
        That means: compute current LST, derive HA_target = LST - RA_target,
        then set ra_ha0_ticks accordingly.
        DEC is assumed to already match physically after you adjust/slew.
        """
        if self.state.target_ra_hours is None or self.state.target_dec_deg is None:
            self.log.warning(
                "SYNC requested but target RA or DEC is not set (RA=%r DEC=%r).",
                self.state.target_ra_hours,
                self.state.target_dec_deg,
            )
            self.log.warning("Please call LX200 commands :Sr <RA> and :Sd <DEC> (or console 'setra'/'setdec') before :CM (sync). Returning '0#'.")
            return "0#"
        t_utc = self.site.now_utc()
        lst = lst_hours(t_utc, self.site.lon_deg_east)
        ha_target = wrap_hours(lst - self.state.target_ra_hours)
        # current ticks -> should correspond to ha_target => ha0_ticks = ticks - ha_to_ticks(ha_target)
        ticks = self.state.last_ra_ticks
        self.state.ra_ha0_ticks = ticks - self._ha_hours_to_ticks(ha_target)

        # DEC: best effort - ask Arduino to set its current coord (non-standard), else just accept.
        # If you want hard sync in DEC, implement :XSC<dec> on Arduino.
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
            await asyncio.to_thread(self.dec.abort)
        except Exception as e:
            self.log.warning("DEC abort failed: %s", e)
        self.log.debug("abort returning '1#'")
        return "1#"

    async def goto_target(self) -> str:
        """
        Perform coordinated GOTO:
        - RA axis: motor controller GOTO to computed target HA
        - DEC axis: Arduino LX200 GOTO using :Sd + :MS
        After slew, (re)enable RA tracking in a safe way.
        """
        if self.state.target_ra_hours is None or self.state.target_dec_deg is None:
            self.log.warning(
                "GOTO requested but target RA or DEC is not set (RA=%r DEC=%r).",
                self.state.target_ra_hours,
                self.state.target_dec_deg,
            )
            self.log.warning("Please call LX200 commands :Sr <RA> and :Sd <DEC> (or console 'setra'/'setdec') before :MS (goto). Returning '0#'.")
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
            self.log.debug("Computed internal state: ra_ha0_ticks=%d last_ra_ticks=%d ra_cpr=%d",
                           self.state.ra_ha0_ticks, self.state.last_ra_ticks, self.state.ra_cpr)

            # DEC first: set and start
            try:
                await asyncio.to_thread(self.dec.set_target_dec, dec_t)
                # if Arduino also needs RA, send current RA as placeholder
                try:
                    await asyncio.to_thread(self.dec.set_target_ra, ra_t)
                except Exception:
                    pass
                await asyncio.to_thread(self.dec.goto)
            except Exception as e:
                self.log.warning("DEC goto failed: %s", e, exc_info=True)

            # RA: stop, set target, goto
            try:
                await asyncio.to_thread(self.ra.instant_stop, self.ra_ch)
                # must be stopped for :S and :G
                await asyncio.sleep(0.1)
                await asyncio.to_thread(self.ra.set_motion_mode, self.ra_ch, tracking=False, ccw=self.ra_ccw)
                await asyncio.to_thread(self.ra.set_goto_target, self.ra_ch, ra_ticks_target)
                await asyncio.to_thread(self.ra.start_motion, self.ra_ch)
            except Exception as e:
                self.log.warning("RA goto failed: %s", e, exc_info=True)

            # monitor until RA close enough, then enable tracking
            await self._wait_slew_finish(ra_ticks_target, dec_t, timeout_s=180.0)
            await self.enable_tracking(True)
            self.log.info("GOTO completed; returning '0#'")
            return "0#"  # LX200 expects "0" for success in many implementations

    async def _wait_slew_finish(self, ra_target_ticks: int, dec_target_deg: float, timeout_s: float) -> None:
        start = time.monotonic()
        while time.monotonic() - start < timeout_s:
            ra_ticks = self.state.last_ra_ticks
            dec_deg = self.state.last_dec_deg
            # conservative threshold: 0.05 deg or 10 ticks min
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
        """
        Enable sidereal tracking on RA via SkyWatcher motor controller tracking mode.
        On Star Adventurer 2i this MAY be redundant (it can track standalone), but when you take over via UART
        it is safer to set it explicitly.
        """
        if self.axis.cpr <= 0 or self.axis.timer_freq <= 0:
            self.log.warning("Tracking not configured (missing CPR/TMR_Freq).")
            return

        # sidereal: 360 deg per 86164.0905 s
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
                await asyncio.to_thread(self.ra.set_motion_mode, self.ra_ch, tracking=True, ccw=self.ra_ccw)
                await asyncio.to_thread(self.ra.set_step_period, self.ra_ch, preset)
                await asyncio.to_thread(self.ra.start_motion, self.ra_ch)
            else:
                await asyncio.to_thread(self.ra.stop_motion, self.ra_ch)
        except Exception as e:
            self.log.warning("Tracking command failed: %s", e, exc_info=True)
        self.log.debug("enable_tracking(%s) completed (preset=%d)", enabled, preset)

    async def move(self, axis: str, direction: str, start: bool, rate_deg_s: float) -> str:
        """
        axis: 'ra' or 'dec'
        direction: 'E','W','N','S'
        """
        self.log.info("move requested: axis=%s direction=%s start=%s rate=%.6f", axis, direction, start, rate_deg_s)
        if axis == "dec":
            if direction == "N":
                await asyncio.to_thread(self.dec.move_ns, True, start)
            elif direction == "S":
                await asyncio.to_thread(self.dec.move_ns, False, start)
            elif direction == "E":
                await asyncio.to_thread(self.dec.move_we, True, start)
            elif direction == "W":
                await asyncio.to_thread(self.dec.move_we, False, start)
            self.log.debug("move (dec) returning '1#'")
            return "1#"

        # RA manual move via tracking mode with custom speed.
        # rate_deg_s is additional speed; we translate to step period. This ignores acceleration on RA (per your request).
        if self.axis.cpr <= 0 or self.axis.timer_freq <= 0:
            self.log.warning("RA move requested but RA CPR or timer_freq not configured.")
            return "0#"
        # convert deg/s to counts/s then preset
        rate_deg_s = abs(rate_deg_s)
        if rate_deg_s < 1e-6:
            return "0#"
        counts_per_s = rate_deg_s * self.axis.cpr / 360.0
        preset = int(round(self.axis.timer_freq / counts_per_s))
        preset = int(clamp(preset, 1, (1 << 24) - 1))

        ccw = self.ra_ccw
        if direction == "E":
            # East/WEST mapping depends on mount; expose --ra-ccw and --ra-sign for tuning.
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
            self.log.debug("move returning '0#' due to exception")
            return "0#"
        self.log.debug("move returning '1#'")
        return "1#"


# -----------------------------
# TCP LX200 server
# -----------------------------

class LX200TCPServer:
    def __init__(self, mount: FrankenMount):
        self.mount = mount
        self.log = logging.getLogger("lx200.tcp")
        self.slew_rate = "C"  # C=fast, M=medium, S=slow, G=guide
        # default rates in deg/s; tune as needed
        self.rates = {"C": 4.0, "M": 1.0, "S": 0.2, "G": 0.02}

    async def handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername")
        self.log.info("Client connected: %s", peer)
        buf = bytearray()
        try:
            while True:
                b = await reader.read(1)
                if not b:
                    return
                buf += b
                if buf.endswith(b"#"):
                    cmd = buf.decode("ascii", errors="replace")
                    buf.clear()
                    resp = await self.dispatch(cmd)
                    if resp is not None:
                        self.log.debug("CMD %r -> %r", cmd, resp)
                        writer.write(resp.encode("ascii"))
                        await writer.drain()
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self.log.warning("Client error: %s", e, exc_info=True)
        finally:
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass
            self.log.info("Client disconnected: %s", peer)

    async def dispatch(self, cmd: str) -> Optional[str]:
        """
        cmd includes leading ':' and trailing '#'.
        Return response WITH trailing '#', or None (no response).
        """
        cmd = cmd.strip()
        if not (cmd.startswith(":") and cmd.endswith("#")):
            return None

        # strip wrappers
        body = cmd[1:-1]

        # --- queries ---
        if body == "GR":
            ra, _ = self.mount.get_ra_dec()
            return fmt_ra(ra)
        if body == "GD":
            _, dec = self.mount.get_ra_dec()
            return fmt_dec(dec)

        if body == "GC":
            # get date MM/DD/YY
            utc = self.mount.site.now_utc()
            # return local date as stored by utc_offset
            offset = dt.timedelta(hours=self.mount.site.utc_offset_hours)
            local = utc.astimezone(dt.timezone(offset))
            return f"{local.month:02d}/{local.day:02d}/{local.year%100:02d}#"
        if body == "GL":
            # get local time HH:MM:SS
            utc = self.mount.site.now_utc()
            offset = dt.timedelta(hours=self.mount.site.utc_offset_hours)
            local = utc.astimezone(dt.timezone(offset))
            return f"{local.hour:02d}:{local.minute:02d}:{local.second:02d}#"
        if body == "Gt":
            return fmt_lx200_lat(self.mount.site.lat_deg)
        if body == "Gg":
            return fmt_lx200_lon(self.mount.site.lon_deg_east)

        # --- setters ---
        if body.startswith("Sr "):
            try:
                ra = parse_ra_hms(body[3:])
                return self.mount.set_target_ra(ra)
            except Exception:
                return "0#"
        if body.startswith("Sd "):
            try:
                dec = parse_dec_dms(body[3:])
                return self.mount.set_target_dec(dec)
            except Exception:
                return "0#"

        if body.startswith("SC "):
            # set date MM/DD/YY
            try:
                mm, dd, yy = body[3:].split("/")
                y = int(yy)
                y = 2000 + y if y < 70 else 1900 + y
                m = int(mm); d = int(dd)
                # keep previous time if set, else 00:00:00
                t = self.mount.site.local_datetime.time() if self.mount.site.local_datetime else dt.time(0,0,0)
                self.mount.site.local_datetime = dt.datetime(y, m, d, t.hour, t.minute, t.second)
                return "1#"
            except Exception:
                return "0#"

        if body.startswith("SL "):
            # set local time HH:MM:SS
            try:
                hh, mm, ss = body[3:].split(":")
                h = int(hh); m = int(mm); s = int(ss)
                # keep previous date if set, else today in local
                utc = self.mount.site.now_utc()
                offset = dt.timedelta(hours=self.mount.site.utc_offset_hours)
                local_now = utc.astimezone(dt.timezone(offset))
                d0 = self.mount.site.local_datetime.date() if self.mount.site.local_datetime else local_now.date()
                self.mount.site.local_datetime = dt.datetime(d0.year, d0.month, d0.day, h, m, s)
                return "1#"
            except Exception:
                return "0#"

        if body.startswith("SG "):
            # set longitude sDDD*MM (we interpret as East-positive by default)
            try:
                lon = parse_lx200_signed_deg(body[3:])
                self.mount.site.lon_deg_east = lon
                return "1#"
            except Exception:
                return "0#"

        if body.startswith("St "):
            # set latitude sDD*MM
            try:
                lat = parse_lx200_signed_deg(body[3:])
                self.mount.site.lat_deg = clamp(lat, -90.0, 90.0)
                return "1#"
            except Exception:
                return "0#"

        if body.startswith("SG") or body.startswith("St"):
            return "0#"

        # --- sync / goto / abort ---
        if body == "MS":
            return await self.mount.goto_target()
        if body == "CM":
            return await self.mount.sync_to_target()
        if body == "Q":
            return await self.mount.abort()

        # --- slew rates (Meade): :RG :RC :RM :RS ---
        if body in ("RG", "RC", "RM", "RS"):
            self.slew_rate = body[1]  # G/C/M/S
            return "1#"

        # --- manual motion start: :Mn :Ms :Me :Mw ; stop: :Qn :Qs :Qe :Qw ---
        if body in ("Mn", "Ms", "Me", "Mw"):
            rate = self.rates.get(self.slew_rate, 0.2)
            if body in ("Mn", "Ms"):
                # DEC axis: North/South
                direction = "N" if body == "Mn" else "S"
                return await self.mount.move("dec", direction, True, rate)
            else:
                # RA axis: East/West (sign depends on config)
                direction = "E" if body == "Me" else "W"
                return await self.mount.move("ra", direction, True, rate)

        if body in ("Qn", "Qs", "Qe", "Qw"):
            rate = self.rates.get(self.slew_rate, 0.2)
            if body in ("Qn", "Qs"):
                direction = "N" if body == "Qn" else "S"
                return await self.mount.move("dec", direction, False, rate)
            else:
                direction = "E" if body == "Qe" else "W"
                return await self.mount.move("ra", direction, False, rate)

        # --- non-standard: DEC accel/vmax runtime tuning (usable via netcat)
        if body.startswith("XAC"):
            try:
                accel = float(body[3:])
                self.mount.dec_accel = accel
                await asyncio.to_thread(self.mount.dec.set_accel, accel)
                return "1#"
            except Exception:
                return "0#"
        if body.startswith("XVM"):
            try:
                vmax = float(body[3:])
                self.mount.dec_vmax = vmax
                await asyncio.to_thread(self.mount.dec.set_max_rate, vmax)
                return "1#"
            except Exception:
                return "0#"

        # fallback: empty response (some LX200 cmds expect that)
        return "#"


# -----------------------------
# main
# -----------------------------

async def console_cli(mount: FrankenMount) -> None:
    """Simple interactive console for testing the mount.

    Commands:
      pos                - show current RA/DEC
      stop               - abort all motion
      setra HH:MM:SS     - set target RA
      setdec +DD*MM:SS   - set target DEC
      goto               - perform GOTO to last set targets
      sidereal on|off    - enable/disable sidereal tracking
      help               - show this help
      exit               - quit process
    """
    print("Test console started. Type 'help' for commands.")
    while True:
        try:
            line = await asyncio.to_thread(input, "console> ")
        except (EOFError, KeyboardInterrupt):
            print("Exiting console.")
            os._exit(0)
        cmd = line.strip()
        if not cmd:
            continue
        if cmd in ("help", "h"):
            print("pos, stop, setra HH:MM:SS, setdec +DD*MM[:SS], goto, sidereal on|off, exit")
            continue
        if cmd == "pos":
            ra, dec = mount.get_ra_dec()
            print(f"RA={fmt_ra(ra)[:-1]} DEC={fmt_dec(dec)[:-1]}")
            continue
        if cmd == "stop":
            await mount.abort()
            print("Abort sent.")
            continue
        if cmd.startswith("setra "):
            try:
                ra = parse_ra_hms(cmd[6:])
                mount.set_target_ra(ra)
                print(f"Target RA set to {fmt_ra(ra)[:-1]}")
            except Exception as e:
                print(f"Bad RA: {e}")
            continue
        if cmd.startswith("setdec "):
            try:
                dec = parse_dec_dms(cmd[7:])
                mount.set_target_dec(dec)
                print(f"Target DEC set to {fmt_dec(dec)[:-1]}")
            except Exception as e:
                print(f"Bad DEC: {e}")
            continue
        if cmd == "goto":
            print("Starting GOTO...")
            res = await mount.goto_target()
            print(f"GOTO result: {res}")
            continue
        if cmd.startswith("sidereal "):
            a = cmd.split()
            if len(a) >= 2 and a[1] in ("on", "off"):
                await mount.enable_tracking(a[1] == "on")
                print(f"Sidereal tracking {'enabled' if a[1]=='on' else 'disabled'}.")
            else:
                print("Usage: sidereal on|off")
            continue
        if cmd in ("exit", "quit"):
            print("Exiting process.")
            os._exit(0)
        print("Unknown command. Type 'help'.")


def setup_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s.%(msecs)03d %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", default="127.0.0.1:10001", help="host:port for LX200 TCP server")
    ap.add_argument("--log-level", default="INFO")

    ap.add_argument("--ra-port", required=True)
    ap.add_argument("--ra-baud", type=int, default=9600)
    ap.add_argument("--ra-timeout", type=float, default=0.5)
    ap.add_argument("--ra-channel", default="1", choices=["1","2"])
    ap.add_argument("--ra-ccw", action="store_true", help="RA tracking direction bit: CCW if set, else CW")
    ap.add_argument("--ra-sign", type=int, default=+1, choices=[-1, +1], help="tick-to-HA sign correction (+1 default)")

    ap.add_argument("--dec-port", required=True)
    ap.add_argument("--dec-baud", type=int, default=115200)
    ap.add_argument("--dec-timeout", type=float, default=0.5)

    ap.add_argument("--site-lat", type=float, default=0.0)
    ap.add_argument("--site-lon", type=float, default=0.0, help="longitude east-positive degrees")
    ap.add_argument("--utc-offset", type=float, default=0.0, help="local time offset hours for :SL/:SC (e.g. +6)")
    ap.add_argument("--dec-accel", type=float, default=5.0, help="DEC accel deg/s^2 (sent via :XAC if supported)")
    ap.add_argument("--dec-vmax", type=float, default=4.0, help="DEC max rate deg/s (sent via :XVM if supported)")
    ap.add_argument("--test-mode", action="store_true", help="Run interactive test console for mount control")

    args = ap.parse_args()
    setup_logging(args.log_level)

    host, port_s = args.listen.split(":")
    port = int(port_s)

    ra_dev = SerialLineDevice(args.ra_port, args.ra_baud, args.ra_timeout, name="serial.ra")
    dec_dev = SerialLineDevice(args.dec_port, args.dec_baud, args.dec_timeout, name="serial.dec")

    ra = SkyWatcherMC(ra_dev)
    dec = LX200SerialClient(dec_dev)
    site = SiteTime(lat_deg=args.site_lat, lon_deg_east=args.site_lon, utc_offset_hours=args.utc_offset)

    mount = FrankenMount(
        ra=ra,
        ra_ch=args.ra_channel,
        dec=dec,
        site=site,
        ra_ccw=args.ra_ccw,
        ra_sign=args.ra_sign,
        dec_accel_deg_s2=args.dec_accel,
        dec_vmax_deg_s=args.dec_vmax,
    )

    server = LX200TCPServer(mount)

    async def runner():
        await mount.start()
        if args.test_mode:
            # spawn console CLI concurrently
            asyncio.create_task(console_cli(mount))
        srv = await asyncio.start_server(server.handle_client, host, port)
        addrs = ", ".join(str(sock.getsockname()) for sock in srv.sockets or [])
        logging.getLogger("main").info("LX200 TCP listening on %s", addrs)
        async with srv:
            await srv.serve_forever()

    try:
        asyncio.run(runner())
    finally:
        ra_dev.close()
        dec_dev.close()

if __name__ == "__main__":
    main()
