import logging
from pathlib import Path
import sys
import time
import threading

SRC_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SRC_DIR.parent
for path in (str(PROJECT_ROOT), str(SRC_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from logging_setup import setup_logging
from lx200.base_server import LX200SimpleServer
from manual_control import ManualControlConsole
from serial_wrapper.wrapper import SerialLine, SerialLineSearchError
from sky.axis import AxisDEC, AxisRA
from sky.combiner import Combiner
from sky.lx200 import SkyLX200
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond
from sky.unavailable_motor import UnavailableMotor
from stdout_dashboard import StdoutDashboard
from skywatcher.motor import SkyWatcherMotor
from tmc2209.motor import TMC2209Motor
from web_control.web import MonitorServer


setup_logging(stream_level=None)
    

if __name__ == "__main__":
    def _axis_motor_connected(axis: AxisRA | AxisDEC) -> bool:
        monitor = axis.command_monitor()
        if "motor_connected" in monitor:
            return bool(monitor["motor_connected"])
        try:
            return bool(axis._motor.status().is_connected)
        except Exception:
            return False

    logger = logging.getLogger("startup")
    ra_search_missing = False
    dec_search_missing = False

    try:
        sw_path = SerialLine.search("PL2303G")
        sw_serial = SerialLine(sw_path, 115200, .05, "sw", terminator="\r")
        axis_ra = AxisRA(SkyWatcherMotor(sw_serial))
    except SerialLineSearchError as exc:
        ra_search_missing = True
        logger.warning("RA axis is unavailable: %s", exc)
        axis_ra = AxisRA(
            UnavailableMotor(
                Ha,
                HaPerSecond,
                SkyWatcherMotor.FORWARD_POSITION_SIGN,
                f"RA axis is unavailable: {exc}",
            )
        )

    try:
        tmc_path = SerialLine.search("tty.usbserial")
        tmc_serial = SerialLine(tmc_path, 115200, 2, "tmc", terminator="\n")
        axis_dec = AxisDEC(TMC2209Motor(tmc_serial))
    except SerialLineSearchError as exc:
        dec_search_missing = True
        logger.warning("DEC axis is unavailable: %s", exc)
        axis_dec = AxisDEC(
            UnavailableMotor(
                Dec,
                DecPerSecond,
                TMC2209Motor.FORWARD_POSITION_SIGN,
                f"DEC axis is unavailable: {exc}",
            )
        )

    combiner = Combiner(axis_ra, axis_dec)
    sky_lx200 = SkyLX200(combiner)

    try:
        web_server = MonitorServer({}, port=8765)
    except OSError as exc:
        logger.warning("Web monitor is unavailable on 127.0.0.1:8765: %s", exc)
    else:
        threading.Thread(target=web_server.serve_forever, name="WEB_CONTROL", daemon=True).start()

    server = LX200SimpleServer(sky_lx200)
    sky_lx200.connect()

    try:
        startup_timeout_s = 0.0 if ra_search_missing or dec_search_missing else 12.0
        deadline = time.monotonic() + startup_timeout_s
        while time.monotonic() < deadline:
            ra_connected = _axis_motor_connected(axis_ra)
            dec_connected = _axis_motor_connected(axis_dec)
            if ra_connected and dec_connected:
                break
            time.sleep(0.1)

        ra_connected = _axis_motor_connected(axis_ra)
        dec_connected = _axis_motor_connected(axis_dec)

        if ra_connected and dec_connected:
            dashboard = StdoutDashboard(combiner, sky_lx200)
            dashboard.start()
            try:
                server.serve_forever()
            finally:
                dashboard.stop()
        else:
            console = ManualControlConsole(
                sky_lx200,
                server,
                lambda: {
                    "ra": _axis_motor_connected(axis_ra),
                    "dec": _axis_motor_connected(axis_dec),
                },
            )
            console.run()
    finally:
        server.stop()
        sky_lx200.stop()
