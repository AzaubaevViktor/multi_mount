import logging
import argparse
from pathlib import Path
import sys
import time

SRC_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SRC_DIR.parent
for path in (str(PROJECT_ROOT), str(SRC_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from logging_setup import setup_logging
from lx200.base_server import LX200SimpleServer
from manual_control import ManualControlConsole
from pointing.api import AgentAPI, AgentAPIServer
from pointing.service import PointingService
from serial_wrapper.wrapper import SerialLine, SerialLineSearchError
from sky.axis import AxisDEC, AxisRA, MotorReadinessState
from sky.combiner import Combiner
from sky.lx200 import SkyLX200
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond
from sky.unavailable_motor import UnavailableMotor
from stdout_dashboard import StdoutDashboard
from skywatcher.motor import SkyWatcherMotor
from tmc2209.motor import TMC2209Motor


setup_logging(stream_level=None)
    

if __name__ == "__main__":
    logger = logging.getLogger("startup")
    ra_search_missing = False
    dec_search_missing = False

    # `python -m src --sim` runs the whole application over simulated boards, so
    # a real external LX200 client can drive the mount with no hardware on the
    # bench. Everything above the serial port is the production graph — see
    # `tools/sim_stack.py` for what is fake and what is not.
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--api-host", default="127.0.0.1")
    parser.add_argument("--api-port", type=int, default=8080)
    parser.add_argument("--sensor-calibration", type=Path)
    options = parser.parse_args()
    simulated = options.sim
    calibration_storage = options.sensor_calibration or Path(
        "state/sensor_calibration_sim.json" if simulated else "state/sensor_calibration.json"
    )

    if simulated:
        from tools.sim_stack import build_realtime_sim_stack

        stack = build_realtime_sim_stack(calibration_storage=calibration_storage)
        axis_ra, axis_dec = stack.axis_ra, stack.axis_dec
        logger.warning("SIMULATION: both axes are simulated boards, nothing will move in the room")

    if not simulated:
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

    if simulated:
        combiner, pointing, sky_lx200 = stack.combiner, stack.pointing, stack.sky_lx200
    else:
        combiner = Combiner(axis_ra, axis_dec)
        pointing = PointingService(axis_dec, calibration_storage)
        sky_lx200 = SkyLX200(combiner, pointing)
    server = LX200SimpleServer(sky_lx200)
    api_server = AgentAPIServer(
        AgentAPI(sky_lx200, pointing, stack.orientation_sensor if simulated else None, lx200_server=server),
        options.api_host, options.api_port,
    )

    sky_lx200.connect()

    try:
        api_server.start()
        logger.info("Agent API listening at %s:%s", *api_server.address)
        startup_timeout_s = 0.0 if ra_search_missing or dec_search_missing else 12.0
        deadline = time.monotonic() + startup_timeout_s
        while time.monotonic() < deadline:
            if all(readiness.is_ready for readiness in combiner.motors_readiness()):
                break
            time.sleep(0.1)

        readiness = combiner.motors_readiness()

        # A motor whose `status()` raises is a broken axis, not a missing one. Say
        # so out loud before falling back to the manual console, instead of the old
        # `except Exception: return False`, which made the two indistinguishable.
        for axis_readiness in readiness:
            if axis_readiness.state is MotorReadinessState.UNKNOWN:
                logger.error("Startup readiness check failed: %s", axis_readiness.describe())
            elif not axis_readiness.is_ready:
                logger.warning("Startup readiness check: %s", axis_readiness.describe())

        if all(axis_readiness.is_ready for axis_readiness in readiness):
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
                    axis_readiness.axis.value: axis_readiness.is_ready
                    for axis_readiness in combiner.motors_readiness()
                },
            )
            console.run()
    finally:
        api_server.stop()
        server.stop()
        sky_lx200.stop()
