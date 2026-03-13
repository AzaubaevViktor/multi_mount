from pathlib import Path
import sys
import threading

SRC_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SRC_DIR.parent
for path in (str(PROJECT_ROOT), str(SRC_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from logging_setup import setup_logging
from lx200.base_server import LX200SimpleServer
from serial_wrapper.wrapper import SerialLine
from sky.axis import AxisDEC, AxisRA
from sky.combiner import Combiner
from sky.lx200 import SkyLX200
from skywatcher.motor import SkyWatcherMotor
from tmc2209.motor import TMC2209Motor
from web_control.web import MonitorServer


setup_logging()
    

if __name__ == "__main__":
    sw_path = SerialLine.search("PL2303G")
    sw_serial = SerialLine(sw_path, 115200, .05, "sw", terminator="\r")
    sw_ra_motor = SkyWatcherMotor(sw_serial)
    axis_ra = AxisRA(sw_ra_motor)

    tmc_path = SerialLine.search("tty.usbserial")
    tmc_serial = SerialLine(tmc_path, 115200, 2, "tmc", terminator="\n")
    tmc_dec_motor = TMC2209Motor(tmc_serial)
    axis_dec = AxisDEC(tmc_dec_motor)

    combiner = Combiner(axis_ra, axis_dec)
    sky_lx200 = SkyLX200(combiner)

    web_server = MonitorServer({}, port=8765)
    threading.Thread(target=web_server.serve_forever, name="WEB_CONTROL", daemon=True).start()

    server = LX200SimpleServer(sky_lx200)

    server.serve_forever()
