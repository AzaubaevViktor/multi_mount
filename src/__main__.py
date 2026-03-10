import threading

from logging_setup import setup_logging
from lx200.base_server import LX200SimpleServer
from lx200.splitter import LX200Splitter
from serial_wrapper.wrapper import SerialLine
from skywatcher.skywatcher_lx200 import SkyWatcherLX200, SkyWatcherMount
from tmc2209.tmc2209_adapter import TMC2209Adapter
from tmc2209.tmc2209_lx200 import TMC2209LX200
from web_control.web import MonitorServer


setup_logging()
    

if __name__ == "__main__":
    sw_path = SerialLine.search("PL2303G")
    sw_serial = SerialLine(sw_path, 115200, .05, "sw")
    sw_ra_mount = SkyWatcherMount(sw_serial)
    sw_lx200 = SkyWatcherLX200(sw_ra_mount)


    tmc_path = SerialLine.search("tty.usbserial") 
    tmc_serial = SerialLine(tmc_path, 115200, 2, "tmc", terminator="\n")
    tmc_dec_mount = TMC2209Adapter(tmc_serial)
    tmc_lx200 = TMC2209LX200(tmc_dec_mount)

    splitter = LX200Splitter(
        ra=sw_lx200,
        dec=tmc_lx200,
    )

    web_server = MonitorServer({"skywatcher_ra": sw_ra_mount}, port=8765)
    threading.Thread(target=web_server.serve_forever, name="WEB_CONTROL", daemon=True).start()

    server = LX200SimpleServer(splitter)

    server.serve_forever()
