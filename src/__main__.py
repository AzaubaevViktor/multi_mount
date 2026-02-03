import logging
from logging_setup import setup_logging
from lx200.base import LX200Base
from lx200.base_server import LX200SimpleServer
from lx200.protocol import AlignmentMode
from lx200.splitter import LX200Splitter
from serial_wrapper.wrapper import SerialLine
from skywatcher.skywatcher_lx200 import SkyWatcherLX200, SkyWatcherMount


setup_logging()

log = logging.getLogger("lx200")

class LX200TestDECServer(LX200Base):
    def handle_alignment(self, data: bytes) -> AlignmentMode:
        return AlignmentMode.POLAR
    
    def connect(self):
        pass
    

if __name__ == "__main__":
    skywatcher_serial = SerialLine("/dev/tty.PL2303G-USBtoUART2120", 112500, .2, "skywatcher")
    skywatcher_ra_mount = SkyWatcherMount(skywatcher_serial)
    skywatcher_lx200 = SkyWatcherLX200(skywatcher_ra_mount)

    dec = LX200TestDECServer()

    splitter = LX200Splitter(
        ra=skywatcher_lx200,
        dec=dec,
    )

    server = LX200SimpleServer(splitter)

    server.serve_forever()
