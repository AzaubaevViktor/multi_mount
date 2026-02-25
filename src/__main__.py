from logging_setup import setup_logging
from lx200.base import LX200Base
from lx200.base_server import LX200SimpleServer
from lx200.protocol import AlignmentMode
from lx200.protocols import Dec
from lx200.splitter import LX200Splitter
from serial_wrapper.wrapper import SerialLine
from skywatcher.skywatcher_lx200 import SkyWatcherLX200, SkyWatcherMount
from tmc2209.tmc2209_adapter import TMC2209Adapter
from tmc2209.tmc2209_lx200 import TMC2209LX200


setup_logging()


class LX200TestDECServer(LX200Base):
    def __init__(self) -> None:
        self.dec = 1

    def get_telescope_dec(self) -> Dec:
        return Dec.from_degrees(self.dec)
    
    def sync_telescope_dec(self, position: Dec) -> bool:
        self.dec = position.to_degrees()
        return True
    
    def halt_all(self) -> bool:
        return True
    
    def move_east(self) -> bool:
        return False

    def move_north(self) -> bool:
        return True

    def move_south(self) -> bool:
        return True

    def move_west(self) -> bool:
        return False

    def halt_east(self) -> bool:
        return False

    def halt_north(self) -> bool:
        return True

    def halt_south(self) -> bool:
        return True

    def halt_west(self) -> bool:
        return False

    def set_slew_to_find(self) -> bool:
        return True

    def slew_to_dec(self, position: Dec) -> bool:
        self.dec = position.to_degrees()
        return True

    def handle_alignment(self, data: bytes) -> AlignmentMode:
        return AlignmentMode.POLAR
    
    def get_site1_name(self) -> str:
        return "noop"
    
    def connect(self):
        pass

    def get_distance(self) -> str:
        return ""
    

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

    server = LX200SimpleServer(splitter)

    server.serve_forever()
