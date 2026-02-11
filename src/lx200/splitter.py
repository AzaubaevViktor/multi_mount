import logging
from lx200.base import LX200Base, LX200Handler
from lx200.protocol import AlignmentMode
from lx200.protocols import LX200Dec, LX200Ha


class LX200Splitter(LX200Handler):
    def __init__(self, ra: LX200Base, dec: LX200Base) -> None:
        super().__init__()
        self.logger = logging.getLogger("splitter")
        self.ra = ra
        self.dec = dec
        self.logger.info("RA: %r; DEC: %r", self.ra, self.dec)
    
    def connect(self):
        self.ra.connect()
        self.dec.connect()
        super().connect()

    def handle_alignment(self, data: bytes) -> AlignmentMode:
        return AlignmentMode.POLAR
    
    def get_telescope_ra(self) -> LX200Ha:
        return self.ra.get_telescope_ra()
    
    def slew_to_ra(self, position: LX200Ha) -> bool:
        return self.ra.slew_to_ra(position)
    
    def slew_to_dec(self, position: LX200Dec) -> bool:
        return self.dec.slew_to_dec(position)

    def sync_telescope_ra(self, position: LX200Ha) -> bool:
        return self.ra.sync_telescope_ra(position)
    
    def get_telescope_dec(self) -> LX200Dec:
        return self.dec.get_telescope_dec()
    
    def sync_telescope_dec(self, position: LX200Dec) -> bool:
        return self.dec.sync_telescope_dec(position)
    
    def get_site1_name(self) -> str:
        return f"splitter_ra_{self.ra.get_site1_name()}_dec_{self.dec.get_site1_name()}"

    def get_distance(self) -> str:
        return self.ra.get_distance() + self.dec.get_distance()
    
    def halt_all(self) -> bool:
        try:
            self.ra.halt_all()
        except Exception:
            self.logger.exception("While stop RA")
        
        try:
            self.dec.halt_all()
        except Exception:
            self.logger.exception("While stop DEC")
        
        return True

    def move_east(self) -> bool:
        try:
            return self.ra.move_east()
        except Exception:
            self.logger.exception("While move RA east")
            return False

    def move_north(self) -> bool:
        try:
            return self.dec.move_north()
        except Exception:
            self.logger.exception("While move DEC north")
            return False

    def move_south(self) -> bool:
        try:
            return self.dec.move_south()
        except Exception:
            self.logger.exception("While move DEC south")
            return False

    def move_west(self) -> bool:
        try:
            return self.ra.move_west()
        except Exception:
            self.logger.exception("While move RA west")
            return False

    def halt_east(self) -> bool:
        try:
            return self.ra.halt_east()
        except Exception:
            self.logger.exception("While stop RA east")
            return False

    def halt_north(self) -> bool:
        try:
            return self.dec.halt_north()
        except Exception:
            self.logger.exception("While stop DEC north")
            return False

    def halt_south(self) -> bool:
        try:
            return self.dec.halt_south()
        except Exception:
            self.logger.exception("While stop DEC south")
            return False

    def halt_west(self) -> bool:
        try:
            return self.ra.halt_west()
        except Exception:
            self.logger.exception("While stop RA west")
            return False

    def set_slew_to_find(self) -> bool:
        ok = True
        try:
            self.ra.set_slew_to_find()
        except Exception:
            self.logger.exception("While set RA slew rate")
            ok = False
        try:
            self.dec.set_slew_to_find()
        except Exception:
            self.logger.exception("While set DEC slew rate")
            ok = False
        return ok
