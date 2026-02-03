from lx200.base import LX200Base
from lx200.protocol import AlignmentMode
from lx200.protocols import LX200Dec, LX200Hours


class LX200Splitter(LX200Base):
    def __init__(self, ra: LX200Base, dec: LX200Base) -> None:
        self.ra = ra
        self.dec = dec
    
    def connect(self):
        self.ra.connect()
        self.dec.connect()

    def handle_alignment(self, data: bytes) -> AlignmentMode:
        return AlignmentMode.POLAR
    
    def get_telescope_ra(self) -> LX200Hours:
        return self.ra.get_telescope_ra()
    
    def slew_to_ra(self, position: LX200Hours) -> bool:
        return self.ra.slew_to_ra(position)
    
    def slew_to_dec(self, position: LX200Dec) -> bool:
        return self.dec.slew_to_dec(position)

    def set_telescope_ra(self, position: LX200Hours) -> bool:
        return self.ra.set_telescope_ra(position)
    
    def get_telescope_dec(self) -> LX200Dec:
        return self.dec.get_telescope_dec()
    
    def set_telescope_dec(self, position: LX200Dec) -> bool:
        return self.dec.set_telescope_dec(position)
    
    def get_site1_name(self) -> str:
        return f"splitter_ra_{self.ra.get_site1_name()}_dec_{self.dec.get_site1_name()}"
