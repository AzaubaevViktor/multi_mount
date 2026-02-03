from lx200.base import LX200Base
from lx200.protocol import AlignmentMode
from lx200.protocols import LX200Hours


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
