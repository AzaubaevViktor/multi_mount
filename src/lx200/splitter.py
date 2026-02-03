from lx200.base import LX200Base
from lx200.protocols import LX200Hours


class LX200Splitter(LX200Base):
    def __init__(self, ra: LX200Base, dec: LX200Base) -> None:
        self.ra = ra
        self.dec = dec

    def get_telescope_ra(self) -> LX200Hours:
        return self.ra.get_telescope_ra()
