from lx200.base import LX200Base
from lx200.protocols import LX200Hours
from .skywatcher import SkyWatcherMount


class SkyWatcherLX200(LX200Base):
    def __init__(self, mount: SkyWatcherMount) -> None:
        self.mount = mount
    
    def connect(self):
        self.mount.connect()

    def get_telescope_ra(self) -> LX200Hours:
        return self.mount.get_telescope_ra()
    
    def set_telescope_ra(self, position: LX200Hours) -> bool:
        return self.mount.set_telescope_ra(position)
    
    def slew_to_ra(self, position: LX200Hours) -> bool:
        return self.mount.slew_to_ra(position)

    def get_site1_name(self) -> str:
        return "skywatcher"
