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
    
    def set_telescope_ra(self, position: LX200Hours) -> str:
        return self.mount.set_telescope_ra(position)
