from lx200.base import LX200Base
from .skywatcher import SkyWatcherMount


class SkyWatcherLX200(LX200Base):
    def __init__(self, mount: SkyWatcherMount) -> None:
        self.mount = mount