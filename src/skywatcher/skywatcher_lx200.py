from lx200.base import LX200Base
from lx200.protocols import LX200Hours
from .skywatcher import SkyWatcherMount, SlewMode


class SkyWatcherLX200(LX200Base):
    """
    В режиме трекинга (монтировка движется со скоростью _STELLAR_SPEED):
    - Ha остаётся постоянным

    В режиме остановки:
    - Ha остаёт (небесная сфера уходит вперёд) со скоростью _STELLAR_SPEED

    В режиме slew:
    - Ha меняется как (current_tick (текущее положение монтировки) - expected_tick (какое положение монтировки должно быть)) -> to Ha
    
    Гайдинг не должен ни на что влиять

    Можно каждый раз смотреть на дельту между ожидаемыми тиками, полученными с монитровки и актуальными тиками

    Т.е будет:
    self.ra
    self.last_mount_ra
    self.last_update_s

    при self.get_telescope_ra:
    mount_ra = self.mount.get_telescope_ra
    expected_ticks = self.last_tick + _STELLAR_SPEED * self.last_update_s
    self.ra += (delta := (expected_ticks - mount_ra)) to lx200hours if delta > 0 else 0
    update last_update_s and last_mount_ra

    при set_telescope_ra
    self.ra = value
    last_mount_ra = self.mount.get_telescope_ra

    """
    def __init__(self, mount: SkyWatcherMount) -> None:
        self.mount = mount
    
    def connect(self):
        self.mount.connect()
        self.mount.set_telescope_ra(LX200Hours.from_hours(0))
        self.mount.start_tracking()

    def get_telescope_ra(self) -> LX200Hours:
        return self.mount.get_telescope_ra()
    
    def set_telescope_ra(self, position: LX200Hours) -> bool:
        return self.mount.set_telescope_ra(position)
    
    def stop(self) -> bool:
        self.mount.gracefully_stop_motor()
        return True
    
    def slew_to_ra(self, position: LX200Hours) -> bool:
        return self.mount.slew_to_ra(position)

    def get_site1_name(self) -> str:
        return "skywatcher"
    
    def get_distance(self) -> str:
        if self.mount.get_status().slew_mode == SlewMode.GOTO:
            return "|"
        else:
            return ""
