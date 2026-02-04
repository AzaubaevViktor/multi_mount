import time

from lx200.base import LX200Base
from lx200.protocols import LX200Ha
from .skywatcher import SkyWatcherMount, SlewMode


DEGREES_PER_HOUR = 15
SECONDS_PER_CIRCLE = 24 * 3600
HALF_CIRCLE_SECONDS = SECONDS_PER_CIRCLE / 2


class SkyWatcherLX200(LX200Base):
    """
    В режиме трекинга (монтировка движется со скоростью STELLAR_SPEED):
    - Ha остаётся постоянным

    В режиме остановки:
    - Ha остаёт (небесная сфера уходит вперёд) со скоростью STELLAR_SPEED

    В режиме slew:
    - Ha меняется как (current_tick (текущее положение монтировки) - expected_tick (какое положение монтировки должно быть)) -> to Ha
    
    Гайдинг не должен ни на что влиять

    Можно каждый раз смотреть на дельту между ожидаемыми тиками, полученными с монитровки и актуальными тиками

    Т.е будет:
    self._ra_seconds
    self._last_mount_seconds
    self._last_update_s

    при self.get_telescope_ra:
    mount_seconds = self.mount.get_telescope_ra().to_seconds()
    expected_delta = elapsed_s * (STELLAR_SPEED / 15)
    actual_delta = mount_seconds - last_mount_seconds (с учётом круга)
    self._ra_seconds += expected_delta - actual_delta
    update _last_update_s and _last_mount_seconds

    при set_telescope_ra
    self._ra_seconds = value.to_seconds()
    _last_mount_seconds = self.mount.get_telescope_ra().to_seconds()

    """
    def __init__(self, mount: SkyWatcherMount) -> None:
        self.mount = mount
        self._ra_seconds = 0.0
        self._last_mount_seconds: float | None = None
        self._last_update_s: float | None = None
    
    def connect(self):
        self.mount.connect()
        self.set_telescope_ra(LX200Ha.from_hours(0))
        self.mount.start_tracking()

    def get_telescope_ra(self) -> LX200Ha:
        now = time.monotonic()
        mount_seconds = self.mount.get_telescope_ra().to_seconds()

        if self._last_update_s is None or self._last_mount_seconds is None:
            self._ra_seconds = float(mount_seconds)
            self._last_mount_seconds = float(mount_seconds)
            self._last_update_s = now
            return LX200Ha.from_seconds(mount_seconds)

        elapsed_s = now - self._last_update_s
        if elapsed_s < 0:
            elapsed_s = 0

        expected_delta_seconds = elapsed_s * (self.mount.STELLAR_SPEED / DEGREES_PER_HOUR)
        actual_delta_seconds = self._signed_delta_seconds(self._last_mount_seconds, mount_seconds)

        self._ra_seconds = self._normalize_seconds(
            self._ra_seconds + expected_delta_seconds - actual_delta_seconds
        )
        self._last_mount_seconds = float(mount_seconds)
        self._last_update_s = now

        ra_seconds = int(round(self._ra_seconds)) % SECONDS_PER_CIRCLE
        return LX200Ha.from_seconds(ra_seconds)
    
    def set_telescope_ra(self, position: LX200Ha) -> bool:
        if not self.mount.set_telescope_ra(position):
            return False

        self._ra_seconds = float(position.to_seconds())
        mount_seconds = self.mount.get_telescope_ra().to_seconds()
        self._last_mount_seconds = float(mount_seconds)
        self._last_update_s = time.monotonic()
        return True
    
    def stop(self) -> bool:
        self.mount.gracefully_stop_motor()
        return True
    
    def slew_to_ra(self, position: LX200Ha) -> bool:
        return self.mount.slew_to_ra(position)

    def get_site1_name(self) -> str:
        return "skywatcher"
    
    def get_distance(self) -> str:
        if self.mount.get_status().slew_mode == SlewMode.GOTO:
            return "|"
        else:
            return ""

    @staticmethod
    def _normalize_seconds(seconds: float) -> float:
        return seconds % SECONDS_PER_CIRCLE

    @staticmethod
    def _signed_delta_seconds(start_seconds: float, end_seconds: float) -> float:
        delta = end_seconds - start_seconds
        if delta > HALF_CIRCLE_SECONDS:
            delta -= SECONDS_PER_CIRCLE
        elif delta < -HALF_CIRCLE_SECONDS:
            delta += SECONDS_PER_CIRCLE
        return delta
