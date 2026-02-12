from dataclasses import dataclass
import logging

from lx200.base import LX200Base
from lx200.protocols import LX200Dec
from tmc2209.tmc2209_adapter import (
    GEAR_RATIO_1,
    GEAR_RATIO_2,
    MICROSTEPS_ALLOWED,
    STEPS_PER_REV,
    TMC2209Adapter,
)


@dataclass
class SpeedProfile:
    microsteps: int
    speed: int
    accel: int


GOTO_DELTA_ARCSEC_THRESHOLD = 5.0
GOTO_FAST_ACCEL_STEPS_PER_MS = 10
GOTO_SLOW_ACCEL_STEPS_PER_MS = 1
GOTO_FAST_MICROSTEPS = 2
GOTO_SLOW_MICROSTEPS = 16
MANUAL_MICROSTEPS = 8
GUIDE_MICROSTEPS = 32
GUIDE_ARCSEC_PER_SEC = 0.5


class TMC2209LX200Error(Exception):
    pass


class TMC2209LX200ConfigError(TMC2209LX200Error):
    pass


class TMC2209LX200(LX200Base):
    FAST_PROFILE_DELTA_DEG = 1

    _goto_fast_profile = SpeedProfile(
        microsteps=2, 
        speed=5000,
        accel=10,
    )
    _goto_slow_profile = SpeedProfile(
        microsteps=16, 
        speed=1000,
        accel=1,
    )
    _slew_profile = SpeedProfile(
        microsteps=8, 
        speed=1000,
        accel=2,
    )
    _guide_profile = SpeedProfile(
        microsteps=128, 
        speed=1000,
        accel=10000,
    )

    def __init__(
        self,
        adapter: TMC2209Adapter,
    ) -> None:
        self._adapter = adapter

        self._dec_steps = 0
        self._microsteps: int = 1

        self.logger = logging.getLogger("TMC2209LX200")

    # Microsteps update logic

    def _refresh_microsteps(self, microsteps: int) -> None:
        microsteps = self._microsteps
        if microsteps != self._microsteps:
            self._set_microsteps(microsteps)
    
    def _set_microsteps(self, microsteps: int) -> None:
        if microsteps not in MICROSTEPS_ALLOWED:
            raise TMC2209LX200ConfigError(f"microsteps not allowed: {microsteps!r}")
        self._adapter.set_param("microsteps", microsteps)

        self._microsteps = microsteps

        # Reset position when microsteps changed
        self._adapter.set_position(0)

    # calculate steps_per_revolution
    @property
    def steps_per_rev(self):
        return STEPS_PER_REV * self._microsteps * GEAR_RATIO_1 * GEAR_RATIO_2

    @property
    def steps_per_degree(self):
        return self.steps_per_rev / 360

    def _initialize(self) -> None:
        status = self._adapter.status()
        self._apply_profile(self._slew_profile)
        self.logger.info("Status: %s", status)

    def connect(self) -> None:
        self._adapter.connect()
        self._initialize()

    def stop(self):
        self._adapter.close()

    def get_telescope_raw_position(self) -> tuple[float, float]:
        position = self._adapter.status().position
        new_position = int(position % self.steps_per_rev)
        if new_position != position:
            self._adapter.set_position(new_position)
        return 0, new_position

    def _steps_from_dec(self, position: LX200Dec) -> int:
        return int(round(position.to_degrees() * self.steps_per_degree % self.steps_per_rev))

    def _dec_from_steps(self, steps: int) -> LX200Dec:
        degrees = steps / self.steps_per_degree
        return LX200Dec.from_degrees(degrees)

    def get_telescope_dec(self) -> LX200Dec:
        return self._dec_from_steps(int(self.get_telescope_raw_position()[1]))

    def sync_telescope_dec(self, position: LX200Dec) -> bool:
        steps = self._steps_from_dec(position)
        self._adapter.set_position(steps)
        return True

    def halt_all(self) -> bool:
        self._adapter.halt()
        return True

    def slew_to_dec(self, position: LX200Dec) -> bool:
        current_position = int(self.get_telescope_raw_position()[1])
        target_steps = self._steps_from_dec(position)
        delta_steps = target_steps - current_position

        profile = self._goto_slow_profile
        if abs(self._dec_from_steps(abs(delta_steps)).to_degrees()) > self.FAST_PROFILE_DELTA_DEG:
            profile = self._goto_fast_profile

        self._apply_profile(profile)
        
        self._adapter.set_target(target_steps)

        self._adapter.run()
        
        return True

    def get_site1_name(self) -> str:
        return "tmc2209"

    def get_distance(self) -> str:
        status = self._adapter.status()
        if status.phase not in ('idle', 'hold'):
            return "|"
        return ""

    def set_slew_to_find(self) -> bool:
        return True

    def move_east(self) -> bool:
        return False

    def move_north(self) -> bool:
        return self._start_manual_move(
            False,
        )

    def move_south(self) -> bool:
        return self._start_manual_move(
            True,
        )

    def move_west(self) -> bool:
        return False

    def halt_east(self) -> bool:
        return False

    def halt_north(self) -> bool:
        return self.halt_all()

    def halt_south(self) -> bool:
        return self.halt_all()

    def halt_west(self) -> bool:
        return False

    def guide_east(self) -> bool:
        return False

    def guide_north(self) -> bool:
        self._apply_profile(self._guide_profile)
        self._adapter.set_direction(False)
        self._adapter.run()
        return True

    def guide_south(self) -> bool:
        self._apply_profile(self._guide_profile)
        self._adapter.set_direction(True)
        self._adapter.run()
        return True

    def guide_west(self) -> bool:
        return False

    def guide_reset(self) -> bool:
        return self.halt_all()

    def _start_manual_move(
        self,
        direction: bool,
    ) -> bool:
        self._apply_profile(self._slew_profile)
        self._adapter.set_direction(direction)
        return self._adapter.run()

    def _apply_profile(
        self, profile: SpeedProfile
    ) -> None:
        self._refresh_microsteps(profile.microsteps)
        self._adapter.set_speed_sps(profile.speed)
        self._adapter.set_acceleration_steps_per_ms(profile.accel)
