from dataclasses import dataclass
import time

from lx200.base import LX200DECHandler
from lx200.protocol import AlignmentMode
from lx200.protocols import LX200Dec, LX200Ha
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


class TMC2209LX200(LX200DECHandler):
    FAST_PROFILE_DELTA_DEG = 1
    _DEFAULT_TRACKING_RATE = 0

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
        accel=0,
    )

    def __init__(
        self,
        adapter: TMC2209Adapter,
    ) -> None:
        self._adapter = adapter
        self._microsteps: int = 1

        self._is_connected = False

        super().__init__()

    def _is_motor_connected(self) -> bool:
        return self._is_connected

    def _get_motor_status(self):
        return self._adapter.status()

    def _get_motor_raw_position(self) -> float:
        return -self._arcseconds_from_steps(self.get_telescope_raw_position()[1])

    def _get_default_tracking_speed(self) -> float:
        return GUIDE_ARCSEC_PER_SEC

    def _wrap_mount_position(self, mount_position: float) -> float:
        return mount_position

    def handle_alignment(self, data: bytes) -> AlignmentMode:
        return AlignmentMode.POLAR

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
        self.logger.info("Connect TMC2209 LX200")
        self._adapter.connect()
        self._initialize()
        self._is_connected = True
        self.sync_telescope_dec(LX200Dec.from_degrees(0))
        self.logger.info("TMC2209 LX200 connected")

    def stop(self):
        self.logger.info("Stop TMC2209 LX200")
        self._is_connected = False
        super().stop()
        self._adapter.close()
        self.logger.info("TMC2209 LX200 stopped")

    def get_telescope_raw_position(self) -> tuple[float, float]:
        return 0, int(self._adapter.status().position)

    def _steps_from_dec(self, position: LX200Dec) -> int:
        return int(round(position.to_degrees() * self.steps_per_degree))

    def _arcseconds_from_steps(self, steps: int) -> float:
        return (steps / self.steps_per_degree) * 3600

    def _dec_from_steps(self, steps: int) -> LX200Dec:
        return LX200Dec.from_arcseconds(self._arcseconds_from_steps(steps))

    def get_telescope_dec(self) -> LX200Dec:
        return LX200Dec.from_arcseconds(self._mount_position_raw)

    def get_telescope_ra(self) -> LX200Ha:
        return LX200Ha.from_hours(0)

    def sync_telescope_dec(self, position: LX200Dec) -> bool:
        self.logger.info("Sync DEC to %s", position)
        steps = self._steps_from_dec(position)
        self._adapter.set_position(steps)
        with self._position_update_lock:
            self._mount_position_raw = position.to_arcseconds()
            self._motor_position_raw = self._get_motor_raw_position()
            self._last_update_s = time.monotonic()
        self._current_track_rate_coef = self._DEFAULT_TRACKING_RATE
        self.logger.info("Sync DEC applied: steps=%s", steps)
        return True

    def sync_telescope_ra(self, position: LX200Ha) -> bool:
        return False

    def halt_all(self) -> bool:
        self.logger.info("Halt all DEC movements")
        self._current_track_rate_coef = self._DEFAULT_TRACKING_RATE
        self._adapter.halt()
        self.logger.info("Halt all DEC movements done")
        return True

    def slew_to_dec(self, position: LX200Dec) -> bool:
        self.logger.info("Start DEC GOTO to %s", position)
        current_position = int(self.get_telescope_raw_position()[1])
        target_steps = self._steps_from_dec(position)
        delta_steps = target_steps - current_position

        profile = self._goto_slow_profile
        if abs(delta_steps / self.steps_per_degree) > self.FAST_PROFILE_DELTA_DEG:
            profile = self._goto_fast_profile

        self.logger.info(
            "DEC GOTO details: current_steps=%s target_steps=%s delta_steps=%s profile=(microsteps=%s speed=%s accel=%s)",
            current_position,
            target_steps,
            delta_steps,
            profile.microsteps,
            profile.speed,
            profile.accel,
        )
        self._apply_profile(profile)
        self._current_track_rate_coef = self._DEFAULT_TRACKING_RATE
        
        self._adapter.set_target(target_steps)

        self._adapter.run()
        self.logger.info("DEC GOTO started: target_steps=%s", target_steps)
        
        return True

    def slew_to_ra(self, position: LX200Ha) -> bool:
        return False

    def get_site1_name(self) -> str:
        return "tmc2209"

    def get_distance(self) -> str:
        status = self._adapter.status()
        if status.phase not in ('idle', 'hold'):
            return "|"
        return ""

    def set_slew_to_find(self) -> bool:
        self.logger.info("Set slew mode to find")
        return True

    def move_east(self) -> bool:
        return False

    def move_north(self) -> bool:
        self._current_track_rate_coef = self._DEFAULT_TRACKING_RATE
        return self._start_manual_move(
            False,
        )

    def move_south(self) -> bool:
        self._current_track_rate_coef = self._DEFAULT_TRACKING_RATE
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
        self.logger.info("Guide north start")
        self._current_track_rate_coef = -1
        self._apply_profile(self._guide_profile)
        self._adapter.set_direction(False)
        self._adapter.run()
        self.logger.info("Guide north applied")
        return True

    def guide_south(self) -> bool:
        self.logger.info("Guide south start")
        self._current_track_rate_coef = 1
        self._apply_profile(self._guide_profile)
        self._adapter.set_direction(True)
        self._adapter.run()
        self.logger.info("Guide south applied")
        return True

    def guide_west(self) -> bool:
        return False

    def guide_reset(self) -> bool:
        self.logger.info("Guide reset")
        self._current_track_rate_coef = self._DEFAULT_TRACKING_RATE
        return self.halt_all()

    def _start_manual_move(
        self,
        direction: bool,
    ) -> bool:
        self.logger.info("Start manual DEC move: backward=%s", direction)
        self._apply_profile(self._slew_profile)
        self._adapter.set_direction(direction)
        result = self._adapter.run()
        self.logger.info("Manual DEC move started: backward=%s running=%s", direction, result)
        return result

    def _apply_profile(
        self, profile: SpeedProfile
    ) -> None:
        self.logger.info(
            "Apply profile: microsteps=%s speed=%s accel=%s",
            profile.microsteps,
            profile.speed,
            profile.accel,
        )
        self._refresh_microsteps(profile.microsteps)
        self._adapter.set_speed_sps(profile.speed)
        self._adapter.set_acceleration_steps_per_ms(profile.accel)
