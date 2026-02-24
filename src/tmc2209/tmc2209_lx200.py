from dataclasses import dataclass
import time

from lx200.base import LX200DECHandler
from lx200.protocol import AlignmentMode
from lx200.protocols import LX200Dec, LX200Ha
from tmc2209.tmc2209_adapter import (
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


class TMC2209LX200(LX200DECHandler):
    FAST_PROFILE_DELTA_ARCSEC = 1 * 60 * 60

    # TODO: Fix microsteps
    _goto_fast_profile = SpeedProfile(
        microsteps=16, 
        speed=5000,
        accel=1000,
    )
    _goto_slow_profile = SpeedProfile(
        microsteps=16, 
        speed=1000,
        accel=1000,
    )
    _slew_profile = SpeedProfile(
        microsteps=16, 
        speed=1000,
        accel=1000,
    )
    _guide_profile = SpeedProfile(
        microsteps=16, 
        speed=100,
        accel=1000,
    )

    def __init__(
        self,
        adapter: TMC2209Adapter,
    ) -> None:
        self._adapter = adapter

        self._is_connected = False

        super().__init__()

    def _is_motor_connected(self) -> bool:
        return self._is_connected

    def _get_motor_status(self):
        return self._adapter.status()

    def _get_motor_raw_position(self) -> float:
        return self.motor_position()[1]

    def _get_default_tracking_speed(self) -> float:
        return self._arcseconds_from_steps(self._guide_profile.speed)

    def _wrap_mount_position(self, mount_position: float) -> float:
        # TODO: Wrap around mount position
        return mount_position
    
    def _wrap_steps(self, motor_position: float) -> float:
        return (motor_position + self._adapter.steps_per_rev / 2) % self._adapter.steps_per_rev - self._adapter.steps_per_rev / 2
    
    def _halt_motion(self):
        self.logger.info("Halt all DEC movements")
        self._adapter.halt()
        self.logger.info("Halt all DEC movements done")

    def _set_tracking_rate(self, rate: float):
        self.logger.info("Tracking start")
        self._apply_profile(self._guide_profile, custom_rate=rate)
        self._adapter.set_free_ride_mode()
        self._adapter.set_direction(rate > 0)
        self._adapter.run()
        self.logger.info("Tracking applied")

    def handle_alignment(self, data: bytes) -> AlignmentMode:
        return AlignmentMode.POLAR

    # Microsteps update logic

    @property
    def steps_per_arcsec(self):
        return self._adapter.steps_per_rev / (360 * 60 * 60)
    
    def _set_microsteps(self, microsteps: int):
        with self._position_update_lock:
            # TODO: Set microsteps changes speed in arduino
            self._adapter.set_microsteps(microsteps)

            # Reset position when microsteps changed
            self._adapter.set_position(0)
            self._motor_position_raw = self._arcseconds_from_steps(0)
            self._last_update_s = time.monotonic()

    def _initialize(self) -> None:
        status = self._adapter.status()
        self._apply_profile(self._slew_profile)
        self._adapter._set_param("irun", 1200)
        self._adapter._set_param("ihold", 200)
        self.logger.info("Status: %s", status)

    def connect(self) -> None:
        self.logger.info("Connect TMC2209 LX200")
        self._adapter.connect()
        self._initialize()
        self._is_connected = True
        self.sync_telescope_dec(LX200Dec.from_arcseconds(0))
        self.resume_tracking()
        self.logger.info("TMC2209 LX200 connected")

    def stop(self):
        self.logger.info("Stop TMC2209 LX200")
        self._is_connected = False
        super().stop()
        self._adapter.close()
        self.logger.info("TMC2209 LX200 stopped")

    def motor_position(self) -> tuple[float, float]:
        return 0, int(self._arcseconds_from_steps(self._adapter.status().position))

    def _steps_from_dec(self, position: LX200Dec) -> int:
        return self._steps_from_arcsec(position.to_arcseconds())
    
    def _steps_from_arcsec(self, arcsecs: float) -> int:
        return int(round(arcsecs * self.steps_per_arcsec))

    def _arcseconds_from_steps(self, steps: int) -> float:
        return (steps / self.steps_per_arcsec)

    def get_telescope_dec(self) -> LX200Dec:
        return LX200Dec.from_arcseconds(self._mount_position_raw)

    def sync_telescope_dec(self, position: LX200Dec) -> bool:
        self.logger.info("Sync DEC to %s", position)
        steps = self._steps_from_dec(position)
        with self._position_update_lock:
            self._adapter.set_position(steps)

            self._mount_position_raw = position.to_arcseconds()
            self._motor_position_raw = self._arcseconds_from_steps(steps)
            self._last_update_s = time.monotonic()

        self.logger.info("Sync DEC applied: steps=%s position=%s", steps, position)
        return True

    def sync_telescope_ra(self, position: LX200Ha) -> bool:
        return False

    def slew_to_dec(self, position: LX200Dec) -> bool:
        self.logger.info("Start DEC GOTO to %s", position)
        # with self._position_update_lock: ???

        current_mount_position_arcs = int(self._mount_position_raw)
        target_position_arcsec = int(position.to_arcseconds())
        delta_arcsec = target_position_arcsec - current_mount_position_arcs

        profile = self._goto_slow_profile
        if abs(delta_arcsec) > self.FAST_PROFILE_DELTA_ARCSEC:
            profile = self._goto_fast_profile

        self._apply_profile(profile)  # XXX: After this position and steps per rev will be changed

        delta_steps = round(self._wrap_steps(self._steps_from_arcsec(delta_arcsec)))

        self.logger.info(
            "DEC GOTO details: %s -> %s (%das -> %das); delta=%sas (%s steps); profile=(microsteps=%s speed=%s accel=%s)",
            LX200Dec.from_arcseconds(current_mount_position_arcs), position,
            current_mount_position_arcs, target_position_arcsec,
            delta_arcsec, delta_steps,
            profile.microsteps,
            profile.speed,
            profile.accel,
        )

        self._adapter.set_target_mode()
        self._adapter.slew_delta(delta_steps)
        if delta_steps > 0:
            self._adapter.set_direction(False)
        else:
            self._adapter.set_direction(True)

        self._adapter.run()
        self.logger.info("DEC GOTO started: delta_steps=%s", delta_steps)
        
        return True

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
        return self._start_manual_move(
            False,
        )

    def move_south(self) -> bool:
        return self._start_manual_move(
            True,
        )

    def move_west(self) -> bool:
        return False

    def _start_manual_move(
        self,
        direction: bool,
    ) -> bool:
        self.logger.info("Start manual DEC move: backward=%s", direction)
        self._apply_profile(self._slew_profile)
        self._adapter.set_free_ride_mode()
        self._adapter.set_direction(direction)
        result = self._adapter.run()
        self.logger.info("Manual DEC move started: backward=%s running=%s", direction, result)
        return result

    def _apply_profile(
        self, profile: SpeedProfile, custom_rate: float = 1
    ) -> None:
        self.logger.info(
            "Apply profile: microsteps=%s speed=%s accel=%s",
            profile.microsteps,
            profile.speed,
            profile.accel,
        )
        self._set_microsteps(profile.microsteps)
        self._adapter.set_acceleration_steps_per_ms(profile.accel)
        self._adapter.set_speed_sps(int(profile.speed * custom_rate))
