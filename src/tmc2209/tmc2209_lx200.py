import logging
import threading

from lx200.base import LX200Base
from lx200.protocols import LX200Dec, SECONDS_PER_DEGREE
from tmc2209.tmc2209_adapter import (
    DEGREES_PER_REV,
    GEAR_RATIO_1,
    GEAR_RATIO_2,
    MAX_ACCEL_STEPS_PER_MS,
    MAX_SPEED_SPS,
    MICROSTEPS_ALLOWED,
    STEPS_PER_REV,
    TMC2209Adapter,
)


class TMC2209LX200Error(Exception):
    pass


class TMC2209LX200ConfigError(TMC2209LX200Error):
    pass


class TMC2209LX200(LX200Base):
    def __init__(
        self,
        adapter: TMC2209Adapter,
        microsteps: int,
        speed_sps: int | None = None,
        accel_steps_per_ms: int | None = None,
    ) -> None:
        if adapter is None:
            raise TMC2209LX200ConfigError("adapter is required")
        if microsteps not in MICROSTEPS_ALLOWED:
            raise TMC2209LX200ConfigError(f"microsteps not allowed: {microsteps!r}")

        steps_per_rev = STEPS_PER_REV * microsteps * GEAR_RATIO_1 * GEAR_RATIO_2
        if steps_per_rev <= 0:
            raise TMC2209LX200ConfigError("steps per revolution must be positive")

        self._steps_per_degree = steps_per_rev / DEGREES_PER_REV
        if self._steps_per_degree <= 0:
            raise TMC2209LX200ConfigError("steps per degree must be positive")

        self._arcseconds_per_step = SECONDS_PER_DEGREE / self._steps_per_degree

        self._adapter = adapter
        self._microsteps = microsteps
        self._max_speed_sps = MAX_SPEED_SPS if speed_sps is None else speed_sps
        self._accel_steps_per_ms = (
            MAX_ACCEL_STEPS_PER_MS if accel_steps_per_ms is None else accel_steps_per_ms
        )
        self._manual_speed_sps = self._max_speed_sps
        self._guide_speed_sps = self._max_speed_sps

        self._dec_steps = 0
        self._goto_to: LX200Dec | None = None
        self._goto_steps: int | None = None
        self._state_lock = threading.Lock()

        self.logger = logging.getLogger("TMC2209LX200")

    def _initialize(self) -> None:
        self._adapter.set_param("microsteps", self._microsteps)
        self._adapter.set_speed_sps(self._max_speed_sps)
        self._adapter.set_acceleration_steps_per_ms(self._accel_steps_per_ms)
        self._adapter.set_enabled(True)
        status = self._adapter.status()
        with self._state_lock:
            self._dec_steps = status.position

    def connect(self) -> None:
        self._adapter.connect()
        self._initialize()

    def get_telescope_dec(self) -> LX200Dec:
        self._sync_status()
        with self._state_lock:
            steps = self._dec_steps
        return self._dec_from_steps(steps)

    def set_telescope_dec(self, position: LX200Dec) -> bool:
        steps = self._steps_from_dec(position)
        self._adapter.set_position(steps)
        with self._state_lock:
            self._dec_steps = steps
            self._goto_to = None
            self._goto_steps = None
        return True

    def halt_all(self) -> bool:
        self._adapter.stop()
        with self._state_lock:
            self._goto_to = None
            self._goto_steps = None
        return True

    def slew_to_dec(self, position: LX200Dec) -> bool:
        target_steps = self._steps_from_dec(position)
        returned_target, target_set = self._adapter.set_target(target_steps)
        if not target_set:
            with self._state_lock:
                self._goto_to = None
                self._goto_steps = None
            return True

        self._adapter.run()
        with self._state_lock:
            self._goto_to = position
            self._goto_steps = returned_target
        return True

    def get_site1_name(self) -> str:
        return "tmc2209"

    def get_distance(self) -> str:
        with self._state_lock:
            has_goto = self._goto_to is not None
        if not has_goto:
            return ""

        status = self._adapter.status()
        if not status.target_set:
            with self._state_lock:
                self._goto_to = None
                self._goto_steps = None
            return ""
        return "|"

    def set_slew_to_find(self) -> bool:
        self._manual_speed_sps = self._max_speed_sps
        return True

    def move_east(self) -> bool:
        return False

    def move_north(self) -> bool:
        return self._start_manual_move(False, self._manual_speed_sps)

    def move_south(self) -> bool:
        return self._start_manual_move(True, self._manual_speed_sps)

    def move_west(self) -> bool:
        return False

    def halt_east(self) -> bool:
        return False

    def halt_north(self) -> bool:
        return self._stop_manual_move()

    def halt_south(self) -> bool:
        return self._stop_manual_move()

    def halt_west(self) -> bool:
        return False

    def guide_east(self) -> bool:
        return False

    def guide_north(self) -> bool:
        return self._start_manual_move(False, self._guide_speed_sps)

    def guide_south(self) -> bool:
        return self._start_manual_move(True, self._guide_speed_sps)

    def guide_west(self) -> bool:
        return False

    def guide_reset(self) -> bool:
        return self._stop_manual_move()

    def _start_manual_move(self, direction: bool, speed_sps: int) -> bool:
        with self._state_lock:
            self._goto_to = None
            self._goto_steps = None
        self._adapter.set_speed_sps(speed_sps)
        self._adapter.set_direction(direction)
        return self._adapter.run()

    def _stop_manual_move(self) -> bool:
        self._adapter.stop()
        return True

    def _sync_status(self) -> None:
        status = self._adapter.status()
        with self._state_lock:
            self._dec_steps = status.position
            if self._goto_to is not None and not status.target_set:
                self._goto_to = None
                self._goto_steps = None

    def _steps_from_dec(self, position: LX200Dec) -> int:
        return int(round(position.to_degrees() * self._steps_per_degree))

    def _dec_from_steps(self, steps: int) -> LX200Dec:
        total_arcseconds = int(round(steps * self._arcseconds_per_step))
        return LX200Dec.from_degrees(total_arcseconds / SECONDS_PER_DEGREE)
