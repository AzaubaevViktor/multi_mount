from dataclasses import dataclass

from lx200.base import LX200DECHandler
from lx200.protocol import AlignmentMode
from sky.physics import Dec, DecPerSecond, Direction, Ha, Second
from tmc2209.tmc2209_adapter import (
    Phase,
    TMC2209Adapter,
)


@dataclass
class SpeedProfile:
    microsteps: int
    speed: int
    accel: int
    name: str

class TMC2209LX200(LX200DECHandler):
    COMPENSATE_MOTOR_SIGN = -1.0
    FAST_PROFILE_DELTA = Dec(1 * 60 * 60)

    # TODO: Fix microsteps
    _goto_fast_profile = SpeedProfile(
        microsteps=16, 
        speed=5000,
        accel=1000,
        name="goto_fast",
    )
    _goto_slow_profile = SpeedProfile(
        microsteps=16, 
        speed=1000,
        accel=1000,
        name="goto_slow",
    )
    _slew_profile = SpeedProfile(
        microsteps=16, 
        speed=1000,
        accel=1000,
        name="slew",
    )
    _guide_profile = SpeedProfile(
        microsteps=16, 
        speed=100,
        accel=10000,
        name="guide",
    )

    def __init__(
        self,
        adapter: TMC2209Adapter,
    ) -> None:
        self._adapter = adapter
        self._current_profile: SpeedProfile | None = None

        self._is_connected = False

        super().__init__()
        self._manual_slew_rate = self._adapter.dec_speed_from_sps(
            self._slew_profile.speed,
            self._slew_profile.microsteps,
        )

    def _is_motor_connected(self) -> bool:
        return self._is_connected

    def _get_motor_status(self):
        return self._adapter.status()

    def _get_motor_raw_position(self) -> Dec:
        return self.motor_position()[1]
    
    def _wrap_steps(self, motor_position_steps: int | float) -> int:
        wrapped_steps = (
            (float(motor_position_steps) + self._adapter.steps_per_rev / 2) % self._adapter.steps_per_rev
            - self._adapter.steps_per_rev / 2
        )
        return int(round(wrapped_steps))
    
    def _halt_motion(self):
        self.logger.info("Halt all DEC movements")
        self._adapter.halt()
        self.logger.info("Halt all DEC movements done")

    def _set_tracking_rate(self, rate: DecPerSecond) -> DecPerSecond | None:
        if rate == DecPerSecond(0):
            self.logger.info("Tracking stop")
            self._adapter.halt()
            return DecPerSecond(0)

        self.logger.info("Tracking start: speed=%s", rate)
        # TMC speed is unsigned; guide direction is controlled separately.
        rounded_rate = self._apply_profile(self._guide_profile, custom_speed=abs(rate))
        rate_sign = 1.0 if rate > DecPerSecond(0) else -1.0
        self._adapter.set_free_ride_mode()
        self._adapter.set_direction(rate > DecPerSecond(0))
        if rate != DecPerSecond(0):
            self._adapter.run()
        self.logger.info("Tracking applied: speed=%s rounded=%s", rate, rounded_rate)
        if rounded_rate is None:
            return None
        return DecPerSecond(float(rounded_rate) * rate_sign)

    def handle_alignment(self, data: bytes) -> AlignmentMode:
        return AlignmentMode.POLAR

    # Microsteps update logic

    def _set_microsteps(self, microsteps: int):
        with self._position_update_lock:
            # TODO: Set microsteps changes speed in arduino
            self._adapter.set_microsteps(microsteps)

            # Reset position when microsteps changed
            self._adapter.set_position(0)
            self._motor_position_raw = self._adapter.dec_from_motor_steps(0)
            self._last_update_s = Second.monotonic()

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
        self.sync_telescope_dec(Dec(0))
        self.resume_tracking()
        self.logger.info("TMC2209 LX200 connected")

    def stop(self):
        self._halt_motion()
        self.logger.info("Stop TMC2209 LX200")
        self._is_connected = False
        super().stop()
        self._adapter.close()
        self.logger.info("TMC2209 LX200 stopped")

    def motor_position(self) -> tuple[Ha, Dec]:
        return Ha(0), self._adapter.dec_from_motor_steps(self._adapter.status().position)

    def get_telescope_dec(self) -> Dec:
        return self._mount_position_raw

    def sync_telescope_dec(self, position: Dec) -> bool:
        self.logger.info("Sync DEC to %s", position)
        steps = self._adapter.motor_steps_from_dec(position)
        with self._position_update_lock:
            self._adapter.set_position(steps)

            self._mount_position_raw = position
            self._motor_position_raw = self._adapter.dec_from_motor_steps(steps)
            self._last_update_s = Second.monotonic()

        self.logger.info("Sync DEC applied: steps=%s position=%s", steps, position)
        return True

    def sync_telescope_ra(self, position: Ha) -> bool:
        return False

    def slew_to_dec(self, position: Dec) -> bool:
        self.logger.info("Start DEC GOTO to %s", position)
        current_mount_position = self._mount_position_raw
        delta = position - current_mount_position

        profile = self._goto_slow_profile
        if abs(delta) > self.FAST_PROFILE_DELTA:
            profile = self._goto_fast_profile

        self._apply_profile(profile)  # XXX: After this position and steps per rev will be changed

        delta_steps = self._wrap_steps(self._adapter.motor_steps_from_dec(delta))

        self.logger.info(
            "DEC GOTO details: %s -> %s; delta=%s (%s steps); profile=(microsteps=%s speed=%s accel=%s)",
            current_mount_position,
            position,
            delta,
            delta_steps,
            profile.microsteps,
            profile.speed,
            profile.accel,
        )

        self._adapter.set_target_mode()
        self._adapter.slew_delta(delta_steps)
        self._adapter.set_direction(delta_steps <= 0)

        self._adapter.run()
        self.logger.info("DEC GOTO started: delta_steps=%s", delta_steps)
        
        return True

    def get_site1_name(self) -> str:
        return "tmc2209"

    def get_distance(self) -> str:
        status = self._adapter.status()
        if status.phase not in (Phase.IDLE, Phase.HOLD):
            return "|"
        return ""

    def set_slew_to_find(self) -> bool:
        self._manual_slew_rate = self._adapter.dec_speed_from_sps(
            self._slew_profile.speed,
            self._slew_profile.microsteps,
        )
        self.logger.info("Set slew mode to find: speed=%s", self._manual_slew_rate)
        return True

    def move_east(self) -> bool:
        return False

    def move_north(self) -> bool:
        return self._start_manual_move(
            Direction.FORWARD,
        )

    def move_south(self) -> bool:
        return self._start_manual_move(
            Direction.BACKWARD,
        )

    def move_west(self) -> bool:
        return False

    def _start_manual_move(
        self,
        direction: Direction,
    ) -> bool:
        is_backward = direction == Direction.BACKWARD
        self.logger.info("Start manual DEC move: direction=%s speed=%s", direction, self._manual_slew_rate)
        self._apply_profile(self._slew_profile, custom_speed=abs(self._manual_slew_rate))
        self._adapter.set_free_ride_mode()
        self._adapter.set_direction(is_backward)
        result = self._adapter.run()
        self.logger.info(
            "Manual DEC move started: direction=%s backward=%s running=%s",
            direction,
            is_backward,
            result,
        )
        return result

    def _apply_profile(
        self,
        profile: SpeedProfile,
        custom_speed: DecPerSecond | None = None,
    ) -> DecPerSecond | None:
        target_speed = (
            self._adapter.dec_speed_from_sps(profile.speed, profile.microsteps)
            if custom_speed is None
            else custom_speed
        )
        new_speed_sps = (
            profile.speed
            if custom_speed is None
            else self._adapter.sps_from_dec_speed(custom_speed, profile.microsteps)
        )
        if new_speed_sps == 0:
            self.logger.info("Stop motor")
            self._adapter.halt()
            return None

        applied_speed = self._adapter.dec_speed_from_sps(new_speed_sps, profile.microsteps)

        if self._current_profile is not profile:
            self.logger.info(
                "Apply profile %s: microsteps=%s speed=%s -> %s accel=%s",
                profile.name,
                profile.microsteps,
                target_speed,
                applied_speed,
                profile.accel,
            )
            self._set_microsteps(profile.microsteps)
            self._adapter.set_acceleration_steps_per_ms(profile.accel)
        else:
            self.logger.info(
                "Update profile %s speed: speed=%s -> %s",
                profile.name,
                target_speed,
                applied_speed,
            )

        self._adapter.set_speed_sps(new_speed_sps)
        
        self._current_profile = profile

        return applied_speed
