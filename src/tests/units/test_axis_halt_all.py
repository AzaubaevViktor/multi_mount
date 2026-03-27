import time

from sky.axis import AxisMotionMode, AxisRA, PointCoordinates
from sky.motor import MotionMode, MotorDirection, MotorStatus
from sky.physics import Dec, Ha, HaPerSecond, SkyDirection


class _StubMotor:
    FORWARD_POSITION_SIGN = 1

    def __init__(self) -> None:
        self._status = MotorStatus(
            is_connected=False,
            steps=0,
            motion_mode=MotionMode.IDLE,
            speed_sps=0,
            accel_sps=None,
            direction=MotorDirection.STOP,
            target=None,
            microsteps=None,
        )

    def connect(self):
        self._status.is_connected = True

    def disconnect(self) -> bool:
        self._status.is_connected = False
        return True

    def status(self) -> MotorStatus:
        return self._status

    def get_power_v(self) -> float | None:
        return None

    def set_steps(self, steps: int) -> bool:
        self._status.steps = steps
        return True

    def set_speed(self, steps_per_second: int) -> int:
        self._status.speed_sps = steps_per_second
        return steps_per_second

    def set_acceleration(self, steps_per_second_square: float) -> bool:
        del steps_per_second_square
        return True

    def set_direction(self, direction: MotorDirection) -> bool:
        self._status.direction = direction
        return True

    def set_delta(self, delta_steps: int) -> bool:
        self._status.target = self._status.steps + delta_steps
        return True

    def get_speed_sps_by_delta(self, delta_steps: int) -> int:
        return max(1, abs(delta_steps))

    def get_speed_by_speed_sps(self, speed_sps: int):
        return HaPerSecond(speed_sps)

    def set_motion_mode(self, motion_mode: MotionMode) -> bool:
        self._status.motion_mode = motion_mode
        return True

    def set_microsteps(self, microsteps: int) -> bool:
        self._status.microsteps = microsteps
        return True

    def convert_position_to_steps(self, position: Ha) -> int:
        return int(float(position))

    def convert_steps_to_position(self, steps: int) -> Ha:
        return Ha(steps)

    def convert_speed_to_steps_per_second(self, speed: HaPerSecond) -> int:
        return int(abs(float(speed)))

    def run(self) -> bool:
        if self._status.target is not None:
            self._status.motion_mode = MotionMode.TARGET
        else:
            self._status.motion_mode = MotionMode.RUN
        return True

    def stop(self) -> bool:
        self._status.motion_mode = MotionMode.IDLE
        self._status.speed_sps = 0
        self._status.direction = MotorDirection.STOP
        self._status.target = None
        return True

    def wait_till_stop(self, do_stop: bool = True, timeout_s: float | None = None) -> None:
        del timeout_s
        if do_stop:
            self.stop()

    def reset(self) -> None:
        self.stop()


def test_axis_halt_all_clears_goto_and_returns_to_tracking() -> None:
    axis = AxisRA(_StubMotor())  # type: ignore[arg-type]
    axis.connect()

    try:
        axis.change_speed(SkyDirection.EAST, HaPerSecond(10), update_sky_speed=True)

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if axis.mode() == AxisMotionMode.TRACK:
                break
            time.sleep(0.01)
        else:
            raise AssertionError("axis did not enter tracking mode")

        axis.goto_to(PointCoordinates(ra=Ha(120), dec=Dec(0)))

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if axis.is_moving_to():
                break
            time.sleep(0.01)
        else:
            raise AssertionError("axis did not enter goto mode")

        axis.halt_all()

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if axis.mode() == AxisMotionMode.TRACK and axis.is_moving_to() is False:
                break
            time.sleep(0.01)
        else:
            raise AssertionError("axis did not return to tracking mode")

        assert axis._goto_target is None
        assert axis._goto_direction is None
    finally:
        axis.disconnect()
