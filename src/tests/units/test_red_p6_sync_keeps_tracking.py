"""Regression guard for П6 (CM SYNC dropped tracking), PLAN.md §1 П6.

The sync bug: ``CM SYNC`` during tracking left the motor in ``mode=idle`` while
the axis logically believed it was tracking (``return to tracking: False`` in
sync_debug.log). The fix (``_do_resume_to_tracking`` / axis.py) makes the axis
resume tracking after ``SET_POSITION``. This test guards against a regression.

NOTE: П6 is already fixed in production code, so this test is expected to be
GREEN. It stays as a regression guard, not a red-first test.
"""

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
        return True

    def set_direction(self, direction: MotorDirection) -> bool:
        self._status.direction = direction
        return True

    def set_delta(self, delta_steps: int) -> bool:
        self._status.target = self._status.steps + delta_steps
        return True

    def get_speed_sps_by_delta(self, delta_steps: int) -> int:
        return max(1, abs(delta_steps))

    def get_speed_by_speed_sps(self, speed_sps: int) -> HaPerSecond:
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
        if do_stop:
            self.stop()

    def reset(self) -> None:
        self.stop()


def _wait_for(predicate, timeout_s: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_sync_during_tracking_keeps_motor_running() -> None:
    """П6: SET_POSITION (CM SYNC) while tracking must resume to run/tracking.

    See PLAN.md §1 П6 / §3 item 5.
    """
    motor = _StubMotor()
    axis = AxisRA(motor)  # type: ignore[arg-type]
    axis.connect()

    try:
        # Enter tracking.
        axis.change_speed(SkyDirection.EAST, HaPerSecond(10), update_sky_speed=True)
        assert _wait_for(lambda: axis.mode() == AxisMotionMode.TRACK), (
            "axis did not enter tracking mode before sync"
        )

        # CM SYNC arrives as a SET_POSITION during tracking.
        axis.set_position(PointCoordinates(ra=Ha(3600), dec=Dec(0)))

        # After sync the axis must be back in tracking with the motor running.
        assert _wait_for(lambda: axis.mode() == AxisMotionMode.TRACK), (
            "axis did not return to tracking mode after CM SYNC"
        )
        assert motor.status().motion_mode == MotionMode.RUN, (
            f"motor left in {motor.status().motion_mode} after sync instead of RUN "
            f"(regression of the sync bug)"
        )
    finally:
        axis.disconnect()
