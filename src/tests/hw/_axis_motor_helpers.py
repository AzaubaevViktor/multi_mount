import time

import pytest

from sky.axis import AxisDEC, AxisMotionMode, AxisRA
from sky.motor import MotorDirection


POLL_INTERVAL_S: float = 0.2


AxisAny = AxisRA | AxisDEC


def _wait_for_tracking_mode(axis: AxisAny, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if axis.mode() == AxisMotionMode.TRACK:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(
        f"Axis did not reach TRACK mode within {timeout_s}s: mode={axis.mode().value}"
    )


def _wait_for_motor_stop(axis: AxisAny, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if axis._motor.status().direction == MotorDirection.STOP:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("Motor did not stop in time")


def _wait_for_motor_running(axis: AxisAny, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if axis._motor.status().direction != MotorDirection.STOP:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("Motor did not start running in time")


def _wait_for_goto_done(axis: AxisAny, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    # Phase 1: wait for motion to actually start (command is async).
    while time.monotonic() < deadline:
        if axis.is_moving_to():
            break
        time.sleep(POLL_INTERVAL_S)
    else:
        pytest.fail("GOTO never started")

    # Phase 2: wait for motion to finish.
    while time.monotonic() < deadline:
        if not axis.is_moving_to():
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("GOTO did not complete in time")


def _measure_motor_speed_sps(axis: AxisAny, duration_s: float) -> float:
    steps1 = axis._motor.status().steps
    t1 = time.monotonic()
    time.sleep(duration_s)
    steps2 = axis._motor.status().steps
    t2 = time.monotonic()
    return abs(steps2 - steps1) / (t2 - t1)

