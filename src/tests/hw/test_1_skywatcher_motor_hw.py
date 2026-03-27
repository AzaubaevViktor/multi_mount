import time
from collections.abc import Iterator

import pytest

from sky.motor import MotionMode, MotorDirection, MotorStateError, MotorStopRequire
from serial_wrapper.wrapper import SerialLine
from sky.constants import STELLAR_SPEED
from sky.physics import HaPerSecond
from skywatcher.motor import SkyWatcherMotor


DEVICE_PATTERN = "PL2303G-USBtoUART"
SERIAL_BAUD = 112500
SERIAL_TIMEOUT_S = 0.2
SERIAL_NAME = "skywatcher_motor"
POLL_INTERVAL_S = 0.1
RUN_TIMEOUT_S = 8.0
TARGET_TIMEOUT_S = 30.0


@pytest.fixture(scope="module")
def skywatcher_motor() -> Iterator[SkyWatcherMotor]:
    serial_line = SerialLine(
        port=SerialLine.search(DEVICE_PATTERN),
        baud=SERIAL_BAUD,
        timeout_s=SERIAL_TIMEOUT_S,
        name=SERIAL_NAME,
        terminator="\r",
    )
    motor = SkyWatcherMotor(serial_line)
    motor.connect()
    try:
        yield motor
    finally:
        try:
            motor.reset()
        finally:
            motor.disconnect()


@pytest.fixture(autouse=True)
def _reset_motor_between_tests(skywatcher_motor: SkyWatcherMotor) -> Iterator[None]:
    skywatcher_motor.wait_till_stop(do_stop=True, timeout_s=5.0)
    skywatcher_motor.reset()
    skywatcher_motor.set_motion_mode(MotionMode.RUN)
    skywatcher_motor.set_steps(0)
    yield
    skywatcher_motor.wait_till_stop(do_stop=True, timeout_s=5.0)
    skywatcher_motor.reset()
    skywatcher_motor.set_motion_mode(MotionMode.RUN)
    skywatcher_motor.set_steps(0)


def _wait_for_position_change(
    motor: SkyWatcherMotor,
    start_position: int,
    direction: MotorDirection,
    timeout_s: float,
) -> int:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        position = motor.status().steps
        signed_delta = position - start_position
        if signed_delta > motor._steps_360 // 2:
            signed_delta -= motor._steps_360
        if signed_delta < -(motor._steps_360 // 2):
            signed_delta += motor._steps_360
        if direction == MotorDirection.FORWARD and signed_delta > 0:
            return position
        if direction == MotorDirection.BACKWARD and signed_delta < 0:
            return position
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("Motor did not start moving in time")


def _wait_for_stop(motor: SkyWatcherMotor, timeout_s: float) -> int:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = motor.status()
        if status.direction == MotorDirection.STOP:
            return status.steps
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("Motor did not stop in time")


SET_POSITION_STEPS = 10_000


@pytest.mark.parametrize("position_steps", [-SET_POSITION_STEPS, 0, SET_POSITION_STEPS])
def test_hw_set_steps_updates_position(
    skywatcher_motor: SkyWatcherMotor,
    position_steps: int,
) -> None:
    assert skywatcher_motor.set_steps(position_steps) is True

    status = skywatcher_motor.status()

    assert status.steps == position_steps
    assert status.direction == MotorDirection.STOP
    assert status.target is None


POSITION_TOLERANCE_STEPS = 64


@pytest.mark.parametrize(
    ("delta_steps", "expected_direction"),
    [
        (0, MotorDirection.STOP),
        (10_000, MotorDirection.FORWARD),
        (-10_000, MotorDirection.BACKWARD),
        (50_000, MotorDirection.FORWARD),
        (-50_000, MotorDirection.BACKWARD),
        (200_000, MotorDirection.FORWARD),
        (-200_000, MotorDirection.BACKWARD),
    ],
)
def test_hw_target_move_reaches_requested_delta(
    skywatcher_motor: SkyWatcherMotor,
    delta_steps: int,
    expected_direction: MotorDirection,
) -> None:
    start_position = skywatcher_motor.status().steps

    assert skywatcher_motor.set_delta(delta_steps) is True
    assert skywatcher_motor.run() is True

    if delta_steps != 0:
        moved_position = _wait_for_position_change(
            skywatcher_motor,
            start_position=start_position,
            direction=expected_direction,
            timeout_s=RUN_TIMEOUT_S,
        )
        signed_delta = moved_position - start_position
        if signed_delta > skywatcher_motor._steps_360 // 2:
            signed_delta -= skywatcher_motor._steps_360
        if signed_delta < -(skywatcher_motor._steps_360 // 2):
            signed_delta += skywatcher_motor._steps_360
        if expected_direction == MotorDirection.FORWARD:
            assert signed_delta > 0
        else:
            assert signed_delta < 0

    final_position = _wait_for_stop(skywatcher_motor, TARGET_TIMEOUT_S)
    final_status = skywatcher_motor.status()

    signed_final_delta = final_position - start_position
    if signed_final_delta > skywatcher_motor._steps_360 // 2:
        signed_final_delta -= skywatcher_motor._steps_360
    if signed_final_delta < -(skywatcher_motor._steps_360 // 2):
        signed_final_delta += skywatcher_motor._steps_360
    assert abs(signed_final_delta - delta_steps) <= POSITION_TOLERANCE_STEPS
    assert final_status.direction == MotorDirection.STOP
    assert final_status.motion_mode == MotionMode.IDLE
    assert final_status.target is None


@pytest.mark.parametrize(
    ("delta_steps", "expected_direction"),
    [
        (10_000, MotorDirection.FORWARD),
        (-10_000, MotorDirection.BACKWARD),
    ],
)
def test_hw_goto_shows_target_motion_mode_while_moving(
    skywatcher_motor: SkyWatcherMotor,
    delta_steps: int,
    expected_direction: MotorDirection,
) -> None:
    assert skywatcher_motor.set_delta(delta_steps) is True
    assert skywatcher_motor.run() is True

    _wait_for_position_change(
        skywatcher_motor,
        start_position=0,
        direction=expected_direction,
        timeout_s=RUN_TIMEOUT_S,
    )

    assert skywatcher_motor.status().motion_mode == MotionMode.TARGET

    _wait_for_stop(skywatcher_motor, TARGET_TIMEOUT_S)


RUN_SETTLE_S = 0.5


@pytest.mark.parametrize(
    ("speed_sps", "direction"),
    [
        (32, MotorDirection.FORWARD),
        (128, MotorDirection.FORWARD),
        (512, MotorDirection.FORWARD),
        (32, MotorDirection.BACKWARD),
        (128, MotorDirection.BACKWARD),
        (512, MotorDirection.BACKWARD),
    ],
)
def test_hw_run_mode_moves_in_requested_direction(
    skywatcher_motor: SkyWatcherMotor,
    speed_sps: int,
    direction: MotorDirection,
) -> None:
    start_position = skywatcher_motor.status().steps

    assert skywatcher_motor.set_motion_mode(MotionMode.RUN) is True
    assert skywatcher_motor.set_speed(speed_sps) == speed_sps
    assert skywatcher_motor.set_direction(direction) is True
    assert skywatcher_motor.run() is True

    moved_position = _wait_for_position_change(
        skywatcher_motor,
        start_position=start_position,
        direction=direction,
        timeout_s=RUN_TIMEOUT_S,
    )
    time.sleep(RUN_SETTLE_S)
    later_status = skywatcher_motor.status()

    assert later_status.motion_mode == MotionMode.RUN
    assert later_status.speed_sps == speed_sps
    assert later_status.direction == direction
    signed_delta = later_status.steps - moved_position
    if signed_delta > skywatcher_motor._steps_360 // 2:
        signed_delta -= skywatcher_motor._steps_360
    if signed_delta < -(skywatcher_motor._steps_360 // 2):
        signed_delta += skywatcher_motor._steps_360
    assert signed_delta > 0 if direction == MotorDirection.FORWARD else signed_delta < 0


def test_hw_set_steps_while_running_raises_stop_require(
    skywatcher_motor: SkyWatcherMotor,
) -> None:
    assert skywatcher_motor.set_motion_mode(MotionMode.RUN) is True
    assert skywatcher_motor.set_speed(128) == 128
    assert skywatcher_motor.set_direction(MotorDirection.FORWARD) is True
    assert skywatcher_motor.run() is True

    _wait_for_position_change(
        skywatcher_motor,
        start_position=0,
        direction=MotorDirection.FORWARD,
        timeout_s=RUN_TIMEOUT_S,
    )

    with pytest.raises(MotorStopRequire, match="cannot change steps while motor is moving"):
        skywatcher_motor.set_steps(1000)


@pytest.mark.parametrize("requested_speed_sps", [32.4, 32.6, 127.6])
def test_hw_set_speed_rounds_fractional_value(
    skywatcher_motor: SkyWatcherMotor,
    requested_speed_sps: float,
) -> None:
    assert skywatcher_motor.set_motion_mode(MotionMode.RUN) is True

    actual_speed_sps = skywatcher_motor.set_speed(requested_speed_sps)
    status = skywatcher_motor.status()

    assert actual_speed_sps == status.speed_sps


SPEED_MEASURE_INTERVAL_S = 1.0
SPEED_TOLERANCE_RATIO = 0.35
SPEED_TOLERANCE_ABS = 16.0


@pytest.mark.parametrize(
    ("speed_sps", "direction"),
    [
        (32, MotorDirection.FORWARD),
        (128, MotorDirection.FORWARD),
        (512, MotorDirection.FORWARD),
        (32, MotorDirection.BACKWARD),
        (128, MotorDirection.BACKWARD),
        (512, MotorDirection.BACKWARD),
    ],
)
def test_hw_run_speed_matches_requested_value(
    skywatcher_motor: SkyWatcherMotor,
    speed_sps: int,
    direction: MotorDirection,
) -> None:
    assert skywatcher_motor.set_motion_mode(MotionMode.RUN) is True
    assert skywatcher_motor.set_speed(speed_sps) == speed_sps
    assert skywatcher_motor.set_direction(direction) is True
    assert skywatcher_motor.run() is True

    _wait_for_position_change(
        skywatcher_motor,
        start_position=0,
        direction=direction,
        timeout_s=RUN_TIMEOUT_S,
    )
    time.sleep(RUN_SETTLE_S)

    sample_start_steps = skywatcher_motor.status().steps
    sample_start_time = time.monotonic()
    time.sleep(SPEED_MEASURE_INTERVAL_S)
    sample_end_status = skywatcher_motor.status()
    measured_steps = sample_end_status.steps - sample_start_steps
    if measured_steps > skywatcher_motor._steps_360 // 2:
        measured_steps -= skywatcher_motor._steps_360
    if measured_steps < -(skywatcher_motor._steps_360 // 2):
        measured_steps += skywatcher_motor._steps_360
    measured_speed_sps = abs(measured_steps) / (time.monotonic() - sample_start_time)

    assert sample_end_status.direction == direction
    assert measured_steps > 0 if direction == MotorDirection.FORWARD else measured_steps < 0
    assert sample_end_status.speed_sps == speed_sps
    assert measured_speed_sps == pytest.approx(speed_sps, rel=SPEED_TOLERANCE_RATIO, abs=SPEED_TOLERANCE_ABS)


STEADY_STATE_SETTLE_S = 3.0
STEADY_STATE_MEASURE_INTERVAL_S = 2.0
STEADY_STATE_TOLERANCE_RATIO = 0.15
STEADY_STATE_TOLERANCE_ABS = 128.0


@pytest.mark.parametrize(
    ("rate_name", "rate_x_sidereal"),
    [
        ("sidereal", 1.0),
        ("log_3p44", 140.0 ** 0.25),
        ("log_11p83", 140.0 ** 0.5),
        ("log_40p67", 140.0 ** 0.75),
        ("x75", 75.0),
        ("x80", 80.0),
        ("find", 140.0),
        ("max", 800.0),
    ],
)
def test_hw_run_speed_matches_effective_speed_for_skywatcher_rates(
    skywatcher_motor: SkyWatcherMotor,
    rate_name: str,
    rate_x_sidereal: float,
) -> None:
    requested_speed_sps = skywatcher_motor.convert_speed_to_steps_per_second(
        HaPerSecond(rate_x_sidereal * float(STELLAR_SPEED))
    )

    assert skywatcher_motor.set_motion_mode(MotionMode.RUN) is True
    effective_speed_sps = skywatcher_motor.set_speed(requested_speed_sps)
    assert skywatcher_motor.set_direction(MotorDirection.FORWARD) is True
    assert skywatcher_motor.run() is True

    _wait_for_position_change(
        skywatcher_motor,
        start_position=0,
        direction=MotorDirection.FORWARD,
        timeout_s=RUN_TIMEOUT_S,
    )
    time.sleep(STEADY_STATE_SETTLE_S)

    sample_start_steps = skywatcher_motor.status().steps
    sample_start_time = time.monotonic()
    time.sleep(STEADY_STATE_MEASURE_INTERVAL_S)
    sample_end_status = skywatcher_motor.status()
    measured_steps = sample_end_status.steps - sample_start_steps
    if measured_steps > skywatcher_motor._steps_360 // 2:
        measured_steps -= skywatcher_motor._steps_360
    if measured_steps < -(skywatcher_motor._steps_360 // 2):
        measured_steps += skywatcher_motor._steps_360
    measured_speed_sps = abs(measured_steps) / (time.monotonic() - sample_start_time)

    assert sample_end_status.direction == MotorDirection.FORWARD
    assert measured_steps > 0, f"{rate_name} did not move forward"
    assert sample_end_status.speed_sps == effective_speed_sps
    assert measured_speed_sps == pytest.approx(
        effective_speed_sps,
        rel=STEADY_STATE_TOLERANCE_RATIO,
        abs=STEADY_STATE_TOLERANCE_ABS,
    )


@pytest.mark.parametrize("delta_steps", [0, 10_000, -10_000, 50_000, -50_000, 200_000, -200_000])
def test_hw_goto_mode_allows_only_status_and_position_updates(
    skywatcher_motor: SkyWatcherMotor,
    delta_steps: int,
) -> None:
    assert skywatcher_motor.set_delta(delta_steps) is True
    assert skywatcher_motor.run() is True

    if delta_steps != 0:
        _wait_for_position_change(
            skywatcher_motor,
            start_position=0,
            direction=MotorDirection.FORWARD if delta_steps > 0 else MotorDirection.BACKWARD,
            timeout_s=RUN_TIMEOUT_S,
        )

        with pytest.raises(MotorStopRequire):
            skywatcher_motor.set_speed(64)
        with pytest.raises(MotorStopRequire):
            skywatcher_motor.set_direction(MotorDirection.FORWARD)
        with pytest.raises(MotorStopRequire):
            skywatcher_motor.set_delta(100)
        with pytest.raises(MotorStopRequire):
            skywatcher_motor.set_motion_mode(MotionMode.RUN)
        with pytest.raises(MotorStopRequire):
            skywatcher_motor.run()

    status = skywatcher_motor.status()
    assert isinstance(status.steps, int)
    assert status.motion_mode in {MotionMode.TARGET, MotionMode.IDLE}


@pytest.mark.parametrize(
    ("speed_sps", "direction"),
    [
        (32, MotorDirection.FORWARD),
        (128, MotorDirection.BACKWARD),
        (512, MotorDirection.FORWARD),
    ],
)
def test_hw_moving_motor_rejects_direction_and_microsteps_changes(
    skywatcher_motor: SkyWatcherMotor,
    speed_sps: int,
    direction: MotorDirection,
) -> None:
    assert skywatcher_motor.set_motion_mode(MotionMode.RUN) is True
    assert skywatcher_motor.set_speed(speed_sps) == speed_sps
    assert skywatcher_motor.set_direction(direction) is True
    assert skywatcher_motor.run() is True

    _wait_for_position_change(
        skywatcher_motor,
        start_position=0,
        direction=direction,
        timeout_s=RUN_TIMEOUT_S,
    )

    with pytest.raises(MotorStopRequire):
        skywatcher_motor.set_direction(MotorDirection.BACKWARD if direction == MotorDirection.FORWARD else MotorDirection.FORWARD)
    with pytest.raises(MotorStopRequire):
        skywatcher_motor.set_microsteps(16)


def test_hw_run_with_target_without_target_mode_raises_state_error(
    skywatcher_motor: SkyWatcherMotor,
) -> None:
    assert skywatcher_motor.set_delta(100) is True
    assert skywatcher_motor.set_motion_mode(MotionMode.RUN) is True

    with pytest.raises(MotorStateError):
        skywatcher_motor.run()
