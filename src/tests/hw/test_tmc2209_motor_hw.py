import time
from collections.abc import Iterator

import pytest
from serial.serialutil import SerialException

from sky.motor import MotionMode, MotorDirection, MotorStateError, MotorStopRequire
from serial_wrapper.wrapper import SerialLine
from tmc2209.motor import TMC2209MotorProtocolError
from tmc2209.motor import TMC2209Motor


DEVICE_PATTERN = r"^tty\.usbserial.*$"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT_S = 2.0
SERIAL_NAME = "tmc2209_motor"
CONNECT_ATTEMPTS = 3
READY_TIMEOUT_S = 10.0
POLL_INTERVAL_S = 0.05
RUN_TIMEOUT_S = 5.0
TARGET_TIMEOUT_S = 20.0
DEFAULT_ACCELERATION_SPS2 = 1_000


@pytest.fixture(scope="session")
def tmc2209_motor() -> Iterator[TMC2209Motor]:
    port = SerialLine.search(DEVICE_PATTERN)
    serial_line = SerialLine(
        port=port,
        baud=SERIAL_BAUD,
        timeout_s=SERIAL_TIMEOUT_S,
        name=SERIAL_NAME,
        terminator="\n",
    )
    motor: TMC2209Motor | None = None
    for attempt in range(CONNECT_ATTEMPTS):
        try:
            serial_line.connect()
            serial_line.reset()
            time.sleep(0.5)
            deadline = time.monotonic() + READY_TIMEOUT_S
            while time.monotonic() < deadline:
                try:
                    ready = serial_line.query(None, timeout=1).strip()
                except SerialException:
                    break
                if ready == "ready":
                    motor = TMC2209Motor(serial_line)
                    motor._is_connected = True
                    break
            if motor is not None:
                break
            else:
                raise TMC2209MotorProtocolError("device did not report ready in time")
        except (SerialException, TMC2209MotorProtocolError):
            try:
                serial_line.close()
            except Exception:
                pass
            if attempt == CONNECT_ATTEMPTS - 1:
                raise
            time.sleep(0.5)
        else:
            raise AssertionError("unreachable")
    if motor is None:
        raise AssertionError("motor fixture failed to initialize")
    try:
        yield motor
    finally:
        try:
            motor.reset()
        finally:
            motor.disconnect()


@pytest.fixture(autouse=True)
def _reset_motor_between_tests(tmc2209_motor: TMC2209Motor) -> Iterator[None]:
    tmc2209_motor.reset()
    tmc2209_motor.set_acceleration(DEFAULT_ACCELERATION_SPS2)
    tmc2209_motor.set_motion_mode(MotionMode.TARGET)
    tmc2209_motor.set_delta(0)
    tmc2209_motor.run()
    _wait_for_target_finish(tmc2209_motor, TARGET_TIMEOUT_S)
    tmc2209_motor.set_motion_mode(MotionMode.RUN)
    tmc2209_motor.set_steps(0)
    yield
    tmc2209_motor.reset()
    tmc2209_motor.set_acceleration(DEFAULT_ACCELERATION_SPS2)
    tmc2209_motor.set_motion_mode(MotionMode.TARGET)
    tmc2209_motor.set_delta(0)
    tmc2209_motor.run()
    _wait_for_target_finish(tmc2209_motor, TARGET_TIMEOUT_S)
    tmc2209_motor.set_motion_mode(MotionMode.RUN)
    tmc2209_motor.set_steps(0)


def _wait_for_position_change(
    motor: TMC2209Motor,
    start_position: int,
    direction: MotorDirection,
    timeout_s: float,
) -> int:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        position = motor.status().steps
        if direction == MotorDirection.FORWARD and position > start_position:
            return position
        if direction == MotorDirection.BACKWARD and position < start_position:
            return position
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("Motor did not start moving in time")


def _wait_for_target_finish(motor: TMC2209Motor, timeout_s: float) -> int:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = motor.status()
        if status.target is None and status.direction == MotorDirection.STOP:
            return status.steps
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("Target move did not finish in time")


def _wait_for_stable_run(
    motor: TMC2209Motor,
    speed_sps: int,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = motor.status()
        actual_speed_sps = abs(motor._status().actual_speed_sps)
        if (
            status.motion_mode == MotionMode.RUN
            and actual_speed_sps == pytest.approx(speed_sps, rel=SPEED_TOLERANCE_RATIO, abs=SPEED_TOLERANCE_ABS)
        ):
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("Motor did not reach stable run speed in time")


SET_POSITION_STEPS = 12_000


@pytest.mark.parametrize("position_steps", [-12_000, 0, 12_000])
def test_hw_set_steps_updates_position(
    tmc2209_motor: TMC2209Motor,
    position_steps: int,
) -> None:
    assert tmc2209_motor.set_steps(position_steps) is True

    status = tmc2209_motor.status()

    assert status.steps == position_steps
    assert status.direction == MotorDirection.STOP
    assert status.target is None


POSITION_TOLERANCE_STEPS = 200


@pytest.mark.parametrize(
    ("delta_steps", "target_speed_sps", "expected_direction"),
    [
        (2_000, 1_000, MotorDirection.FORWARD),
        (10_000, 2_500, MotorDirection.FORWARD),
        (25_000, 4_000, MotorDirection.FORWARD),
        (-2_000, 1_000, MotorDirection.BACKWARD),
        (-10_000, 2_500, MotorDirection.BACKWARD),
        (-25_000, 4_000, MotorDirection.BACKWARD),
    ],
)
def test_hw_target_move_reaches_requested_delta(
    tmc2209_motor: TMC2209Motor,
    delta_steps: int,
    target_speed_sps: int,
    expected_direction: MotorDirection,
) -> None:
    start_position = tmc2209_motor.status().steps

    assert tmc2209_motor.set_motion_mode(MotionMode.TARGET) is True
    assert tmc2209_motor.set_speed(target_speed_sps) == target_speed_sps
    assert tmc2209_motor.set_delta(delta_steps) is True
    assert tmc2209_motor.run() is True

    moving_position = _wait_for_position_change(
        tmc2209_motor,
        start_position=start_position,
        direction=expected_direction,
        timeout_s=RUN_TIMEOUT_S,
    )
    final_position = _wait_for_target_finish(tmc2209_motor, TARGET_TIMEOUT_S)
    final_status = tmc2209_motor.status()

    if expected_direction == MotorDirection.FORWARD:
        assert moving_position > start_position
    else:
        assert moving_position < start_position
    assert abs(final_position - (start_position + delta_steps)) <= POSITION_TOLERANCE_STEPS
    assert final_status.direction == MotorDirection.STOP
    assert final_status.target is None
    assert final_status.motion_mode == MotionMode.RUN


RUN_SETTLE_S = 0.3


@pytest.mark.parametrize(
    ("speed_sps", "direction", "expected_sign"),
    [
        (600, MotorDirection.FORWARD, 1),
        (1_200, MotorDirection.FORWARD, 1),
        (2_400, MotorDirection.FORWARD, 1),
        (600, MotorDirection.BACKWARD, -1),
        (1_200, MotorDirection.BACKWARD, -1),
        (2_400, MotorDirection.BACKWARD, -1),
    ],
)
def test_hw_run_mode_moves_in_requested_direction(
    tmc2209_motor: TMC2209Motor,
    speed_sps: int,
    direction: MotorDirection,
    expected_sign: int,
) -> None:
    start_position = tmc2209_motor.status().steps

    assert tmc2209_motor.set_motion_mode(MotionMode.RUN) is True
    assert tmc2209_motor.set_speed(speed_sps) == speed_sps
    assert tmc2209_motor.set_direction(direction) is True
    assert tmc2209_motor.run() is True

    moved_position = _wait_for_position_change(
        tmc2209_motor,
        start_position=start_position,
        direction=direction,
        timeout_s=RUN_TIMEOUT_S,
    )
    time.sleep(RUN_SETTLE_S)
    later_status = tmc2209_motor.status()

    assert later_status.motion_mode in {MotionMode.RUN, MotionMode.ACCELERATION, MotionMode.DECELERATION}
    assert later_status.speed_sps == speed_sps
    assert (later_status.steps - moved_position) * expected_sign > 0


SPEED_MEASURE_INTERVAL_S = 0.8
SPEED_TOLERANCE_RATIO = 0.2
SPEED_TOLERANCE_ABS = 120.0


@pytest.mark.parametrize(
    ("speed_sps", "direction", "expected_sign"),
    [
        (600, MotorDirection.FORWARD, 1),
        (1_200, MotorDirection.FORWARD, 1),
        (2_400, MotorDirection.FORWARD, 1),
        (600, MotorDirection.BACKWARD, -1),
        (1_200, MotorDirection.BACKWARD, -1),
        (2_400, MotorDirection.BACKWARD, -1),
    ],
)
def test_hw_run_speed_matches_requested_value_after_acceleration(
    tmc2209_motor: TMC2209Motor,
    speed_sps: int,
    direction: MotorDirection,
    expected_sign: int,
) -> None:
    start_position = tmc2209_motor.status().steps

    assert tmc2209_motor.set_motion_mode(MotionMode.RUN) is True
    assert tmc2209_motor.set_speed(speed_sps) == speed_sps
    assert tmc2209_motor.set_direction(direction) is True
    assert tmc2209_motor.run() is True

    _wait_for_position_change(
        tmc2209_motor,
        start_position=start_position,
        direction=direction,
        timeout_s=RUN_TIMEOUT_S,
    )
    _wait_for_stable_run(
        tmc2209_motor,
        speed_sps=speed_sps,
        timeout_s=RUN_TIMEOUT_S,
    )

    sample_start_steps = tmc2209_motor.status().steps
    sample_start_time = time.monotonic()
    time.sleep(SPEED_MEASURE_INTERVAL_S)
    sample_end_status = tmc2209_motor.status()
    measured_speed_sps = abs(sample_end_status.steps - sample_start_steps) / (time.monotonic() - sample_start_time)
    actual_speed_sps = abs(motor_status.actual_speed_sps) if (motor_status := tmc2209_motor._status()) else 0.0

    assert (sample_end_status.steps - sample_start_steps) * expected_sign > 0
    assert measured_speed_sps == pytest.approx(speed_sps, rel=SPEED_TOLERANCE_RATIO, abs=SPEED_TOLERANCE_ABS)
    assert actual_speed_sps == pytest.approx(speed_sps, rel=SPEED_TOLERANCE_RATIO, abs=SPEED_TOLERANCE_ABS)


@pytest.mark.parametrize("delta_steps", [0, 10, -10, 100, -100, 2_000, 10_000, -2_000, -10_000])
def test_hw_goto_mode_allows_only_status_and_position_updates(
    tmc2209_motor: TMC2209Motor,
    delta_steps: int,
) -> None:
    speed_sps = 50 if abs(delta_steps) <= 100 else 2_000
    acceleration_sps2 = 20 if abs(delta_steps) <= 10 else 100 if abs(delta_steps) <= 100 else DEFAULT_ACCELERATION_SPS2

    assert tmc2209_motor.set_motion_mode(MotionMode.TARGET) is True
    assert tmc2209_motor.set_acceleration(acceleration_sps2) is True
    assert tmc2209_motor.set_speed(speed_sps) == speed_sps
    assert tmc2209_motor.set_delta(delta_steps) is True
    assert tmc2209_motor.run() is True

    if delta_steps == 0:
        final_steps = _wait_for_target_finish(tmc2209_motor, TARGET_TIMEOUT_S)
        assert final_steps == 0
        assert tmc2209_motor.status().target is None
        return

    _wait_for_position_change(
        tmc2209_motor,
        start_position=0,
        direction=MotorDirection.FORWARD if delta_steps > 0 else MotorDirection.BACKWARD,
        timeout_s=RUN_TIMEOUT_S,
    )

    with pytest.raises(MotorStopRequire):
        tmc2209_motor.set_speed(1_500)
    with pytest.raises(MotorStopRequire):
        tmc2209_motor.set_direction(MotorDirection.FORWARD)
    with pytest.raises(MotorStopRequire):
        tmc2209_motor.set_delta(100)
    with pytest.raises(MotorStopRequire):
        tmc2209_motor.set_motion_mode(MotionMode.RUN)
    with pytest.raises(MotorStopRequire):
        tmc2209_motor.run()

    status = tmc2209_motor.status()
    assert status.motion_mode in {MotionMode.TARGET, MotionMode.ACCELERATION, MotionMode.DECELERATION, MotionMode.RUN}
    assert isinstance(status.steps, int)


@pytest.mark.parametrize(
    ("speed_sps", "direction"),
    [
        (600, MotorDirection.FORWARD),
        (1_200, MotorDirection.BACKWARD),
        (2_400, MotorDirection.FORWARD),
    ],
)
def test_hw_moving_motor_rejects_direction_and_microsteps_changes(
    tmc2209_motor: TMC2209Motor,
    speed_sps: int,
    direction: MotorDirection,
) -> None:
    assert tmc2209_motor.set_motion_mode(MotionMode.RUN) is True
    assert tmc2209_motor.set_speed(speed_sps) == speed_sps
    assert tmc2209_motor.set_direction(direction) is True
    assert tmc2209_motor.run() is True

    _wait_for_position_change(
        tmc2209_motor,
        start_position=0,
        direction=direction,
        timeout_s=RUN_TIMEOUT_S,
    )

    with pytest.raises(MotorStopRequire):
        tmc2209_motor.set_direction(MotorDirection.BACKWARD if direction == MotorDirection.FORWARD else MotorDirection.FORWARD)
    with pytest.raises(MotorStopRequire):
        tmc2209_motor.set_microsteps(32)


def test_hw_run_with_target_without_target_mode_raises_state_error(
    tmc2209_motor: TMC2209Motor,
) -> None:
    assert tmc2209_motor.set_motion_mode(MotionMode.RUN) is True
    assert tmc2209_motor.set_speed(1_200) == 1_200
    assert tmc2209_motor.set_delta(2_000) is True

    with pytest.raises(MotorStateError):
        tmc2209_motor.run()
