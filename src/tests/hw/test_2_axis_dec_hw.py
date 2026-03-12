import time
from collections.abc import Iterator

import pytest
from serial.serialutil import SerialException

from serial_wrapper.wrapper import SerialLine
from sky.axis import AxisMotionMode, AxisDEC, PointCoordinates
from sky.motor import MotorDirection
from sky.physics import Dec, DecPerSecond, Ha, SkyDirection
from tmc2209.motor import TMC2209Motor, TMC2209MotorProtocolError


DEVICE_PATTERN = r"^tty\.usbserial.*$"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT_S = 2.0
SERIAL_NAME = "tmc2209_axis_dec"
CONNECT_ATTEMPTS = 3
READY_TIMEOUT_S = 10.0

POLL_INTERVAL_S = 0.2
COMMAND_PROCESS_TIMEOUT_S = 5.0
MOTOR_STOP_TIMEOUT_S = 10.0
GOTO_TIMEOUT_S = 60.0
GOTO_POSITION_TOLERANCE_AS = 15.0
DEC_CHANGE_THRESHOLD_AS = 0.5
POSITION_SET_TOLERANCE_AS = 2.0
DRIFT_TOLERANCE_AS = 5.0
SLEW_SPEED = DecPerSecond(20)
SLEW_OBSERVE_S = 3.0
DEFAULT_START_DEC = Dec(0)
GOTO_START = DEFAULT_START_DEC
DEC_TRACK_SPEED = DecPerSecond(10)
SPEED_MEASURE_S = 3.0
SPEED_STABILIZE_S = 1.0
MOTOR_SPEED_REL_TOL = 0.15
COORD_RATE_ABS_TOL = 3.0


@pytest.fixture(scope="session")
def axis_dec() -> Iterator[AxisDEC]:
    port = SerialLine.search(DEVICE_PATTERN)
    serial_line = SerialLine(
        port=port,
        baud=SERIAL_BAUD,
        timeout_s=SERIAL_TIMEOUT_S,
        name=SERIAL_NAME,
        terminator="\n",
    )
    motor: TMC2209Motor = TMC2209Motor(serial_line)
    axis = AxisDEC(motor)
    for attempt in range(CONNECT_ATTEMPTS):
        try:
            axis.connect()
            break
        except (SerialException, TMC2209MotorProtocolError):
            try:
                serial_line.close()
            except Exception:
                pass
            if attempt == CONNECT_ATTEMPTS - 1:
                raise
            time.sleep(1)
    try:
        yield axis
    finally:
        axis.halt_all()
        time.sleep(2.0)
        axis.disconnect()


@pytest.fixture(autouse=True)
def _reset_axis_between_tests(axis_dec: AxisDEC) -> Iterator[None]:
    _do_reset(axis_dec)
    yield
    _do_reset(axis_dec)


def _do_reset(axis: AxisDEC) -> None:
    axis.halt_all()
    _wait_for_tracking_mode(axis, MOTOR_STOP_TIMEOUT_S + COMMAND_PROCESS_TIMEOUT_S)
    axis.set_position(PointCoordinates(ra=Ha(0), dec=DEFAULT_START_DEC))
    _wait_for_dec_near(axis, DEFAULT_START_DEC, tolerance_as=POSITION_SET_TOLERANCE_AS, timeout_s=COMMAND_PROCESS_TIMEOUT_S)
    axis.change_speed(axis.FORWARD_DIRECTION, DecPerSecond(0), update_sky_speed=True)
    _wait_for_motor_stop(axis, COMMAND_PROCESS_TIMEOUT_S)
    _wait_for_tracking_mode(axis, COMMAND_PROCESS_TIMEOUT_S)
    assert axis.mode() == AxisMotionMode.TRACK


# ---------------------------------------------------------------------------
# Polling helpers
# ---------------------------------------------------------------------------

def _wait_for_dec_near(
    axis: AxisDEC,
    expected_dec: Dec,
    tolerance_as: float,
    timeout_s: float,
) -> float:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        current = float(axis.get_position().dec)
        if abs(current - float(expected_dec)) < tolerance_as:
            return current
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(
        f"DEC did not reach {float(expected_dec):.1f} within {timeout_s}s "
        f"(last={float(axis.get_position().dec):.1f}, tol={tolerance_as})"
    )


def _wait_for_dec_change(
    axis: AxisDEC,
    start_dec: Dec,
    direction: SkyDirection,
    timeout_s: float,
) -> float:
    """Wait until DEC changes from *start_dec* toward *direction*.

    Convention: NORTH → DEC increases, SOUTH → DEC decreases.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        current = float(axis.get_position().dec)
        delta = current - float(start_dec)
        if direction == SkyDirection.NORTH and delta > DEC_CHANGE_THRESHOLD_AS:
            return current
        if direction == SkyDirection.SOUTH and delta < -DEC_CHANGE_THRESHOLD_AS:
            return current
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(f"DEC did not change toward {direction.value} from {float(start_dec):.1f}")


def _wait_for_tracking_mode(axis: AxisDEC, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if axis.mode() == AxisMotionMode.TRACK:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(
        f"DEC axis did not reach TRACK mode within {timeout_s}s: mode={axis.mode().value}"
    )


def _wait_for_motor_stop(axis: AxisDEC, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if axis._motor.status().direction == MotorDirection.STOP:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("Motor did not stop in time")


def _wait_for_motor_running(axis: AxisDEC, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if axis._motor.status().direction != MotorDirection.STOP:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("Motor did not start running in time")


def _wait_for_motor_direction(
    axis: AxisDEC,
    expected_direction: MotorDirection,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if axis._motor.status().direction == expected_direction:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(f"Motor did not reach {expected_direction.value} direction in time")


def _wait_for_goto_done(axis: AxisDEC, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if axis.is_moving_to():
            break
        time.sleep(POLL_INTERVAL_S)
    else:
        pytest.fail("GOTO never started")
    while time.monotonic() < deadline:
        if not axis.is_moving_to():
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("GOTO did not complete in time")


def _measure_motor_speed_sps(axis: AxisDEC, duration_s: float) -> float:
    steps1 = axis._motor.status().steps
    t1 = time.monotonic()
    time.sleep(duration_s)
    steps2 = axis._motor.status().steps
    t2 = time.monotonic()
    return abs(steps2 - steps1) / (t2 - t1)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("target_dec", [Dec(0), Dec(45 * 3600), Dec(-45 * 3600), Dec(80 * 3600)])
def test_hw_axis_dec_set_position(
    axis_dec: AxisDEC,
    target_dec: Dec,
) -> None:
    axis_dec.set_position(PointCoordinates(ra=Ha(0), dec=target_dec))
    _wait_for_dec_near(axis_dec, target_dec, tolerance_as=POSITION_SET_TOLERANCE_AS, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    position = axis_dec.get_position()
    assert abs(float(position.dec) - float(target_dec)) < POSITION_SET_TOLERANCE_AS


@pytest.mark.parametrize(
    ("direction", "expected_motor_direction"),
    [
        (SkyDirection.NORTH, MotorDirection.FORWARD),
        (SkyDirection.SOUTH, MotorDirection.BACKWARD),
    ],
)
def test_hw_axis_dec_track_at_speed(
    axis_dec: AxisDEC,
    direction: SkyDirection,
    expected_motor_direction: MotorDirection,
) -> None:
    axis_dec.change_speed(direction, DEC_TRACK_SPEED, update_sky_speed=True)
    _wait_for_motor_direction(axis_dec, expected_motor_direction, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    status = axis_dec._motor.status()
    assert status.direction == expected_motor_direction

    time.sleep(3.0)

    position = axis_dec.get_position()
    assert abs(float(position.dec) - float(DEFAULT_START_DEC)) < DRIFT_TOLERANCE_AS


def test_hw_axis_dec_zero_sky_speed_is_track_mode(axis_dec: AxisDEC) -> None:
    assert axis_dec._sky_speed == DecPerSecond(0)
    assert axis_dec._motor.status().direction == MotorDirection.STOP
    time.sleep(1.0)
    assert axis_dec._mode == AxisMotionMode.TRACK

    position = axis_dec.get_position()
    assert abs(float(position.dec) - float(DEFAULT_START_DEC)) < DRIFT_TOLERANCE_AS


@pytest.mark.parametrize(
    ("direction", "expect_dec_sign"),
    [
        (SkyDirection.NORTH, 1),
        (SkyDirection.SOUTH, -1),
    ],
)
def test_hw_axis_dec_move_in_direction(
    axis_dec: AxisDEC,
    direction: SkyDirection,
    expect_dec_sign: int,
) -> None:
    start_dec = axis_dec.get_position().dec
    axis_dec.move(direction, SLEW_SPEED)
    _wait_for_dec_change(axis_dec, start_dec, direction, timeout_s=COMMAND_PROCESS_TIMEOUT_S + SLEW_OBSERVE_S)

    time.sleep(SLEW_OBSERVE_S)
    end_dec = float(axis_dec.get_position().dec)

    assert (end_dec - float(start_dec)) * expect_dec_sign > 0


@pytest.mark.parametrize(
    ("direction", "speed"),
    [
        (SkyDirection.NORTH, DecPerSecond(10)),
        (SkyDirection.NORTH, DecPerSecond(20)),
        (SkyDirection.SOUTH, DecPerSecond(10)),
        (SkyDirection.SOUTH, DecPerSecond(20)),
    ],
)
def test_hw_axis_dec_motor_speed_matches_requested(
    axis_dec: AxisDEC,
    direction: SkyDirection,
    speed: DecPerSecond,
) -> None:
    axis_dec.move(direction, speed)
    _wait_for_motor_running(axis_dec, timeout_s=COMMAND_PROCESS_TIMEOUT_S)
    time.sleep(SPEED_STABILIZE_S)

    actual_sps = _measure_motor_speed_sps(axis_dec, SPEED_MEASURE_S)
    expected_sps = axis_dec._motor.convert_speed_to_steps_per_second(speed)
    assert actual_sps == pytest.approx(expected_sps, rel=MOTOR_SPEED_REL_TOL)


@pytest.mark.parametrize(
    ("move_direction", "move_speed"),
    [
        (SkyDirection.NORTH, DecPerSecond(20)),
        (SkyDirection.SOUTH, DecPerSecond(20)),
    ],
)
def test_hw_axis_dec_tracking_plus_movement(
    axis_dec: AxisDEC,
    move_direction: SkyDirection,
    move_speed: DecPerSecond,
) -> None:
    track_speed = DEC_TRACK_SPEED
    axis_dec.change_speed(SkyDirection.NORTH, track_speed, update_sky_speed=True)
    _wait_for_motor_running(axis_dec, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    axis_dec.move(move_direction, move_speed)
    _wait_for_motor_running(axis_dec, timeout_s=COMMAND_PROCESS_TIMEOUT_S)
    time.sleep(SPEED_STABILIZE_S)

    dec1 = float(axis_dec.get_position().dec)
    t1 = time.monotonic()
    time.sleep(SPEED_MEASURE_S)
    dec2 = float(axis_dec.get_position().dec)
    t2 = time.monotonic()

    coord_rate = (dec2 - dec1) / (t2 - t1)
    if move_direction == SkyDirection.NORTH:
        expected_rate = float(move_speed) - float(track_speed)
    else:
        expected_rate = -(float(move_speed) + float(track_speed))
    assert coord_rate == pytest.approx(expected_rate, abs=COORD_RATE_ABS_TOL)

    actual_sps = _measure_motor_speed_sps(axis_dec, SPEED_MEASURE_S)
    expected_sps = axis_dec._motor.convert_speed_to_steps_per_second(move_speed)
    assert actual_sps == pytest.approx(expected_sps, rel=MOTOR_SPEED_REL_TOL)


@pytest.mark.parametrize("direction", [SkyDirection.NORTH, SkyDirection.SOUTH])
def test_hw_axis_dec_halt_matching_direction(
    axis_dec: AxisDEC,
    direction: SkyDirection,
) -> None:
    axis_dec.move(direction, SLEW_SPEED)
    _wait_for_motor_running(axis_dec, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    axis_dec.halt_direction(direction)


@pytest.mark.parametrize(
    ("move_direction", "halt_direction"),
    [
        (SkyDirection.NORTH, SkyDirection.SOUTH),
        (SkyDirection.SOUTH, SkyDirection.NORTH),
    ],
)
def test_hw_axis_dec_halt_non_matching_direction_is_ignored(
    axis_dec: AxisDEC,
    move_direction: SkyDirection,
    halt_direction: SkyDirection,
) -> None:
    axis_dec.move(move_direction, SLEW_SPEED)
    expected_direction = MotorDirection.FORWARD if move_direction == SkyDirection.NORTH else MotorDirection.BACKWARD
    _wait_for_motor_direction(axis_dec, expected_direction, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    axis_dec.halt_direction(halt_direction)
    time.sleep(1.5)

    assert axis_dec._motor.status().direction != MotorDirection.STOP


@pytest.mark.parametrize("direction", [SkyDirection.NORTH, SkyDirection.SOUTH])
def test_hw_axis_dec_halt_all(
    axis_dec: AxisDEC,
    direction: SkyDirection,
) -> None:
    axis_dec.move(direction, SLEW_SPEED)
    _wait_for_motor_running(axis_dec, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    axis_dec.halt_all()


@pytest.mark.parametrize("ra_direction", [SkyDirection.EAST, SkyDirection.WEST])
def test_hw_axis_dec_ra_directions_are_ignored(
    axis_dec: AxisDEC,
    ra_direction: SkyDirection,
) -> None:
    assert axis_dec._motor.status().direction == MotorDirection.STOP

    axis_dec.move(ra_direction, SLEW_SPEED)
    axis_dec.change_speed(ra_direction, DEC_TRACK_SPEED, update_sky_speed=True)
    time.sleep(1.5)
    assert axis_dec._motor.status().direction == MotorDirection.STOP

    axis_dec.move(SkyDirection.NORTH, SLEW_SPEED)
    _wait_for_motor_direction(axis_dec, MotorDirection.FORWARD, timeout_s=COMMAND_PROCESS_TIMEOUT_S)
    axis_dec.halt_direction(ra_direction)
    time.sleep(1.5)
    assert axis_dec._motor.status().direction != MotorDirection.STOP


@pytest.mark.parametrize(
    ("delta_dec", "expect_fast"),
    [
        (Dec(0), None),
        (Dec(30), False),
        (Dec(-30), False),
        (Dec(300), True),
        (Dec(-300), True),
        (Dec(3600), True),
        (Dec(-3600), True),
        (Dec(10 * 3600), True),
    ],
)
def test_hw_axis_dec_goto_reaches_target(
    axis_dec: AxisDEC,
    delta_dec: Dec,
    expect_fast: bool | None,
) -> None:
    axis_dec.set_position(PointCoordinates(ra=Ha(0), dec=GOTO_START))
    _wait_for_dec_near(axis_dec, GOTO_START, tolerance_as=POSITION_SET_TOLERANCE_AS, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    target = GOTO_START + delta_dec
    axis_dec.goto_to(PointCoordinates(ra=Ha(0), dec=target))

    if float(delta_dec) == 0:
        time.sleep(1.0)
        assert not axis_dec.is_moving_to()
    else:
        deadline = time.monotonic() + COMMAND_PROCESS_TIMEOUT_S
        while time.monotonic() < deadline:
            if axis_dec.is_moving_to():
                break
            time.sleep(POLL_INTERVAL_S)

        if expect_fast is not None and axis_dec.is_moving_to():
            goto_speed_sps = axis_dec._motor.status().speed_sps
            slow_sps = axis_dec._motor.convert_speed_to_steps_per_second(DecPerSecond(30))
            if expect_fast:
                assert goto_speed_sps > slow_sps, (
                    f"expected fast GOTO but got {goto_speed_sps} sps "
                    f"(slow threshold={slow_sps})"
                )
            else:
                assert goto_speed_sps <= slow_sps, (
                    f"expected slow GOTO but got {goto_speed_sps} sps "
                    f"(slow threshold={slow_sps})"
                )

        deadline = time.monotonic() + GOTO_TIMEOUT_S
        while time.monotonic() < deadline:
            if not axis_dec.is_moving_to():
                break
            time.sleep(POLL_INTERVAL_S)
        else:
            pytest.fail("GOTO did not complete in time")

    final_dec = axis_dec.get_position().dec
    assert abs(float(final_dec) - float(target)) < GOTO_POSITION_TOLERANCE_AS


def test_hw_axis_dec_is_moving_to_during_goto(axis_dec: AxisDEC) -> None:
    axis_dec.set_position(PointCoordinates(ra=Ha(0), dec=GOTO_START))
    _wait_for_dec_near(axis_dec, GOTO_START, tolerance_as=POSITION_SET_TOLERANCE_AS, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    target = GOTO_START + Dec(600)
    axis_dec.goto_to(PointCoordinates(ra=Ha(0), dec=target))

    deadline = time.monotonic() + COMMAND_PROCESS_TIMEOUT_S
    while time.monotonic() < deadline:
        if axis_dec.is_moving_to():
            break
        time.sleep(POLL_INTERVAL_S)
    assert axis_dec.is_moving_to() is True

    _wait_for_goto_done(axis_dec, timeout_s=GOTO_TIMEOUT_S)
    assert axis_dec.is_moving_to() is False
