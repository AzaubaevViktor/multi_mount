import time
from collections.abc import Iterator

import pytest

from serial_wrapper.wrapper import SerialLine
from sky.axis import AxisMotionMode, AxisRA, PointCoordinates
from sky.constants import STELLAR_SPEED
from sky.motor import MotorDirection
from sky.physics import Dec, Ha, HaPerSecond, SkyDirection
from skywatcher.motor import SkyWatcherMotor


DEVICE_PATTERN = "PL2303G-USBtoUART"
SERIAL_BAUD = 112500
SERIAL_TIMEOUT_S = 0.2
SERIAL_NAME = "skywatcher_axis_ra"

POLL_INTERVAL_S = 0.2
COMMAND_PROCESS_TIMEOUT_S = 5.0
MOTOR_STOP_TIMEOUT_S = 10.0
GOTO_TIMEOUT_S = 60.0
GOTO_POSITION_TOLERANCE_S = 15.0
RA_CHANGE_THRESHOLD_S = 0.5
POSITION_SET_TOLERANCE_S = 2.0
TRACKING_DRIFT_TOLERANCE_S = 5.0
SLEW_SPEED = HaPerSecond(3)
SLEW_OBSERVE_S = 3.0
DEFAULT_START_RA = Ha(12 * 3600)
GOTO_START = DEFAULT_START_RA
SPEED_MEASURE_S = 3.0
SPEED_STABILIZE_S = 1.0
MOTOR_SPEED_REL_TOL = 0.10
COORD_RATE_ABS_TOL = 0.5


@pytest.fixture(scope="session")
def axis_ra() -> Iterator[AxisRA]:
    serial_line = SerialLine(
        port=SerialLine.search(DEVICE_PATTERN),
        baud=SERIAL_BAUD,
        timeout_s=SERIAL_TIMEOUT_S,
        name=SERIAL_NAME,
        terminator="\r",
    )
    motor = SkyWatcherMotor(serial_line)
    axis = AxisRA(motor)
    axis.connect()
    try:
        yield axis
    finally:
        axis.halt_all()
        time.sleep(2.0)
        axis.disconnect()


@pytest.fixture(autouse=True)
def _reset_axis_between_tests(axis_ra: AxisRA) -> Iterator[None]:
    _do_reset(axis_ra)
    yield
    _do_reset(axis_ra)


def _do_reset(axis: AxisRA) -> None:
    axis.halt_all()
    _wait_for_tracking_mode(axis, MOTOR_STOP_TIMEOUT_S + COMMAND_PROCESS_TIMEOUT_S)
    axis.set_position(PointCoordinates(ra=DEFAULT_START_RA, dec=Dec(0)))
    _wait_for_ra_near(axis, DEFAULT_START_RA, tolerance_s=POSITION_SET_TOLERANCE_S, timeout_s=COMMAND_PROCESS_TIMEOUT_S)
    axis.change_speed(axis.FORWARD_DIRECTION, HaPerSecond(0), update_sky_speed=True)
    _wait_for_motor_stop(axis, COMMAND_PROCESS_TIMEOUT_S)
    _wait_for_tracking_mode(axis, COMMAND_PROCESS_TIMEOUT_S)
    assert axis.mode() == AxisMotionMode.TRACK


# ---------------------------------------------------------------------------
# Polling helpers
# ---------------------------------------------------------------------------

def _wait_for_ra_near(
    axis: AxisRA,
    expected_ra: Ha,
    tolerance_s: float,
    timeout_s: float,
) -> float:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        current = float(axis.get_position().ra)
        if abs(current - float(expected_ra)) < tolerance_s:
            return current
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(
        f"RA did not reach {float(expected_ra):.1f} within {timeout_s}s "
        f"(last={float(axis.get_position().ra):.1f}, tol={tolerance_s})"
    )


def _wait_for_ra_change(
    axis: AxisRA,
    start_ra: Ha,
    direction: SkyDirection,
    timeout_s: float,
) -> float:
    """Wait until RA changes from *start_ra* toward *direction*.

    Convention: EAST → RA decreases, WEST → RA increases.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        current = float(axis.get_position().ra)
        delta = current - float(start_ra)
        if direction == SkyDirection.EAST and delta < -RA_CHANGE_THRESHOLD_S:
            return current
        if direction == SkyDirection.WEST and delta > RA_CHANGE_THRESHOLD_S:
            return current
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(f"RA did not change toward {direction.value} from {float(start_ra):.1f}")


def _wait_for_tracking_mode(axis: AxisRA, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if axis.mode() == AxisMotionMode.TRACK:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(
        f"RA axis did not reach TRACK mode within {timeout_s}s: mode={axis.mode().value}"
    )


def _wait_for_motor_stop(axis: AxisRA, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if axis._motor.status().direction == MotorDirection.STOP:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("Motor did not stop in time")


def _wait_for_motor_running(axis: AxisRA, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if axis._motor.status().direction != MotorDirection.STOP:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("Motor did not start running in time")


def _wait_for_goto_done(axis: AxisRA, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    # Phase 1: wait for GOTO to actually start (command is async)
    while time.monotonic() < deadline:
        if axis.is_moving_to():
            break
        time.sleep(POLL_INTERVAL_S)
    else:
        pytest.fail("GOTO never started")
    # Phase 2: wait for GOTO to finish
    while time.monotonic() < deadline:
        if not axis.is_moving_to():
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("GOTO did not complete in time")


def _measure_motor_speed_sps(axis: AxisRA, duration_s: float) -> float:
    steps1 = axis._motor.status().steps
    t1 = time.monotonic()
    time.sleep(duration_s)
    steps2 = axis._motor.status().steps
    t2 = time.monotonic()
    return abs(steps2 - steps1) / (t2 - t1)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("target_ra", [Ha(0), Ha(6 * 3600), Ha(12 * 3600), Ha(23 * 3600)])
def test_hw_axis_ra_set_position(
    axis_ra: AxisRA,
    target_ra: Ha,
) -> None:
    axis_ra.set_position(PointCoordinates(ra=target_ra, dec=Dec(0)))
    _wait_for_ra_near(axis_ra, target_ra, tolerance_s=POSITION_SET_TOLERANCE_S, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    position = axis_ra.get_position()
    assert abs(float(position.ra) - float(target_ra)) < POSITION_SET_TOLERANCE_S


@pytest.mark.parametrize(
    ("direction", "expected_motor_direction"),
    [
        (SkyDirection.EAST, MotorDirection.FORWARD),
        (SkyDirection.WEST, MotorDirection.BACKWARD),
    ],
)
def test_hw_axis_ra_track_at_sidereal_rate(
    axis_ra: AxisRA,
    direction: SkyDirection,
    expected_motor_direction: MotorDirection,
) -> None:
    axis_ra.change_speed(direction, STELLAR_SPEED, update_sky_speed=True)
    _wait_for_motor_running(axis_ra, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    status = axis_ra._motor.status()
    assert status.direction == expected_motor_direction

    time.sleep(3.0)

    position = axis_ra.get_position()
    assert abs(float(position.ra) - float(DEFAULT_START_RA)) < TRACKING_DRIFT_TOLERANCE_S


def test_hw_axis_ra_zero_sky_speed_is_track_mode(axis_ra: AxisRA) -> None:
    assert axis_ra._sky_speed == HaPerSecond(0)
    assert axis_ra._motor.status().direction == MotorDirection.STOP
    time.sleep(1.0)
    assert axis_ra._mode == AxisMotionMode.TRACK

    position = axis_ra.get_position()
    assert abs(float(position.ra) - float(DEFAULT_START_RA)) < TRACKING_DRIFT_TOLERANCE_S


@pytest.mark.parametrize(
    ("direction", "expect_ra_sign"),
    [
        (SkyDirection.EAST, -1),
        (SkyDirection.WEST, 1),
    ],
)
def test_hw_axis_ra_move_in_direction(
    axis_ra: AxisRA,
    direction: SkyDirection,
    expect_ra_sign: int,
) -> None:
    start_ra = axis_ra.get_position().ra
    axis_ra.move(direction, SLEW_SPEED)
    _wait_for_ra_change(axis_ra, start_ra, direction, timeout_s=COMMAND_PROCESS_TIMEOUT_S + SLEW_OBSERVE_S)

    time.sleep(SLEW_OBSERVE_S)
    end_ra = float(axis_ra.get_position().ra)

    assert (end_ra - float(start_ra)) * expect_ra_sign > 0


@pytest.mark.parametrize(
    ("direction", "speed"),
    [
        (SkyDirection.EAST, HaPerSecond(1)),
        (SkyDirection.EAST, HaPerSecond(3)),
        (SkyDirection.WEST, HaPerSecond(1)),
        (SkyDirection.WEST, HaPerSecond(3)),
    ],
)
def test_hw_axis_ra_motor_speed_matches_requested(
    axis_ra: AxisRA,
    direction: SkyDirection,
    speed: HaPerSecond,
) -> None:
    axis_ra.move(direction, speed)
    _wait_for_motor_running(axis_ra, timeout_s=COMMAND_PROCESS_TIMEOUT_S)
    time.sleep(SPEED_STABILIZE_S)

    actual_sps = _measure_motor_speed_sps(axis_ra, SPEED_MEASURE_S)
    expected_sps = axis_ra._motor.convert_speed_to_steps_per_second(speed)
    assert actual_sps == pytest.approx(expected_sps, rel=MOTOR_SPEED_REL_TOL)


@pytest.mark.parametrize(
    ("move_direction", "move_speed"),
    [
        (SkyDirection.EAST, HaPerSecond(3)),
        (SkyDirection.WEST, HaPerSecond(3)),
    ],
)
def test_hw_axis_ra_tracking_plus_movement(
    axis_ra: AxisRA,
    move_direction: SkyDirection,
    move_speed: HaPerSecond,
) -> None:
    axis_ra.change_speed(SkyDirection.EAST, STELLAR_SPEED, update_sky_speed=True)
    _wait_for_motor_running(axis_ra, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    axis_ra.move(move_direction, move_speed)
    _wait_for_motor_running(axis_ra, timeout_s=COMMAND_PROCESS_TIMEOUT_S)
    time.sleep(SPEED_STABILIZE_S)

    ra1 = float(axis_ra.get_position().ra)
    t1 = time.monotonic()
    time.sleep(SPEED_MEASURE_S)
    ra2 = float(axis_ra.get_position().ra)
    t2 = time.monotonic()

    coord_rate = (ra2 - ra1) / (t2 - t1)
    if move_direction == SkyDirection.EAST:
        expected_rate = -(float(move_speed) - float(STELLAR_SPEED))
    else:
        expected_rate = float(move_speed) + float(STELLAR_SPEED)
    assert coord_rate == pytest.approx(expected_rate, abs=COORD_RATE_ABS_TOL)

    actual_sps = _measure_motor_speed_sps(axis_ra, SPEED_MEASURE_S)
    expected_sps = axis_ra._motor.convert_speed_to_steps_per_second(move_speed)
    assert actual_sps == pytest.approx(expected_sps, rel=MOTOR_SPEED_REL_TOL)


@pytest.mark.parametrize("direction", [SkyDirection.EAST, SkyDirection.WEST])
def test_hw_axis_ra_halt_matching_direction(
    axis_ra: AxisRA,
    direction: SkyDirection,
) -> None:
    axis_ra.move(direction, SLEW_SPEED)
    _wait_for_motor_running(axis_ra, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    axis_ra.halt_direction(direction)


@pytest.mark.parametrize(
    ("move_direction", "halt_direction"),
    [
        (SkyDirection.EAST, SkyDirection.WEST),
        (SkyDirection.WEST, SkyDirection.EAST),
    ],
)
def test_hw_axis_ra_halt_non_matching_direction_is_ignored(
    axis_ra: AxisRA,
    move_direction: SkyDirection,
    halt_direction: SkyDirection,
) -> None:
    axis_ra.move(move_direction, SLEW_SPEED)
    _wait_for_motor_running(axis_ra, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    axis_ra.halt_direction(halt_direction)
    time.sleep(1.5)

    assert axis_ra._motor.status().direction != MotorDirection.STOP


@pytest.mark.parametrize("direction", [SkyDirection.EAST, SkyDirection.WEST])
def test_hw_axis_ra_halt_all(
    axis_ra: AxisRA,
    direction: SkyDirection,
) -> None:
    axis_ra.move(direction, SLEW_SPEED)
    _wait_for_motor_running(axis_ra, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    axis_ra.halt_all()


@pytest.mark.parametrize("dec_direction", [SkyDirection.NORTH, SkyDirection.SOUTH])
def test_hw_axis_ra_dec_directions_are_ignored(
    axis_ra: AxisRA,
    dec_direction: SkyDirection,
) -> None:
    assert axis_ra._motor.status().direction == MotorDirection.STOP

    axis_ra.move(dec_direction, SLEW_SPEED)
    axis_ra.change_speed(dec_direction, STELLAR_SPEED, update_sky_speed=True)
    time.sleep(1.5)
    assert axis_ra._motor.status().direction == MotorDirection.STOP

    axis_ra.move(SkyDirection.EAST, SLEW_SPEED)
    _wait_for_motor_running(axis_ra, timeout_s=COMMAND_PROCESS_TIMEOUT_S)
    axis_ra.halt_direction(dec_direction)
    time.sleep(1.5)
    assert axis_ra._motor.status().direction != MotorDirection.STOP


@pytest.mark.parametrize(
    ("delta_ha", "expect_highspeed"),
    [
        (Ha(0), None),
        (Ha(30), False),
        (Ha(-30), False),
        (Ha(300), False),
        (Ha(-300), False),
        (Ha(900), True),
        (Ha(-900), True),
        (Ha(1800), True),
    ],
)
def test_hw_axis_ra_goto_reaches_target(
    axis_ra: AxisRA,
    delta_ha: Ha,
    expect_highspeed: bool | None,
) -> None:
    axis_ra.set_position(PointCoordinates(ra=GOTO_START, dec=Dec(0)))
    _wait_for_ra_near(axis_ra, GOTO_START, tolerance_s=POSITION_SET_TOLERANCE_S, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    target = GOTO_START + delta_ha
    axis_ra.goto_to(PointCoordinates(ra=target, dec=Dec(0)))

    if float(delta_ha) == 0:
        time.sleep(1.0)
        assert not axis_ra.is_moving_to()
    else:
        deadline = time.monotonic() + COMMAND_PROCESS_TIMEOUT_S
        while time.monotonic() < deadline:
            if axis_ra.is_moving_to():
                break
            time.sleep(POLL_INTERVAL_S)

        if expect_highspeed is not None and axis_ra.is_moving_to():
            goto_speed_sps = axis_ra._motor.status().speed_sps
            lowspeed_sps = axis_ra._motor.convert_speed_to_steps_per_second(
                HaPerSecond(float(STELLAR_SPEED) * 128)
            )
            if expect_highspeed:
                assert goto_speed_sps > lowspeed_sps * 2, (
                    f"expected highspeed but got {goto_speed_sps} sps "
                    f"(lowspeed threshold={lowspeed_sps})"
                )
            else:
                assert goto_speed_sps <= lowspeed_sps * 2, (
                    f"expected lowspeed but got {goto_speed_sps} sps "
                    f"(lowspeed threshold={lowspeed_sps})"
                )

        deadline = time.monotonic() + GOTO_TIMEOUT_S
        while time.monotonic() < deadline:
            if not axis_ra.is_moving_to():
                break
            time.sleep(POLL_INTERVAL_S)
        else:
            pytest.fail("GOTO did not complete in time")

    final_ra = axis_ra.get_position().ra
    assert abs(float(final_ra) - float(target)) < GOTO_POSITION_TOLERANCE_S


def test_hw_axis_ra_is_moving_to_during_goto(axis_ra: AxisRA) -> None:
    axis_ra.set_position(PointCoordinates(ra=GOTO_START, dec=Dec(0)))
    _wait_for_ra_near(axis_ra, GOTO_START, tolerance_s=POSITION_SET_TOLERANCE_S, timeout_s=COMMAND_PROCESS_TIMEOUT_S)

    target = GOTO_START + Ha(600)
    axis_ra.goto_to(PointCoordinates(ra=target, dec=Dec(0)))

    deadline = time.monotonic() + COMMAND_PROCESS_TIMEOUT_S
    while time.monotonic() < deadline:
        if axis_ra.is_moving_to():
            break
        time.sleep(POLL_INTERVAL_S)
    assert axis_ra.is_moving_to() is True

    _wait_for_goto_done(axis_ra, timeout_s=GOTO_TIMEOUT_S)
    assert axis_ra.is_moving_to() is False
