import time
from collections.abc import Iterator

import pytest
from serial.serialutil import SerialException

from serial_wrapper.wrapper import SerialLine
from sky.axis import AxisDEC, AxisMotionMode, AxisRA, PointCoordinates
from sky.combiner import Combiner
from sky.constants import STELLAR_SPEED
from sky.motor import MotorDirection
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond, Second, SkyDirection
from skywatcher.motor import SkyWatcherMotor
from tmc2209.motor import TMC2209Motor, TMC2209MotorProtocolError


RA_DEVICE_PATTERN = "PL2303G-USBtoUART"
RA_SERIAL_BAUD = 112500
RA_SERIAL_TIMEOUT_S = 0.2

DEC_DEVICE_PATTERN = r"^tty\.usbserial.*$"
DEC_SERIAL_BAUD = 115200
DEC_SERIAL_TIMEOUT_S = 2.0
DEC_CONNECT_ATTEMPTS = 3
DEC_READY_TIMEOUT_S = 10.0

POLL_INTERVAL_S = 0.2
COMMAND_PROCESS_TIMEOUT_S = 5.0
MOTOR_STOP_TIMEOUT_S = 10.0
GOTO_TIMEOUT_S = 60.0
SLEW_OBSERVE_S = 3.0
SPEED_STABILIZE_S = 1.0
SPEED_MEASURE_S = 3.0

RA_POSITION_TOLERANCE_S = 2.0
DEC_POSITION_TOLERANCE_AS = 2.0
RA_DRIFT_TOLERANCE_S = 5.0
DEC_DRIFT_TOLERANCE_AS = 5.0
RA_CHANGE_THRESHOLD_S = 0.5
DEC_CHANGE_THRESHOLD_AS = 0.5
GOTO_RA_TOLERANCE_S = 15.0
GOTO_DEC_TOLERANCE_AS = 15.0
MOTOR_SPEED_REL_TOL = 0.15

RA_SLEW_SPEED = HaPerSecond(3)
DEC_SLEW_SPEED = DecPerSecond(20)

ZERO_POSITION = PointCoordinates(ra=Ha(0), dec=Dec(0))


def _connect_dec_axis(dec_serial: SerialLine) -> AxisDEC:
    """Connect DEC axis with retry logic for TMC2209 serial handshake."""
    for attempt in range(DEC_CONNECT_ATTEMPTS):
        try:
            axis = AxisDEC(TMC2209Motor(dec_serial))
            axis.connect()
            return axis
        except (SerialException, TMC2209MotorProtocolError):
            try:
                dec_serial.close()
            except Exception:
                pass
            if attempt == DEC_CONNECT_ATTEMPTS - 1:
                raise
            time.sleep(0.5)
    raise RuntimeError("Unreachable")


@pytest.fixture(scope="session")
def combiner() -> Iterator[Combiner]:
    ra_serial = SerialLine(
        port=SerialLine.search(RA_DEVICE_PATTERN),
        baud=RA_SERIAL_BAUD,
        timeout_s=RA_SERIAL_TIMEOUT_S,
        name="skywatcher_axis_ra",
        terminator="\r",
    )
    axis_ra = AxisRA(SkyWatcherMotor(ra_serial))

    dec_serial = SerialLine(
        port=SerialLine.search(DEC_DEVICE_PATTERN),
        baud=DEC_SERIAL_BAUD,
        timeout_s=DEC_SERIAL_TIMEOUT_S,
        name="tmc2209_axis_dec",
        terminator="\n",
    )
    axis_dec = _connect_dec_axis(dec_serial)

    comb = Combiner(axis_ra, axis_dec)
    comb.connect()

    try:
        yield comb
    finally:
        comb.halt_all()
        time.sleep(2.0)
        comb.disconnect()


@pytest.fixture(autouse=True)
def _reset_between_tests(combiner: Combiner) -> Iterator[None]:
    _do_reset(combiner)
    yield
    _do_reset(combiner)


def _do_reset(comb: Combiner) -> None:
    comb.halt_all()
    comb.set_sky_speed(STELLAR_SPEED, DecPerSecond(0))
    comb.set_position(ZERO_POSITION)
    # Force polar compensator to reset to default speeds on next iteration
    comb._polar_compensator.last_guide_pulse = Second(0)
    _wait_while_queue_empty(comb, COMMAND_PROCESS_TIMEOUT_S * 5)
    _wait_for_position_near(
        comb,
        ZERO_POSITION,
        ra_tol=RA_POSITION_TOLERANCE_S,
        dec_tol=DEC_POSITION_TOLERANCE_AS,
        timeout_s=COMMAND_PROCESS_TIMEOUT_S,
    )
    _wait_for_tracking_mode(comb, COMMAND_PROCESS_TIMEOUT_S)
    assert comb.ra.mode() == AxisMotionMode.TRACK
    assert comb.dec.mode() == AxisMotionMode.TRACK


def _wait_while_queue_empty(comb: Combiner, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if comb.ra._queue.empty() and comb.dec._queue.empty():
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("Queue did not empty in time")


def _wait_for_tracking_mode(comb: Combiner, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if comb.ra.mode() == AxisMotionMode.TRACK and comb.dec.mode() == AxisMotionMode.TRACK:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(
        f"Axes did not reach TRACK mode within {timeout_s}s: "
        f"ra={comb.ra.mode().value}, dec={comb.dec.mode().value}"
    )


# ---------------------------------------------------------------------------
# Polling helpers
# ---------------------------------------------------------------------------

def _wait_for_position_near(
    comb: Combiner,
    target: PointCoordinates,
    ra_tol: float,
    dec_tol: float,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pos = comb.get_position()
        if abs(float(pos.ra) - float(target.ra)) < ra_tol and abs(float(pos.dec) - float(target.dec)) < dec_tol:
            return
        time.sleep(POLL_INTERVAL_S)
    pos = comb.get_position()
    pytest.fail(
        f"Position did not reach target within {timeout_s}s: "
        f"ra={float(pos.ra):.1f} (expected {float(target.ra):.1f}±{ra_tol}), "
        f"dec={float(pos.dec):.1f} (expected {float(target.dec):.1f}±{dec_tol})"
    )


def _wait_for_ra_change(
    comb: Combiner,
    start_ra: Ha,
    direction: SkyDirection,
    timeout_s: float,
) -> float:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        current = float(comb.get_position().ra)
        delta = current - float(start_ra)
        if direction == SkyDirection.EAST and delta < -RA_CHANGE_THRESHOLD_S:
            return current
        if direction == SkyDirection.WEST and delta > RA_CHANGE_THRESHOLD_S:
            return current
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(f"RA did not change toward {direction.value} from {float(start_ra):.1f}")


def _wait_for_dec_change(
    comb: Combiner,
    start_dec: Dec,
    direction: SkyDirection,
    timeout_s: float,
) -> float:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        current = float(comb.get_position().dec)
        delta = current - float(start_dec)
        if direction == SkyDirection.NORTH and delta > DEC_CHANGE_THRESHOLD_AS:
            return current
        if direction == SkyDirection.SOUTH and delta < -DEC_CHANGE_THRESHOLD_AS:
            return current
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(f"DEC did not change toward {direction.value} from {float(start_dec):.1f}")


def _wait_for_ra_motor_stop(comb: Combiner, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if comb.ra._motor.status().direction == MotorDirection.STOP:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("RA motor did not stop in time")


def _wait_for_dec_motor_stop(comb: Combiner, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if comb.dec._motor.status().direction == MotorDirection.STOP:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("DEC motor did not stop in time")


def _wait_for_ra_motor_running(comb: Combiner, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if comb.ra._motor.status().direction != MotorDirection.STOP:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("RA motor did not start running in time")


def _wait_for_dec_motor_running(comb: Combiner, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if comb.dec._motor.status().direction != MotorDirection.STOP:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("DEC motor did not start running in time")


def _wait_for_goto_done(comb: Combiner, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if comb.is_moving_to():
            break
        time.sleep(POLL_INTERVAL_S)
    else:
        pytest.fail("GOTO never started")
    while time.monotonic() < deadline:
        if not comb.is_moving_to():
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail("GOTO did not complete in time")


def _measure_ra_motor_speed_sps(comb: Combiner, duration_s: float) -> float:
    steps1 = comb.ra._motor.status().steps
    t1 = time.monotonic()
    time.sleep(duration_s)
    steps2 = comb.ra._motor.status().steps
    t2 = time.monotonic()
    return abs(steps2 - steps1) / (t2 - t1)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_hw_combiner_is_connected(combiner: Combiner) -> None:
    assert combiner.is_connected() is True


def test_hw_combiner_initial_position(combiner: Combiner) -> None:
    pos = combiner.get_position()
    assert abs(float(pos.ra)) < RA_POSITION_TOLERANCE_S
    assert abs(float(pos.dec)) < DEC_POSITION_TOLERANCE_AS


def test_hw_combiner_sidereal_tracking_stable(combiner: Combiner) -> None:
    """RA tracks at sidereal rate; sky position should stay nearly constant."""
    _wait_for_ra_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)
    time.sleep(3.0)
    pos = combiner.get_position()
    assert abs(float(pos.ra)) < RA_DRIFT_TOLERANCE_S
    assert abs(float(pos.dec)) < DEC_DRIFT_TOLERANCE_AS


@pytest.mark.parametrize(
    ("ra_dir", "dec_dir", "expect_ra_sign", "expect_dec_sign"),
    [
        (SkyDirection.EAST,  None,              -1,  0),
        (SkyDirection.WEST,  None,               1,  0),
        (None,               SkyDirection.NORTH,  0,  1),
        (None,               SkyDirection.SOUTH,  0, -1),
        (SkyDirection.EAST,  SkyDirection.NORTH, -1,  1),
        (SkyDirection.WEST,  SkyDirection.NORTH,  1,  1),
        (SkyDirection.EAST,  SkyDirection.SOUTH, -1, -1),
        (SkyDirection.WEST,  SkyDirection.SOUTH,  1, -1),
    ],
    ids=["east", "west", "north", "south", "ne", "nw", "se", "sw"],
)
def test_hw_combiner_move(
    combiner: Combiner,
    ra_dir: SkyDirection | None,
    dec_dir: SkyDirection | None,
    expect_ra_sign: int,
    expect_dec_sign: int,
) -> None:
    start = combiner.get_position()

    if ra_dir is not None:
        combiner.move(ra_dir, RA_SLEW_SPEED)
    if dec_dir is not None:
        combiner.move(dec_dir, DEC_SLEW_SPEED)

    timeout = COMMAND_PROCESS_TIMEOUT_S + SLEW_OBSERVE_S
    if ra_dir is not None:
        _wait_for_ra_change(combiner, start.ra, ra_dir, timeout)
    if dec_dir is not None:
        _wait_for_dec_change(combiner, start.dec, dec_dir, timeout)

    time.sleep(SLEW_OBSERVE_S)
    end = combiner.get_position()

    ra_delta = float(end.ra) - float(start.ra)
    dec_delta = float(end.dec) - float(start.dec)

    if expect_ra_sign != 0:
        assert ra_delta * expect_ra_sign > 0
    else:
        assert abs(ra_delta) < RA_DRIFT_TOLERANCE_S

    if expect_dec_sign != 0:
        assert dec_delta * expect_dec_sign > 0
    else:
        assert abs(dec_delta) < DEC_DRIFT_TOLERANCE_AS


@pytest.mark.parametrize("direction", [SkyDirection.EAST, SkyDirection.WEST])
def test_hw_combiner_halt_ra_direction(
    combiner: Combiner,
    direction: SkyDirection,
) -> None:
    combiner.move(direction, RA_SLEW_SPEED)
    _wait_for_ra_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)

    combiner.halt_direction(direction)
    _wait_for_tracking_mode(combiner, COMMAND_PROCESS_TIMEOUT_S)
    pos_after_halt = combiner.get_position()
    time.sleep(SPEED_STABILIZE_S)
    pos_after_tracking = combiner.get_position()

    assert combiner.ra.mode() == AxisMotionMode.TRACK
    assert abs(float(pos_after_tracking.ra) - float(pos_after_halt.ra)) < RA_DRIFT_TOLERANCE_S


@pytest.mark.parametrize("direction", [SkyDirection.NORTH, SkyDirection.SOUTH])
def test_hw_combiner_halt_dec_direction(
    combiner: Combiner,
    direction: SkyDirection,
) -> None:
    combiner.move(direction, DEC_SLEW_SPEED)
    _wait_for_dec_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)

    combiner.halt_direction(direction)
    _wait_for_dec_motor_stop(combiner, MOTOR_STOP_TIMEOUT_S)
    _wait_for_tracking_mode(combiner, COMMAND_PROCESS_TIMEOUT_S)
    pos_after_halt = combiner.get_position()
    time.sleep(SPEED_STABILIZE_S)
    pos_after_tracking = combiner.get_position()

    assert combiner.dec.mode() == AxisMotionMode.TRACK
    assert abs(float(pos_after_tracking.dec) - float(pos_after_halt.dec)) < DEC_DRIFT_TOLERANCE_AS


@pytest.mark.parametrize(
    ("move_dir", "halt_dir"),
    [
        (SkyDirection.EAST, SkyDirection.WEST),
        (SkyDirection.WEST, SkyDirection.EAST),
        (SkyDirection.NORTH, SkyDirection.SOUTH),
        (SkyDirection.SOUTH, SkyDirection.NORTH),
    ],
)
def test_hw_combiner_halt_wrong_direction_ignored(
    combiner: Combiner,
    move_dir: SkyDirection,
    halt_dir: SkyDirection,
) -> None:
    is_ra = move_dir in (SkyDirection.EAST, SkyDirection.WEST)
    speed: HaPerSecond | DecPerSecond = RA_SLEW_SPEED if is_ra else DEC_SLEW_SPEED

    combiner.move(move_dir, speed)
    if is_ra:
        _wait_for_ra_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)
    else:
        _wait_for_dec_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)

    combiner.halt_direction(halt_dir)
    time.sleep(1.5)

    motor = combiner.ra._motor if is_ra else combiner.dec._motor
    assert motor.status().direction != MotorDirection.STOP


def test_hw_combiner_halt_all(combiner: Combiner) -> None:
    combiner.move(SkyDirection.WEST, RA_SLEW_SPEED)
    combiner.move(SkyDirection.NORTH, DEC_SLEW_SPEED)
    _wait_for_ra_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)
    _wait_for_dec_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)

    combiner.halt_all()


@pytest.mark.parametrize("axis_name", ["ra", "dec"], ids=["ra", "dec"])
@pytest.mark.parametrize("tracking_name", ["default", "custom"], ids=["default_tracking", "custom_tracking"])
@pytest.mark.parametrize("action_name", ["move", "goto"], ids=["move", "goto"])
@pytest.mark.parametrize("halt_name", ["halt_direction", "halt_all"], ids=["halt_direction", "halt_all"])
def test_hw_combiner_halt_restores_tracking_state(
    combiner: Combiner,
    axis_name: str,
    tracking_name: str,
    action_name: str,
    halt_name: str,
) -> None:
    if tracking_name == "default":
        expected_ra_sky_speed = STELLAR_SPEED
        expected_dec_sky_speed = DecPerSecond(0)
    else:
        expected_ra_sky_speed = STELLAR_SPEED + HaPerSecond(2)
        expected_dec_sky_speed = DecPerSecond(2)

    combiner.set_sky_speed(expected_ra_sky_speed, expected_dec_sky_speed)
    _wait_for_tracking_mode(combiner, COMMAND_PROCESS_TIMEOUT_S)

    if axis_name == "ra":
        direction = SkyDirection.WEST
        move_speed: HaPerSecond | DecPerSecond = RA_SLEW_SPEED
        target = PointCoordinates(ra=Ha(300), dec=Dec(0))
        drift_tolerance = RA_DRIFT_TOLERANCE_S
    else:
        direction = SkyDirection.SOUTH
        move_speed = DEC_SLEW_SPEED
        target = PointCoordinates(ra=Ha(0), dec=Dec(-300))
        drift_tolerance = DEC_DRIFT_TOLERANCE_AS

    if action_name == "move":
        combiner.move(direction, move_speed)
        if axis_name == "ra":
            _wait_for_ra_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)
        else:
            _wait_for_dec_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)
    else:
        combiner.goto_to(target)
        deadline = time.monotonic() + COMMAND_PROCESS_TIMEOUT_S
        while time.monotonic() < deadline:
            if combiner.is_moving_to():
                break
            time.sleep(POLL_INTERVAL_S)
        else:
            pytest.fail("GOTO never started before halt")

    if halt_name == "halt_direction":
        combiner.halt_direction(direction)
    else:
        combiner.halt_all()

    _wait_for_tracking_mode(combiner, COMMAND_PROCESS_TIMEOUT_S + MOTOR_STOP_TIMEOUT_S)
    pos_after_halt = combiner.get_position()
    time.sleep(SPEED_STABILIZE_S)
    pos_after_tracking = combiner.get_position()

    assert combiner.ra.mode() == AxisMotionMode.TRACK
    assert combiner.dec.mode() == AxisMotionMode.TRACK

    if axis_name == "ra":
        assert abs(float(pos_after_tracking.ra) - float(pos_after_halt.ra)) < drift_tolerance
    else:
        assert abs(float(pos_after_tracking.dec) - float(pos_after_halt.dec)) < drift_tolerance

    if halt_name == "halt_all":
        assert combiner.ra._sky_speed == STELLAR_SPEED and combiner.dec._sky_speed == DecPerSecond(0)
    else:
        assert combiner.ra._sky_speed == expected_ra_sky_speed and combiner.dec._sky_speed == expected_dec_sky_speed


def test_hw_combiner_set_sky_speed(combiner: Combiner) -> None:
    combiner.set_sky_speed(STELLAR_SPEED, DecPerSecond(0))
    _wait_for_ra_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)

    time.sleep(3.0)

    pos = combiner.get_position()
    assert abs(float(pos.ra)) < RA_DRIFT_TOLERANCE_S
    assert abs(float(pos.dec)) < DEC_DRIFT_TOLERANCE_AS


def test_hw_combiner_set_moving_speed(combiner: Combiner) -> None:
    combiner.set_moving_speed(RA_SLEW_SPEED, DEC_SLEW_SPEED)
    _wait_for_ra_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)
    _wait_for_dec_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)

    time.sleep(SPEED_STABILIZE_S)

    assert combiner.ra._motor.status().direction != MotorDirection.STOP
    assert combiner.dec._motor.status().direction != MotorDirection.STOP


@pytest.mark.parametrize("direction", [SkyDirection.EAST, SkyDirection.WEST])
def test_hw_combiner_guide_ra(
    combiner: Combiner,
    direction: SkyDirection,
) -> None:
    combiner.guide(direction, 2500)
    _wait_for_ra_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)
    assert combiner.ra._motor.status().direction != MotorDirection.STOP


@pytest.mark.parametrize("direction", [SkyDirection.NORTH, SkyDirection.SOUTH])
def test_hw_combiner_guide_dec(
    combiner: Combiner,
    direction: SkyDirection,
) -> None:
    combiner.guide(direction, 2500)
    _wait_for_dec_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)
    assert combiner.dec._motor.status().direction != MotorDirection.STOP


def test_hw_combiner_guide_ra_changes_tracking_speed(combiner: Combiner) -> None:
    combiner.set_sky_speed(STELLAR_SPEED, DecPerSecond(0))
    _wait_for_ra_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)
    time.sleep(SPEED_STABILIZE_S)

    speed_before = _measure_ra_motor_speed_sps(combiner, SPEED_MEASURE_S)

    combiner.guide(SkyDirection.EAST, 2500)
    time.sleep(SPEED_STABILIZE_S)

    speed_after = _measure_ra_motor_speed_sps(combiner, SPEED_MEASURE_S)
    assert speed_after != pytest.approx(speed_before, rel=0.02)


@pytest.mark.parametrize(
    ("delta_ra", "delta_dec"),
    [
        (Ha(600), Dec(0)),
        (Ha(-600), Dec(0)),
        (Ha(0), Dec(600)),
        (Ha(0), Dec(-600)),
        (Ha(300), Dec(300)),
        (Ha(-300), Dec(-300)),
    ],
)
def test_hw_combiner_goto_reaches_target(
    combiner: Combiner,
    delta_ra: Ha,
    delta_dec: Dec,
) -> None:
    target = PointCoordinates(ra=Ha(float(delta_ra)), dec=Dec(float(delta_dec)))
    combiner.goto_to(target)
    _wait_for_goto_done(combiner, GOTO_TIMEOUT_S)

    final = combiner.get_position()
    assert abs(float(final.ra) - float(target.ra)) < GOTO_RA_TOLERANCE_S
    assert abs(float(final.dec) - float(target.dec)) < GOTO_DEC_TOLERANCE_AS


GUIDE_RA_FAST = STELLAR_SPEED + HaPerSecond(5)
GUIDE_RA_SLOW = STELLAR_SPEED - HaPerSecond(5)
GUIDE_DEC_NORTH = DecPerSecond(3)
GUIDE_DEC_SOUTH = DecPerSecond(-3)


@pytest.mark.parametrize(
    ("sky_ra", "sky_dec", "delta_ra", "delta_dec"),
    [
        (GUIDE_RA_FAST,  GUIDE_DEC_NORTH, Ha(600),  Dec(0)),
        (GUIDE_RA_SLOW,  DecPerSecond(0), Ha(-600), Dec(0)),
        (STELLAR_SPEED,  GUIDE_DEC_NORTH, Ha(0),    Dec(600)),
        (STELLAR_SPEED,  GUIDE_DEC_SOUTH, Ha(0),    Dec(-600)),
        (GUIDE_RA_FAST,  GUIDE_DEC_SOUTH, Ha(300),  Dec(300)),
        (GUIDE_RA_SLOW,  GUIDE_DEC_NORTH, Ha(-300), Dec(-300)),
    ],
    ids=[
        "fast_ra+dec_n__goto_ra+",
        "slow_ra__goto_ra-",
        "dec_n__goto_dec+",
        "dec_s__goto_dec-",
        "fast_ra+dec_s__goto_diag+",
        "slow_ra+dec_n__goto_diag-",
    ],
)
def test_hw_combiner_goto_from_guiding(
    combiner: Combiner,
    sky_ra: HaPerSecond,
    sky_dec: DecPerSecond,
    delta_ra: Ha,
    delta_dec: Dec,
) -> None:
    """GOTO must reach the target when sky speed differs from default sidereal."""
    combiner.set_sky_speed(sky_ra, sky_dec)
    _wait_for_ra_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)
    time.sleep(SPEED_STABILIZE_S)

    target = PointCoordinates(ra=Ha(float(delta_ra)), dec=Dec(float(delta_dec)))
    combiner.goto_to(target)
    _wait_for_goto_done(combiner, GOTO_TIMEOUT_S)

    final = combiner.get_position()
    assert abs(float(final.ra) - float(target.ra)) < GOTO_RA_TOLERANCE_S
    assert abs(float(final.dec) - float(target.dec)) < GOTO_DEC_TOLERANCE_AS


def test_hw_combiner_goto_zero_delta_is_noop(combiner: Combiner) -> None:
    combiner.goto_to(ZERO_POSITION)
    time.sleep(1.0)

    assert not combiner.is_moving_to()

    pos = combiner.get_position()
    assert abs(float(pos.ra)) < RA_POSITION_TOLERANCE_S
    assert abs(float(pos.dec)) < DEC_POSITION_TOLERANCE_AS


def test_hw_combiner_is_moving_to_during_goto(combiner: Combiner) -> None:
    target = PointCoordinates(ra=Ha(600), dec=Dec(600))
    combiner.goto_to(target)

    deadline = time.monotonic() + COMMAND_PROCESS_TIMEOUT_S
    while time.monotonic() < deadline:
        if combiner.is_moving_to():
            break
        time.sleep(POLL_INTERVAL_S)
    assert combiner.is_moving_to() is True

    _wait_for_goto_done(combiner, GOTO_TIMEOUT_S)
    assert combiner.is_moving_to() is False


def test_hw_combiner_move_wrong_speed_type_raises(combiner: Combiner) -> None:
    with pytest.raises(ValueError):
        combiner.move(SkyDirection.EAST, DecPerSecond(10))

    with pytest.raises(ValueError):
        combiner.move(SkyDirection.NORTH, HaPerSecond(10))
