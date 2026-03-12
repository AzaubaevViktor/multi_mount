import time
from collections.abc import Iterator

import pytest

from serial_wrapper.wrapper import SerialLine
from sky.axis import AxisDEC, AxisMotionMode, AxisRA, PointCoordinates
from sky.combiner import Combiner
from sky.constants import STELLAR_SPEED
from sky.lx200 import SkyLX200
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond, SkyDirection
from skywatcher.motor import SkyWatcherMotor
from tmc2209.motor import TMC2209Motor


RA_DEVICE_PATTERN: str = "PL2303G-USBtoUART"
RA_SERIAL_BAUD: int = 112500
RA_SERIAL_TIMEOUT_S: float = 0.2

DEC_DEVICE_PATTERN: str = r"^tty\.usbserial.*$"
DEC_SERIAL_BAUD: int = 115200
DEC_SERIAL_TIMEOUT_S: float = 2.0

POLL_INTERVAL_S: float = 0.2
COMMAND_PROCESS_TIMEOUT_S: float = 5.0
GOTO_TIMEOUT_S: float = 60.0

RA_POSITION_TOLERANCE_S: float = 15.0
DEC_POSITION_TOLERANCE_AS: float = 60.0

RA_DRIFT_TOLERANCE_S: float = 5.0
DEC_DRIFT_TOLERANCE_AS: float = 5.0

RA_SLEW_SPEED: HaPerSecond = HaPerSecond(3)
DEC_SLEW_SPEED: DecPerSecond = DecPerSecond(20)

ZERO_POSITION: PointCoordinates = PointCoordinates(ra=Ha(0), dec=Dec(0))

GUIDE_PULSE_MS_VALUES: tuple[int, int] = (2500, 5000)


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
    axis_dec = AxisDEC(TMC2209Motor(dec_serial))

    comb: Combiner = Combiner(axis_ra, axis_dec)
    comb.connect()

    try:
        yield comb
    finally:
        comb.halt_all()
        time.sleep(2.0)
        comb.disconnect()


@pytest.fixture(scope="session")
def sky_lx200(combiner: Combiner) -> Iterator[SkyLX200]:
    handler = SkyLX200(combiner)
    handler.connect()
    try:
        yield handler
    finally:
        handler.stop()


@pytest.fixture(autouse=True)
def _reset_between_tests(combiner: Combiner) -> Iterator[None]:
    _do_reset(combiner)
    yield
    _do_reset(combiner)


def _do_reset(comb: Combiner) -> None:
    comb.halt_all()
    comb.set_sky_speed(STELLAR_SPEED, DecPerSecond(0))
    comb.set_position(ZERO_POSITION)
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


def _wait_for_tracking_mode(comb: Combiner, timeout_s: float) -> None:
    deadline: float = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if comb.ra.mode() == AxisMotionMode.TRACK and comb.dec.mode() == AxisMotionMode.TRACK:
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(
        f"Axes did not reach TRACK mode within {timeout_s}s: "
        f"ra={comb.ra.mode().value}, dec={comb.dec.mode().value}"
    )


def _wait_for_position_near(
    comb: Combiner,
    target: PointCoordinates,
    ra_tol: float,
    dec_tol: float,
    timeout_s: float,
) -> None:
    deadline: float = time.monotonic() + timeout_s
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


def test_sky_lx200_alignment_mode_is_polar(sky_lx200: SkyLX200) -> None:
    assert sky_lx200.handle_alignment(b"") is not None


def test_sky_lx200_sync_telescope_updates_combiner_coordinates(
    sky_lx200: SkyLX200,
    combiner: Combiner,
) -> None:
    target_ra: Ha = Ha.from_string("12:00:00")
    target_dec: Dec = Dec.from_string("+20*00:00")

    assert sky_lx200.sync_telescope(target_ra, target_dec) is True

    pos = combiner.get_position()
    assert abs(float(pos.ra) - float(target_ra)) < RA_POSITION_TOLERANCE_S
    assert abs(float(pos.dec) - float(target_dec)) < DEC_POSITION_TOLERANCE_AS


def test_sky_lx200_get_telescope_coordinates_match_combiner(
    sky_lx200: SkyLX200,
    combiner: Combiner,
) -> None:
    combiner.set_position(PointCoordinates(ra=Ha(3600), dec=Dec(600)))
    _wait_for_position_near(
        combiner,
        PointCoordinates(ra=Ha(3600), dec=Dec(600)),
        ra_tol=RA_POSITION_TOLERANCE_S,
        dec_tol=DEC_POSITION_TOLERANCE_AS,
        timeout_s=COMMAND_PROCESS_TIMEOUT_S,
    )

    assert float(sky_lx200.get_telescope_ra()) == pytest.approx(3600, abs=RA_POSITION_TOLERANCE_S)
    assert float(sky_lx200.get_telescope_dec()) == pytest.approx(600, abs=DEC_POSITION_TOLERANCE_AS)


@pytest.mark.parametrize(
    ("set_speed_method", "expect_ra_speed", "expect_dec_speed"),
    [
        ("set_slew_to_guide", SkyLX200.GUIDE_RA_SPEED, SkyLX200.GUIDE_DEC_SPEED),
        ("set_slew_to_center", SkyLX200.CENTER_RA_SPEED, SkyLX200.CENTER_DEC_SPEED),
        ("set_slew_to_find", SkyLX200.FIND_RA_SPEED, SkyLX200.FIND_DEC_SPEED),
        ("set_slew_to_max", SkyLX200.MAX_RA_SPEED, SkyLX200.MAX_DEC_SPEED),
    ],
)
def test_sky_lx200_slew_speed_presets_update_manual_speeds(
    sky_lx200: SkyLX200,
    combiner: Combiner,
    set_speed_method: str,
    expect_ra_speed: HaPerSecond,
    expect_dec_speed: DecPerSecond,
) -> None:
    getattr(sky_lx200, set_speed_method)()

    # Execute a small move to ensure that preset speeds are actually used.
    sky_lx200.move_west()
    sky_lx200.move_north()

    time.sleep(2.0)
    pos = combiner.get_position()
    assert abs(float(pos.ra)) > RA_DRIFT_TOLERANCE_S
    assert abs(float(pos.dec)) > DEC_DRIFT_TOLERANCE_AS


@pytest.mark.parametrize(
    ("ra_sign", "dec_sign"),
    (
        pytest.param(-1, 0, id="east"),
        pytest.param(1, 0, id="west"),
        pytest.param(0, 1, id="north"),
        pytest.param(0, -1, id="south"),
        pytest.param(1, 1, id="west-north"),
        pytest.param(1, -1, id="west-south"),
        pytest.param(-1, 1, id="east-north"),
        pytest.param(-1, -1, id="east-south"),
    ),
)
def test_sky_lx200_manual_slew_directions(
    sky_lx200: SkyLX200,
    combiner: Combiner,
    ra_sign: int,
    dec_sign: int,
) -> None:
    combiner.set_moving_speed(RA_SLEW_SPEED, DEC_SLEW_SPEED)
    start = combiner.get_position()

    if ra_sign > 0:
        sky_lx200.move_west()
    elif ra_sign < 0:
        sky_lx200.move_east()

    if dec_sign > 0:
        sky_lx200.move_north()
    elif dec_sign < 0:
        sky_lx200.move_south()

    time.sleep(3.0)
    end = combiner.get_position()

    ra_delta = float(end.ra) - float(start.ra)
    dec_delta = float(end.dec) - float(start.dec)

    if ra_sign != 0:
        assert ra_delta * ra_sign > 0
    else:
        assert abs(ra_delta) < RA_DRIFT_TOLERANCE_S

    if dec_sign != 0:
        assert dec_delta * dec_sign > 0
    else:
        assert abs(dec_delta) < DEC_DRIFT_TOLERANCE_AS


GOTO_DIRECTION_TARGETS = (
    pytest.param(600.0, 0.0, id="goto-ra-plus"),
    pytest.param(-600.0, 0.0, id="goto-ra-minus"),
    pytest.param(0.0, 1200.0, id="goto-dec-plus"),
    pytest.param(0.0, -1200.0, id="goto-dec-minus"),
    pytest.param(600.0, 1200.0, id="goto-ra-plus-dec-plus"),
    pytest.param(600.0, -1200.0, id="goto-ra-plus-dec-minus"),
    pytest.param(-600.0, 1200.0, id="goto-ra-minus-dec-plus"),
    pytest.param(-600.0, -1200.0, id="goto-ra-minus-dec-minus"),
)


@pytest.mark.parametrize(
    ("target_ra_s", "target_dec_as"),
    GOTO_DIRECTION_TARGETS,
)
def test_sky_lx200_slew_to_moves_to_target(
    sky_lx200: SkyLX200,
    combiner: Combiner,
    target_ra_s: float,
    target_dec_as: float,
) -> None:
    target = PointCoordinates(ra=Ha(target_ra_s), dec=Dec(target_dec_as))

    assert sky_lx200.slew_to(target.ra, target.dec) is True

    _wait_for_position_near(
        combiner,
        target,
        ra_tol=RA_POSITION_TOLERANCE_S,
        dec_tol=DEC_POSITION_TOLERANCE_AS,
        timeout_s=GOTO_TIMEOUT_S,
    )


def test_sky_lx200_halt_all_stops_motion_and_restores_tracking(
    sky_lx200: SkyLX200,
    combiner: Combiner,
) -> None:
    # Start motion in both axes
    combiner.set_moving_speed(RA_SLEW_SPEED, DEC_SLEW_SPEED)
    sky_lx200.move_west()
    sky_lx200.move_north()
    time.sleep(2.0)

    assert sky_lx200.halt_all() is True
    _wait_for_tracking_mode(combiner, COMMAND_PROCESS_TIMEOUT_S)

    pos_after_halt = combiner.get_position()
    time.sleep(2.0)
    pos_after_tracking = combiner.get_position()

    assert combiner.ra.mode() == AxisMotionMode.TRACK
    assert combiner.dec.mode() == AxisMotionMode.TRACK
    assert abs(float(pos_after_tracking.ra) - float(pos_after_halt.ra)) < RA_DRIFT_TOLERANCE_S
    assert abs(float(pos_after_tracking.dec) - float(pos_after_halt.dec)) < DEC_DRIFT_TOLERANCE_AS


@pytest.mark.parametrize(
    "pulse_ms",
    (
        pytest.param(GUIDE_PULSE_MS_VALUES[0], id="pulse-1000ms"),
        pytest.param(GUIDE_PULSE_MS_VALUES[1], id="pulse-2500ms"),
    ),
)
@pytest.mark.parametrize(
    ("guide_method", "direction"),
    (
        ("guide_east", SkyDirection.EAST),
        ("guide_west", SkyDirection.WEST),
        ("guide_north", SkyDirection.NORTH),
        ("guide_south", SkyDirection.SOUTH),
    ),
)
def test_sky_lx200_guide_pulses_change_tracking_speed(
    sky_lx200: SkyLX200,
    combiner: Combiner,
    pulse_ms: int,
    guide_method: str,
    direction: SkyDirection,
) -> None:
    ra_speed_before = combiner.ra._sky_speed
    dec_speed_before = combiner.dec._sky_speed

    getattr(sky_lx200, guide_method)(pulse_ms)

    time.sleep(0.5)

    if direction in (SkyDirection.EAST, SkyDirection.WEST):
        assert combiner.ra._sky_speed != ra_speed_before
        assert combiner.dec._sky_speed == dec_speed_before
    else:
        assert combiner.dec._sky_speed != dec_speed_before
        assert combiner.ra._sky_speed == ra_speed_before


def test_sky_lx200_distance_indicator_tracks_goto_state(
    sky_lx200: SkyLX200,
    combiner: Combiner,
) -> None:
    assert sky_lx200.get_distance() == ""

    target = PointCoordinates(ra=Ha(600), dec=Dec(600))
    sky_lx200.slew_to(target.ra, target.dec)

    deadline = time.monotonic() + COMMAND_PROCESS_TIMEOUT_S
    while time.monotonic() < deadline:
        if combiner.is_moving_to():
            break
        time.sleep(POLL_INTERVAL_S)

    assert sky_lx200.get_distance() == "|"

    end_deadline = time.monotonic() + GOTO_TIMEOUT_S
    while time.monotonic() < end_deadline:
        if not combiner.is_moving_to():
            break
        time.sleep(POLL_INTERVAL_S)

    assert sky_lx200.get_distance() == ""


@pytest.mark.parametrize(
    ("move_method", "halt_method", "axis_name", "expect_sign"),
    (
        pytest.param("move_east", "halt_east", "ra", -1, id="halt-east"),
        pytest.param("move_west", "halt_west", "ra", 1, id="halt-west"),
        pytest.param("move_north", "halt_north", "dec", 1, id="halt-north"),
        pytest.param("move_south", "halt_south", "dec", -1, id="halt-south"),
    ),
)
def test_sky_lx200_halt_direction_stops_manual_slew(
    sky_lx200: SkyLX200,
    combiner: Combiner,
    move_method: str,
    halt_method: str,
    axis_name: str,
    expect_sign: int,
) -> None:
    combiner.set_moving_speed(RA_SLEW_SPEED, DEC_SLEW_SPEED)
    start = combiner.get_position()

    getattr(sky_lx200, move_method)()
    time.sleep(2.0)
    mid = combiner.get_position()

    if axis_name == "ra":
        delta = float(mid.ra) - float(start.ra)
    else:
        delta = float(mid.dec) - float(start.dec)
    assert delta * expect_sign > 0

    getattr(sky_lx200, halt_method)()
    _wait_for_tracking_mode(combiner, COMMAND_PROCESS_TIMEOUT_S)

    pos_after_halt = combiner.get_position()
    time.sleep(2.0)
    pos_after_tracking = combiner.get_position()

    assert combiner.ra.mode() == AxisMotionMode.TRACK
    assert combiner.dec.mode() == AxisMotionMode.TRACK

    if axis_name == "ra":
        assert abs(float(pos_after_tracking.ra) - float(pos_after_halt.ra)) < RA_DRIFT_TOLERANCE_S
    else:
        assert abs(float(pos_after_tracking.dec) - float(pos_after_halt.dec)) < DEC_DRIFT_TOLERANCE_AS


def test_sky_lx200_halt_all_resets_sky_speed(
    sky_lx200: SkyLX200,
    combiner: Combiner,
) -> None:
    custom_ra = STELLAR_SPEED + HaPerSecond(2)
    custom_dec = DecPerSecond(2)
    combiner.set_sky_speed(custom_ra, custom_dec)
    _wait_for_tracking_mode(combiner, COMMAND_PROCESS_TIMEOUT_S)

    assert combiner.ra._sky_speed == custom_ra
    assert combiner.dec._sky_speed == custom_dec

    sky_lx200.halt_all()
    _wait_for_tracking_mode(combiner, COMMAND_PROCESS_TIMEOUT_S)

    assert combiner.ra._sky_speed == STELLAR_SPEED
    assert combiner.dec._sky_speed == DecPerSecond(0)


def test_sky_lx200_halt_all_stops_goto(
    sky_lx200: SkyLX200,
    combiner: Combiner,
) -> None:
    start = combiner.get_position()
    target = PointCoordinates(ra=Ha(float(start.ra) + 600.0), dec=Dec(float(start.dec) + 600.0))

    sky_lx200.slew_to(target.ra, target.dec)
    deadline = time.monotonic() + COMMAND_PROCESS_TIMEOUT_S
    while time.monotonic() < deadline:
        if combiner.is_moving_to():
            break
        time.sleep(POLL_INTERVAL_S)

    assert combiner.is_moving_to() is True

    sky_lx200.halt_all()
    _wait_for_tracking_mode(combiner, COMMAND_PROCESS_TIMEOUT_S + GOTO_TIMEOUT_S)

    assert combiner.is_moving_to() is False
    assert combiner.ra.mode() == AxisMotionMode.TRACK
    assert combiner.dec.mode() == AxisMotionMode.TRACK
