import time
from collections.abc import Iterator

import pytest

from serial_wrapper.wrapper import SerialLine
from sky.axis import AxisDEC, AxisMotionMode, AxisRA, PointCoordinates
from sky.combiner import Combiner
from sky.constants import STELLAR_SPEED
from sky.lx200 import SkyLX200
from sky.physics import Dec, DecPerSecond, Ha
from skywatcher.motor import SkyWatcherMotor
from tmc2209.motor import TMC2209Motor
from tests.hw._sky_lx200_polar_helpers import (
    GUIDE_SETTLE_MARGIN_S,
    get_stable_compensation_via_lx200,
)


RA_DEVICE_PATTERN: str = "PL2303G-USBtoUART"
RA_SERIAL_BAUD: int = 112500
RA_SERIAL_TIMEOUT_S: float = 0.2

DEC_DEVICE_PATTERN: str = r"^tty\.usbserial.*$"
DEC_SERIAL_BAUD: int = 115200
DEC_SERIAL_TIMEOUT_S: float = 2.0


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

    try:
        yield comb
    finally:
        comb.disconnect()


@pytest.fixture(scope="module")
def sky_lx200(combiner: Combiner) -> Iterator[SkyLX200]:
    handler = SkyLX200(combiner)
    handler.connect()
    try:
        yield handler
    finally:
        handler.stop()


def test_sky_lx200_polar_compensator_replays_stable_guide_after_external_guiding_stops(
    combiner: Combiner,
    sky_lx200: SkyLX200,
) -> None:
    (expected_ra_speed, expected_dec_speed), (takeover_ra_speed, takeover_dec_speed) = (
        get_stable_compensation_via_lx200(combiner, sky_lx200)
    )
    time.sleep(float(combiner.GUIDE_INTERVAL_S) + GUIDE_SETTLE_MARGIN_S)

    assert combiner.ra.mode() == AxisMotionMode.TRACK
    assert combiner.dec.mode() == AxisMotionMode.TRACK
    assert takeover_ra_speed == pytest.approx(expected_ra_speed, abs=0.2)
    assert float(takeover_dec_speed) == pytest.approx(float(expected_dec_speed), abs=0.2)
    assert combiner.ra._sky_speed == pytest.approx(expected_ra_speed, abs=0.2)
    assert float(combiner.dec._sky_speed) == pytest.approx(float(expected_dec_speed), abs=0.2)


def test_sky_lx200_polar_compensator_changes_compensation_after_position_change(
    combiner: Combiner,
    sky_lx200: SkyLX200,
) -> None:
    (_, _), (initial_ra_speed, initial_dec_speed) = get_stable_compensation_via_lx200(combiner, sky_lx200)

    sky_lx200.sync_telescope(Ha(1800), Dec(1800))
    time.sleep(float(combiner.GUIDE_INTERVAL_S) + GUIDE_SETTLE_MARGIN_S)

    updated_ra_speed = combiner.ra._sky_speed
    updated_dec_speed = combiner.dec._sky_speed

    assert float(updated_ra_speed) != pytest.approx(float(initial_ra_speed), abs=0.2)
    assert float(updated_dec_speed) != pytest.approx(float(initial_dec_speed), abs=0.2)


def test_sky_lx200_polar_compensator_resets_to_sidereal_after_new_external_guiding(
    combiner: Combiner,
    sky_lx200: SkyLX200,
) -> None:
    (_, _), (initial_ra_speed, initial_dec_speed) = get_stable_compensation_via_lx200(combiner, sky_lx200)
    assert combiner.ra._sky_speed == pytest.approx(initial_ra_speed, abs=0.2)
    assert float(combiner.dec._sky_speed) == pytest.approx(float(initial_dec_speed), abs=0.2)

    # New external guiding with different speeds should disable internal compensation.
    sky_lx200.guide_east(1000)
    sky_lx200.guide_north(1000)
    time.sleep(0.5)

    assert combiner._polar_compensator.get_guide_speeds() is None

    combiner._polar_compensator.last_ra_guide_pulse -= combiner._polar_compensator.STOP_AXIS_AFTER
    assert combiner._polar_compensator.get_guide_speeds() == (STELLAR_SPEED, None)
    assert combiner._polar_compensator.ra_speed == STELLAR_SPEED

    combiner._polar_compensator.last_guide_pulse -= combiner._polar_compensator.DROP_GUIDE_PULSES_COUNT_AFTER
    # After guide pulses stop and stability is lost, compensator must reset to sidereal.
    time.sleep(float(combiner._polar_compensator.DROP_GUIDE_PULSES_COUNT_AFTER) + GUIDE_SETTLE_MARGIN_S)

    speeds_after = combiner._polar_compensator.get_guide_speeds()
    assert speeds_after is None
    assert combiner._polar_compensator.ra_speed == STELLAR_SPEED
    assert combiner._polar_compensator.dec_speed == DecPerSecond(0)
    assert combiner._polar_compensator.eps_E is None
    assert combiner._polar_compensator.eps_N is None
    assert combiner.ra._sky_speed == STELLAR_SPEED
    assert combiner.dec._sky_speed == DecPerSecond(0)
    assert combiner.ra.mode() == AxisMotionMode.TRACK
    assert combiner.dec.mode() == AxisMotionMode.TRACK
