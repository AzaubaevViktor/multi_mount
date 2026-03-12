import time

import pytest

from sky.axis import AxisMotionMode, PointCoordinates
from sky.combiner import Combiner
from sky.constants import STELLAR_SPEED
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond, SkyDirection
from tests.hw.test_3_combiner_hw import (
    COMMAND_PROCESS_TIMEOUT_S,
    POLL_INTERVAL_S,
    RA_SLEW_SPEED,
    SPEED_STABILIZE_S,
    _wait_for_dec_motor_running,
    _wait_for_goto_done,
    _wait_for_ra_motor_running,
    _wait_for_tracking_mode,
    combiner,
)


GUIDE_PULSE_MS = 2500
GUIDE_SETTLE_MARGIN_S = 1.5
GUIDE_RA_DIRECTION = SkyDirection.EAST
GUIDE_DEC_DIRECTION = SkyDirection.NORTH


def _prime_stable_polar_solution(comb: Combiner) -> tuple[HaPerSecond, DecPerSecond]:
    comb.set_position(PointCoordinates(ra=Ha(900), dec=Dec(1200)))
    _wait_for_tracking_mode(comb, COMMAND_PROCESS_TIMEOUT_S)
    time.sleep(SPEED_STABILIZE_S)

    stable_pulses = comb._polar_compensator.STABLE_GUIDE_PULSES_COUNT + 1

    for _ in range(stable_pulses):
        comb.guide(GUIDE_RA_DIRECTION, GUIDE_PULSE_MS)
        comb.guide(GUIDE_DEC_DIRECTION, GUIDE_PULSE_MS)
        time.sleep(0.1)

    deadline = time.monotonic() + COMMAND_PROCESS_TIMEOUT_S
    while time.monotonic() < deadline:
        if (
            comb._polar_compensator.stable_guide_ra_pulses_count >= comb._polar_compensator.STABLE_GUIDE_PULSES_COUNT
            and comb._polar_compensator.stable_guide_dec_pulses_count >= comb._polar_compensator.STABLE_GUIDE_PULSES_COUNT
        ):
            return comb._polar_compensator.ra_speed, comb._polar_compensator.dec_speed
        time.sleep(POLL_INTERVAL_S)

    pytest.fail("Polar compensator did not reach stable guide pulse counts")


def _wait_for_compensation_takeover(comb: Combiner) -> tuple[HaPerSecond, DecPerSecond]:
    deadline = time.monotonic() + float(comb.GUIDE_INTERVAL_S + comb._polar_compensator.DROP_GUIDE_PULSES_COUNT_AFTER) + GUIDE_SETTLE_MARGIN_S
    while time.monotonic() < deadline:
        speeds = comb._polar_compensator.get_guide_speeds()
        if speeds is not None:
            return speeds
        time.sleep(POLL_INTERVAL_S)

    pytest.fail("Polar compensator did not take over after external guiding stopped")


def test_hw_polar_compensator_replays_stable_guide_after_external_guiding_stops(combiner: Combiner) -> None:
    expected_ra_speed, expected_dec_speed = _prime_stable_polar_solution(combiner)

    takeover_ra_speed, takeover_dec_speed = _wait_for_compensation_takeover(combiner)
    time.sleep(float(combiner.GUIDE_INTERVAL_S) + GUIDE_SETTLE_MARGIN_S)

    assert combiner.ra.mode() == AxisMotionMode.TRACK
    assert combiner.dec.mode() == AxisMotionMode.TRACK
    assert takeover_ra_speed == pytest.approx(expected_ra_speed, abs=0.2)
    assert float(takeover_dec_speed) == pytest.approx(float(expected_dec_speed), abs=0.2)
    assert combiner.ra._sky_speed == pytest.approx(expected_ra_speed, abs=0.2)

    assert float(combiner.dec._sky_speed) == pytest.approx(float(expected_dec_speed), abs=0.2)

def test_hw_polar_compensator_changes_compensation_after_position_change(combiner: Combiner) -> None:
    _prime_stable_polar_solution(combiner)

    initial_ra_speed, initial_dec_speed = _wait_for_compensation_takeover(combiner)

    combiner.set_position(PointCoordinates(ra=Ha(1800), dec=Dec(1800)))
    time.sleep(float(combiner.GUIDE_INTERVAL_S) + GUIDE_SETTLE_MARGIN_S)

    updated_ra_speed = combiner.ra._sky_speed
    updated_dec_speed = combiner.dec._sky_speed

    assert float(updated_ra_speed) != pytest.approx(float(initial_ra_speed), abs=0.2)
    assert float(updated_dec_speed) != pytest.approx(float(initial_dec_speed), abs=0.2)


def test_hw_polar_compensator_does_not_apply_compensation_during_goto(combiner: Combiner) -> None:
    expected_ra_speed, expected_dec_speed = _prime_stable_polar_solution(combiner)

    target = PointCoordinates(ra=Ha(600), dec=Dec(600))
    combiner.goto_to(target)

    deadline = time.monotonic() + COMMAND_PROCESS_TIMEOUT_S
    while time.monotonic() < deadline:
        if combiner.is_moving_to():
            break
        time.sleep(POLL_INTERVAL_S)
    else:
        pytest.fail("GOTO never started")

    time.sleep(float(combiner.GUIDE_INTERVAL_S) + GUIDE_SETTLE_MARGIN_S)

    assert combiner.is_moving_to() is True
    assert combiner.ra.mode() == AxisMotionMode.GOTO or combiner.dec.mode() == AxisMotionMode.GOTO
    assert combiner.ra._sky_speed == pytest.approx(expected_ra_speed, abs=0.2)
    assert combiner.dec._sky_speed == pytest.approx(expected_dec_speed, abs=0.2)

    _wait_for_goto_done(combiner, COMMAND_PROCESS_TIMEOUT_S * 4)
    _wait_for_tracking_mode(combiner, COMMAND_PROCESS_TIMEOUT_S)


def test_hw_polar_compensator_does_not_apply_compensation_during_move(combiner: Combiner) -> None:
    expected_ra_speed, expected_dec_speed = _prime_stable_polar_solution(combiner)

    combiner.move(SkyDirection.WEST, RA_SLEW_SPEED)
    _wait_for_ra_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)
    time.sleep(float(combiner.GUIDE_INTERVAL_S) + GUIDE_SETTLE_MARGIN_S)

    assert combiner.ra.mode() == AxisMotionMode.SLEW
    assert combiner.ra._sky_speed == pytest.approx(expected_ra_speed, abs=0.2)
    assert combiner.dec._sky_speed == pytest.approx(expected_dec_speed, abs=0.2)

    combiner.halt_direction(SkyDirection.WEST)
    _wait_for_tracking_mode(combiner, COMMAND_PROCESS_TIMEOUT_S)
    _wait_for_dec_motor_running(combiner, COMMAND_PROCESS_TIMEOUT_S)


def test_hw_polar_compensator_resets_to_sidereal_without_stable_guiding(combiner: Combiner) -> None:
    combiner.guide(SkyDirection.EAST, 2500)
    combiner.guide(SkyDirection.NORTH, 1000)
    combiner.guide(SkyDirection.WEST, 2500)
    combiner.guide(SkyDirection.SOUTH, 1000)

    time.sleep(float(combiner._polar_compensator.DROP_GUIDE_PULSES_COUNT_AFTER) + GUIDE_SETTLE_MARGIN_S)

    assert combiner._polar_compensator.get_guide_speeds() == (STELLAR_SPEED, DecPerSecond(0))
    assert combiner._polar_compensator.ra_speed == STELLAR_SPEED
    assert combiner._polar_compensator.dec_speed == DecPerSecond(0)
    assert combiner.ra._sky_speed == STELLAR_SPEED
    assert combiner.dec._sky_speed == DecPerSecond(0)
    assert combiner.ra.mode() == AxisMotionMode.TRACK
    assert combiner.dec.mode() == AxisMotionMode.TRACK
