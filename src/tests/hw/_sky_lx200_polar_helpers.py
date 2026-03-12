import time

import pytest

from sky.axis import AxisMotionMode, PointCoordinates
from sky.combiner import Combiner
from sky.lx200 import SkyLX200
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond, SkyDirection


GUIDE_PULSE_MS: int = 2500
GUIDE_SETTLE_MARGIN_S: float = 1.5
GUIDE_RA_DIRECTION: SkyDirection = SkyDirection.EAST
GUIDE_DEC_DIRECTION: SkyDirection = SkyDirection.NORTH
COMMAND_PROCESS_TIMEOUT_S: float = 5.0
POLL_INTERVAL_S: float = 0.2


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


def prime_stable_polar_solution_via_lx200(comb: Combiner, sky: SkyLX200) -> tuple[HaPerSecond, DecPerSecond]:
    comb.set_position(PointCoordinates(ra=Ha(900), dec=Dec(1200)))
    _wait_for_tracking_mode(comb, COMMAND_PROCESS_TIMEOUT_S)
    time.sleep(1.0)

    stable_pulses = comb._polar_compensator.STABLE_GUIDE_PULSES_COUNT + 1

    for _ in range(stable_pulses):
        sky.guide_east(GUIDE_PULSE_MS)
        sky.guide_north(GUIDE_PULSE_MS)
        time.sleep(0.1)

    deadline: float = time.monotonic() + COMMAND_PROCESS_TIMEOUT_S
    while time.monotonic() < deadline:
        if (
            comb._polar_compensator.stable_guide_ra_pulses_count >= comb._polar_compensator.STABLE_GUIDE_PULSES_COUNT
            and comb._polar_compensator.stable_guide_dec_pulses_count >= comb._polar_compensator.STABLE_GUIDE_PULSES_COUNT
        ):
            return comb._polar_compensator.ra_speed, comb._polar_compensator.dec_speed
        time.sleep(POLL_INTERVAL_S)

    pytest.fail("Polar compensator did not reach stable guide pulse counts via SkyLX200")


def wait_for_compensation_takeover(comb: Combiner) -> tuple[HaPerSecond, DecPerSecond]:
    deadline: float = time.monotonic() + float(
        comb.GUIDE_INTERVAL_S + comb._polar_compensator.DROP_GUIDE_PULSES_COUNT_AFTER
    ) + GUIDE_SETTLE_MARGIN_S
    while time.monotonic() < deadline:
        speeds = comb._polar_compensator.get_guide_speeds()
        if speeds is not None:
            return speeds
        time.sleep(POLL_INTERVAL_S)

    pytest.fail("Polar compensator did not take over after external guiding stopped (SkyLX200)")


def get_stable_compensation_via_lx200(
    comb: Combiner,
    sky: SkyLX200,
) -> tuple[tuple[HaPerSecond, DecPerSecond], tuple[HaPerSecond, DecPerSecond]]:
    expected_ra_speed, expected_dec_speed = prime_stable_polar_solution_via_lx200(comb, sky)
    takeover_ra_speed, takeover_dec_speed = wait_for_compensation_takeover(comb)
    return (expected_ra_speed, expected_dec_speed), (takeover_ra_speed, takeover_dec_speed)

