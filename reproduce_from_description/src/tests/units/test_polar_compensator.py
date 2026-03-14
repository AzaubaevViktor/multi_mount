from __future__ import annotations

import pytest

from sky.constants import SIDEREAL_RATE_HOURS_PER_SECOND
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond, PointCoordinates
from sky.polar_compensator import PolarCompensator, compute_guide_speeds, compute_pole_offset


def test_compute_pole_offset_round_trips_with_forward_model() -> None:
    eps_east = Ha(0.001)
    eps_north = Dec(0.25)
    position = PointCoordinates(ra=Ha(3.0), dec=Dec(45.0))

    ra_speed, dec_speed = compute_guide_speeds(eps_east, eps_north, position.ra, position.dec)
    recovered_east, recovered_north = compute_pole_offset(dec_speed, ra_speed, position.ra, position.dec)

    assert recovered_east.hours == pytest.approx(eps_east.hours, abs=1e-6)
    assert recovered_north.degrees == pytest.approx(eps_north.degrees, abs=1e-6)


def test_compute_pole_offset_rejects_declination_near_zero() -> None:
    with pytest.raises(ValueError):
        compute_pole_offset(DecPerSecond(0.001), HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND), Ha(1.0), Dec(0.0))


def test_stable_guiding_produces_offset_and_takeover_speeds_after_timeout() -> None:
    compensator = PolarCompensator(min_samples=3, guide_timeout_seconds=2.0, axis_stop_after_seconds=0.5)
    position = PointCoordinates(ra=Ha(2.0), dec=Dec(35.0))
    expected_ra = HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND + 0.0001)
    expected_dec = DecPerSecond(0.0002)
    compensator.update_position(position)
    for timestamp in [1.0, 1.3, 1.6]:
        compensator.guide_ra(expected_ra, timestamp=timestamp)
        compensator.guide_dec(expected_dec, timestamp=timestamp)

    offset = compensator.compute_pole_offset()
    takeover = compensator.takeover_speeds(position, timestamp=4.0)

    assert offset is not None
    assert takeover is not None
    assert takeover[0].hours_per_second != pytest.approx(SIDEREAL_RATE_HOURS_PER_SECOND, abs=1e-9)
    assert takeover[1].degrees_per_second != pytest.approx(0.0, abs=1e-9)


def test_speed_jump_resets_stability_before_offset_is_available() -> None:
    compensator = PolarCompensator(min_samples=3, guide_timeout_seconds=2.0, max_speed_jump_degrees_per_second=0.001)
    position = PointCoordinates(ra=Ha(2.0), dec=Dec(35.0))
    compensator.update_position(position)
    compensator.guide_ra(HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND + 0.0001), timestamp=1.0)
    compensator.guide_dec(DecPerSecond(0.0002), timestamp=1.0)
    compensator.guide_ra(HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND + 0.0001), timestamp=1.3)
    compensator.guide_dec(DecPerSecond(0.0002), timestamp=1.3)
    compensator.guide_ra(HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND + 0.01), timestamp=1.6)
    compensator.guide_dec(DecPerSecond(0.0002), timestamp=1.6)

    assert compensator.compute_pole_offset() is None


def test_unstable_guiding_resets_back_to_defaults_after_timeout() -> None:
    compensator = PolarCompensator(min_samples=3, guide_timeout_seconds=1.0, axis_stop_after_seconds=0.25)
    position = PointCoordinates(ra=Ha(2.0), dec=Dec(35.0))
    compensator.update_position(position)
    compensator.guide_ra(HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND + 0.0001), timestamp=1.0)
    compensator.guide_dec(DecPerSecond(0.0002), timestamp=1.0)
    compensator.guide_ra(HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND + 0.0001), timestamp=1.3)
    compensator.guide_dec(DecPerSecond(0.0002), timestamp=1.3)

    takeover = compensator.takeover_speeds(position, timestamp=3.0)

    assert takeover == (HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND), DecPerSecond(0.0))
