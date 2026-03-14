from __future__ import annotations

from sky.constants import SIDEREAL_RATE_HOURS_PER_SECOND
from sky.combiner import Combiner, GuideDirection
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond
from sky.polar_compensator import PolarCompensator
from tests.base.fakes import AxisRecorder


def test_dec_guide_speed_uses_default_midpoint_and_forward_values() -> None:
    combiner = Combiner(
        ra_axis=AxisRecorder(position=Ha(0.0), tracking_speed=HaPerSecond(0.0), moves=[], speed_changes=[]),
        dec_axis=AxisRecorder(position=Dec(0.0), tracking_speed=DecPerSecond(0.0), moves=[], speed_changes=[]),
        polar_compensator=PolarCompensator(),
    )

    zero = combiner.guide_speed(GuideDirection.NORTH, 0)
    midpoint = combiner.guide_speed(GuideDirection.NORTH, 2000)
    full = combiner.guide_speed(GuideDirection.NORTH, 4000)

    assert zero == DecPerSecond(0.0)
    assert midpoint == DecPerSecond(full.degrees_per_second / 2.0)
    assert full.degrees_per_second > 0.0


def test_ra_guide_uses_tracking_centered_speed_profile() -> None:
    combiner = Combiner(
        ra_axis=AxisRecorder(position=Ha(0.0), tracking_speed=HaPerSecond(0.0), moves=[], speed_changes=[]),
        dec_axis=AxisRecorder(position=Dec(0.0), tracking_speed=DecPerSecond(0.0), moves=[], speed_changes=[]),
        polar_compensator=PolarCompensator(),
    )

    east = combiner.guide_speed(GuideDirection.EAST, 4000)
    west = combiner.guide_speed(GuideDirection.WEST, 4000)

    assert east.hours_per_second > SIDEREAL_RATE_HOURS_PER_SECOND
    assert west.hours_per_second < SIDEREAL_RATE_HOURS_PER_SECOND
    assert east.hours_per_second > west.hours_per_second


def test_guide_routes_direction_to_speed_updates_on_the_matching_axis() -> None:
    ra_axis = AxisRecorder(position=Ha(0.0), tracking_speed=HaPerSecond(0.0), moves=[], speed_changes=[])
    dec_axis = AxisRecorder(position=Dec(0.0), tracking_speed=DecPerSecond(0.0), moves=[], speed_changes=[])
    combiner = Combiner(ra_axis=ra_axis, dec_axis=dec_axis, polar_compensator=PolarCompensator())

    combiner.guide(GuideDirection.SOUTH, 1000)

    assert ra_axis.moves == []
    assert dec_axis.moves == []
    assert dec_axis.speed_changes is not None
    assert len(dec_axis.speed_changes) == 1
    assert dec_axis.speed_changes[0].degrees_per_second < 0.0
