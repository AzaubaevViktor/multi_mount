from __future__ import annotations

from sky.axis import AxisMotionMode
from sky.combiner import Combiner, GuideDirection
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond
from sky.polar_compensator import PolarCompensator
from tests.base.fakes import AxisRecorder


def test_guide_speed_scales_with_duration() -> None:
    combiner = Combiner(
        ra_axis=AxisRecorder(position=Ha(0.0), tracking_speed=HaPerSecond(0.0), moves=[]),
        dec_axis=AxisRecorder(position=Dec(0.0), tracking_speed=DecPerSecond(0.0), moves=[]),
        polar_compensator=PolarCompensator(),
    )

    short_speed = combiner.guide_speed(GuideDirection.NORTH, 500)
    long_speed = combiner.guide_speed(GuideDirection.NORTH, 2000)

    assert short_speed.degrees_per_second > 0
    assert long_speed.degrees_per_second > short_speed.degrees_per_second


def test_guide_routes_direction_to_the_matching_axis() -> None:
    ra_axis = AxisRecorder(position=Ha(0.0), tracking_speed=HaPerSecond(0.0), moves=[])
    dec_axis = AxisRecorder(position=Dec(0.0), tracking_speed=DecPerSecond(0.0), moves=[])
    combiner = Combiner(ra_axis=ra_axis, dec_axis=dec_axis, polar_compensator=PolarCompensator())

    combiner.guide(GuideDirection.SOUTH, 1000)

    assert ra_axis.moves == []
    assert len(dec_axis.moves) == 1
    speed, mode = dec_axis.moves[0]
    assert mode == AxisMotionMode.GUIDE
    assert speed.degrees_per_second < 0
