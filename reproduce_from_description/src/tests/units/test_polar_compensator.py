from __future__ import annotations

from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond, PointCoordinates
from sky.polar_compensator import PolarCompensator


def test_compute_offset_and_takeover_speeds_from_stable_guiding() -> None:
    compensator = PolarCompensator(min_samples=3, takeover_delay_seconds=0.5, takeover_max_seconds=10.0)
    position = PointCoordinates(ra=Ha(1.0), dec=Dec(20.0))
    timestamps = [1.0, 1.2, 1.4]
    for timestamp in timestamps:
        compensator.record_guide_speeds(HaPerSecond(0.001), DecPerSecond(0.002), position, timestamp=timestamp)

    offset = compensator.compute_pole_offset()
    takeover = compensator.takeover_speeds(position, timestamp=2.1)

    assert offset is not None
    assert round(offset.ra_bias.hours_per_second, 6) == 0.001
    assert round(offset.dec_bias.degrees_per_second, 6) == 0.002
    assert takeover == (HaPerSecond(-0.001), DecPerSecond(-0.002))


def test_reset_after_speed_jump() -> None:
    compensator = PolarCompensator(min_samples=3, max_speed_jump_degrees_per_second=0.01)
    position = PointCoordinates(ra=Ha(1.0), dec=Dec(20.0))
    compensator.record_guide_speeds(HaPerSecond(0.001), DecPerSecond(0.002), position, timestamp=1.0)
    compensator.record_guide_speeds(HaPerSecond(0.001), DecPerSecond(0.002), position, timestamp=1.1)
    compensator.record_guide_speeds(HaPerSecond(0.05), DecPerSecond(0.002), position, timestamp=1.2)

    assert compensator.compute_pole_offset() is None


def test_reset_after_takeover_timeout() -> None:
    compensator = PolarCompensator(min_samples=3, takeover_delay_seconds=0.5, takeover_max_seconds=1.0)
    position = PointCoordinates(ra=Ha(1.0), dec=Dec(20.0))
    for timestamp in [1.0, 1.2, 1.4]:
        compensator.record_guide_speeds(HaPerSecond(0.001), DecPerSecond(0.002), position, timestamp=timestamp)

    assert compensator.takeover_speeds(position, timestamp=3.0) is None
    assert compensator.compute_pole_offset() is None
