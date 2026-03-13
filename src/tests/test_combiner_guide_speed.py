from sky.combiner import Combiner
from sky.physics import Second, SkyDirection


def test_dec_guide_north_small_pulse_produces_negative_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.NORTH,
        Second.from_milliseconds(1000),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert float(speed) < 0


def test_dec_guide_north_large_pulse_produces_positive_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.NORTH,
        Second.from_milliseconds(4000),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert float(speed) > 0

