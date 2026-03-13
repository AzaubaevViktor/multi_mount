from sky.combiner import Combiner
from sky.physics import Second, SkyDirection


def test_dec_guide_north_zero_pulse_uses_backward_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.NORTH,
        Second.from_milliseconds(0),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert speed == Combiner.DEC_GUIDE_SPEED.backward


def test_dec_guide_north_half_interval_uses_default_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.NORTH,
        Second.from_milliseconds(2500),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert speed == Combiner.DEC_GUIDE_SPEED.default


def test_dec_guide_north_full_interval_uses_forward_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.NORTH,
        Second.from_milliseconds(5000),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert speed == Combiner.DEC_GUIDE_SPEED.forward


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


def test_dec_guide_south_half_interval_uses_default_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.SOUTH,
        Second.from_milliseconds(2500),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert speed == Combiner.DEC_GUIDE_SPEED.default


def test_dec_guide_south_small_pulse_produces_positive_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.SOUTH,
        Second.from_milliseconds(1000),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert float(speed) > 0


def test_dec_guide_south_large_pulse_produces_negative_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.SOUTH,
        Second.from_milliseconds(4000),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert float(speed) < 0
