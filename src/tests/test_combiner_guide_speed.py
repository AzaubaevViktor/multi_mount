from sky.combiner import Combiner
from sky.physics import DecPerSecond, Second, SkyDirection
from sky.axis import AxisDEC, AxisRA


class _StubPolarCompensator:
    def __init__(self) -> None:
        self.ra_speeds = []
        self.dec_speeds = []

    def guide_ra(self, speed) -> None:
        self.ra_speeds.append(speed)

    def guide_dec(self, speed) -> None:
        self.dec_speeds.append(speed)


class _StubAxisRA(AxisRA):
    def __init__(self) -> None:
        self.DIRECTIONS = (self.FORWARD_DIRECTION, self.BACKWARD_DIRECTION)
        self.calls = []

    def change_speed(self, direction, speed, update_sky_speed=False) -> None:
        self.calls.append((direction, speed, update_sky_speed))


class _StubAxisDEC(AxisDEC):
    def __init__(self) -> None:
        self.DIRECTIONS = (self.FORWARD_DIRECTION, self.BACKWARD_DIRECTION)
        self.calls = []

    def change_speed(self, direction, speed, update_sky_speed=False) -> None:
        self.calls.append((direction, speed, update_sky_speed))


def test_dec_guide_north_zero_pulse_uses_default_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.NORTH,
        Second.from_milliseconds(0),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert speed == Combiner.DEC_GUIDE_SPEED.default


def test_dec_guide_north_half_interval_uses_midpoint_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.NORTH,
        Second.from_milliseconds(2000),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert speed == DecPerSecond(1)


def test_dec_guide_north_full_interval_uses_forward_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.NORTH,
        Second.from_milliseconds(4000),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert speed == Combiner.DEC_GUIDE_SPEED.forward


def test_dec_guide_north_small_pulse_produces_positive_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.NORTH,
        Second.from_milliseconds(1000),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert float(speed) > 0


def test_dec_guide_north_large_pulse_produces_positive_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.NORTH,
        Second.from_milliseconds(4000),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert float(speed) > 0


def test_dec_guide_south_zero_pulse_uses_default_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.SOUTH,
        Second.from_milliseconds(0),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert speed == Combiner.DEC_GUIDE_SPEED.default


def test_dec_guide_south_half_interval_uses_midpoint_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.SOUTH,
        Second.from_milliseconds(2000),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert speed == DecPerSecond(-1)


def test_dec_guide_south_small_pulse_produces_negative_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.SOUTH,
        Second.from_milliseconds(1000),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert float(speed) < 0


def test_dec_guide_south_large_pulse_produces_negative_speed() -> None:
    speed = Combiner.DEC_GUIDE_SPEED.calculate_speed(
        SkyDirection.SOUTH,
        Second.from_milliseconds(4000),
        Combiner.GUIDE_INTERVAL_S,
    )

    assert float(speed) < 0


def test_combiner_guide_uses_ra_forward_direction_for_west_guide() -> None:
    ra = _StubAxisRA()
    dec = _StubAxisDEC()
    combiner = Combiner(ra, dec)  # type: ignore[arg-type]
    combiner._polar_compensator = _StubPolarCompensator()  # type: ignore[assignment]

    combiner.guide(SkyDirection.WEST, int(Combiner.GUIDE_INTERVAL_S.to_milliseconds()))

    assert ra.calls == [(SkyDirection.EAST, Combiner.RA_GUIDE_SPEED.backward, True)]


def test_combiner_guide_uses_dec_forward_direction_for_south_guide() -> None:
    ra = _StubAxisRA()
    dec = _StubAxisDEC()
    combiner = Combiner(ra, dec)  # type: ignore[arg-type]
    combiner._polar_compensator = _StubPolarCompensator()  # type: ignore[assignment]

    combiner.guide(SkyDirection.SOUTH, int(Combiner.GUIDE_INTERVAL_S.to_milliseconds()))

    assert dec.calls == [(SkyDirection.NORTH, Combiner.DEC_GUIDE_SPEED.backward, True)]
