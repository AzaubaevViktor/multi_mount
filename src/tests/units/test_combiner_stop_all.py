from sky.combiner import Combiner
from sky.physics import DecPerSecond, HaPerSecond, Second, SkyDirection


class _StubPolarCompensator:
    def __init__(self) -> None:
        self.eps_E = object()
        self.eps_N = object()
        self.ra_speed = object()
        self.dec_speed = object()
        self.stable_guide_ra_pulses_count = 3
        self.stable_guide_dec_pulses_count = 4
        self.last_guide_pulse = Second(100)
        self.last_ra_guide_pulse = Second(101)
        self.last_dec_guide_pulse = Second(102)
        self.reset_calls = 0

    def reset(self, last_guide_pulse: Second = Second(0)) -> None:
        self.reset_calls += 1
        self.eps_E = None
        self.eps_N = None
        self.ra_speed = None
        self.dec_speed = None
        self.stable_guide_ra_pulses_count = 0
        self.stable_guide_dec_pulses_count = 0
        self.last_guide_pulse = last_guide_pulse
        self.last_ra_guide_pulse = last_guide_pulse
        self.last_dec_guide_pulse = last_guide_pulse


class _StubAxisRA:
    DIRECTIONS = (SkyDirection.EAST, SkyDirection.WEST)
    FORWARD_DIRECTION = SkyDirection.EAST

    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []

    def change_speed(self, direction, speed, update_sky_speed=False) -> None:
        self.calls.append(("change_speed", direction, speed, update_sky_speed))

    def halt_all(self) -> None:
        self.calls.append(("halt_all",))


class _StubAxisDEC:
    DIRECTIONS = (SkyDirection.NORTH, SkyDirection.SOUTH)
    FORWARD_DIRECTION = SkyDirection.NORTH

    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []

    def change_speed(self, direction, speed, update_sky_speed=False) -> None:
        self.calls.append(("change_speed", direction, speed, update_sky_speed))

    def halt_all(self) -> None:
        self.calls.append(("halt_all",))


def test_combiner_stop_all_stops_tracking_and_resets_guide_state() -> None:
    ra = _StubAxisRA()
    dec = _StubAxisDEC()
    combiner = Combiner(ra, dec)  # type: ignore[arg-type]
    combiner._polar_compensator = _StubPolarCompensator()  # type: ignore[assignment]

    combiner.stop_all()

    assert ra.calls == [
        ("halt_all",),
        ("change_speed", SkyDirection.EAST, HaPerSecond(0), False),
    ]
    assert dec.calls == [
        ("halt_all",),
        ("change_speed", SkyDirection.NORTH, DecPerSecond(0), False),
    ]
    assert combiner._polar_compensator.reset_calls == 1
    assert combiner._polar_compensator.eps_E is None
    assert combiner._polar_compensator.eps_N is None
    assert combiner._polar_compensator.stable_guide_ra_pulses_count == 0
    assert combiner._polar_compensator.stable_guide_dec_pulses_count == 0
    assert combiner._polar_compensator.last_guide_pulse == Second(0)
    assert combiner._polar_compensator.last_ra_guide_pulse == Second(0)
    assert combiner._polar_compensator.last_dec_guide_pulse == Second(0)
    assert combiner._guide_updated.is_set() is True
