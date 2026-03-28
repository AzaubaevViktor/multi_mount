import pytest

from sky.constants import STELLAR_SPEED
from sky.lx200 import SkyLX200


class _FakeCombiner:
    def __init__(self) -> None:
        self.calls: list[tuple[object, object]] = []

    def set_moving_speed(self, ra_speed, dec_speed) -> None:
        self.calls.append((ra_speed, dec_speed))


@pytest.mark.parametrize(
    ("method_name", "expected_ra_multiplier", "expected_dec_speed"),
    [
        ("set_slew_to_guide", 1.0, 5.0),
        ("set_slew_to_center", 20.0, 100.0),
        ("set_slew_to_find", 40.0, 1000.0),
        ("set_slew_to_max", 80.0, 2000.0),
    ],
)
def test_sky_lx200_slew_speed_presets_use_ranked_ra_speeds(
    method_name: str,
    expected_ra_multiplier: float,
    expected_dec_speed: float,
) -> None:
    combiner = _FakeCombiner()
    sky_lx200 = SkyLX200(combiner)  # type: ignore[arg-type]

    assert getattr(sky_lx200, method_name)() is True

    assert float(sky_lx200._manual_ra_speed) == pytest.approx(expected_ra_multiplier * float(STELLAR_SPEED))
    assert float(sky_lx200._manual_dec_speed) == pytest.approx(expected_dec_speed)
    assert len(combiner.calls) == 1
    assert float(combiner.calls[0][0]) == pytest.approx(expected_ra_multiplier * float(STELLAR_SPEED))
    assert float(combiner.calls[0][1]) == pytest.approx(expected_dec_speed)
