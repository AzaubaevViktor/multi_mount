import pytest

from skywatcher.skywatcher import SkyWatcherMount


class _DummySerial:
    terminator = b"\r"
    encoding = "ascii"


@pytest.mark.parametrize(
    ("delta_seconds", "expected_abs_seconds", "expected_rate_sign"),
    (
        pytest.param(1000.0, 1000.0, 1, id="positive_delta_keeps_positive_rate"),
        pytest.param(-1000.0, 1000.0, -1, id="negative_delta_keeps_negative_rate"),
        pytest.param(50000.0, 36400.0, -1, id="positive_wraps_to_shortest_negative"),
        pytest.param(-50000.0, 36400.0, 1, id="negative_wraps_to_shortest_positive"),
    ),
)
def test_wrap_delta_move_keeps_direction(delta_seconds: float, expected_abs_seconds: float, expected_rate_sign: int) -> None:
    mount = SkyWatcherMount(_DummySerial())

    wrapped_seconds, rate = mount._do_wrap_delta_move(delta_seconds)

    assert wrapped_seconds == pytest.approx(expected_abs_seconds)
    assert (rate > 0) == (expected_rate_sign > 0)
