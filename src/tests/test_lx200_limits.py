from lx200.base import LX200Commands, LX200Handler
from sky.physics import Dec


def test_handle_set_highest_elevation_accepts_valid_limit() -> None:
    handler = LX200Handler()

    result = handler._do_handle(LX200Commands.SET_HIGHEST_ELEVATION, "00*")

    assert result is True
    assert handler._highest_elevation == Dec(0)


def test_handle_set_minimum_elevation_accepts_valid_limit() -> None:
    handler = LX200Handler()

    result = handler._do_handle(LX200Commands.SET_MINIMUM_ELEVATION, "00*")

    assert result is True
    assert handler._minimum_elevation == Dec(0)
