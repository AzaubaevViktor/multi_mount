import random

import pytest

from skywatcher.skywatcher import SkyWatcherRevu24, SkyWatcherRevu24Error


@pytest.mark.parametrize(
    "value, expected",
    [
        (0, "000000"),
        (1, "010000"),
        (0x123456, "563412"),
        (0xFFFFFF, "FFFFFF"),
    ],
)
def test_revu24_from_int(value, expected):
    assert SkyWatcherRevu24.from_int(value) == expected


@pytest.mark.parametrize(
    "data, expected",
    [
        ("000000", 0),
        ("010000", 1),
        ("563412", 0x123456),
        ("ffffff", 0xFFFFFF),
    ],
)
def test_revu24_from_mount(data, expected):
    assert SkyWatcherRevu24.from_mount(data) == expected


def test_revu24_roundtrip_random_values():
    rng = random.Random(0)
    for _ in range(200):
        value = rng.randrange(0x1000000)
        encoded = SkyWatcherRevu24.from_int(value)
        assert SkyWatcherRevu24.from_mount(encoded) == value


@pytest.mark.parametrize(
    "data",
    [
        "",
        "12345",
        "G00000",
        "00000G",
    ],
)
def test_revu24_from_mount_invalid(data):
    with pytest.raises(SkyWatcherRevu24Error):
        SkyWatcherRevu24.from_mount(data)


@pytest.mark.parametrize(
    "value",
    [
        -1,
        0x1000000,
        None,
        "123",
    ],
)
def test_revu24_from_int_invalid(value):
    with pytest.raises(SkyWatcherRevu24Error):
        SkyWatcherRevu24.from_int(value)
