import random

import pytest

from sky.physics import Dec, Ha, HaFormatError


@pytest.mark.parametrize(
    "value, expected",
    [
        ("00:00:00", (0, 0, 0)),
        ("01:02:03", (1, 2, 3)),
        ("23:59:59", (23, 59, 59)),
    ],
)
def test_hours_from_string_valid(value, expected):
    hours = Ha.from_string(value)
    assert (hours.hours, hours.minutes, hours.seconds) == expected
    assert str(hours) == value


@pytest.mark.parametrize(
    "value",
    [
        "1:02:03",
        "01:2:03",
        "01:02",
        "01-02-03",
        "ab:cd:ef",
        "",
    ],
)
def test_hours_from_string_invalid_format(value):
    with pytest.raises(HaFormatError):
        Ha.from_string(value)


@pytest.mark.parametrize(
    "value, expected",
    [
        (0, "00:00:00"),
        (1, "00:00:01"),
        (60, "00:01:00"),
        (3600, "01:00:00"),
        (86399, "23:59:59"),
    ],
)
def test_hours_from_seconds_valid(value, expected):
    hours = Ha(value)
    assert str(hours) == expected
    assert float(hours) == value


@pytest.mark.parametrize(
    "value, expected",
    [
        (0.0, "00:00:00"),
        (1.5, "01:30:00"),
        (2.25, "02:15:00"),
    ],
)
def test_hours_from_hours_valid(value, expected):
    hours = Ha(value * 3600)
    assert str(hours) == expected
    assert hours.to_hours_deg() == pytest.approx(value * 15)


@pytest.mark.parametrize(
    "value, expected",
    [
        (59.9, "00:01:00"),
        (3599.9, "01:00:00"),
        (86399.9, "00:00:00"),
    ],
)
def test_hours_rounding_for_components(value, expected):
    hours = Ha(value)
    assert str(hours) == expected


@pytest.mark.parametrize(
    "value, expected",
    [
        ("+00*00:00", (0, 0, 0)),
        ("-12*34:56", (12, 34, 56)),
        ("+12*34:56", (12, 34, 56)),
    ],
)
def test_dec_from_string_valid(value, expected):
    dec = Dec.from_string(value)
    assert (dec.degrees, dec.arcminutes, dec.arcseconds) == expected
    assert str(dec) == value


@pytest.mark.parametrize(
    "value",
    [
        "12*34:56",
        "+1*02:03",
        "+12:34:56",
        "+12*3456",
        "+12*34:5",
        "+12*34\"56",
        "+12*34'56",
        "+12*34\u201956",
        "",
    ],
)
def test_dec_from_string_invalid_format(value):
    with pytest.raises(ValueError):
        Dec.from_string(value)


@pytest.mark.parametrize(
    "value, expected",
    [
        (0.0, "+00*00:00"),
        (12.5, "+12*30:00"),
        (-12.0, "-12*00:00"),
        (45.25, "+45*15:00"),
    ],
)
def test_dec_from_degrees_valid(value, expected):
    dec = Dec(value * 3600)
    assert str(dec) == expected
    assert dec.to_degrees() == pytest.approx(value)


def test_dec_to_degrees_negative():
    dec = Dec(-5400)
    assert dec.to_degrees() == pytest.approx(-1.5)


@pytest.mark.parametrize(
    "value, expected_text",
    [
        (59.9, "+00*01:00"),
        (-59.9, "-00*01:00"),
        (3599.9, "+01*00:00"),
        (-3599.9, "-01*00:00"),
    ],
)
def test_dec_from_arcseconds(value, expected_text):
    dec = Dec(value)
    assert str(dec) == expected_text
    assert float(dec) == pytest.approx(value)
    assert dec.to_degrees() == pytest.approx(value / 3600)


@pytest.mark.parametrize(
    "arcseconds",
    (
        "random",
        -233381,
        244108,
        65134,
    ),
)
def test_dec_roundtrip_random_degrees(arcseconds):
    if arcseconds == "random":
        rng = random.Random()
        arcseconds_ = [rng.randrange(-90 * 3600, 90 * 3600 + 1) for _ in range(100)]
    else:
        arcseconds_ = [arcseconds]

    for total_arcseconds in arcseconds_:
        degrees = total_arcseconds / 3600
        value = str(Dec(degrees * 3600))
        roundtrip = Dec.from_string(value)
        assert roundtrip.to_degrees() == pytest.approx(degrees, abs=1.0 / 3600), total_arcseconds
