import random

import pytest

from lx200.protocols import (
    LX200Dec,
    LX200DecFormatError,
    LX200DecRangeError,
    LX200Ha,
    LX200HoursFormatError,
    LX200HoursRangeError,
)


@pytest.mark.parametrize(
    "value, expected",
    [
        ("00:00:00", (0, 0, 0)),
        ("01:02:03", (1, 2, 3)),
        ("23:59:59", (23, 59, 59)),
    ],
)
def test_hours_from_string_valid(value, expected):
    hours = LX200Ha.from_string(value)
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
    with pytest.raises(LX200HoursFormatError):
        LX200Ha.from_string(value)


@pytest.mark.parametrize(
    "value",
    [
        "24:00:00",
        "00:60:00",
        "00:00:60",
        "99:00:00",
    ],
)
def test_hours_from_string_range_errors(value):
    with pytest.raises(LX200HoursRangeError):
        LX200Ha.from_string(value)


@pytest.mark.parametrize(
    "parts",
    [
        (-1, 0, 0),
        (24, 0, 0),
        (0, -1, 0),
        (0, 60, 0),
        (0, 0, -1),
        (0, 0, 60),
    ],
)
def test_hours_init_range_errors(parts):
    with pytest.raises(LX200HoursRangeError):
        LX200Ha(*parts)


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
    hours = LX200Ha.from_seconds(value)
    assert str(hours) == expected
    assert hours.to_seconds() == value


@pytest.mark.parametrize(
    "value",
    [
        86400,
    ],
)
def test_hours_from_seconds_invalid(value):
    with pytest.raises(LX200HoursRangeError):
        LX200Ha.from_seconds(value)


@pytest.mark.parametrize(
    "value, expected",
    [
        (0.0, "00:00:00"),
        (1.5, "01:30:00"),
        (2.25, "02:15:00"),
    ],
)
def test_hours_from_hours_valid(value, expected):
    hours = LX200Ha.from_hours(value)
    assert str(hours) == expected
    assert hours.to_hours() == pytest.approx(value)


@pytest.mark.parametrize(
    "value",
    [
        24.0,
    ],
)
def test_hours_from_hours_invalid(value):
    with pytest.raises(LX200HoursRangeError):
        LX200Ha.from_hours(value)


def test_hours_repr():
    hours = LX200Ha(1, 2, 3)
    assert repr(hours) == "LX200Hours('01:02:03')"


@pytest.mark.parametrize(
    "value, expected",
    [
        (59.9, "00:01:00"),
        (3599.9, "01:00:00"),
        (86399.9, "00:00:00"),
    ],
)
def test_hours_rounding_for_components(value, expected):
    hours = LX200Ha.from_seconds(value)
    assert str(hours) == expected


@pytest.mark.parametrize(
    "value, expected",
    [
        ("+00*00:00", ("+", 0, 0, 0)),
        ("-12*34:56", ("-", 12, 34, 56)),
        ("+12*34:56", ("+", 12, 34, 56)),
    ],
)
def test_dec_from_string_valid(value, expected):
    dec = LX200Dec.from_string(value)
    assert (dec.sign, dec.degrees, dec.minutes, dec.seconds) == expected
    assert str(dec) == f"{expected[0]}{expected[1]:02d}*{expected[2]:02d}:{expected[3]:02d}"


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
    with pytest.raises(LX200DecFormatError):
        LX200Dec.from_string(value)


@pytest.mark.parametrize(
    "parts",
    [
        ("x", 0, 0, 0),
        ("+", -1, 0, 0),
        ("+", 91, 0, 0),
        ("+", 90, 1, 0),
        ("+", 90, 0, 1),
        ("+", 0, 60, 0),
        ("+", 0, 0, 60),
    ],
)
def test_dec_init_range_errors(parts):
    with pytest.raises(LX200DecRangeError):
        LX200Dec(*parts)


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
    dec = LX200Dec.from_degrees(value)
    assert str(dec) == expected
    assert dec.to_degrees() == pytest.approx(value)


@pytest.mark.parametrize(
    "value",
    [
        90.0002777778,
        -90.1,
    ],
)
def test_dec_from_degrees_invalid(value):
    with pytest.raises(LX200DecRangeError):
        LX200Dec.from_degrees(value)


def test_dec_to_degrees_negative():
    dec = LX200Dec("-", 1, 30, 0)
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
    dec = LX200Dec.from_arcseconds(value)
    assert str(dec) == expected_text
    assert dec.to_arcseconds() == pytest.approx(value)
    assert dec.to_degrees() == pytest.approx(value / 3600)


@pytest.mark.parametrize(
    "value",
    [
        90 * 3600 + 0.1,
        -(90 * 3600 + 0.1),
    ],
)
def test_dec_from_arcseconds_invalid(value):
    with pytest.raises(LX200DecRangeError):
        LX200Dec.from_arcseconds(value)


def test_dec_repr():
    dec = LX200Dec("+", 5, 6, 7)
    assert repr(dec) == "LX200Dec('+05*06:07')"

@pytest.mark.parametrize(
    'arcseconds', (
        'random',
        -233381,
        244108,
        65134,
    )
)
def test_dec_roundtrip_random_degrees(arcseconds):
    if arcseconds == "random":
        rng = random.Random()
        arcseconds_ = [rng.randrange(-90 * 3600, 90 * 3600 + 1) for _ in range(100)]
    else:
        arcseconds_: list[float] = [arcseconds]

    for total_arcseconds in arcseconds_:
        degrees = total_arcseconds / 3600
        value = str(LX200Dec.from_degrees(degrees))
        roundtrip = LX200Dec.from_string(value)
        assert roundtrip.to_degrees() == pytest.approx(degrees, abs=1./3600), total_arcseconds

