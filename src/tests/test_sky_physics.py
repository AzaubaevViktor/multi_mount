import pytest

from sky.physics import Dec, DecPerSecond, HA, HAPerSecond, HaFormatError, Second


@pytest.mark.parametrize(
    "value, expected",
    [
        ("00:00:00", (0, 0, 0, 0)),
        ("01:02:03", (3723, 1, 2, 3)),
        ("23:59:59", (86399, 23, 59, 59)),
    ],
)
def test_ha_from_string_valid(value, expected):
    ha = HA.from_string(value)
    assert ha.to_raw() == expected[0]
    assert (ha.hours, ha.minutes, ha.seconds) == expected[1:]
    assert str(ha) == value


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
def test_ha_from_string_invalid_format(value):
    with pytest.raises(HaFormatError):
        HA.from_string(value)


@pytest.mark.parametrize(
    "value, expected_text",
    [
        (59.9, "00:01:00"),
        (3599.9, "01:00:00"),
        (86399.9, "00:00:00"),
    ],
)
def test_ha_rounding_for_components(value, expected_text):
    ha = HA(value)
    assert str(ha) == expected_text


@pytest.mark.parametrize(
    "value, expected_raw, expected_text",
    [
        (86461, 61, "00:01:01"),
        (-86461, -61, "-00:01:01"),
    ],
)
def test_ha_wraps_values_outside_single_circle(value, expected_raw, expected_text):
    ha = HA(value)
    assert ha.to_raw() == pytest.approx(expected_raw)
    assert str(ha) == expected_text


def test_ha_negation_returns_same_type_with_inverted_raw():
    ha = -HA(12.5)
    assert isinstance(ha, HA)
    assert ha.to_raw() == pytest.approx(-12.5)


def test_ha_divide_and_multiply_by_seconds_roundtrip():
    speed = HA(30) / Second(2)
    assert isinstance(speed, HAPerSecond)

    ha = speed * Second(4)
    assert isinstance(ha, HA)
    assert ha.to_raw() == pytest.approx(60)


def test_ha_division_rejects_unsupported_type():
    with pytest.raises(NotImplementedError):
        HA(30) / 2


@pytest.mark.parametrize(
    "value, expected_raw, expected_parts",
    [
        ("+00*00:00", 0, (0, 0, 0)),
        ("+12*34:56", 45296, (12, 34, 56)),
        ("-12*34:56", -45296, (12, 34, 56)),
    ],
)
def test_dec_from_string_valid(value, expected_raw, expected_parts):
    dec = Dec.from_string(value)
    assert dec.to_raw() == expected_raw
    assert (dec.degrees, dec.arcminutes, dec.arcseconds) == expected_parts
    assert str(dec) == value


@pytest.mark.parametrize(
    "value",
    [
        "12*34:56",
        "+1*02:03",
        "+12:34:56",
        "+12*3456",
        "+12*34:5",
        "",
    ],
)
def test_dec_from_string_invalid_format(value):
    with pytest.raises(ValueError):
        Dec.from_string(value)


@pytest.mark.parametrize(
    "value, expected_text",
    [
        (59.9, "+00*01:00"),
        (-59.9, "-00*01:00"),
        (3599.9, "+01*00:00"),
        (-3599.9, "-01*00:00"),
    ],
)
def test_dec_rounding_for_components(value, expected_text):
    dec = Dec(value)
    assert str(dec) == expected_text


@pytest.mark.parametrize(
    "value, expected_raw, expected_text",
    [
        (324061, 61, "+00*01:01"),
        (-324061, -61, "-00*01:01"),
    ],
)
def test_dec_wraps_values_outside_single_quarter_circle(value, expected_raw, expected_text):
    dec = Dec(value)
    assert dec.to_raw() == pytest.approx(expected_raw)
    assert str(dec) == expected_text


def test_dec_negation_returns_same_type_with_inverted_raw():
    dec = -Dec(12.5)
    assert isinstance(dec, Dec)
    assert dec.to_raw() == pytest.approx(-12.5)
    assert str(dec) == "-00*00:12"


def test_dec_divide_and_multiply_by_seconds_roundtrip():
    speed = Dec(30) / Second(2)
    assert isinstance(speed, DecPerSecond)

    dec = speed * Second(4)
    assert isinstance(dec, Dec)
    assert dec.to_raw() == pytest.approx(60)


def test_dec_division_rejects_unsupported_type():
    with pytest.raises(NotImplementedError):
        Dec(30) / 2
