import pytest

from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond, Second


@pytest.mark.parametrize(
    ("operation", "expected_type", "expected_value"),
    [
        pytest.param(lambda: Second(6) + Second(2), Second, 8, id="second_add"),
        pytest.param(lambda: Second(6) - Second(2), Second, 4, id="second_sub"),
        pytest.param(lambda: Second(6) * 2, Second, 12, id="second_mul_scalar"),
        pytest.param(lambda: 2 * Second(6), Second, 12, id="second_rmul_scalar"),
        pytest.param(lambda: Second(6) / 2, Second, 3, id="second_div_scalar"),
        pytest.param(lambda: Second(6) / Second(2), float, 3, id="second_div_second"),
        pytest.param(lambda: Ha(6) + Ha(2), Ha, 8, id="ha_add"),
        pytest.param(lambda: Ha(6) - Ha(2), Ha, 4, id="ha_sub"),
        pytest.param(lambda: Ha(6) / 2, Ha, 3, id="ha_div_scalar"),
        pytest.param(lambda: Ha(6) / Ha(2), float, 3, id="ha_div_ha"),
        pytest.param(lambda: Ha(6) / Second(2), HaPerSecond, 3, id="ha_div_second"),
        pytest.param(lambda: Ha(6) / HaPerSecond(2), Second, 3, id="ha_div_ha_speed"),
        pytest.param(lambda: HaPerSecond(6) + HaPerSecond(2), HaPerSecond, 8, id="ha_speed_add"),
        pytest.param(lambda: HaPerSecond(6) - HaPerSecond(2), HaPerSecond, 4, id="ha_speed_sub"),
        pytest.param(lambda: HaPerSecond(2) * 3, HaPerSecond, 6, id="ha_speed_mul_scalar"),
        pytest.param(lambda: HaPerSecond(6) / 2, HaPerSecond, 3, id="ha_speed_div_scalar"),
        pytest.param(lambda: HaPerSecond(6) / HaPerSecond(2), float, 3, id="ha_speed_div_ha_speed"),
        pytest.param(lambda: HaPerSecond(2) * Second(3), Ha, 6, id="ha_speed_mul_second"),
        pytest.param(lambda: Dec(6) + Dec(2), Dec, 8, id="dec_add"),
        pytest.param(lambda: Dec(6) - Dec(2), Dec, 4, id="dec_sub"),
        pytest.param(lambda: Dec(6) / 2, Dec, 3, id="dec_div_scalar"),
        pytest.param(lambda: Dec(6) / Dec(2), float, 3, id="dec_div_dec"),
        pytest.param(lambda: Dec(6) / Second(2), DecPerSecond, 3, id="dec_div_second"),
        pytest.param(lambda: Dec(6) / DecPerSecond(2), Second, 3, id="dec_div_dec_speed"),
        pytest.param(lambda: DecPerSecond(6) + DecPerSecond(2), DecPerSecond, 8, id="dec_speed_add"),
        pytest.param(lambda: DecPerSecond(6) - DecPerSecond(2), DecPerSecond, 4, id="dec_speed_sub"),
        pytest.param(lambda: DecPerSecond(2) * 3, DecPerSecond, 6, id="dec_speed_mul_scalar"),
        pytest.param(lambda: DecPerSecond(6) / 2, DecPerSecond, 3, id="dec_speed_div_scalar"),
        pytest.param(lambda: DecPerSecond(6) / DecPerSecond(2), float, 3, id="dec_speed_div_dec_speed"),
        pytest.param(lambda: DecPerSecond(2) * Second(3), Dec, 6, id="dec_speed_mul_second"),
    ],
)
def test_sky_physics_test_model(operation, expected_type, expected_value):
    result = operation()
    assert isinstance(result, expected_type)
    if expected_type is float:
        assert result == pytest.approx(expected_value)
    else:
        assert float(result) == pytest.approx(expected_value)
