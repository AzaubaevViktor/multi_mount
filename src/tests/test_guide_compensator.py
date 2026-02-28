

import math

import pytest

from lx200.guide_compensator import compute_pole_offset


def simulate_guide_rates(eps_N: float, eps_E: float, HA_deg: float, dec_deg: float) -> tuple[float, float]:
    """
    По известному смещению полюса и позиции звезды вычисляет
    теоретические d и k — прямая задача.
    """
    omega = 15.0
    HA  = math.radians(HA_deg)
    dec = math.radians(dec_deg)
    
    d = omega * (eps_N * math.cos(HA) - eps_E * math.sin(HA))
    k = 1.0 + math.tan(dec) * (eps_N * math.sin(HA) + eps_E * math.cos(HA))
    return d, k


@pytest.mark.parametrize(
        ("e_n, e_e, ha_deg, dec_deg"),
        (
            pytest.param(  60.0,    0.0,    0.0,  45.0, id="low_alt_mer_mid_dec"),
            pytest.param(   0.0,  120.0,    0.0,  45.0, id="west_axis_mer"),
            pytest.param( -80.0,   50.0,   30.0,  30.0, id="high_west_ha2h_d30"),
            pytest.param( 200.0, -150.0,  -45.0,  60.0, id="big_err_ha_m3h_d60"),
            pytest.param(  10.0,   10.0,   90.0,  20.0, id="small_err_ha6h_d20"),
            pytest.param(-300.0,  300.0,  -30.0,  50.0, id="very_big_err"),
            pytest.param(  45.0,  -45.0,   60.0,  70.0, id="high_dec_ha4h"),
            pytest.param( -20.0,  -80.0,  -60.0,  35.0, id="ha_m4h"),
            pytest.param( 100.0,    5.0,   15.0,  55.0, id="mostly_alt"),
            pytest.param(  -5.0,  180.0,  -15.0,  25.0, id="mostly_az"),
        ),
)
def test_simulated_polar_missaligment(e_n: float, e_e: float, ha_deg: float, dec_deg: float):
    # Прямая задача: получаем d и k
    d, k = simulate_guide_rates(e_n, e_e, ha_deg, dec_deg)
    
    # Обратная задача: восстанавливаем смещение
    eps_N_calc, eps_E_calc = compute_pole_offset(d, k, ha_deg, dec_deg)

    err_N = eps_N_calc - e_n
    err_E = eps_E_calc - e_e
    assert abs(err_N) < 1e-9 and abs(err_E) < 1e-9

    print(f"{e_n:>10.2f} {e_e:>10.2f} {ha_deg:>7.1f} {dec_deg:>6.1f} "
            f"{eps_N_calc:>10.2f} {eps_E_calc:>10.2f} {err_N:>8.2e} {err_E:>8.2e}")
