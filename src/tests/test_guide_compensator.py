

import math

import pytest

from lx200.guide_compensator import compute_pole_offset, compute_guide_rates
from sky.physics import Dec, Ha


@pytest.mark.parametrize(
        ("eps_n, eps_e, ha_deg, dec_deg"),
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
            pytest.param(  -1.0,  -1.0,  -15.0,  25.0, id="min_diag_1"),
            pytest.param(  -1.0,  1.0,  -15.0,  25.0, id="min_diag_2"),
            pytest.param(  1.0,  -1.0,  -15.0,  25.0, id="min_diag_3"),
            pytest.param(  1.0,  1.0,  -15.0,  25.0, id="min_diag_4"),
            pytest.param(  -.001,  -.001,  -15.0,  25.0, id="small_diag_1"),
            pytest.param(  -.001,  .001,  -15.0,  25.0, id="small_diag_2"),
            pytest.param(  .001,  -.001,  -15.0,  25.0, id="small_diag_3"),
            pytest.param(  .001,  .001,  -15.0,  25.0, id="small_diag_4"),
        ),
)
def test_simulated_polar_missaligment(eps_n: float, eps_e: float, ha_deg: float, dec_deg: float):
    # Forward problem: obtain d and k
    ha_drift, dec_drift = compute_guide_rates(Dec(eps_n), Ha(eps_e), Ha(ha_deg), Dec(dec_deg))
    
    # Inverse problem: recover the offset
    eps_N_calc, eps_E_calc = compute_pole_offset(dec_drift, ha_drift, Ha(ha_deg), Dec(dec_deg))

    err_N = eps_N_calc - Dec(eps_n)
    err_E = eps_E_calc - Ha(eps_e)
    assert float(abs(err_N)) < 1e-9 and float(abs(err_E)) < 1e-9

    print(f"{eps_n:>10.2f} {eps_e:>10.2f} {ha_deg:>7.1f} {dec_deg:>6.1f} "
            f"{eps_N_calc:>10.2f} {eps_E_calc:>10.2f} {err_N:>8.2e} {err_E:>8.2e}")
