import random
from unittest.mock import patch

import pytest

from sky.constants import STELLAR_SPEED
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond, Second
from sky.polar_compensator import PolarCompensator, compute_guide_speeds, compute_pole_offset


# ---------------------------------------------------------------------------
#  Standalone functions
# ---------------------------------------------------------------------------

class TestComputePoleOffset:
    def test_raises_on_zero_declination(self):
        with pytest.raises(ValueError, match="too close to 0"):
            compute_pole_offset(DecPerSecond(0.1), HaPerSecond(1.0), Ha(3600), Dec(0))

    @pytest.mark.parametrize("ha_hours,dec_deg", [
        (0, 45),
        (3, 30),
        (6, 60),
        (12, 80),
        (18, 20),
    ])
    def test_round_trip_with_compute_guide_speeds(self, ha_hours: float, dec_deg: float):
        eps_E = Ha(50)
        eps_N = Dec(75)
        ha = Ha(ha_hours * 3600)
        dec = Dec(dec_deg * 3600)

        ra_speed, dec_speed = compute_guide_speeds(eps_E, eps_N, ha, dec)
        recovered_E, recovered_N = compute_pole_offset(dec_speed, ra_speed, ha, dec)

        assert float(recovered_E) == pytest.approx(float(eps_E), abs=1e-6)
        assert float(recovered_N) == pytest.approx(float(eps_N), abs=1e-6)


    @pytest.mark.parametrize("ha_hours", [0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22])
    def test_round_trip_small_offset_low_declination(self, ha_hours: float):
        eps_E = Ha(3)
        eps_N = Dec(5)
        ha = Ha(ha_hours * 3600)
        dec = Dec(1 * 3600)

        ra_speed, dec_speed = compute_guide_speeds(eps_E, eps_N, ha, dec)
        recovered_E, recovered_N = compute_pole_offset(dec_speed, ra_speed, ha, dec)

        assert float(recovered_E) == pytest.approx(float(eps_E), abs=1e-6), \
            f"eps_E mismatch at HA={ha_hours}h dec=1°: expected {float(eps_E)}, got {float(recovered_E)}"
        assert float(recovered_N) == pytest.approx(float(eps_N), abs=1e-6), \
            f"eps_N mismatch at HA={ha_hours}h dec=1°: expected {float(eps_N)}, got {float(recovered_N)}"

    @pytest.mark.parametrize("eps_E_hours", [0, 3, 6, 9, 12, 15, 18, 21])
    def test_round_trip_small_eps_N_any_eps_E(self, eps_E_hours: float):
        eps_E = Ha(eps_E_hours * 3600)
        eps_N = Dec(1 * 3600)

        rng = random.Random(eps_E_hours)
        for i in range(200):
            ha_hours = rng.uniform(0, 24)
            dec_deg = rng.choice([-1, 1]) * rng.uniform(1, 89)
            ha = Ha(ha_hours * 3600)
            dec = Dec(dec_deg * 3600)

            ra_speed, dec_speed = compute_guide_speeds(eps_E, eps_N, ha, dec)
            recovered_E, recovered_N = compute_pole_offset(dec_speed, ra_speed, ha, dec)

            tag = f"eps_E={eps_E_hours}h eps_N=1° HA={ha_hours:.2f}h dec={dec_deg:.2f}° (sample {i})"
            assert float(recovered_E) == pytest.approx(float(eps_E), abs=1e-4), \
                f"eps_E mismatch: {tag}, got {float(recovered_E)}"
            assert float(recovered_N) == pytest.approx(float(eps_N), abs=1e-4), \
                f"eps_N mismatch: {tag}, got {float(recovered_N)}"

    def test_round_trip_random_values(self):
        rng = random.Random(42)
        n_samples = 200

        for i in range(n_samples):
            eps_E_val = rng.uniform(-300, 300)
            eps_N_val = rng.uniform(-300, 300)
            ha_hours = rng.uniform(0, 24)
            dec_deg = rng.choice([-1, 1]) * rng.uniform(1, 89)

            eps_E = Ha(eps_E_val)
            eps_N = Dec(eps_N_val)
            ha = Ha(ha_hours * 3600)
            dec = Dec(dec_deg * 3600)

            ra_speed, dec_speed = compute_guide_speeds(eps_E, eps_N, ha, dec)
            recovered_E, recovered_N = compute_pole_offset(dec_speed, ra_speed, ha, dec)

            tag = f"sample {i}: eps_E={eps_E_val:.4f} eps_N={eps_N_val:.4f} HA={ha_hours:.2f}h dec={dec_deg:.2f}°"
            assert float(recovered_E) == pytest.approx(float(eps_E), abs=1e-4), \
                f"eps_E mismatch — {tag}"
            assert float(recovered_N) == pytest.approx(float(eps_N), abs=1e-4), \
                f"eps_N mismatch — {tag}"


class TestComputeGuideSpeeds:
    def test_zero_offset_returns_sidereal_and_zero(self):
        ra, dec = compute_guide_speeds(Ha(0), Dec(0), Ha(6 * 3600), Dec(45 * 3600))
        assert float(ra) == pytest.approx(float(STELLAR_SPEED), abs=1e-9)
        assert float(dec) == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
#  PolarCompensator class
# ---------------------------------------------------------------------------

def _make_compensator(t0: float = 0.0) -> PolarCompensator:
    with patch("time.monotonic", return_value=t0):
        return PolarCompensator()


def _send_pulse_pair(
    comp: PolarCompensator,
    ra_speed: HaPerSecond,
    dec_speed: DecPerSecond,
    t: float,
) -> None:
    with patch("time.monotonic", return_value=t):
        comp.guide_ra(ra_speed)
        comp.guide_dec(dec_speed)


def _send_stable_pulses(
    comp: PolarCompensator,
    ra_speed: HaPerSecond,
    dec_speed: DecPerSecond,
    count: int,
    t0: float,
    interval: float = 1.0,
) -> None:
    for i in range(count):
        _send_pulse_pair(comp, ra_speed, dec_speed, t0 + i * interval)


class TestPolarCompensatorInitial:
    def test_get_polar_offset_returns_zero(self):
        comp = _make_compensator()
        eps_E, eps_N = comp.get_polar_offset()
        assert float(eps_E) == 0.0
        assert float(eps_N) == 0.0

    def test_get_guide_speeds_returns_sidereal(self):
        comp = _make_compensator()
        with patch("time.monotonic", return_value=1.0):
            speeds = comp.get_guide_speeds()
        assert speeds is None
        assert float(comp.ra_speed) == pytest.approx(float(STELLAR_SPEED), abs=1e-9)
        assert float(comp.dec_speed) == pytest.approx(0.0, abs=1e-9)


class TestStablePulseCounting:
    RA_SPEED = STELLAR_SPEED + HaPerSecond(0.01)
    DEC_SPEED = DecPerSecond(0.02)

    def test_enough_stable_pulses_produce_nonzero_offset(self):
        comp = _make_compensator()
        comp.update_position(Ha(6 * 3600), Dec(45 * 3600))

        _send_stable_pulses(comp, self.RA_SPEED, self.DEC_SPEED,
                            count=PolarCompensator.STABLE_GUIDE_PULSES_COUNT, t0=1.0)

        eps_E, eps_N = comp.get_polar_offset()
        assert float(eps_E) != 0.0 or float(eps_N) != 0.0

    def test_one_fewer_pulse_returns_zero_offset(self):
        comp = _make_compensator()
        comp.update_position(Ha(6 * 3600), Dec(45 * 3600))

        _send_stable_pulses(comp, self.RA_SPEED, self.DEC_SPEED,
                            count=PolarCompensator.STABLE_GUIDE_PULSES_COUNT - 1, t0=1.0)

        eps_E, eps_N = comp.get_polar_offset()
        assert float(eps_E) == 0.0
        assert float(eps_N) == 0.0


class TestSpeedChangeResetsCounter:
    BASE_RA = STELLAR_SPEED + HaPerSecond(0.01)
    BASE_DEC = DecPerSecond(0.02)

    def test_ra_speed_jump_resets_ra_counter(self):
        comp = _make_compensator()
        comp.update_position(Ha(6 * 3600), Dec(45 * 3600))

        _send_stable_pulses(comp, self.BASE_RA, self.BASE_DEC, count=4, t0=1.0)

        jumped_ra = STELLAR_SPEED + HaPerSecond(0.2)
        _send_pulse_pair(comp, jumped_ra, self.BASE_DEC, t=5.0)

        _send_stable_pulses(comp, jumped_ra, self.BASE_DEC, count=4, t0=6.0)

        eps_E, eps_N = comp.get_polar_offset()
        assert float(eps_E) == 0.0 and float(eps_N) == 0.0

    def test_dec_speed_jump_resets_dec_counter(self):
        comp = _make_compensator()
        comp.update_position(Ha(6 * 3600), Dec(45 * 3600))

        _send_stable_pulses(comp, self.BASE_RA, self.BASE_DEC, count=4, t0=1.0)

        jumped_dec = DecPerSecond(0.2)
        _send_pulse_pair(comp, self.BASE_RA, jumped_dec, t=5.0)

        _send_stable_pulses(comp, self.BASE_RA, jumped_dec, count=4, t0=6.0)

        eps_E, eps_N = comp.get_polar_offset()
        assert float(eps_E) == 0.0 and float(eps_N) == 0.0


class TestTimeoutResetsCounters:
    RA_SPEED = STELLAR_SPEED + HaPerSecond(0.01)
    DEC_SPEED = DecPerSecond(0.02)

    def test_long_gap_resets_first_axis_counter(self):
        comp = _make_compensator()
        comp.update_position(Ha(6 * 3600), Dec(45 * 3600))

        _send_stable_pulses(comp, self.RA_SPEED, self.DEC_SPEED, count=4, t0=1.0)

        t_late = 4.0 + float(PolarCompensator.DROP_GUIDE_PULSES_COUNT_AFTER) + 1
        _send_pulse_pair(comp, self.RA_SPEED, self.DEC_SPEED, t=t_late)

        eps_E, eps_N = comp.get_polar_offset()
        assert float(eps_E) == 0.0 and float(eps_N) == 0.0


class TestGetGuideSpeeds:
    RA_SPEED = STELLAR_SPEED + HaPerSecond(0.01)
    DEC_SPEED = DecPerSecond(0.02)
    HA = Ha(6 * 3600)
    DEC = Dec(45 * 3600)

    def test_returns_last_known_speeds_when_unstable(self):
        comp = _make_compensator()
        _send_stable_pulses(comp, self.RA_SPEED, self.DEC_SPEED, count=2, t0=1.0)
        current_ra = comp.ra_speed
        current_dec = comp.dec_speed

        with patch("time.monotonic", return_value=3.5):
            speeds = comp.get_guide_speeds()

        assert speeds is None
        assert float(comp.ra_speed) == pytest.approx(float(current_ra), abs=1e-9)
        assert float(comp.dec_speed) == pytest.approx(float(current_dec), abs=1e-9)

    def test_resets_to_sidereal_after_timeout_when_unstable(self):
        comp = _make_compensator()
        _send_stable_pulses(comp, self.RA_SPEED, self.DEC_SPEED, count=2, t0=1.0)

        t_late = 100.0
        with patch("time.monotonic", return_value=t_late):
            speeds = comp.get_guide_speeds()

        assert speeds is None
        assert float(comp.ra_speed) == pytest.approx(float(STELLAR_SPEED), abs=1e-9)
        assert float(comp.dec_speed) == pytest.approx(0.0, abs=1e-9)

    def test_computes_from_polar_offset_when_stable(self):
        comp = _make_compensator()
        comp.update_position(self.HA, self.DEC)

        _send_stable_pulses(comp, self.RA_SPEED, self.DEC_SPEED,
                            count=PolarCompensator.STABLE_GUIDE_PULSES_COUNT, t0=1.0)

        with patch("time.monotonic", return_value=100.0):
            ra, dec = comp.get_guide_speeds()

        eps_E, eps_N = comp.get_polar_offset()
        expected_ra, expected_dec = compute_guide_speeds(eps_E, eps_N, self.HA, self.DEC)

        assert float(ra) == pytest.approx(float(expected_ra), abs=1e-9)
        assert float(dec) == pytest.approx(float(expected_dec), abs=1e-9)

    def test_polar_offset_changes_after_position_update(self):
        comp = _make_compensator()
        comp.update_position(self.HA, self.DEC)

        _send_stable_pulses(comp, self.RA_SPEED, self.DEC_SPEED,
                            count=PolarCompensator.STABLE_GUIDE_PULSES_COUNT, t0=1.0)

        eps_E1, eps_N1 = comp.get_polar_offset()

        comp.update_position(Ha(0), Dec(20 * 3600))
        eps_E2, eps_N2 = comp.get_polar_offset()

        assert float(eps_E1) != pytest.approx(float(eps_E2), abs=1e-9) or \
               float(eps_N1) != pytest.approx(float(eps_N2), abs=1e-9)


class TestWithinToleranceBoundary:
    def test_exactly_at_tolerance_is_stable(self):
        comp = _make_compensator()
        comp.update_position(Ha(6 * 3600), Dec(45 * 3600))

        base_ra = STELLAR_SPEED
        shifted_ra = HaPerSecond(float(STELLAR_SPEED) * (1 + (PolarCompensator.RA_SPEED_TOLERANCE_PERCENT * 0.99 / 100)))

        _send_pulse_pair(comp, base_ra, DecPerSecond(0), t=1.0)
        _send_stable_pulses(comp, shifted_ra, DecPerSecond(0),
                            count=PolarCompensator.STABLE_GUIDE_PULSES_COUNT, t0=2.0)

        assert comp.stable_guide_ra_pulses_count >= PolarCompensator.STABLE_GUIDE_PULSES_COUNT

    def test_beyond_tolerance_is_unstable(self):
        comp = _make_compensator()

        base_ra = STELLAR_SPEED

        _send_pulse_pair(comp, base_ra, DecPerSecond(0), t=1.0)

        jumped_ra = HaPerSecond(float(STELLAR_SPEED) * (1 + (PolarCompensator.RA_SPEED_TOLERANCE_PERCENT * 1.01 / 100)))
        _send_pulse_pair(comp, jumped_ra, DecPerSecond(0), t=2.0)

        assert comp.stable_guide_ra_pulses_count == 0


class TestAverageGuideSpeeds:
    def test_keeps_requested_polar_deviations_in_average(self):
        comp = _make_compensator()
        target_ra = STELLAR_SPEED * 1.112
        target_dec = DecPerSecond(-1.0)

        for i in range(PolarCompensator.STABLE_GUIDE_PULSES_COUNT * 10):
            _send_pulse_pair(comp, target_ra, target_dec, t=1.0 + i)

        assert float(comp.ra_speed) == pytest.approx(float(target_ra), abs=1e-9)
        assert float(comp.dec_speed) == pytest.approx(float(target_dec), abs=1e-9)
