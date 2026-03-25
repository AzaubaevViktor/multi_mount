from unittest.mock import patch

import pytest

from sky.constants import STELLAR_SPEED
from sky.physics import DecPerSecond, HaPerSecond, Second
from sky.polar_compensator import PolarCompensator


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
    def test_get_guide_speeds_returns_none_with_default_speeds(self):
        comp = _make_compensator()

        with patch("time.monotonic", return_value=1.0):
            speeds = comp.get_guide_speeds()

        assert speeds is None
        assert float(comp.ra_speed) == pytest.approx(float(STELLAR_SPEED), abs=1e-9)
        assert float(comp.dec_speed) == pytest.approx(0.0, abs=1e-9)
        assert comp.is_guiding is False

    def test_reset_clears_guide_history(self):
        comp = _make_compensator()
        comp.current_ha = object()  # type: ignore[assignment]
        comp.current_dec = object()  # type: ignore[assignment]
        comp.eps_E = object()  # type: ignore[assignment]
        comp.eps_N = object()  # type: ignore[assignment]
        comp.is_guiding = True
        _send_stable_pulses(
            comp,
            STELLAR_SPEED + HaPerSecond(0.01),
            DecPerSecond(0.02),
            count=PolarCompensator.STABLE_GUIDE_PULSES_COUNT,
            t0=1.0,
        )

        comp.reset(Second(123))

        assert comp.current_ha is None
        assert comp.current_dec is None
        assert comp.eps_E is None
        assert comp.eps_N is None
        assert comp.is_guiding is False
        assert comp.stable_guide_ra_pulses_count == 0
        assert comp.stable_guide_dec_pulses_count == 0
        assert comp.last_guide_pulse == Second(123)
        assert comp.last_ra_guide_pulse == Second(123)
        assert comp.last_dec_guide_pulse == Second(123)
        assert float(comp.ra_speed) == pytest.approx(float(STELLAR_SPEED), abs=1e-9)
        assert float(comp.dec_speed) == pytest.approx(0.0, abs=1e-9)


class TestStablePulseCounting:
    RA_SPEED = STELLAR_SPEED + HaPerSecond(0.01)
    DEC_SPEED = DecPerSecond(0.02)

    def test_enough_stable_pulses_mark_both_axes_stable(self):
        comp = _make_compensator()

        _send_stable_pulses(
            comp,
            self.RA_SPEED,
            self.DEC_SPEED,
            count=PolarCompensator.STABLE_GUIDE_PULSES_COUNT,
            t0=1.0,
        )

        assert comp.stable_guide_ra_pulses_count >= PolarCompensator.STABLE_GUIDE_PULSES_COUNT
        assert comp.stable_guide_dec_pulses_count >= PolarCompensator.STABLE_GUIDE_PULSES_COUNT

    def test_one_fewer_pulse_keeps_state_unstable(self):
        comp = _make_compensator()

        _send_stable_pulses(
            comp,
            self.RA_SPEED,
            self.DEC_SPEED,
            count=PolarCompensator.STABLE_GUIDE_PULSES_COUNT - 1,
            t0=1.0,
        )

        assert comp.stable_guide_ra_pulses_count < PolarCompensator.STABLE_GUIDE_PULSES_COUNT
        assert comp.stable_guide_dec_pulses_count < PolarCompensator.STABLE_GUIDE_PULSES_COUNT


class TestSpeedChangeResetsCounter:
    BASE_RA = STELLAR_SPEED + HaPerSecond(0.01)
    BASE_DEC = DecPerSecond(0.02)

    def test_ra_speed_jump_resets_ra_counter(self):
        comp = _make_compensator()

        _send_stable_pulses(comp, self.BASE_RA, self.BASE_DEC, count=4, t0=1.0)

        jumped_ra = STELLAR_SPEED + HaPerSecond(0.2)
        _send_pulse_pair(comp, jumped_ra, self.BASE_DEC, t=5.0)

        assert comp.stable_guide_ra_pulses_count == 0
        assert comp.stable_guide_dec_pulses_count >= 4

    def test_dec_speed_jump_resets_dec_counter(self):
        comp = _make_compensator()

        _send_stable_pulses(comp, self.BASE_RA, self.BASE_DEC, count=4, t0=1.0)

        jumped_dec = DecPerSecond(0.2)
        _send_pulse_pair(comp, self.BASE_RA, jumped_dec, t=5.0)

        assert comp.stable_guide_ra_pulses_count >= 4
        assert comp.stable_guide_dec_pulses_count == 0


class TestTimeoutResetsCounters:
    RA_SPEED = STELLAR_SPEED + HaPerSecond(0.01)
    DEC_SPEED = DecPerSecond(0.02)

    def test_long_gap_resets_first_axis_counter(self):
        comp = _make_compensator()

        _send_stable_pulses(comp, self.RA_SPEED, self.DEC_SPEED, count=4, t0=1.0)

        t_late = 4.0 + float(PolarCompensator.DROP_GUIDE_PULSES_COUNT_AFTER) + 1
        _send_pulse_pair(comp, self.RA_SPEED, self.DEC_SPEED, t=t_late)

        assert comp.stable_guide_ra_pulses_count == 1
        assert comp.stable_guide_dec_pulses_count >= 1


class TestGetGuideSpeeds:
    RA_SPEED = STELLAR_SPEED + HaPerSecond(0.01)
    DEC_SPEED = DecPerSecond(0.02)

    def test_returns_none_while_external_guiding_is_recent(self):
        comp = _make_compensator()
        _send_stable_pulses(
            comp,
            self.RA_SPEED,
            self.DEC_SPEED,
            count=PolarCompensator.STABLE_GUIDE_PULSES_COUNT,
            t0=1.0,
        )

        with patch("time.monotonic", return_value=6.0):
            speeds = comp.get_guide_speeds()

        assert speeds is None
        assert comp.is_guiding is False

    def test_replays_last_stable_speeds_after_external_guiding_stops(self):
        comp = _make_compensator()
        _send_stable_pulses(
            comp,
            self.RA_SPEED,
            self.DEC_SPEED,
            count=PolarCompensator.STABLE_GUIDE_PULSES_COUNT,
            t0=1.0,
        )
        expected_ra = comp.ra_speed
        expected_dec = comp.dec_speed

        with patch("time.monotonic", return_value=100.0):
            ra, dec = comp.get_guide_speeds()

        assert float(ra) == pytest.approx(float(expected_ra), abs=1e-9)
        assert float(dec) == pytest.approx(float(expected_dec), abs=1e-9)
        assert comp.eps_E is None
        assert comp.eps_N is None
        assert comp.is_guiding is True

    def test_resets_to_sidereal_after_timeout_when_unstable(self):
        comp = _make_compensator()
        _send_stable_pulses(comp, self.RA_SPEED, self.DEC_SPEED, count=2, t0=1.0)

        with patch("time.monotonic", return_value=100.0):
            speeds = comp.get_guide_speeds()

        assert speeds == (STELLAR_SPEED, DecPerSecond(0))
        assert float(comp.ra_speed) == pytest.approx(float(STELLAR_SPEED), abs=1e-9)
        assert float(comp.dec_speed) == pytest.approx(0.0, abs=1e-9)
        assert comp.is_guiding is False

    def test_stops_ra_axis_when_only_dec_guiding_continues(self):
        comp = _make_compensator()
        _send_stable_pulses(
            comp,
            self.RA_SPEED,
            self.DEC_SPEED,
            count=PolarCompensator.STABLE_GUIDE_PULSES_COUNT,
            t0=1.0,
        )

        with patch("time.monotonic", return_value=10.0):
            comp.last_ra_guide_pulse = Second(0)
            comp.last_dec_guide_pulse = Second(9.0)
            comp.last_guide_pulse = Second(9.0)
            speeds = comp.get_guide_speeds()

        assert speeds == (STELLAR_SPEED, None)
        assert comp.ra_speed == STELLAR_SPEED

    def test_stops_dec_axis_when_only_ra_guiding_continues(self):
        comp = _make_compensator()
        _send_stable_pulses(
            comp,
            self.RA_SPEED,
            self.DEC_SPEED,
            count=PolarCompensator.STABLE_GUIDE_PULSES_COUNT,
            t0=1.0,
        )

        with patch("time.monotonic", return_value=10.0):
            comp.last_ra_guide_pulse = Second(9.0)
            comp.last_dec_guide_pulse = Second(0)
            comp.last_guide_pulse = Second(9.0)
            speeds = comp.get_guide_speeds()

        assert speeds == (None, DecPerSecond(0))
        assert comp.dec_speed == DecPerSecond(0)


class TestWithinToleranceBoundary:
    def test_exactly_within_tolerance_is_stable(self):
        comp = _make_compensator()

        base_ra = STELLAR_SPEED
        shifted_ra = HaPerSecond(
            float(STELLAR_SPEED) * (1 + (PolarCompensator.RA_SPEED_TOLERANCE_PERCENT * 0.99 / 100))
        )

        _send_pulse_pair(comp, base_ra, DecPerSecond(0), t=1.0)
        _send_stable_pulses(
            comp,
            shifted_ra,
            DecPerSecond(0),
            count=PolarCompensator.STABLE_GUIDE_PULSES_COUNT,
            t0=2.0,
        )

        assert comp.stable_guide_ra_pulses_count >= PolarCompensator.STABLE_GUIDE_PULSES_COUNT

    def test_beyond_tolerance_is_unstable(self):
        comp = _make_compensator()

        base_ra = STELLAR_SPEED
        jumped_ra = HaPerSecond(
            float(STELLAR_SPEED) * (1 + (PolarCompensator.RA_SPEED_TOLERANCE_PERCENT * 1.01 / 100))
        )

        _send_pulse_pair(comp, base_ra, DecPerSecond(0), t=1.0)
        _send_pulse_pair(comp, jumped_ra, DecPerSecond(0), t=2.0)

        assert comp.stable_guide_ra_pulses_count == 0


class TestAverageGuideSpeeds:
    def test_keeps_requested_guide_speeds_in_average(self):
        comp = _make_compensator()
        target_ra = STELLAR_SPEED * 1.112
        target_dec = DecPerSecond(-1.0)

        for i in range(PolarCompensator.STABLE_GUIDE_PULSES_COUNT * 10):
            _send_pulse_pair(comp, target_ra, target_dec, t=1.0 + i)

        assert float(comp.ra_speed) == pytest.approx(float(target_ra), abs=1e-9)
        assert float(comp.dec_speed) == pytest.approx(float(target_dec), abs=1e-9)
