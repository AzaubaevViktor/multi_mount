"""The ``Second.monotonic()`` clock seam: default is real time, tests swap it.

``Axis`` and ``PolarCompensator`` take no clock in their constructors; they read
elapsed time through ``sky.physics.Second.monotonic()``. ``set_clock()`` points
that classmethod at the simulator's virtual clock, which is what makes
axis-level chaos tests possible without spending real seconds.
"""

import time

import pytest

from clock import REAL_CLOCK
from sim.clock import Clock as VirtualClock
from sky.axis import AxisMotionMode, AxisRA
from sky.constants import STELLAR_SPEED
from sky.motor import MotionMode, MotorDirection, MotorStatus
from sky.physics import DecPerSecond, Ha, HaPerSecond, Second, SkyDirection
from sky.polar_compensator import PolarCompensator

_SKY_SPEED = HaPerSecond(1)
"""1 hour-angle second per second: keeps the expected numbers exact."""

_SLEW_SPEED = HaPerSecond(10)
_VIRTUAL_SLEW_S = 100.0
_WAIT_DEADLINE_S = 5.0
_WAIT_POLL_S = 0.002


class _VirtualMotor:
    """Encoder that integrates over the very same virtual clock as the axis.

    One step == one hour-angle second, so the arithmetic in the test stays
    readable: running FORWARD at ``speed_sps`` for ``dt`` virtual seconds moves
    the reported position by ``speed_sps * dt``.
    """

    FORWARD_POSITION_SIGN = 1

    def __init__(self, clock: VirtualClock) -> None:
        self._clock = clock
        self._integrated_at_s = clock.monotonic()
        self._steps = 0.0
        self._status = MotorStatus(
            is_connected=False,
            steps=0,
            motion_mode=MotionMode.IDLE,
            speed_sps=0,
            accel_sps=None,
            direction=MotorDirection.STOP,
            target=None,
            microsteps=None,
        )

    def _integrate(self) -> None:
        now_s = self._clock.monotonic()
        elapsed_s = now_s - self._integrated_at_s
        self._integrated_at_s = now_s

        if self._status.motion_mode != MotionMode.RUN:
            return

        if self._status.direction == MotorDirection.FORWARD:
            self._steps += self._status.speed_sps * elapsed_s
        elif self._status.direction == MotorDirection.BACKWARD:
            self._steps -= self._status.speed_sps * elapsed_s

    def connect(self):
        self._status.is_connected = True

    def disconnect(self) -> bool:
        self._status.is_connected = False
        return True

    def status(self) -> MotorStatus:
        self._integrate()
        self._status.steps = self._steps  # type: ignore[assignment]
        return self._status

    def get_power_v(self) -> float | None:
        return None

    def set_steps(self, steps: int) -> bool:
        self._integrate()
        self._steps = float(steps)
        return True

    def set_speed(self, steps_per_second: int) -> int:
        self._integrate()
        self._status.speed_sps = steps_per_second
        return steps_per_second

    def set_acceleration(self, steps_per_second_square: float) -> bool:
        del steps_per_second_square
        return True

    def set_direction(self, direction: MotorDirection) -> bool:
        self._integrate()
        self._status.direction = direction
        return True

    def set_delta(self, delta_steps: int) -> bool:
        self._status.target = int(self._steps) + delta_steps
        return True

    def get_speed_sps_by_delta(self, delta_steps: int) -> int:
        return max(1, abs(delta_steps))

    def get_speed_by_speed_sps(self, speed_sps: int) -> HaPerSecond:
        return HaPerSecond(speed_sps)

    def set_motion_mode(self, motion_mode: MotionMode) -> bool:
        self._integrate()
        self._status.motion_mode = motion_mode
        return True

    def set_microsteps(self, microsteps: int) -> bool:
        self._status.microsteps = microsteps
        return True

    def convert_position_to_steps(self, position: Ha) -> int:
        return int(float(position))

    def convert_steps_to_position(self, steps: float) -> Ha:
        return Ha(steps)

    def convert_speed_to_steps_per_second(self, speed: HaPerSecond) -> int:
        return int(abs(float(speed)))

    def run(self) -> bool:
        self._integrate()
        self._status.motion_mode = MotionMode.RUN
        return True

    def stop(self) -> bool:
        self._integrate()
        self._status.motion_mode = MotionMode.IDLE
        self._status.speed_sps = 0
        self._status.direction = MotorDirection.STOP
        self._status.target = None
        return True

    def wait_till_stop(self, do_stop: bool = True, timeout_s: float | None = None) -> None:
        del timeout_s
        if do_stop:
            self.stop()

    def reset(self) -> None:
        self.stop()


def _wait_until(what: str, predicate) -> None:
    deadline = time.monotonic() + _WAIT_DEADLINE_S
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(_WAIT_POLL_S)
    raise AssertionError(f"timeout while waiting for {what}")


class TestClockSeam:
    def test_default_clock_is_real_time(self):
        assert Second.CLOCK is REAL_CLOCK

        before = Second.monotonic()
        time.sleep(0.02)
        after = Second.monotonic()

        assert float(after - before) >= 0.01
        assert float(after) == pytest.approx(time.monotonic(), abs=0.05)

    def test_installed_clock_replaces_real_time(self, virtual_clock: VirtualClock):
        assert Second.CLOCK is virtual_clock
        assert float(Second.monotonic()) == 0.0

        started_at = time.monotonic()
        virtual_clock.advance(3600.0)
        elapsed_real_s = time.monotonic() - started_at

        assert float(Second.monotonic()) == 3600.0
        assert elapsed_real_s < 0.1

    def test_previous_test_did_not_leak_its_clock(self):
        """Runs right after the virtual-clock test above: the swap must be gone."""
        assert Second.CLOCK is REAL_CLOCK
        assert float(Second.monotonic()) == pytest.approx(time.monotonic(), abs=0.05)

        before = Second.monotonic()
        time.sleep(0.02)

        assert float(Second.monotonic() - before) >= 0.01


class TestPolarCompensatorOnVirtualClock:
    RA_SPEED = STELLAR_SPEED + HaPerSecond(0.01)
    DEC_SPEED = DecPerSecond(0.02)

    def test_guide_timeouts_elapse_without_spending_real_time(self, virtual_clock: VirtualClock):
        started_at = time.monotonic()
        comp = PolarCompensator()

        for _ in range(PolarCompensator.STABLE_GUIDE_PULSES_COUNT):
            comp.guide_ra(self.RA_SPEED)
            comp.guide_dec(self.DEC_SPEED)
            virtual_clock.advance(1.0)

        assert comp.get_guide_speeds() is None  # external guiding still recent
        expected_ra = comp.ra_speed
        expected_dec = comp.dec_speed

        # Both DROP_GUIDE_PULSES_COUNT_AFTER (20s) and STOP_AXIS_AFTER (4.1s) expire here.
        virtual_clock.advance(float(PolarCompensator.DROP_GUIDE_PULSES_COUNT_AFTER) + 1)
        ra, dec = comp.get_guide_speeds()
        elapsed_real_s = time.monotonic() - started_at

        assert float(ra) == pytest.approx(float(expected_ra), abs=1e-9)
        assert float(dec) == pytest.approx(float(expected_dec), abs=1e-9)
        assert comp.is_guiding is True
        assert float(comp.last_guide_pulse) == 4.0  # virtual, not wall clock
        assert virtual_clock.now == 26.0
        assert elapsed_real_s < 0.5


class TestAxisOnVirtualClock:
    def test_tracking_compensation_follows_virtual_time(
        self,
        virtual_clock: VirtualClock,
        monkeypatch: pytest.MonkeyPatch,
    ):
        """Slew while tracking: the logical position must gain (slew - sky) * virtual dt.

        Nothing here sleeps for the 100 seconds the axis integrates over: the
        test advances the clock in one step and the background convertor thread
        reads the new instant through ``Second.monotonic()``.
        """
        monkeypatch.setattr(AxisRA, "THREAD_ITERATION_DELAY_S", Second(0.01))
        motor = _VirtualMotor(virtual_clock)
        axis = AxisRA(motor)  # type: ignore[arg-type]
        axis.connect()

        try:
            axis.change_speed(SkyDirection.EAST, _SKY_SPEED, update_sky_speed=True)
            _wait_until("tracking mode", lambda: axis.mode() == AxisMotionMode.TRACK)

            axis.move(SkyDirection.EAST, _SLEW_SPEED)
            _wait_until("slew mode", lambda: axis.mode() == AxisMotionMode.SLEW)
            _wait_until("both commands logged", lambda: len(axis.command_monitor()["processed"]) == 2)

            assert float(axis.get_position().ra) == 0.0
            assert [float(at) for at, _ in axis.command_monitor()["processed"]] == [0.0, 0.0]

            started_at = time.monotonic()
            virtual_clock.advance(_VIRTUAL_SLEW_S)
            expected_ha = (float(_SLEW_SPEED) - float(_SKY_SPEED)) * _VIRTUAL_SLEW_S
            _wait_until(
                f"compensated position {expected_ha}",
                lambda: float(axis.get_position().ra) == pytest.approx(expected_ha, abs=1e-6),
            )
            elapsed_real_s = time.monotonic() - started_at

            assert float(motor.status().steps) == pytest.approx(
                float(_SLEW_SPEED) * _VIRTUAL_SLEW_S,
                abs=1e-6,
            )
            assert elapsed_real_s < 1.0
        finally:
            axis.disconnect()
