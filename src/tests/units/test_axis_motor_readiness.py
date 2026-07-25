"""Startup readiness probe (PLAN.md #31).

`src/__main__.py` decides between "serve LX200 + dashboard" and "drop into the
manual console" by asking each axis whether its motor is there. The old probe
did that behind `except Exception: return False`, so a motor whose `status()`
raised was reported as *absent* — the operator got "no hardware" instead of the
error, and nothing appeared in the log. `Axis.motor_readiness()` is the typed
replacement: three states, the exception carried out and logged.
"""

import logging

import pytest

from serial_wrapper.wrapper import SerialLineClosedError
from sky.axis import AxisDEC, AxisRA, MotorReadinessState
from sky.combiner import Combiner
from sky.motor import MotionMode, MotorDirection, MotorStatus
from sky.physics import AxisSpeed, Dec, DecPerSecond, Ha, HaPerSecond, StepsPerSecond


class _ProbeMotor:
    """Just enough motor to answer (or refuse to answer) `status()`."""

    FORWARD_POSITION_SIGN = 1

    def __init__(self, pos_cls, speed_cls, *, is_connected: bool = True, error: BaseException | None = None) -> None:
        self._pos_cls = pos_cls
        self._speed_cls = speed_cls
        self._is_connected = is_connected
        self.error = error
        self.status_calls = 0

    def status(self) -> MotorStatus[AxisSpeed]:
        self.status_calls += 1
        if self.error is not None:
            raise self.error
        return MotorStatus(
            is_connected=self._is_connected,
            steps=0,
            motion_mode=MotionMode.IDLE,
            speed_sps=StepsPerSecond(0),
            accel_sps=None,
            direction=MotorDirection.STOP,
            target=None,
            microsteps=None,
        )


def _ra(**kwargs) -> AxisRA:
    return AxisRA(_ProbeMotor(Ha, HaPerSecond, **kwargs))  # type: ignore[arg-type]


def _dec(**kwargs) -> AxisDEC:
    return AxisDEC(_ProbeMotor(Dec, DecPerSecond, **kwargs))  # type: ignore[arg-type]


def test_connected_motor_is_ready() -> None:
    readiness = _ra(is_connected=True).motor_readiness()

    assert readiness.state is MotorReadinessState.READY
    assert readiness.is_ready is True
    assert readiness.error is None


def test_disconnected_motor_is_not_connected_and_carries_no_error() -> None:
    readiness = _ra(is_connected=False).motor_readiness()

    assert readiness.state is MotorReadinessState.NOT_CONNECTED
    assert readiness.is_ready is False
    assert readiness.error is None


def test_failing_status_is_unknown_and_not_confused_with_a_missing_motor() -> None:
    """The whole point: a broken axis must not look like an absent one."""
    boom = SerialLineClosedError("port is gone")

    readiness = _ra(error=boom).motor_readiness()

    # UNKNOWN, *not* NOT_CONNECTED: that is the distinction the old probe could not make.
    assert readiness.state is MotorReadinessState.UNKNOWN
    assert readiness.is_ready is False
    assert readiness.error is boom
    assert "SerialLineClosedError" in readiness.describe()
    assert "port is gone" in readiness.describe()


def test_a_failing_status_never_escapes_the_probe() -> None:
    """A diagnostic that crashes tells the operator less than one that reports."""
    axis = _ra(error=RuntimeError("motion convertor is on fire"))

    readiness = axis.motor_readiness()

    assert readiness.state is MotorReadinessState.UNKNOWN
    assert isinstance(readiness.error, RuntimeError)


def test_failing_status_is_logged_once_per_distinct_failure(caplog) -> None:
    """Visible in the log, but a dead port must not write a line per poll (PLAN.md P1)."""
    axis = _ra(error=OSError(6, "Device not configured"))

    with caplog.at_level(logging.WARNING, logger=axis.logger.name):
        for _ in range(5):
            axis.motor_readiness()

    warnings = [record for record in caplog.records if record.levelno >= logging.WARNING]
    assert len(warnings) == 1
    assert "readiness is unknown" in warnings[0].getMessage()
    assert "Device not configured" in warnings[0].getMessage()


def test_a_recovered_then_broken_motor_warns_again(caplog) -> None:
    axis = _ra(error=OSError(6, "Device not configured"))
    motor: _ProbeMotor = axis._motor  # type: ignore[assignment]

    with caplog.at_level(logging.WARNING, logger=axis.logger.name):
        axis.motor_readiness()
        motor.error = None
        assert axis.motor_readiness().state is MotorReadinessState.READY
        motor.error = OSError(6, "Device not configured")
        axis.motor_readiness()

    warnings = [record for record in caplog.records if record.levelno >= logging.WARNING]
    assert len(warnings) == 2


@pytest.mark.parametrize(
    "ra_kwargs,dec_kwargs,expected",
    [
        ({"is_connected": True}, {"is_connected": True}, (MotorReadinessState.READY, MotorReadinessState.READY)),
        (
            {"is_connected": True},
            {"is_connected": False},
            (MotorReadinessState.READY, MotorReadinessState.NOT_CONNECTED),
        ),
        (
            {"error": OSError(6, "Device not configured")},
            {"is_connected": True},
            (MotorReadinessState.UNKNOWN, MotorReadinessState.READY),
        ),
    ],
)
def test_combiner_reports_both_axes_in_ra_dec_order(ra_kwargs, dec_kwargs, expected) -> None:
    combiner = Combiner(_ra(**ra_kwargs), _dec(**dec_kwargs))

    ra_readiness, dec_readiness = combiner.motors_readiness()

    assert (ra_readiness.state, dec_readiness.state) == expected
    assert ra_readiness.axis.value == "ra"
    assert dec_readiness.axis.value == "dec"
