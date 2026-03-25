from types import MethodType

import pytest
from sky.physics import Dec
from skywatcher.motor import SkyWatcherMotor, _Direction, _MotionStatus, _SpeedMode, _Status, _SlewMode
from tmc2209.motor import TMC2209Motor, _Mode, _Phase, _Status as _TmcStatus


@pytest.mark.parametrize("speed_sps", [32.4, 32.6, 127.6])
def test_skywatcher_set_speed_returns_quantized_speed(speed_sps: float) -> None:
    motor = SkyWatcherMotor(object())  # type: ignore[arg-type]
    motor._steps_360 = 12_489_074
    motor._steps_worm = 15_400_960
    motor._highspeed_ratio = 1
    motor._get_status = MethodType(
        lambda self: _Status(
            raw=0,
            running=False,
            initialized=True,
            slew_mode=_SlewMode.SLEW,
            direction=_Direction.FORWARD,
            speed_mode=_SpeedMode.LOWSPEED,
        ),
        motor,
    )
    written_commands: list[tuple[object, str | None]] = []
    motor._transact = MethodType(lambda self, command, arg=None: written_commands.append((command, arg)) or "", motor)

    actual_speed = motor.set_speed(speed_sps)

    assert actual_speed == motor._speed_sps_from_period(motor._period_from_speed_sps(speed_sps), _SpeedMode.LOWSPEED)
    assert motor._last_speed_sps == actual_speed
    assert len(written_commands) == 2
    assert written_commands[0][0].name == "SET_MOTION_MODE"
    assert written_commands[0][1] == _MotionStatus(_SlewMode.SLEW, _Direction.FORWARD, _SpeedMode.LOWSPEED).to_command()
    assert written_commands[1][0].name == "SET_STEP_PERIOD"


def test_skywatcher_set_speed_switches_to_highspeed_mode_for_fast_speed() -> None:
    motor = SkyWatcherMotor(object())  # type: ignore[arg-type]
    motor._steps_360 = 12_489_074
    motor._steps_worm = 15_400_960
    motor._highspeed_ratio = 2
    motor._get_status = MethodType(
        lambda self: _Status(
            raw=0,
            running=False,
            initialized=True,
            slew_mode=_SlewMode.SLEW,
            direction=_Direction.FORWARD,
            speed_mode=_SpeedMode.LOWSPEED,
        ),
        motor,
    )
    written_commands: list[tuple[object, str | None]] = []
    motor._transact = MethodType(lambda self, command, arg=None: written_commands.append((command, arg)) or "", motor)

    speed_sps = motor.convert_speed_to_steps_per_second(motor._LOWSPEED_SPEED) + 1

    actual_speed = motor.set_speed(speed_sps)

    assert actual_speed == motor._speed_sps_from_period(motor._period_from_speed_sps(speed_sps), _SpeedMode.HIGHSPEED)
    assert written_commands[0][0].name == "SET_MOTION_MODE"
    assert written_commands[0][1] == _MotionStatus(_SlewMode.SLEW, _Direction.FORWARD, _SpeedMode.HIGHSPEED).to_command()
    assert written_commands[1][0].name == "SET_STEP_PERIOD"


@pytest.mark.parametrize("speed_sps", [-0.1, -1, -10.5])
def test_skywatcher_set_speed_rejects_negative_values(speed_sps: float) -> None:
    motor = SkyWatcherMotor(object())  # type: ignore[arg-type]
    motor._get_status = MethodType(
        lambda self: _Status(
            raw=0,
            running=False,
            initialized=True,
            slew_mode=_SlewMode.SLEW,
            direction=_Direction.FORWARD,
            speed_mode=_SpeedMode.LOWSPEED,
        ),
        motor,
    )

    with pytest.raises(ValueError, match="steps_per_second must be positive"):
        motor.set_speed(speed_sps)


@pytest.mark.parametrize(("speed_sps", "expected"), [(10.4, 10), (10.6, 11), (120.0, 120)])
def test_tmc2209_set_speed_rounds_to_nearest_integer(speed_sps: float, expected: int) -> None:
    motor = TMC2209Motor(object())  # type: ignore[arg-type]
    motor._status = MethodType(
        lambda self: _TmcStatus(
            initialised=True,
            enabled=True,
            mode=_Mode.FREE_RIDE,
            position=0,
            phase=_Phase.IDLE,
            target=0,
            target_set=False,
            speed_sps=0.0,
            actual_speed_sps=0.0,
            accel_steps_per_s=0.0,
        ),
        motor,
    )
    calls: list[list[str]] = []
    motor._transact = MethodType(lambda self, command, args=None: calls.append(args or []) or None, motor)

    actual_speed = motor.set_speed(speed_sps)

    assert actual_speed == expected
    assert calls == [[str(expected)]]


@pytest.mark.parametrize("speed_sps", [-0.1, -1, -10.5])
def test_tmc2209_set_speed_rejects_negative_values(speed_sps: float) -> None:
    motor = TMC2209Motor(object())  # type: ignore[arg-type]
    motor._status = MethodType(
        lambda self: _TmcStatus(
            initialised=True,
            enabled=True,
            mode=_Mode.FREE_RIDE,
            position=0,
            phase=_Phase.IDLE,
            target=0,
            target_set=False,
            speed_sps=0.0,
            actual_speed_sps=0.0,
            accel_steps_per_s=0.0,
        ),
        motor,
    )

    with pytest.raises(ValueError, match="steps_per_second must be non-negative"):
        motor.set_speed(speed_sps)


def test_tmc2209_dec_position_conversion_uses_calibrated_scale() -> None:
    motor = TMC2209Motor(object())  # type: ignore[arg-type]

    one_degree_steps = motor.convert_position_to_steps(Dec(3600))

    assert one_degree_steps == 1889
    assert float(motor.convert_steps_to_position(one_degree_steps)) == pytest.approx(3600.0, abs=1.0)
