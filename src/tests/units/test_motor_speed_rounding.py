from types import MethodType

import pytest
from sky.constants import STELLAR_DAY, STELLAR_SPEED
from sky.motor import MotorDirection
from sky.physics import Dec, DecStepsPerSecond, HaStepsPerSecond
from skywatcher.board import SkyWatcherBoard, TimerPeriod
from skywatcher.codec import Command, Direction, MotionStatus, SkyWatcherCodec, SlewMode, SpeedMode, Status
from skywatcher.motor import SkyWatcherMotor
from tmc2209.motor import TMC2209Motor, _Mode, _Phase, _Response, _Status as _TmcStatus

_IDLE_STATUS = Status(
    raw=0,
    running=False,
    initialized=True,
    slew_mode=SlewMode.SLEW,
    direction=Direction.FORWARD,
    speed_mode=SpeedMode.LOWSPEED,
)


def _idle_tmc_status(_self: TMC2209Motor) -> _TmcStatus:
    return _TmcStatus(
        initialised=True,
        enabled=True,
        mode=_Mode.FREE_RIDE,
        position=0,
        phase=_Phase.IDLE,
        target=0,
        target_set=False,
        speed_sps=DecStepsPerSecond(0),
        actual_speed_sps=DecStepsPerSecond(0),
        accel_steps_per_s=0.0,
    )


def _motor_on_board(board: SkyWatcherBoard) -> SkyWatcherMotor:
    """A driver whose board snapshot is given instead of read off a wire.

    The three numbers below used to be assigned to the driver one attribute at a
    time; they are one immutable object now, which is the point of the board
    layer — a half-filled snapshot is not a state the driver can be in.
    """
    motor = SkyWatcherMotor(object())  # type: ignore[arg-type]
    motor._session._board = board
    return motor


def _recording_session(
    motor: SkyWatcherMotor, monkeypatch: pytest.MonkeyPatch
) -> list[tuple[Command, str | None]]:
    """Silence the wire and record what the driver would have sent."""
    written_commands: list[tuple[Command, str | None]] = []

    def _transact(command: Command, arg: str | None = None) -> str:
        written_commands.append((command, arg))
        return ""

    monkeypatch.setattr(motor._session, "status", lambda: _IDLE_STATUS)
    monkeypatch.setattr(motor._session, "transact", _transact)
    return written_commands


def _recording_tmc_transact(motor: TMC2209Motor, calls: list[list[str]]) -> MethodType:
    def _transact(_self: TMC2209Motor, command: str, args: list[str] | None = None) -> _Response:
        calls.append(args or [])
        # The driver now reads back the value the controller acknowledged instead of
        # trusting the one it sent, so a stub that answers nothing is no longer a
        # controller: it echoes the write the way both dialects do.
        return _Response(ok=True, values={"speed": f"{float(args[0]):.2f}"} if args else {}, error=None)

    return MethodType(_transact, motor)


@pytest.mark.parametrize("speed_sps", [HaStepsPerSecond(32.4), HaStepsPerSecond(32.6), HaStepsPerSecond(127.6)])
def test_skywatcher_set_speed_returns_quantized_speed(speed_sps: HaStepsPerSecond, monkeypatch: pytest.MonkeyPatch) -> None:
    motor = _motor_on_board(SkyWatcherBoard(cpr=12_489_074, timer_freq=15_400_960, highspeed_ratio=1))
    written_commands = _recording_session(motor, monkeypatch)

    # Quantization coverage: a step rate is not a count of anything, and the
    # fractional values here are what the period arithmetic has to round consistently.
    actual_speed = motor.set_speed(speed_sps)

    board = motor.board
    assert board is not None
    assert actual_speed == board.speed_sps_from_period(
        board.period_from_speed_sps(speed_sps, SpeedMode.LOWSPEED),
        SpeedMode.LOWSPEED,
    )
    assert motor._last_speed_sps == actual_speed
    assert len(written_commands) == 2
    assert written_commands[0][0].name == "SET_MOTION_MODE"
    assert written_commands[0][1] == MotionStatus(SlewMode.SLEW, Direction.FORWARD, SpeedMode.LOWSPEED).to_command()
    assert written_commands[1][0].name == "SET_STEP_PERIOD"


def test_skywatcher_set_speed_switches_to_highspeed_mode_for_fast_speed(monkeypatch: pytest.MonkeyPatch) -> None:
    motor = _motor_on_board(SkyWatcherBoard(cpr=12_489_074, timer_freq=15_400_960, highspeed_ratio=2))
    written_commands = _recording_session(motor, monkeypatch)

    speed_sps = motor.convert_speed_to_steps_per_second(motor._LOWSPEED_SPEED) + HaStepsPerSecond(1)

    actual_speed = motor.set_speed(speed_sps)

    board = motor.board
    assert board is not None
    assert actual_speed == board.speed_sps_from_period(
        board.period_from_speed_sps(speed_sps, SpeedMode.HIGHSPEED), SpeedMode.HIGHSPEED
    )
    assert written_commands[0][0].name == "SET_MOTION_MODE"
    assert written_commands[0][1] == MotionStatus(SlewMode.SLEW, Direction.FORWARD, SpeedMode.HIGHSPEED).to_command()
    assert written_commands[1][0].name == "SET_STEP_PERIOD"


def test_skywatcher_highspeed_period_uses_lowspeed_threshold_not_ratio() -> None:
    motor = _motor_on_board(SkyWatcherBoard(cpr=12_489_074, timer_freq=15_400_960, highspeed_ratio=256))
    board = motor.board
    assert board is not None

    speed_sps = motor.convert_speed_to_steps_per_second(motor._LOWSPEED_SPEED) + HaStepsPerSecond(1)

    period = motor._period_from_speed_sps(speed_sps)
    rate = float(speed_sps) * (24 * 60 * 60) / board.cpr / float(STELLAR_SPEED)
    expected = TimerPeriod(int(float(STELLAR_DAY) * board.timer_freq / board.cpr / (rate / board.highspeed_ratio)))

    assert period == expected


def test_skywatcher_set_speed_clamps_period_to_mount_minimum(monkeypatch: pytest.MonkeyPatch) -> None:
    motor = _motor_on_board(SkyWatcherBoard(cpr=12_489_074, timer_freq=15_400_960, highspeed_ratio=11))
    speed_sps = motor.convert_speed_to_steps_per_second(motor._HIGHSPEED_SPEED)
    unclamped_period = motor._period_from_speed_sps(speed_sps)
    min_period = TimerPeriod(int(unclamped_period) + 123)
    motor = _motor_on_board(
        SkyWatcherBoard(cpr=12_489_074, timer_freq=15_400_960, highspeed_ratio=11, min_period=min_period)
    )
    written_commands = _recording_session(motor, monkeypatch)

    actual_speed = motor.set_speed(speed_sps)

    board = motor.board
    assert board is not None
    assert actual_speed == board.speed_sps_from_period(min_period, SpeedMode.HIGHSPEED)
    assert motor._last_speed_sps == actual_speed
    assert written_commands[1][0].name == "SET_STEP_PERIOD"
    assert written_commands[1][1] == SkyWatcherCodec.encode_revu24(int(min_period))


def test_skywatcher_set_speed_clamps_lowspeed_period_to_mount_minimum(monkeypatch: pytest.MonkeyPatch) -> None:
    min_period = TimerPeriod(0x0600)
    motor = _motor_on_board(
        SkyWatcherBoard(cpr=12_489_074, timer_freq=15_400_960, highspeed_ratio=1, min_period=min_period)
    )
    written_commands = _recording_session(motor, monkeypatch)

    speed_sps = HaStepsPerSecond(11_598)

    actual_speed = motor.set_speed(speed_sps)

    board = motor.board
    assert board is not None
    assert actual_speed == board.speed_sps_from_period(min_period, SpeedMode.LOWSPEED)
    assert written_commands[1][0].name == "SET_STEP_PERIOD"
    assert written_commands[1][1] == SkyWatcherCodec.encode_revu24(int(min_period))


def test_skywatcher_set_direction_preserves_highspeed_mode_from_last_speed(monkeypatch: pytest.MonkeyPatch) -> None:
    motor = _motor_on_board(SkyWatcherBoard(cpr=12_489_074, timer_freq=15_400_960, highspeed_ratio=11))
    motor._last_speed_sps = motor.convert_speed_to_steps_per_second(motor._HIGHSPEED_SPEED)
    written_commands = _recording_session(motor, monkeypatch)

    assert motor.set_direction(MotorDirection.FORWARD) is True

    assert written_commands == [
        (
            Command.SET_MOTION_MODE,
            MotionStatus(SlewMode.SLEW, Direction.FORWARD, SpeedMode.HIGHSPEED).to_command(),
        )
    ]


@pytest.mark.parametrize("speed_sps", [-0.1, -1, -10.5])
def test_skywatcher_set_speed_rejects_negative_values(speed_sps: float, monkeypatch: pytest.MonkeyPatch) -> None:
    motor = SkyWatcherMotor(object())  # type: ignore[arg-type]
    monkeypatch.setattr(motor._session, "status", lambda: _IDLE_STATUS)

    with pytest.raises(ValueError, match="steps_per_second must be positive"):
        # Negative test: floats are passed on purpose, the annotation only admits int.
        motor.set_speed(speed_sps)  # type: ignore[arg-type]


@pytest.mark.parametrize(("speed_sps", "expected"), [(10.4, 10), (10.6, 11), (120.0, 120)])
def test_tmc2209_set_speed_rounds_to_nearest_integer(
    speed_sps: float, expected: int, monkeypatch: pytest.MonkeyPatch
) -> None:
    motor = TMC2209Motor(object())  # type: ignore[arg-type]
    monkeypatch.setattr(motor, "_status", MethodType(_idle_tmc_status, motor))
    calls: list[list[str]] = []
    monkeypatch.setattr(motor, "_transact", _recording_tmc_transact(motor, calls))

    # Rounding coverage: set_speed is annotated int but rounds floats at runtime.
    actual_speed = motor.set_speed(speed_sps)  # type: ignore[arg-type]

    assert actual_speed == expected
    assert calls == [[str(expected)]]


@pytest.mark.parametrize("speed_sps", [-0.1, -1, -10.5])
def test_tmc2209_set_speed_rejects_negative_values(speed_sps: float, monkeypatch: pytest.MonkeyPatch) -> None:
    motor = TMC2209Motor(object())  # type: ignore[arg-type]
    monkeypatch.setattr(motor, "_status", MethodType(_idle_tmc_status, motor))

    with pytest.raises(ValueError, match="steps_per_second must be non-negative"):
        # Negative test: floats are passed on purpose, the annotation only admits int.
        motor.set_speed(speed_sps)  # type: ignore[arg-type]


def test_tmc2209_dec_position_conversion_uses_calibrated_scale() -> None:
    motor = TMC2209Motor(object())  # type: ignore[arg-type]

    one_degree_steps = motor.convert_position_to_steps(Dec(3600))

    assert one_degree_steps == 1889
    assert float(motor.convert_steps_to_position(one_degree_steps)) == pytest.approx(3600.0, abs=1.0)
