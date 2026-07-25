import pytest

from sim import Clock, FaultKind, SimSerialLine, SkyWatcherSim, TMC2209Sim
from sky.constants import STELLAR_DAY, STELLAR_SPEED
from sky.motor import MotionMode, MotorDirection
from sky.physics import Ha
from skywatcher.motor import SkyWatcherMotor, SkyWatcherMotorProtocolError
from tmc2209.motor import TMC2209Motor

# The RA board of this project, as measured in `docs/protocol/RA_PROTOCOL.md` §2.
# The simulator defaults to it, so nothing is passed in here on purpose: these
# tests must exercise the controller the driver really talks to.
_RA_CPR = 12_492_146
_RA_TIMER_FREQ = 16_000_000


def _make_ra(**config: int) -> tuple[Clock, SkyWatcherSim, SkyWatcherMotor]:
    clock = Clock()
    sim = SkyWatcherSim(clock, **config)
    line = SimSerialLine(sim, clock, port="sim://ra", timeout_s=0, name="sim-ra", terminator="\r")
    return clock, sim, SkyWatcherMotor(line, clock)


def _make_dec() -> tuple[Clock, TMC2209Sim, TMC2209Motor]:
    clock = Clock()
    sim = TMC2209Sim(clock)
    line = SimSerialLine(sim, clock, port="sim://dec", timeout_s=2, name="sim-dec", terminator="\n")
    return clock, sim, TMC2209Motor(line, clock)


def test_skywatcher_connect_reads_valid_mount_config() -> None:
    clock, sim, motor = _make_ra()

    motor.connect()

    assert motor._steps_360 == _RA_CPR
    assert motor._steps_worm == _RA_TIMER_FREQ
    # Highspeed ratio 1 (§2.1): this board has no high-speed multiplier at all,
    # so `G '3'` changes nothing. Mount code 0x0A picks the driver's 0x0600
    # minimum period — stricter than the board's own clamp of 1103 (§2.2).
    assert motor._highspeed_ratio == 1
    assert motor._min_period == 0x0600

    status = motor.status()
    assert status.is_connected is True
    assert status.steps == 0
    assert status.motion_mode == MotionMode.IDLE
    assert status.direction == MotorDirection.STOP
    assert status.target is None


@pytest.mark.parametrize("mount_code,min_period", [(0x0A, 0x0600), (0xF0, 12), (0x42, 6)])
def test_skywatcher_mount_code_selects_min_period(mount_code: int, min_period: int) -> None:
    clock, sim, motor = _make_ra(mount_code=mount_code)

    motor.connect()

    assert motor._mount_code == mount_code
    assert motor._min_period == min_period


def test_skywatcher_tracking_drifts_at_sidereal_rate() -> None:
    clock, sim, motor = _make_ra()
    motor.connect()

    speed_sps = motor.convert_speed_to_steps_per_second(STELLAR_SPEED)
    motor.set_speed(speed_sps)
    motor.set_direction(MotorDirection.FORWARD)
    motor.set_motion_mode(MotionMode.RUN)
    motor.run()

    clock.advance(60)

    status = motor.status()
    assert status.motion_mode == MotionMode.RUN
    assert status.direction == MotorDirection.FORWARD

    # 1.002738x sidereal: the axis makes one turn (CPR steps) per stellar day.
    # The tolerance is the quantization budget only: the driver rounds the
    # requested steps/s and truncates the step period, which together bias the
    # axis ~0.20% fast. It has to stay below the 0.273% gap between a stellar
    # and a solar day, otherwise "tracking on solar time" passes unnoticed.
    expected_steps = 60 * _RA_CPR / float(STELLAR_DAY)
    assert status.steps == pytest.approx(expected_steps, rel=0.003)


def test_skywatcher_goto_highspeed_reaches_target() -> None:
    clock, sim, motor = _make_ra()
    motor.connect()

    delta_steps = motor.convert_position_to_steps(Ha(1800))
    motor.set_delta(delta_steps)
    motor.run()

    status = motor.status()
    assert status.motion_mode == MotionMode.TARGET
    assert status.target == delta_steps
    assert sim.highspeed is True

    # 30s, not 10: the driver asks for 800x sidereal but its own `min_period`
    # holds the axis at 71.8x (§2.2), so half an hour of RA takes ~25s here.
    clock.advance(30)

    status = motor.status()
    assert status.steps == delta_steps
    assert status.motion_mode == MotionMode.IDLE
    assert status.direction == MotorDirection.STOP
    assert status.target is None


def test_skywatcher_goto_lowspeed_reaches_target() -> None:
    clock, sim, motor = _make_ra()
    motor.connect()

    delta_steps = motor.convert_position_to_steps(Ha(300))
    motor.set_delta(delta_steps)
    motor.run()

    assert sim.highspeed is False

    clock.advance(5)

    status = motor.status()
    assert status.steps == delta_steps
    assert status.motion_mode == MotionMode.IDLE


def test_skywatcher_board_has_no_voltage_to_report() -> None:
    """§6: no command reads the supply voltage off this board.

    `:fL#` was invented by a commit of ours; the live controller stays silent
    (only `\\r` terminates a command) and answers `!3` to `:fL\\r`, because `L`
    is not a hex channel. The driver therefore has no voltage to report, and
    the dashboard must take it from the DEC board instead.
    """
    clock, sim, motor = _make_ra()
    motor.connect()

    assert motor.get_power_v() is None
    assert sim.drain() == b""


def test_skywatcher_driver_retries_over_scripted_faults() -> None:
    clock, sim, motor = _make_ra()
    motor.connect()

    sim.faults.push(FaultKind.EMPTY, command="f")
    sim.faults.push(FaultKind.TRUNCATE, command="f")

    status = motor.status()
    assert status.motion_mode == MotionMode.IDLE


def test_skywatcher_dead_motor_fails_connect() -> None:
    clock, sim, motor = _make_ra()
    sim.faults.make_dead()

    with pytest.raises(SkyWatcherMotorProtocolError):
        motor.connect()


def test_tmc_connect_receives_ready_after_dtr_reset() -> None:
    clock, sim, motor = _make_dec()

    motor.connect()

    status = motor.status()
    assert status.is_connected is True
    assert status.steps == 0
    assert status.motion_mode == MotionMode.IDLE
    assert status.direction == MotorDirection.STOP
    assert status.microsteps == 16
    assert motor.get_power_v() == pytest.approx(12.0)


def test_tmc_goto_reaches_target_over_virtual_time() -> None:
    clock, sim, motor = _make_dec()
    motor.connect()

    motor.set_motion_mode(MotionMode.TARGET)
    motor.set_acceleration(1000)
    motor.set_speed(1000)
    motor.set_delta(5000)
    motor.run()

    clock.advance(8)

    status = motor.status()
    assert status.steps == 5000
    assert status.target is None
    assert status.direction == MotorDirection.STOP


def test_tmc_free_ride_integrates_speed() -> None:
    clock, sim, motor = _make_dec()
    motor.connect()

    motor.set_motion_mode(MotionMode.RUN)
    motor.set_acceleration(0)
    motor.set_speed(100)
    motor.set_direction(MotorDirection.FORWARD)
    motor.run()

    clock.advance(10)

    status = motor.status()
    assert status.steps == 1000
    assert status.motion_mode == MotionMode.RUN
    assert status.direction == MotorDirection.FORWARD


def test_tmc_backward_direction_decreases_steps() -> None:
    clock, sim, motor = _make_dec()
    motor.connect()

    motor.set_motion_mode(MotionMode.RUN)
    motor.set_acceleration(0)
    motor.set_speed(200)
    motor.set_direction(MotorDirection.BACKWARD)
    motor.run()

    clock.advance(5)

    assert motor.status().steps == -1000
