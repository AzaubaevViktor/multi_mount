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
    # so `G '3'` changes nothing. The minimum period is the board's own clamp,
    # 1103 = 100x sidereal (§10.5), asked of the board and not guessed.
    assert motor._highspeed_ratio == 1
    assert motor._min_period == sim.min_period == 1103

    status = motor.status()
    assert status.is_connected is True
    assert status.steps == 0
    assert status.motion_mode == MotionMode.IDLE
    assert status.direction == MotorDirection.STOP
    assert status.target is None


@pytest.mark.parametrize("mount_code", [0x0A, 0xF0, 0x42])
def test_skywatcher_min_period_comes_from_the_board_not_from_the_mount_code(mount_code: int) -> None:
    """§10.5 and §13 #5/#6: the clamp is the board's, and only the board knows it.

    The driver used to look the minimum period up by mount code (0x0A -> 0x0600,
    0xF0 -> 12, else 6). Three independent sources say no such table exists: the
    command-set PDF never mentions a minimum, the EQMod reference has no 0x0A at
    all, and the vendor's own app (RA_SA_CONSOLE_PROTOCOL.md) clamps nothing in
    `rpsToStride`. So the same board must yield the same limit whatever mount
    code it reports — the number can only come off the wire.
    """
    clock, sim, motor = _make_ra(mount_code=mount_code)

    motor.connect()

    assert motor._mount_code == mount_code
    assert motor._min_period == sim.min_period


def test_skywatcher_min_period_probe_follows_a_board_with_another_clamp() -> None:
    """A measurement, not a constant: another board, another answer.

    `cpr`/`timer_freq` here give a 1x tracking period of 10 000, so this
    simulated controller clamps at 100 instead of 1103. If the driver kept a
    table (or a hard-coded 1103) it would report the wrong ceiling.
    """
    clock, sim, motor = _make_ra(cpr=8_616_410, timer_freq=1_000_000)
    assert sim.min_period == 100

    motor.connect()

    assert motor._min_period == 100


def test_skywatcher_min_period_probe_puts_back_the_period_it_found() -> None:
    """The probe writes the fastest period the board has; it must not leave it there.

    The board powers up at the 1x tracking period (§12.1) and a `:J1` from
    anywhere would start the axis at whatever `:I1` left loaded. Connecting is
    not a reason to arm the mount at 100x sidereal.
    """
    clock, sim, motor = _make_ra()
    period_before = sim.step_period
    assert period_before == sim.tracking_period_1x

    motor.connect()

    assert sim.step_period == period_before


def test_skywatcher_goto_reports_the_speed_the_board_will_really_run() -> None:
    """§2.2: what goes up to `Axis` is the achievable speed, not the wish.

    `Axis._run_goto_to` divides the delta by this number to get the GOTO ETA and
    subtracts the sky drift over that ETA from the target. The driver asks for
    800x sidereal and the board holds it at 100x (§10.5), so reporting the
    request made the axis expect a move eight times shorter than it is and
    under-compensate the sky by the same factor.
    """
    clock, sim, motor = _make_ra()
    motor.connect()

    delta_steps = motor.convert_position_to_steps(Ha(1800))
    reported_sps = motor.get_speed_sps_by_delta(delta_steps)
    requested_sps = motor.convert_speed_to_steps_per_second(motor._HIGHSPEED_SPEED)

    # The request is 800x sidereal, the board gives 100.05x: the two must not be
    # confused for each other.
    assert requested_sps == pytest.approx(_RA_CPR * 800 / float(STELLAR_DAY), rel=1e-3)
    assert reported_sps == pytest.approx(_RA_CPR * 100.05 / float(STELLAR_DAY), rel=1e-3)

    # And the reported speed is the one the axis really moves at: predicted ETA
    # against the arrival measured on the simulated board.
    predicted_eta_s = delta_steps / reported_sps
    motor.set_delta(delta_steps)
    motor.run()

    elapsed_s = 0.0
    while motor.status().motion_mode != MotionMode.IDLE and elapsed_s < 10 * predicted_eta_s:
        clock.advance(0.1)
        elapsed_s += 0.1

    assert motor.status().steps == delta_steps
    assert elapsed_s == pytest.approx(predicted_eta_s, rel=0.02)


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

    # 30s, not 4: the driver asks for 800x sidereal but the board clamps the
    # step period at 1103 (§10.5), so half an hour of RA runs at 100x and takes
    # ~18s here. The driver knows it — see the ETA test above.
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


def test_skywatcher_reports_the_supply_voltage_in_volts() -> None:
    """§6.8: the numbers measured on the board, end to end through the driver.

    Battery 604 hundredths -> 6.04 V (the multimeter said 6.08 V at the same
    moment), USB 470 -> 4.70 V. Raw counts, or bytes taken big-endian, would
    give 604 V and 236.44 V respectively — three orders of magnitude of
    difference between a right and a wrong reading.
    """
    clock, sim, motor = _make_ra()
    motor.connect()

    assert motor.get_power_v() == pytest.approx(6.04)
    assert motor.protocol_monitor()["battery_v"] == "6.04"
    assert motor.protocol_monitor()["usb_v"] == "4.70"


def test_skywatcher_reports_the_higher_of_the_two_supply_rails() -> None:
    """The board is fed by whichever source is higher, so that is what is shown.

    The vendor's own UI does `max(usbVolt, batteryVolt)` too. The point is not
    cosmetic: on USB alone the battery channel reads 0, and a driver reporting
    the battery only would put a flat 0.00 V on the dashboard of a perfectly
    healthy mount.
    """
    clock, sim, motor = _make_ra(battery_volt_hundredths=0, usb_volt_hundredths=498)
    motor.connect()

    assert motor.get_power_v() == pytest.approx(4.98)
    # Both rails stay separately visible, so a sagging battery is not lost in
    # the maximum.
    assert motor.protocol_monitor()["battery_v"] == "0.00"
    assert motor.protocol_monitor()["usb_v"] == "4.98"


def test_skywatcher_status_carries_the_ra_voltage_without_polling_for_it() -> None:
    """The dashboard reads `power_v` off both boards; RA is no longer a blank.

    `status()` is called far more often than the voltage moves, so it reports
    what the last poll brought and sends nothing of its own.
    """
    clock, sim, motor = _make_ra()
    motor.connect()

    assert motor.status().power_v is None
    assert motor.get_power_v() == pytest.approx(6.04)

    received = bytearray()
    original_feed = sim.feed

    def _recording_feed(data: bytes) -> None:
        received.extend(data)
        original_feed(data)

    sim.feed = _recording_feed  # type: ignore[method-assign]

    assert motor.status().power_v == pytest.approx(6.04)
    assert b":C1" not in bytes(received)
    assert b":n1" not in bytes(received)


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
