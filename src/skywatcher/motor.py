import dataclasses
import logging
from enum import IntEnum, StrEnum

from clock import NEVER, REAL_CLOCK, Clock
from serial_wrapper.wrapper import SerialLine
from sky.constants import STELLAR_DAY, STELLAR_SPEED
from sky.motor import MotionMode, Motor, MotorDirection, MotorStateError, MotorStatus, MotorStopRequire
from sky.physics import Ha, HaPerSecond
from skywatcher.protocol import Protocol


class SkyWatcherMotorError(Exception):
    def __init__(self, message: str) -> None:
        self.message = message
        super().__init__(message)


class SkyWatcherMotorProtocolError(SkyWatcherMotorError):
    pass


class SkyWatcherMotorCommandError(SkyWatcherMotorError):
    pass


class SkyWatcherMotorTimeoutError(SkyWatcherMotorError, TimeoutError):
    pass


class _Command(StrEnum):
    INQUIRE_TIMER_FREQ = "b"
    INQUIRE_CPR = "a"
    INQUIRE_MOTOR_BOARD_VERSION = "e"
    INQUIRE_POSITION = "j"
    INQUIRE_STATUS = "f"
    INQUIRE_HIGHSPEED_RATIO = "g"
    INQUIRE_STEP_PERIOD = "i"
    SET_STEP_PERIOD = "I"
    SET_MEMORY_ADDRESS = "C"
    INQUIRE_MEMORY_BYTE = "n"
    SET_GOTO_TARGET_INCREMENT = "H"
    SET_BREAK_POINT_INCREMENT = "M"
    SET_AXIS_POSITION = "E"
    SET_MOTION_MODE = "G"
    START_MOTION = "J"
    STOP_MOTION = "K"
    INITIALIZE = "F"


class _Axis(StrEnum):
    RA = "1"


class _Direction(IntEnum):
    BACKWARD = 0
    FORWARD = 1


class _SlewMode(IntEnum):
    SLEW = 0
    GOTO = 1


class _SpeedMode(IntEnum):
    LOWSPEED = 0
    HIGHSPEED = 1


@dataclasses.dataclass(frozen=True)
class _Status:
    raw: int
    running: bool
    initialized: bool
    slew_mode: _SlewMode
    direction: _Direction
    speed_mode: _SpeedMode

    @classmethod
    def from_bytes(cls, data: bytes) -> "_Status":
        # A short body is a damaged frame, not a status with the missing flags cleared:
        # zero-padding `=41` reports "stopped, not initialized" for a slewing axis, and
        # `wait_till_stop` would then return while the mount keeps moving.
        if len(data) != 3:
            raise SkyWatcherMotorProtocolError(f"expected 3 status chars, got {data!r}")
        b1, b2, b3 = data
        raw = b2 | (b1 << 8) | (b3 << 16)
        return cls(
            raw=raw,
            running=bool(b2 & 0x01),
            initialized=bool(b3 & 0x01),
            slew_mode=_SlewMode.SLEW if (b1 & 0x01) else _SlewMode.GOTO,
            direction=_Direction.BACKWARD if (b1 & 0x02) else _Direction.FORWARD,
            speed_mode=_SpeedMode.HIGHSPEED if (b1 & 0x04) else _SpeedMode.LOWSPEED,
        )


@dataclasses.dataclass(frozen=True)
class _MotionStatus:
    slew_mode: _SlewMode
    direction: _Direction
    speed_mode: _SpeedMode

    def to_command(self) -> str:
        if self.slew_mode == _SlewMode.SLEW:
            motion_mode = "1" if self.speed_mode == _SpeedMode.LOWSPEED else "3"
        else:
            motion_mode = "2" if self.speed_mode == _SpeedMode.LOWSPEED else "0"
        direction_mode = "0" if self.direction == _Direction.FORWARD else "1"
        return f"{motion_mode}{direction_mode}"


class _Revu24:
    @staticmethod
    def from_mount(data: str, hex_chars: int = 6) -> int:
        # `hex_chars` is the width the queried value really has (reference §4.5: 24-bit -> 6
        # hex chars, 8-bit -> 2), and only that narrower answer may be zero-extended. Doing
        # it for every command instead — as this did — turns a damaged frame into a
        # plausible number: a position answer that lost four of its six digits would decode
        # as a valid position on the other side of the axis.
        if len(data) == hex_chars < 6:
            data = f"{data}{'0' * (6 - hex_chars)}"
        if len(data) != 6:
            raise SkyWatcherMotorProtocolError(f"expected {hex_chars} hex chars, got {data!r}")
        reordered = data[4:6] + data[2:4] + data[0:2]
        try:
            return int(reordered, 16)
        except ValueError as exc:
            raise SkyWatcherMotorProtocolError(f"invalid hex data: {data!r}") from exc

    @staticmethod
    def from_int(value: int) -> str:
        if value < 0 or value > 0xFFFFFF:
            raise SkyWatcherMotorProtocolError(f"expected value in range 0..{0xFFFFFF}, got {value}")
        return value.to_bytes(3, "little").hex().upper()


class SkyWatcherMotor(Motor[Ha, HaPerSecond]):
    FORWARD_POSITION_SIGN = -1

    _POSITION_OFFSET = 0x800000
    # Fallback for a board whose own clamp could not be measured (§10.5 probe below):
    # the value the INDI reference uses for every mount it does not recognize.
    _DEFAULT_MIN_PERIOD = 6
    # Written during the probe: below every plausible clamp, and harmless — the board
    # answers `=` and stores its own minimum instead, even for 0 (§10.5).
    _MIN_PERIOD_PROBE = 1
    _LOWSPEED_MARGIN = Ha(10 * 60)
    _LOWSPEED_SPEED = STELLAR_SPEED * 128
    _HIGHSPEED_SPEED = STELLAR_SPEED * 800
    _CONNECT_ATTEMPTS = 3
    _CONNECT_RETRY_DELAY_S = 0.25
    # The measured brake ramp is 0.6s at the fastest the board goes (§10.3); the timeout
    # is an order of magnitude above it, so it only fires when the axis is really stuck.
    _STOP_TIMEOUT_S = 10.0
    _STOP_POLL_S = 0.05
    # §11: the initialization flag drops "asynchronously, hundreds of milliseconds" after
    # an `:E`. The board was seen taking ~0.3s; the check waits longer than that on purpose.
    _INIT_FLAG_SETTLE_S = 0.5

    # §6.8: both supply voltages sit in the `:C`/`:n` window as int16 little-endian in
    # hundredths of a volt. The address is 16-bit and travels low byte first.
    _BATTERY_VOLT_ADDRESS = 0x0004
    _USB_VOLT_ADDRESS = 0x001C
    _VOLT_PER_COUNT = 0.01
    # One reading is eight commands (`:C1` + `:n1` per byte, two bytes per channel,
    # two channels), and the dashboard asks on every tick. A supply that
    # sags fast enough to matter (§12: motor load -> brownout -> reboot) still sags over
    # seconds, so a few seconds of staleness costs nothing and keeps the line free.
    _POWER_CACHE_TTL_S = 5.0
    # A board that answers `!0` or garbage will keep doing so. Retrying every tick would
    # be the `:fL#` storm all over again, only eight commands wide instead of one.
    _POWER_FAILURE_BACKOFF_S = 60.0

    def __init__(self, serial: SerialLine, clock: Clock = REAL_CLOCK) -> None:
        self._serial = serial
        self._clock = clock
        self._logger = logging.getLogger(type(self).__name__)
        self._is_connected = False
        self._steps_360 = 0
        self._steps_worm = 0
        self._highspeed_ratio = 0
        self._mount_code: int | None = None
        self._min_period = self._DEFAULT_MIN_PERIOD
        self._last_status: _Status | None = None
        self._last_speed_sps = 0
        self._last_direction = MotorDirection.STOP
        self._last_target: int | None = None
        self._zero_target_pending = False
        self._mount_position_cache = Ha(0)
        self._mount_position_cache_updated = NEVER
        self._battery_v: float | None = None
        self._usb_v: float | None = None
        self._power_v_updated = NEVER
        self._power_v_retry_at = NEVER

    def connect(self) -> None:
        if self._serial.terminator != Protocol.ANSWER_END_BYTE:
            raise SkyWatcherMotorProtocolError(
                f"invalid SerialLine terminator: expected {Protocol.ANSWER_END_BYTE!r}, got {self._serial.terminator!r}"
            )
        self._serial.connect()
        # A reconnect is a different session (possibly a different power source):
        # nothing measured before it may be reported afterwards.
        self._forget_power_v()

        # A mount that stays silent right after the port opens is usually transient (PLAN.md §1 П3):
        # retry the handshake a bounded number of times before declaring the mount unreachable.
        for attempt in range(self._CONNECT_ATTEMPTS):
            try:
                self._transact(_Command.INITIALIZE)
                break
            except SkyWatcherMotorError as error:
                if attempt + 1 >= self._CONNECT_ATTEMPTS:
                    raise SkyWatcherMotorProtocolError(
                        f"mount is not responding on {self._serial.port}: no answer to INITIALIZE after {self._CONNECT_ATTEMPTS} attempts"
                    ) from error

                self._logger.warning("Mount is not responding on %s, attempt %d/%d: %s", self._serial.port, attempt + 1, self._CONNECT_ATTEMPTS, error)
                self._clock.sleep(self._CONNECT_RETRY_DELAY_S * 2 ** attempt)

        mount_version = _Revu24.from_mount(self._transact(_Command.INQUIRE_MOTOR_BOARD_VERSION))
        mount_version = ((mount_version & 0xFF) << 16) | (mount_version & 0xFF00) | ((mount_version & 0xFF0000) >> 16)
        self._mount_code = mount_version & 0xFF
        self._steps_360 = _Revu24.from_mount(self._transact(_Command.INQUIRE_CPR))
        self._steps_worm = _Revu24.from_mount(self._transact(_Command.INQUIRE_TIMER_FREQ))
        self._highspeed_ratio = _Revu24.from_mount(self._transact(_Command.INQUIRE_HIGHSPEED_RATIO), hex_chars=2)
        if self._steps_360 <= 0 or self._steps_worm <= 0 or self._highspeed_ratio <= 0:
            raise SkyWatcherMotorProtocolError(
                f"invalid mount config: steps_360={self._steps_360} steps_worm={self._steps_worm} highspeed_ratio={self._highspeed_ratio}"
            )
        self._min_period = self._measure_min_period()
        self._is_connected = True

    def _measure_min_period(self) -> int:
        """Ask the board itself how fast it is willing to go (§10.5).

        The board takes `:I1` with any value, answers `=`, and stores
        `max(value, its own minimum)` — 1103 on this controller, i.e. exactly 100x
        sidereal, a limit neither the specification nor the INDI reference mentions.
        The driver used to guess that limit from the mount code (`0x0A -> 0x0600`, a
        number with no source anywhere), which was wrong in both directions: it cut 28%
        off the range the board offers, and it disagreed with the board, so the speed
        reported upwards was not the speed the axis ran at. The vendor's own app has no
        such table either (RA_SA_CONSOLE_PROTOCOL.md: `rpsToStride` clamps nothing).
        Four commands once per connect replace the guess with the fact; the axis must be
        stopped, which is what the probe checks first.
        """
        if self._get_status().running:
            self._logger.warning(
                "Axis is running at connect, cannot probe the board's minimum step period; "
                "falling back to %d, so reported speeds may be optimistic",
                self._DEFAULT_MIN_PERIOD,
            )
            return self._DEFAULT_MIN_PERIOD
        try:
            saved_period = _Revu24.from_mount(self._transact(_Command.INQUIRE_STEP_PERIOD))
        except SkyWatcherMotorError as error:
            self._logger.warning("Could not read the step period back, falling back to %d: %s", self._DEFAULT_MIN_PERIOD, error)
            return self._DEFAULT_MIN_PERIOD

        try:
            self._transact(_Command.SET_STEP_PERIOD, _Revu24.from_int(self._MIN_PERIOD_PROBE))
            measured = _Revu24.from_mount(self._transact(_Command.INQUIRE_STEP_PERIOD))
        except SkyWatcherMotorError as error:
            self._logger.warning("Could not probe the board's minimum step period, falling back to %d: %s", self._DEFAULT_MIN_PERIOD, error)
            measured = self._DEFAULT_MIN_PERIOD
        finally:
            # The probe leaves the fastest period the board has loaded. Whatever happened
            # above, put back what was there: a `:J1` from anywhere else (a leftover GOTO,
            # a manual session) would otherwise start the axis at the board's top speed
            # instead of at the tracking period the board powers up with (§12.1).
            self._restore_step_period(saved_period)

        min_period = max(measured, self._DEFAULT_MIN_PERIOD)
        self._logger.info("Board clamps the step period at %d (%.1fx sidereal at most)", min_period, self._times_sidereal(min_period))
        return min_period

    def _restore_step_period(self, period: int) -> None:
        try:
            self._transact(_Command.SET_STEP_PERIOD, _Revu24.from_int(period))
        except SkyWatcherMotorError as error:
            self._logger.warning("Could not restore the step period %d after the probe: %s", period, error)

    def _times_sidereal(self, period: int) -> float:
        return float(STELLAR_DAY) * self._steps_worm / self._steps_360 / period

    def disconnect(self) -> bool:
        self._is_connected = False
        self._forget_power_v()
        self._serial.close()
        return True

    def _forget_power_v(self) -> None:
        self._battery_v = None
        self._usb_v = None
        self._power_v_updated = NEVER
        self._power_v_retry_at = NEVER

    def status(self) -> MotorStatus:
        status = self._get_status()
        if status.running and status.slew_mode == _SlewMode.GOTO:
            motion_mode = MotionMode.TARGET
        elif status.running:
            motion_mode = MotionMode.RUN
        else:
            motion_mode = MotionMode.IDLE
        if not status.running:
            direction = MotorDirection.STOP
        elif status.direction == _Direction.FORWARD:
            direction = MotorDirection.FORWARD
        else:
            direction = MotorDirection.BACKWARD

        return MotorStatus(
            is_connected=self._is_connected,
            steps=self.convert_position_to_steps(self._get_position()),
            motion_mode=motion_mode,
            speed_sps=self._last_speed_sps,
            accel_sps=None,
            direction=direction,
            target=self._last_target if status.slew_mode == _SlewMode.GOTO else None,
            microsteps=None,
            # Whatever the last poll brought, never a poll of its own: `status()` is
            # called far more often than the voltage changes, and it must not turn
            # into eight extra commands per call.
            power_v=self._cached_power_v(),
            initialized=status.initialized,
        )

    def get_power_v(self) -> float | None:
        """Supply voltage of the RA board, or None while it is unknown.

        §6.8: the board does have a voltage query after all — not a command of its
        own but the `:C`/`:n` memory window, found by disassembling the vendor's
        SAM Console app and then confirmed on the wire. Battery volts * 100 sit at
        0x0004/0x0005, USB volts * 100 at 0x001C/0x001D, both int16 little-endian.
        The measured pair (6.04 V against 6.08 V on the multimeter, 4.70 V on USB)
        is what the simulator defaults to.

        The number reported is `max(battery, usb)`, as the vendor's own UI shows
        it. The board is fed from whichever source is higher, so that maximum is
        the rail the regulator actually sees — and it is the only formula that
        says something true in all three wirings: on USB only the battery channel
        reads ~0, on batteries only the USB channel does, and reporting a flat 0
        would be a false alarm. The two channels stay separately visible in
        `protocol_monitor()`, so a battery sagging under a healthy USB is not
        hidden by the maximum.
        """
        if not self._is_connected:
            return None

        now = self._clock.monotonic()
        if now - self._power_v_updated < self._POWER_CACHE_TTL_S or now < self._power_v_retry_at:
            return self._cached_power_v()

        try:
            battery_v = self._read_voltage(self._BATTERY_VOLT_ADDRESS)
            usb_v = self._read_voltage(self._USB_VOLT_ADDRESS)
        except SkyWatcherMotorError as error:
            # A board that cannot answer this will not answer it in 40ms either, and
            # the caller is a dashboard tick. Back off, report "unknown", stay quiet.
            self._battery_v = None
            self._usb_v = None
            self._power_v_retry_at = now + self._POWER_FAILURE_BACKOFF_S
            self._logger.warning(
                "Could not read the RA supply voltage, next try in %.0fs: %s", self._POWER_FAILURE_BACKOFF_S, error
            )
            return None

        self._battery_v = battery_v
        self._usb_v = usb_v
        self._power_v_updated = now
        self._power_v_retry_at = NEVER
        return self._cached_power_v()

    def _cached_power_v(self) -> float | None:
        voltages = [v for v in (self._battery_v, self._usb_v) if v is not None]
        return max(voltages) if voltages else None

    def _read_voltage(self, address: int) -> float:
        # int16 little-endian: the low byte is at `address`, the high byte one above.
        # Swapping them turns 6.04 V into 236.44 V, which is why both halves are read
        # here and not by two independent callers.
        low = self._read_memory_byte(address)
        high = self._read_memory_byte(address + 1)
        return ((high << 8) | low) * self._VOLT_PER_COUNT

    def _read_memory_byte(self, address: int) -> int:
        # `:C1<lo><hi>\r` then `:n1\r` -> `=XX\r`. The address is 16-bit, low byte
        # first (`:C11C00` is 0x001C, not 0x1C00), and the window does not auto
        # increment: it has to be set again before every single byte.
        self._transact(_Command.SET_MEMORY_ADDRESS, f"{address & 0xFF:02X}{(address >> 8) & 0xFF:02X}")
        data = self._transact(_Command.INQUIRE_MEMORY_BYTE)
        if len(data) != 2 or any(char not in "0123456789abcdefABCDEF" for char in data):
            raise SkyWatcherMotorProtocolError(f"expected 2 hex chars from the memory window, got {data!r}")
        return int(data, 16)

    def protocol_monitor(self) -> dict[str, object]:
        return {
            "speed_mode": "-" if self._last_status is None else f"{self._last_status.speed_mode.name.lower()}({int(self._last_status.speed_mode)})",
            "highspeed_ratio": self._highspeed_ratio or "-",
            "initialized": "-" if self._last_status is None else ("yes" if self._last_status.initialized else "NO"),
            # Both rails, because `get_power_v` reports only the higher one (§6.8).
            "battery_v": "-" if self._battery_v is None else f"{self._battery_v:.2f}",
            "usb_v": "-" if self._usb_v is None else f"{self._usb_v:.2f}",
        }

    def _get_preferred_speed_mode(self, fallback: _SpeedMode) -> _SpeedMode:
        if self._last_speed_sps > 0:
            return self._get_speed_mode_for_speed_sps(self._last_speed_sps)
        return fallback

    def set_steps(self, steps: int) -> bool:
        status = self._get_status()
        self._ensure_not_goto(status, "cannot change steps while GOTO is in progress")
        if status.running:
            # §11, the one firmware defect found on this board: `:E` sent while the axis
            # moves is *accepted* with `=` (the spec's `!2` never comes) and then clears
            # the initialization flag asynchronously, 5 runs out of 5. The board offers no
            # protection, so this host-side check is the whole of it.
            raise MotorStopRequire("cannot change steps while motor is moving")
        self._transact(_Command.SET_AXIS_POSITION, _Revu24.from_int((steps + self._POSITION_OFFSET) % self._steps_360))
        self._mount_position_cache = self.convert_steps_to_position(steps)
        self._mount_position_cache_updated = self._clock.monotonic()
        self._recover_initialization_after_set_position()
        return True

    def _recover_initialization_after_set_position(self) -> None:
        # §11 requirements 3 and 4: after *any* `:E` re-read `:f1` and re-initialize if the
        # flag went down — and do the re-read late, because the flag drops hundreds of
        # milliseconds after the command was acknowledged. An immediate check would pass
        # while the flag is still up and miss the very thing it is there to catch.
        self._clock.sleep(self._INIT_FLAG_SETTLE_S)
        if self._get_status().initialized:
            return
        self._logger.warning(
            "Initialization flag went down after SET_AXIS_POSITION (RA_PROTOCOL.md §11), re-initializing the axis"
        )
        self._transact(_Command.INITIALIZE)
        if not self._get_status().initialized:
            raise SkyWatcherMotorProtocolError("axis stays uninitialized after re-sending INITIALIZE")

    def set_speed(self, steps_per_second: int) -> int:
        status = self._get_status()
        self._ensure_not_goto(status, "cannot change speed while GOTO is in progress")
        if steps_per_second <= 0:
            raise ValueError(f"steps_per_second must be positive, got {steps_per_second}")
        target_speed_mode = self._get_speed_mode_for_speed_sps(steps_per_second)
        self._set_motion(
            _MotionStatus(
                slew_mode=status.slew_mode,
                direction=status.direction,
                speed_mode=target_speed_mode,
            ),
            status,
        )
        period = self._clamp_period(self._period_from_speed_sps(steps_per_second), target_speed_mode)
        self._transact(_Command.SET_STEP_PERIOD, _Revu24.from_int(period))
        self._last_speed_sps = self._speed_sps_from_period(period, target_speed_mode)
        return self._last_speed_sps

    def set_acceleration(self, steps_per_second_square: float) -> bool:
        self._ensure_not_goto(self._get_status(), "cannot change acceleration while GOTO is in progress")
        return False

    def set_direction(self, direction: MotorDirection) -> bool:
        status = self._get_status()
        self._ensure_not_goto(status, "cannot change direction while GOTO is in progress")
        if status.running:
            raise MotorStopRequire("cannot change direction while motor is moving")
        if direction == MotorDirection.STOP:
            self._last_direction = direction
            return True
        self._set_motion(
            _MotionStatus(
                slew_mode=_SlewMode.SLEW,
                direction=_Direction.FORWARD if direction == MotorDirection.FORWARD else _Direction.BACKWARD,
                speed_mode=self._get_preferred_speed_mode(status.speed_mode),
            ),
            status,
        )
        self._last_direction = direction
        return True

    def set_delta(self, delta_steps: int) -> bool:
        status = self._get_status()
        self._ensure_not_goto(status, "cannot change target while GOTO is in progress")
        delta = self.convert_steps_to_position(delta_steps).moving_wrap()
        if delta == Ha(0):
            self._last_target = None
            self._zero_target_pending = True
            return True
        self._zero_target_pending = False
        speed = self._get_goto_speed(delta)
        target_speed_mode = self._get_speed_mode_for_speed_sps(self.convert_speed_to_steps_per_second(abs(speed)))
        self._set_motion(
            _MotionStatus(
                slew_mode=_SlewMode.GOTO,
                direction=_Direction.FORWARD if speed > HaPerSecond(0) else _Direction.BACKWARD,
                speed_mode=target_speed_mode,
            ),
            status,
        )
        requested_speed_sps = self.convert_speed_to_steps_per_second(abs(speed))
        period = self._clamp_period(self._period_from_speed_sps(requested_speed_sps), target_speed_mode)
        self._last_speed_sps = self._speed_sps_from_period(period, target_speed_mode)
        self._transact(_Command.SET_STEP_PERIOD, _Revu24.from_int(period))
        self._last_target = abs(delta_steps) % self._steps_360
        self._transact(_Command.SET_GOTO_TARGET_INCREMENT, _Revu24.from_int(self._last_target))
        self._transact(_Command.SET_BREAK_POINT_INCREMENT, _Revu24.from_int(min(200, self._last_target)))
        self._last_direction = MotorDirection.FORWARD if speed > HaPerSecond(0) else MotorDirection.BACKWARD
        return True

    def get_speed_sps_by_delta(self, delta_steps: int) -> int:
        # What the axis above gets is what the mount will really do, not what the driver
        # would like it to do. `Axis._run_goto_to` turns this number into the GOTO ETA and
        # subtracts the sky drift over that ETA, so an optimistic answer is not cosmetic:
        # asking for 800x and getting 100x made the move take 8 times longer than the axis
        # expected and cut the sky compensation by the same factor — an undershoot and a
        # second GOTO. The request therefore goes through the same round trip the board
        # imposes (speed -> period -> clamp -> speed) before it is reported.
        requested_speed = self._get_goto_speed(self.convert_steps_to_position(delta_steps).moving_wrap())
        return self._achievable_speed_sps(self.convert_speed_to_steps_per_second(requested_speed))

    def _achievable_speed_sps(self, speed_sps: int) -> int:
        speed_mode = self._get_speed_mode_for_speed_sps(speed_sps)
        period = self._clamp_period(self._period_from_speed_sps(speed_sps), speed_mode)
        return self._speed_sps_from_period(period, speed_mode)

    def get_speed_by_speed_sps(self, speed_sps: int) -> HaPerSecond:
        if speed_sps < 0:
            raise ValueError(f"speed_sps must be non-negative, got {speed_sps}")
        return HaPerSecond(float(speed_sps) * 24 * 60 * 60 / self._steps_360)

    def set_motion_mode(self, motion_mode: MotionMode) -> bool:
        status = self._get_status()
        self._ensure_not_goto(status, "cannot change motion mode while GOTO is in progress")
        if motion_mode == MotionMode.RUN:
            self._set_motion(
                _MotionStatus(
                    slew_mode=_SlewMode.SLEW,
                    direction=_Direction.FORWARD if self._last_direction != MotorDirection.BACKWARD else _Direction.BACKWARD,
                    speed_mode=self._get_preferred_speed_mode(status.speed_mode),
                ),
                status,
            )
            return True
        if motion_mode == MotionMode.TARGET:
            self._set_motion(
                _MotionStatus(
                    slew_mode=_SlewMode.GOTO,
                    direction=_Direction.FORWARD if self._last_direction != MotorDirection.BACKWARD else _Direction.BACKWARD,
                    speed_mode=self._get_preferred_speed_mode(status.speed_mode),
                ),
                status,
            )
            return True
        if motion_mode == MotionMode.IDLE:
            return True
        raise MotorStateError(f"unsupported motion mode: {motion_mode}")

    def set_microsteps(self, microsteps: int) -> bool:
        status = self._get_status()
        self._ensure_not_goto(status, "cannot change microsteps while GOTO is in progress")
        if status.running:
            raise MotorStopRequire("cannot change microsteps while motor is moving")
        return False

    def convert_position_to_steps(self, position: Ha) -> int:
        self._ensure_geometry_ready()
        return int(round(float(position) / (24 * 60 * 60) * self._steps_360))

    def convert_steps_to_position(self, steps: int) -> Ha:
        self._ensure_geometry_ready()
        return Ha(steps / self._steps_360 * 24 * 60 * 60)

    def convert_speed_to_steps_per_second(self, speed: HaPerSecond) -> int:
        self._ensure_geometry_ready()
        return int(round(abs(float(speed)) * self._steps_360 / (24 * 60 * 60)))

    def run(self) -> bool:
        status = self._get_status()
        self._ensure_not_goto(status, "cannot run while GOTO is in progress")
        if self._zero_target_pending:
            self._zero_target_pending = False
            return True
        if self._last_target is not None and status.slew_mode != _SlewMode.GOTO:
            raise MotorStateError("cannot run before motor motion mode is switched")
        self._transact(_Command.START_MOTION)
        return True

    def stop(self) -> bool:
        # §10.3: `K` starts a braking *ramp*. The board answers `=` immediately, but at
        # the top speed it allows the axis keeps running for another 0.6s and coasts
        # ~1148 counts. Returning True on the `=` alone made `Axis.disconnect` (reset ->
        # stop -> disconnect) close the port on a physically moving axis, so the caller
        # gets control back only once the Running bit is really down.
        self._request_stop()
        self._wait_till_running_clears(self._STOP_TIMEOUT_S)
        return True

    def wait_till_stop(self, do_stop: bool = True, timeout_s: float | None = None) -> None:
        if do_stop:
            self._request_stop()
        self._wait_till_running_clears(timeout_s)

    def _request_stop(self) -> None:
        self._transact(_Command.STOP_MOTION)
        self._last_target = None
        self._zero_target_pending = False

    def _wait_till_running_clears(self, timeout_s: float | None) -> None:
        deadline = None if timeout_s is None else self._clock.monotonic() + timeout_s
        while self._get_status().running:
            if deadline is not None and self._clock.monotonic() > deadline:
                raise SkyWatcherMotorTimeoutError(f"motor did not stop within {timeout_s}s")
            self._clock.sleep(self._STOP_POLL_S)

    def reset(self) -> None:
        self.stop()
        self._last_speed_sps = 0
        self._last_direction = MotorDirection.STOP
        self._last_target = None
        self._zero_target_pending = False

    def _get_status(self) -> _Status:
        self._last_status = _Status.from_bytes(self._transact(_Command.INQUIRE_STATUS).encode("ascii"))
        return self._last_status

    def _get_position(self) -> Ha:
        self._ensure_geometry_ready()
        if self._clock.monotonic() - self._mount_position_cache_updated <= 0.25:
            return self._mount_position_cache
        ticks = (_Revu24.from_mount(self._transact(_Command.INQUIRE_POSITION)) - self._POSITION_OFFSET) % self._steps_360
        self._mount_position_cache = self.convert_steps_to_position(ticks).wrap()
        self._mount_position_cache_updated = self._clock.monotonic()
        return self._mount_position_cache

    def _set_motion(self, target: _MotionStatus, current: _Status) -> None:
        if current.running and (
            current.slew_mode != target.slew_mode
            or current.direction != target.direction
            or current.speed_mode != target.speed_mode
        ):
            raise MotorStopRequire("cannot change motion mode while motor is moving")
        self._transact(_Command.SET_MOTION_MODE, target.to_command())

    def _get_goto_speed(self, delta: Ha) -> HaPerSecond:
        speed = self._HIGHSPEED_SPEED if abs(delta) > self._LOWSPEED_MARGIN else self._LOWSPEED_SPEED
        return speed if delta >= Ha(0) else -speed

    def _get_speed_mode_for_speed_sps(self, speed_sps: int) -> _SpeedMode:
        return _SpeedMode.HIGHSPEED if speed_sps > self.convert_speed_to_steps_per_second(self._LOWSPEED_SPEED) else _SpeedMode.LOWSPEED

    def _period_from_speed_sps(self, speed_sps: int) -> int:
        self._ensure_timing_ready()
        if speed_sps <= 0:
            raise MotorStateError("speed must be positive")
        rate = speed_sps * (24 * 60 * 60) / self._steps_360 / float(STELLAR_SPEED)
        if self._get_speed_mode_for_speed_sps(speed_sps) == _SpeedMode.HIGHSPEED:
            rate /= self._highspeed_ratio
        return int(STELLAR_DAY * self._steps_worm / self._steps_360 / rate)

    def _clamp_period(self, period: int, speed_mode: _SpeedMode) -> int:
        if period < self._min_period:
            return self._min_period
        return period

    def _speed_sps_from_period(self, period: int, speed_mode: _SpeedMode) -> int:
        self._ensure_timing_ready()
        if period <= 0:
            raise MotorStateError("period must be positive")
        rate = float(STELLAR_DAY) * self._steps_worm / self._steps_360 / period
        if speed_mode == _SpeedMode.HIGHSPEED:
            rate *= self._highspeed_ratio
        return int(round(rate * float(STELLAR_SPEED) * self._steps_360 / (24 * 60 * 60)))

    def _ensure_not_goto(self, status: _Status, message: str) -> None:
        if status.running and status.slew_mode == _SlewMode.GOTO:
            raise MotorStopRequire(message)

    def _ensure_geometry_ready(self) -> None:
        if self._steps_360 <= 0:
            raise SkyWatcherMotorProtocolError("motor geometry is not initialized")

    def _ensure_timing_ready(self) -> None:
        # The timer frequency and the high-speed ratio are read after the CPR, so a
        # handshake that died in between leaves them at 0 while the axis conversions
        # already work. Dividing by them then is worse than a ZeroDivisionError: a zero
        # `_steps_worm` makes `_period_from_speed_sps` return 0, the period clamps to the
        # board minimum, and the axis silently slews at the fastest rate the board has.
        self._ensure_geometry_ready()
        if self._steps_worm <= 0 or self._highspeed_ratio <= 0:
            raise SkyWatcherMotorProtocolError(
                f"motor timing is not initialized: steps_worm={self._steps_worm} highspeed_ratio={self._highspeed_ratio}"
            )

    REPEATS = 3
    def _transact(self, command: _Command, arg: str | None = None) -> str:
        # Every command this board understands has the same frame: `:<cmd><channel><data>\r`,
        # answered with `=<data>\r` or `!<code>\r`. There is no second framing (§8: only `\r`
        # terminates anything), so there is no second branch here either.
        payload = f"{Protocol.COMMAND_PREFIX}{command.value}{_Axis.RA}{arg or ''}{Protocol.COMMAND_TERMINATOR}"
        response_prefixes = (Protocol.RESPONSE_PREFIX_BYTE, Protocol.COMMAND_ERROR_PREFIX_BYTE)

        count = self.REPEATS
        response = None
        while count > 0:
            try:
                response = self._serial.query(payload, response_prefixes=response_prefixes)
                if not response:
                    raise SkyWatcherMotorProtocolError(f"empty response: {response!r}")
                if not response.endswith(Protocol.ANSWER_END):
                    raise SkyWatcherMotorProtocolError(f"unterminated response: {response!r}")
                if response[0] == Protocol.COMMAND_ERROR_PREFIX:
                    raise SkyWatcherMotorCommandError(f"command error: {response!r}")
                if response[0] != Protocol.RESPONSE_PREFIX:
                    raise SkyWatcherMotorProtocolError(f"invalid response: {response!r}")

                return response[1:-len(Protocol.ANSWER_END)]

            except SkyWatcherMotorProtocolError:
                # An in-transaction retry is a recoverable transient: the caller decides whether the
                # final failure is worth an error record, so keep the per-attempt trace on DEBUG.
                self._logger.debug("While quering %s(%s) `%s` -> `%s`, %d last", command.name, arg, payload, response, count, exc_info=True)
                # Read the leftovers *before* dropping them. The old order was the reverse —
                # `drop_buffers()` and then a read that could only come back empty — which is
                # why the March logs hold 20 601 records of `['']` and not one byte of the
                # garbage that actually confused the parser.
                data = self._serial.read_all_data(timeout=.5)
                if data:
                    self._logger.info("Discarding %d leftover byte-groups after a protocol error: %s", len(data), data)
                self._serial.drop_buffers()
                count -= 1
                if count == 0:
                    raise
                self._clock.sleep(0.1)
        raise SkyWatcherMotorProtocolError(f"failed to execute command {command} with payload {payload}")
