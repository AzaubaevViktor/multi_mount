import dataclasses
import logging
from enum import StrEnum

from clock import REAL_CLOCK, Clock, TtlCache
from pointing.sensor import SensorReading, SensorState
from serial_wrapper.wrapper import SerialLine
from sky.motor import MotionMode, Motor, MotorDirection, MotorStateError, MotorStatus, MotorStopRequire
from sky.physics import Dec, DecPerSecond, DecStepsPerSecond, StepsPerSecond
from tmc2209.protocol import (
    KEY_VALUE_SEPARATOR,
    RESPONSE_DELIMITER,
    Response as _Response,
    TMC2209MotorCommandError,
    TMC2209MotorConcatenatedResponseError,
    TMC2209MotorEchoMismatchError,
    TMC2209MotorError,
    TMC2209MotorIntegrityError,
    TMC2209MotorLegacyResponseError,
    TMC2209MotorProtocolError,
    TMC2209MotorStaleResponseError,
    TMC2209MotorTimeoutError,
    TMC2209MotorTruncatedResponseError,
    build_request,
    decode_response,
    encode_frame,
)
from utils.polar_align_lib import Vec3

# Re-exported: callers (and the hardware suite) have always imported the error
# hierarchy from this module, and the codec split must not move their imports.
__all__ = [
    "MICROSTEPS_ALLOWED",
    "TMC2209Motor",
    "TMC2209MotorCommandError",
    "TMC2209MotorConcatenatedResponseError",
    "TMC2209MotorEchoMismatchError",
    "TMC2209MotorError",
    "TMC2209MotorIntegrityError",
    "TMC2209MotorLegacyResponseError",
    "TMC2209MotorProtocolError",
    "TMC2209MotorStaleResponseError",
    "TMC2209MotorTimeoutError",
    "TMC2209MotorTruncatedResponseError",
]

COMMAND_TERMINATOR = "\n"
MICROSTEPS_ALLOWED = {1, 2, 4, 8, 16, 32, 64, 128, 256}
DRIVER_FLAG_STALL = 0x10
DEGREES_PER_REV = 360.0
STEPS_PER_REV = 200
GEAR_RATIO_1 = 44 / 26
# Calibrated from DEC under-travel: start 47d23m15s, expected 43d06m14s,
# actual 43d23m00s, so the DEC scale needs 1.069788414846x more travel.
GEAR_RATIO_2 = 125.608671834894


class _Dialect(StrEnum):
    AUTO = "auto"
    """Probe the board on connect and pick one of the two below."""

    FRAMED = "framed"
    """Protocol v3: length + CRC + sequence number (``tmc2209/protocol.py``)."""

    LEGACY = "legacy"
    """Protocol v2: the ``1;key=value;`` lines of the firmware still in the field."""


class _Phase(StrEnum):
    IDLE = "idle"
    HOLD = "hold"
    ACCELERATION = "acceleration"
    RUNNING = "running"
    DECELERATION = "deceleration"


class _Mode(StrEnum):
    TARGET = "target"
    FREE_RIDE = "free_ride"


class _Safety(StrEnum):
    NORMAL = "normal"
    DERATED = "derated"
    STOPPING = "stopping"
    SHUTDOWN = "shutdown"


@dataclasses.dataclass(frozen=True)
class _Status:
    initialised: bool
    enabled: bool
    mode: _Mode
    position: int
    phase: _Phase
    target: int
    target_set: bool
    speed_sps: DecStepsPerSecond
    actual_speed_sps: DecStepsPerSecond
    accel_steps_per_s: float
    power_v: float | None = None
    # Firmware older than the TX-ring fix does not report it; None means "unknown", not zero.
    tx_overflow: int | None = None
    driver_flags: int | None = None
    safety: _Safety | None = None
    active_microsteps: int | None = None
    fine_microsteps: int | None = None
    speed_limit_sps: int | None = None
    safety_events: int | None = None

    @classmethod
    def from_response(cls, response: _Response) -> "_Status":
        # A status frame that lost a field, or a field that lost its digits, used to
        # leave here as a bare KeyError/ValueError: past the retry in `_transact`
        # (which only catches command errors) and past every caller, all of which
        # expect a TMC2209MotorError. On the framed dialect the CRC stops a damaged
        # frame long before this point; on the legacy one this is the last line of
        # defence, and the reason PLAN.md #33 exists.
        try:
            return cls(
                initialised=response.values["initialised"] == "1",
                enabled=response.values["enabled"] == "1",
                mode=_Mode(response.values.get("mode", _Mode.TARGET.value)),
                position=int(response.values["position"]),
                phase=_Phase(response.values["phase"]),
                target=int(response.values["target"]),
                target_set=response.values["target_set"] == "1",
                speed_sps=DecStepsPerSecond(float(response.values["speed"])),
                actual_speed_sps=DecStepsPerSecond(float(response.values["actual_speed"])),
                accel_steps_per_s=float(response.values["accel_per_s"]),
                power_v=float(response.values["power_v"]) if "power_v" in response.values else None,
                tx_overflow=int(response.values["tx_overflow"]) if "tx_overflow" in response.values else None,
                driver_flags=int(response.values["drv_flags"]) if "drv_flags" in response.values else None,
                safety=_Safety(response.values["safety"]) if "safety" in response.values else None,
                active_microsteps=int(response.values["mres"]) if "mres" in response.values else None,
                fine_microsteps=int(response.values["fine_mres"]) if "fine_mres" in response.values else None,
                speed_limit_sps=int(response.values["limit"]) if "limit" in response.values else None,
                safety_events=int(response.values["events"]) if "events" in response.values else None,
            )
        except KeyError as error:
            raise TMC2209MotorTruncatedResponseError(
                f"status response has no {error.args[0]!r} field: {response.values}"
            ) from error
        except ValueError as error:
            raise TMC2209MotorProtocolError(f"status response carries an unusable value: {response.values}") from error


class TMC2209Motor(Motor[Dec, DecPerSecond]):
    FORWARD_POSITION_SIGN = 1
    _READY_RETRIES = 3
    _READY_TIMEOUT_S = 10.0
    _POWER_CACHE_TTL_S = 2.0

    _PROBE_ATTEMPTS = 3

    def __init__(self, serial: SerialLine, clock: Clock = REAL_CLOCK, dialect: _Dialect = _Dialect.AUTO) -> None:
        self._serial = serial
        self._clock = clock
        self._logger = logging.getLogger(type(self).__name__)
        self._microsteps = 16
        self._is_connected = False
        self._direction = MotorDirection.STOP
        # Firmware without the `power_v` field answers None, and that answer is
        # cached like any other: re-asking every tick would cost a status frame
        # per call to learn the same nothing.
        self._power_v: TtlCache[float | None] = TtlCache(self._POWER_CACHE_TTL_S, clock)
        self._last_tx_overflow: int | None = None
        self._last_driver_flags: int | None = None
        self._last_safety: _Safety | None = None
        self._last_safety_events: int | None = None
        self._last_active_microsteps: int | None = None
        self._dialect = dialect
        self._seq = 0

    def connect(self) -> None:
        ready = ""
        for attempt in range(self._READY_RETRIES):
            self._serial.connect()
            self._serial.reset()
            self._serial.read_all_data()

            deadline = self._clock.monotonic() + self._READY_TIMEOUT_S
            while self._clock.monotonic() < deadline:
                ready = self._serial.query(None, timeout=1)
                if ready and ready.strip() == "ready":
                    if self._dialect is _Dialect.AUTO:
                        self._probe_dialect()
                    self._is_connected = True
                    return

            self._serial.close()
            if attempt < self._READY_RETRIES - 1:
                self._clock.sleep(0.5)

        raise TMC2209MotorProtocolError(f"device not ready: {ready!r}")

    def _probe_dialect(self) -> None:
        """Decide which protocol the board on the other end speaks.

        The probe is one HELLO frame. Firmware that predates the framed dialect
        sees an unknown command and answers ``0;error=unknown_cmd;`` — a v2 line,
        which is *positive* proof and the only thing that switches the driver down
        to the legacy protocol. Silence proves nothing (a chewed-up line looks the
        same on both firmwares), so an inconclusive probe stays on the framed
        dialect: a board that then rejects every frame fails loudly, whereas a
        needless downgrade would go back to corrupting data quietly.
        """
        for _ in range(self._PROBE_ATTEMPTS):
            try:
                hello = self._exchange("hello", None)
            except TMC2209MotorLegacyResponseError:
                self._dialect = _Dialect.LEGACY
                self._logger.warning(
                    "DEC controller speaks the v2 line protocol: no frame length, no CRC, "
                    "damaged values cannot be detected. Flash the current firmware to fix it."
                )
                return
            except TMC2209MotorError as error:
                self._logger.info("Protocol probe attempt failed, retrying: %s", error)
                continue
            self._dialect = _Dialect.FRAMED
            self._logger.info("DEC controller speaks framed protocol v%s", hello.values.get("protocol"))
            return

        self._dialect = _Dialect.FRAMED
        self._logger.warning("Protocol probe was inconclusive, assuming the framed protocol")

    def disconnect(self) -> bool:
        self._is_connected = False
        self._power_v.forget()
        self._last_driver_flags = None
        self._last_safety = None
        self._last_safety_events = None
        self._last_active_microsteps = None
        self._serial.close()
        return True

    def status(self) -> MotorStatus[DecPerSecond]:
        status = self._status()
        if status.phase == _Phase.IDLE:
            motion_mode = MotionMode.IDLE
        elif status.mode == _Mode.TARGET and status.phase in (_Phase.ACCELERATION, _Phase.RUNNING, _Phase.DECELERATION):
            motion_mode = MotionMode.TARGET
        elif status.phase == _Phase.ACCELERATION:
            motion_mode = MotionMode.ACCELERATION
        elif status.phase == _Phase.DECELERATION:
            motion_mode = MotionMode.DECELERATION
        else:
            motion_mode = MotionMode.RUN

        if status.phase in (_Phase.IDLE, _Phase.HOLD):
            direction = MotorDirection.STOP
        else:
            direction = self._direction

        return MotorStatus(
            is_connected=self._is_connected,
            steps=status.position,
            motion_mode=motion_mode,
            speed_sps=DecStepsPerSecond(round(abs(float(status.speed_sps)))),
            accel_sps=int(round(status.accel_steps_per_s)),
            direction=direction,
            target=status.target if status.target_set else None,
            microsteps=self._microsteps,
            power_v=None,
        )

    def get_power_v(self) -> float | None:
        if self._is_connected and not self._power_v.is_fresh():
            try:
                # Stored, not returned directly: a failed read must leave the last
                # known voltage standing rather than replace it with None.
                self._power_v.store(self._status().power_v)
            except TMC2209MotorError:
                self._logger.exception("While querying tmc2209 voltage")

        return self._power_v.value

    def set_steps(self, steps: int) -> bool:
        self._ensure_not_goto(self._status(), "cannot change steps while GOTO is in progress")
        self._confirm(self._transact("position", [str(steps)]), "position", steps)
        return True

    def set_speed(self, steps_per_second: StepsPerSecond[DecPerSecond]) -> DecStepsPerSecond:
        self._ensure_not_goto(self._status(), "cannot change speed while GOTO is in progress")
        if steps_per_second < 0:
            raise ValueError(f"steps_per_second must be non-negative, got {steps_per_second}")
        speed = round(float(steps_per_second))
        # The applied speed comes back from the controller, not from the argument:
        # "the command was sent" and "the axis now runs at that rate" are different
        # statements, and only the echo can make the second one.
        applied = self._confirm(self._transact("speed", [str(speed)]), "speed", speed)
        return DecStepsPerSecond(round(float(applied)))

    def set_acceleration(self, steps_per_second_square: float) -> bool:
        self._ensure_not_goto(self._status(), "cannot change acceleration while GOTO is in progress")
        acceleration = int(round(steps_per_second_square))
        if acceleration < 0:
            raise ValueError(f"steps_per_second_square must be non-negative, got {steps_per_second_square}")
        self._confirm(self._transact("acceleration", [str(acceleration)]), "accel_per_s", acceleration)
        return True

    def set_direction(self, direction: MotorDirection) -> bool:
        status = self._status()
        self._ensure_not_goto(status, "cannot change direction while GOTO is in progress")
        if status.phase not in (_Phase.IDLE, _Phase.HOLD):
            raise MotorStopRequire("cannot change direction while motor is moving")
        if direction == MotorDirection.STOP:
            return True
        backward = "1" if direction == MotorDirection.BACKWARD else "0"
        self._confirm(self._transact("direction", [backward]), "direction", backward)
        self._direction = direction
        return True

    def set_delta(self, delta_steps: int) -> bool:
        self._ensure_not_goto(self._status(), "cannot change target while GOTO is in progress")
        self._confirm(self._transact("delta", [str(delta_steps)]), "delta", delta_steps)
        return True

    def get_speed_sps_by_delta(self, delta_steps: int) -> DecStepsPerSecond:
        return DecStepsPerSecond(min(max(abs(delta_steps), 1), 6000))

    def get_speed_by_speed_sps(self, speed_sps: StepsPerSecond[DecPerSecond]) -> DecPerSecond:
        if speed_sps < 0:
            raise ValueError(f"speed_sps must be non-negative, got {speed_sps}")
        return self.convert_steps_to_speed(speed_sps)

    def set_motion_mode(self, motion_mode: MotionMode) -> bool:
        status = self._status()
        self._ensure_not_goto(status, "cannot change motion mode while GOTO is in progress")
        if motion_mode in (MotionMode.IDLE, MotionMode.RUN):
            self._confirm(self._transact("mode", [_Mode.FREE_RIDE.value]), "mode", _Mode.FREE_RIDE.value)
            return True
        if motion_mode == MotionMode.TARGET:
            self._confirm(self._transact("mode", [_Mode.TARGET.value]), "mode", _Mode.TARGET.value)
            return True
        raise MotorStateError(f"unsupported motion mode: {motion_mode}")

    def set_microsteps(self, microsteps: int) -> bool:
        status = self._status()
        self._ensure_not_goto(status, "cannot change microsteps while GOTO is in progress")
        if status.phase not in (_Phase.IDLE, _Phase.HOLD):
            raise MotorStopRequire("cannot change microsteps while motor is moving")
        if microsteps not in MICROSTEPS_ALLOWED:
            raise ValueError(f"microsteps not allowed: {microsteps}")
        echoed = self._transact("set", [f"microsteps={microsteps}"]).values.get("microsteps")
        if echoed != str(microsteps):
            # Never update the host-side scale unless the board echoed the value it
            # actually applied. A broken UART or rejected register write would
            # otherwise corrupt every angle derived from `_steps_per_arcsecond()`.
            raise TMC2209MotorEchoMismatchError(
                f"microsteps stayed at {echoed} after writing {microsteps}: the UART to the driver chip "
                f"did not confirm the requested register value. "
                f"Run `python -m tools.dec_wirescan` to check the wiring and baud rate."
            )
        self._microsteps = microsteps
        return True

    def convert_position_to_steps(self, position: Dec) -> int:
        return int(round(float(position) * self._steps_per_arcsecond()))

    def convert_steps_to_position(self, steps: int) -> Dec:
        return Dec(float(steps) / self._steps_per_arcsecond())

    def convert_speed_to_steps_per_second(self, speed: DecPerSecond) -> DecStepsPerSecond:
        return DecStepsPerSecond(round(abs(float(speed)) * self._steps_per_arcsecond()))

    def run(self) -> bool:
        status = self._status()
        self._ensure_not_goto(status, "cannot run while GOTO is in progress")
        if status.target_set and status.mode != _Mode.TARGET:
            raise MotorStateError("cannot run before motor motion mode is switched")
        self._confirm(self._transact("run"), "running", 1)
        return True

    def stop(self) -> bool:
        self._confirm(self._transact("stop"), "stopping", 1)
        return True

    def wait_till_stop(self, do_stop: bool = True, timeout_s: float | None = None) -> None:
        if do_stop:
            self.stop()
        deadline = None if timeout_s is None else self._clock.monotonic() + timeout_s
        while True:
            status = self._status()
            if status.phase in (_Phase.IDLE, _Phase.HOLD):
                return
            if deadline is not None and self._clock.monotonic() > deadline:
                raise TimeoutError(f"motor did not stop within {timeout_s}s")
            self._clock.sleep(0.05)

    def reset(self) -> None:
        self.wait_till_stop(do_stop=True)
        self._power_v.forget()

    def convert_steps_to_speed(self, speed_sps: StepsPerSecond[DecPerSecond]) -> DecPerSecond:
        return DecPerSecond(float(speed_sps) / self._steps_per_arcsecond())

    def _steps_per_arcsecond(self) -> float:
        return STEPS_PER_REV * self._microsteps * GEAR_RATIO_1 * GEAR_RATIO_2 / (DEGREES_PER_REV * 60 * 60)

    def _confirm(self, response: _Response, key: str, sent: int | str) -> str:
        """Prove the controller acknowledged the value that was actually sent.

        Both dialects answer a write by echoing the field back, and until now the
        driver ignored that echo and reported success on the ``1;`` prefix alone —
        so a value the board clamped, rejected or received damaged was reported as
        applied (DEC_PROTOCOL.md §4, §9.3).
        """
        echoed = response.values.get(key)
        if echoed is None:
            raise TMC2209MotorTruncatedResponseError(f"reply carries no `{key}` to confirm the write: {response.values}")
        try:
            matches = float(echoed) == float(sent)
        except ValueError:
            matches = echoed == str(sent)
        if not matches:
            raise TMC2209MotorEchoMismatchError(f"controller acknowledged {key}={echoed!r} for a written {sent!r}")
        return echoed

    def _ensure_not_goto(self, status: _Status, message: str) -> None:
        if status.mode == _Mode.TARGET and status.phase not in (_Phase.IDLE, _Phase.HOLD):
            raise MotorStopRequire(message)

    def _status(self) -> _Status:
        status = _Status.from_response(self._transact("status"))
        if status.tx_overflow:
            lost = status.tx_overflow - (self._last_tx_overflow or 0)
            if lost > 0:
                self._logger.warning(
                    "TMC2209 dropped %d response(s) to TX ring overflow, %d since controller boot",
                    lost,
                    status.tx_overflow,
                )
        self._last_tx_overflow = status.tx_overflow
        if status.fine_microsteps is not None:
            self._microsteps = status.fine_microsteps
        if status.active_microsteps is not None and status.active_microsteps != self._last_active_microsteps:
            self._logger.info(
                "TMC2209 active microstep resolution changed to 1/%d (fine position units: 1/%d)",
                status.active_microsteps,
                status.fine_microsteps or self._microsteps,
            )
        if status.safety is not None and status.safety != self._last_safety:
            if status.safety == _Safety.NORMAL and self._last_safety is not None:
                self._logger.info("TMC2209 safety state returned to normal")
            elif status.safety != _Safety.NORMAL:
                self._logger.warning(
                    "TMC2209 safety state is %s: flags=0x%02X, speed limit=%s steps/s",
                    status.safety,
                    status.driver_flags or 0,
                    status.speed_limit_sps,
                )
        if status.safety_events is not None:
            new_events = status.safety_events - (self._last_safety_events or 0)
            if new_events > 0:
                self._logger.warning(
                    "TMC2209 reported %d new safety event(s), %d since controller boot",
                    new_events,
                    status.safety_events,
                )
        if status.driver_flags and status.driver_flags != self._last_driver_flags:
            self._logger.warning("TMC2209 driver flags changed to 0x%02X", status.driver_flags)
        if (
            status.driver_flags is not None
            and status.driver_flags & DRIVER_FLAG_STALL
            and not ((self._last_driver_flags or 0) & DRIVER_FLAG_STALL)
        ):
            self._logger.warning("TMC2209 stall detected; controller stopped and disabled motor")
        self._last_driver_flags = status.driver_flags
        self._last_safety = status.safety
        self._last_safety_events = status.safety_events
        self._last_active_microsteps = status.active_microsteps
        return status

    def _exchange(self, command: str, args: list[str] | None) -> _Response:
        """One request, one reply, in whichever dialect this board speaks.

        The framed branch is where the integrity guarantees live: the sequence
        number is bumped per request and checked on the way back, so a reply left
        over from a previous command cannot be mistaken for this one, and the CRC
        rejects anything the line altered.
        """
        if self._dialect is _Dialect.LEGACY:
            payload = command if not args else f"{command} {' '.join(args)}"
            return _Response.from_line(self._serial.query(f"{payload}{COMMAND_TERMINATOR}"))

        op, arg_payload = build_request(command, args)
        self._seq = (self._seq + 1) & 0xFF
        raw = self._serial.query(encode_frame(op, self._seq, arg_payload))
        return _Response.from_frame(decode_response(raw, op=op, seq=self._seq))

    def read_orientation_sensor(self) -> SensorReading:
        if not self._is_connected:
            return SensorReading(SensorState.NOT_CONNECTED)
        try:
            values = self._transact("sensor", attempts=1).values
        except TMC2209MotorCommandError as error:
            if str(error) == "unknown_cmd":
                return SensorReading(SensorState.UNSUPPORTED)
            raise
        try:
            flags = int(values["sensor_flags"])
            if flags not in (0, 1, 3, 7):
                raise ValueError("inconsistent sensor flags")
            if flags < 3:
                return SensorReading(SensorState.DEVICE_NOT_FOUND if flags == 0 else SensorState.NO_DATA)
            sequence, age_ms = int(values["sample"]), int(values["age_ms"])
            keys = ("gx", "gy", "gz", "mx", "my", "mz") if flags & 4 else ("gx", "gy", "gz")
            components = tuple(int(values[key]) for key in keys)
            # The legacy text reply must obey the same bounds as the v3 binary
            # fields; corrupt text must not become an apparently valid vector.
            if (
                not 0 <= sequence <= 0xFFFFFFFF
                or not 0 <= age_ms <= 65535
                or any(not -32768 <= v <= 32767 for v in components)
            ):
                raise ValueError("raw sensor value exceeds wire range")
            return SensorReading(
                SensorState.AVAILABLE,
                sequence,
                age_ms,
                Vec3(*(v / 1000 for v in components[:3])),
                Vec3(*(v / 100 for v in components[3:])) if flags & 4 else None,
            )
        except (ValueError, KeyError) as error:
            raise TMC2209MotorProtocolError("invalid raw sensor response") from error

    def _transact(self, command: str, args: list[str] | None = None, *, attempts: int = 3) -> _Response:
        payload = command if not args else f"{command} {' '.join(args)}"
        count = attempts
        response = None
        while count > 0:
            try:
                response = self._exchange(command, args)
                if not response.ok:
                    raise TMC2209MotorCommandError(response.error or "tmc2209 error")
                return response
            except TMC2209MotorCommandError:
                count -= 1
                if count == 0:
                    raise
                self._logger.exception("TMC2209 WHILE TRANSACTING: %s(%s) `%s` -> `%s`, %d last", command, args, payload, response, count)
                self._serial.drain_after_error(self._logger, "a command error")
                self._clock.sleep(0.1)
        # Unreachable: the retry above re-raises once count hits 0. Mirrors the same
        # guard in SkyWatcherMotor._transact and makes the return type total.
        raise TMC2209MotorProtocolError(f"failed to execute command {command} with payload {payload}")
