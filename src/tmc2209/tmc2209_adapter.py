import dataclasses
from enum import StrEnum
import logging
import time

from lx200.protocols import LX200Dec
from serial_wrapper.wrapper import SerialLine

COMMAND_TERMINATOR = "\n"
RESPONSE_DELIMITER = ";"
KEY_VALUE_SEPARATOR = "="
RESPONSE_OK_VALUE = "1"
RESPONSE_ERROR_VALUE = "0"
HEX_PREFIX = "0x"

DEGREES_PER_REV = 360.0

# Placeholder values; adjust when real mechanics are known.
STEPS_PER_REV = 200
GEAR_RATIO_1 = 1.5
GEAR_RATIO_2 = 300

MICROSTEPS_ALLOWED = {1, 2, 4, 8, 16, 32, 64, 128, 256}


class Phase(StrEnum):
    IDLE = "idle"
    HOLD = "hold"
    ACCELERATION = "acceleration"
    RUNNING = "running"
    DECELERATION = "deceleration"


PHASE_BY_VALUE = {phase.value: phase for phase in Phase}
BOOL_VALUES = {"0", "1"}

MIN_SPEED_SPS = 0
MAX_SPEED_SPS = 6000
MIN_ACCEL_STEPS_PER_MS = 0
MAX_ACCEL_STEPS_PER_MS = 100000


class TMC2209AdapterError(Exception):
    pass


class TMC2209ConfigError(TMC2209AdapterError):
    pass


class TMC2209ProtocolError(TMC2209AdapterError):
    pass


class TMC2209ResponseError(TMC2209AdapterError):
    pass


class TMC2209CommandError(TMC2209AdapterError):
    pass


@dataclasses.dataclass(frozen=True)
class TMC2209Response:
    ok: bool
    values: dict[str, str]
    error: str | None
    raw: str

    def __post_init__(self) -> None:
        if self.ok and self.error is not None:
            raise TMC2209ResponseError("ok response cannot include error")

    @classmethod
    def from_line(cls, line: str) -> "TMC2209Response":
        cleaned = line.strip()
        if not cleaned:
            raise TMC2209ResponseError("empty response")

        tokens = [token for token in cleaned.split(RESPONSE_DELIMITER) if token]
        if not tokens:
            raise TMC2209ResponseError(f"invalid response: {line!r}")

        ok_raw = tokens[0]
        if ok_raw not in (RESPONSE_OK_VALUE, RESPONSE_ERROR_VALUE):
            raise TMC2209ResponseError(f"invalid response prefix: {ok_raw!r}")

        values: dict[str, str] = {}
        for token in tokens[1:]:
            key, value = _split_key_value(token)
            if key in values:
                raise TMC2209ResponseError(f"duplicate key: {key!r}")
            values[key] = value

        ok = ok_raw == RESPONSE_OK_VALUE
        error = values.get("error") if not ok else None
        return cls(ok=ok, values=values, error=error, raw=cleaned)


@dataclasses.dataclass(frozen=True)
class TMC2209Status:
    initialised: bool
    enabled: bool
    position: int
    phase: Phase
    target: int
    target_set: bool
    speed_sps: float
    actual_speed_sps: float
    accel_steps_per_s: float

    @classmethod
    def from_response(cls, response: TMC2209Response) -> "TMC2209Status":
        values = response.values
        phase_raw = _require_value(values, "phase")
        phase = PHASE_BY_VALUE.get(phase_raw)
        if phase is None:
            raise TMC2209ProtocolError(f"unexpected phase: {phase_raw!r}")

        return cls(
            initialised=_parse_bool(_require_value(values, "initialised"), "initialised"),
            enabled=_parse_bool(_require_value(values, "enabled"), "enabled"),
            position=_parse_int(_require_value(values, "position"), "position"),
            phase=phase,
            target=_parse_int(_require_value(values, "target"), "target"),
            target_set=_parse_bool(_require_value(values, "target_set"), "target_set"),
            speed_sps=_parse_float(_require_value(values, "speed"), "speed"),
            actual_speed_sps=_parse_float(_require_value(values, "actual_speed"), "actual_speed"),
            accel_steps_per_s=_parse_float(_require_value(values, "accel_per_s"), "accel_per_s"),
        )


@dataclasses.dataclass(frozen=True)
class TMC2209DriverStatus:
    gconf: int
    drv_status: int
    sg_result: int

    @classmethod
    def from_response(cls, response: TMC2209Response) -> "TMC2209DriverStatus":
        values = response.values
        return cls(
            gconf=_parse_hex(_require_value(values, "gconf"), "gconf"),
            drv_status=_parse_hex(_require_value(values, "drv_status"), "drv_status"),
            sg_result=_parse_int(_require_value(values, "sg_result"), "sg_result"),
        )


def steps_from_dec(dec: LX200Dec, microsteps: int) -> int:
    _validate_microsteps(microsteps)

    steps_per_rev = STEPS_PER_REV * microsteps * GEAR_RATIO_1 * GEAR_RATIO_2
    if steps_per_rev <= 0:
        raise TMC2209ConfigError("steps per revolution must be positive")

    steps_per_degree = steps_per_rev / DEGREES_PER_REV
    return int(round(dec.to_degrees() * steps_per_degree))


class TMC2209Adapter:
    def __init__(self, serial: SerialLine) -> None:
        self._serial = serial
        self._log = logging.getLogger("tmc2209.adapter")
        self._last_status_snapshot: tuple[bool, bool, Phase, bool] | None = None

        self._validate_serial()

    def connect(self) -> None:
        self._serial.connect()
        self._serial.reset()
        self._serial.read_all_data()
        self._log.debug("Wait while ready for ...")
        start = time.monotonic()
        while not (ready := self._serial.query(None, timeout=10)):
            if time.monotonic() - start > 10:
                break

        if ready.strip() != "ready":
            raise ValueError(f"Device not ready `{ready}`")
        
        self._log.info("Connected")

    def close(self) -> None:
        self._serial.close()

    def status(self) -> TMC2209Status:
        response = self._transact("status")
        status = TMC2209Status.from_response(response)
        current_snapshot = (
            status.initialised,
            status.enabled,
            status.phase,
            status.target_set,
        )

        if self._last_status_snapshot is None:
            self._log.info(
                "Status initial: initialised=%s enabled=%s phase=%s target_set=%s",
                status.initialised,
                status.enabled,
                status.phase,
                status.target_set,
            )
        elif current_snapshot != self._last_status_snapshot:
            previous = self._last_status_snapshot
            self._log.info(
                "Status changed: initialised %s -> %s enabled %s -> %s phase %s -> %s target_set %s -> %s",
                previous[0],
                status.initialised,
                previous[1],
                status.enabled,
                previous[2],
                status.phase,
                previous[3],
                status.target_set,
            )
        else:
            self._log.debug(
                "Status poll: initialised=%s enabled=%s phase=%s target_set=%s position=%s",
                status.initialised,
                status.enabled,
                status.phase,
                status.target_set,
                status.position,
            )

        self._last_status_snapshot = current_snapshot
        return status

    def driver_status(self) -> TMC2209DriverStatus:
        response = self._transact("driver_status")
        return TMC2209DriverStatus.from_response(response)

    def full_status(self) -> TMC2209DriverStatus:
        response = self._transact("full_status")
        return TMC2209DriverStatus.from_response(response)

    def get_param(self, name: str) -> str:
        name = _normalize_param_name(name)
        response = self._transact("get", [name])
        return _require_value(response.values, name)

    def set_param(self, name: str, value: str | int | float | bool) -> str:
        name = _normalize_param_name(name)
        value_str = _format_param_value(value)
        response = self._transact("set", [f"{name}{KEY_VALUE_SEPARATOR}{value_str}"])
        return _require_value(response.values, name)

    def set_position(self, position: int) -> int:
        self._log.info("Set position request: position=%s", position)
        response = self._transact("position", [str(position)])
        response_position = _parse_int(_require_value(response.values, "position"), "position")
        self._log.info("Set position applied: position=%s", response_position)
        return response_position

    def set_enabled(self, enabled: bool) -> bool:
        self._log.info("Set enabled request: enabled=%s", enabled)
        value = "1" if enabled else "0"
        response = self._transact("enabled", [value])
        is_enabled = _parse_bool(_require_value(response.values, "enabled"), "enabled")
        self._log.info("Set enabled applied: enabled=%s", is_enabled)
        return is_enabled

    def set_direction(self, direction: bool) -> bool:
        self._log.info("Set direction request: backward=%s", direction)
        value = "1" if direction else "0"
        response = self._transact("direction", [value])
        is_backward = _parse_bool(_require_value(response.values, "direction"), "direction")
        self._log.info("Set direction applied: backward=%s", is_backward)
        return is_backward

    def set_speed_sps(self, speed_sps: int) -> float:
        if speed_sps < MIN_SPEED_SPS or speed_sps > MAX_SPEED_SPS:
            raise TMC2209ConfigError(f"speed_sps out of range: {speed_sps!r}")
        self._log.info("Set speed request: speed_sps=%s", speed_sps)
        response = self._transact("speed", [str(speed_sps)])
        applied_speed = _parse_float(_require_value(response.values, "speed"), "speed")
        self._log.info("Set speed applied: speed_sps=%s", applied_speed)
        return applied_speed

    def set_acceleration_steps_per_ms(self, accel_steps_per_ms: int) -> float:
        if accel_steps_per_ms < MIN_ACCEL_STEPS_PER_MS or accel_steps_per_ms > MAX_ACCEL_STEPS_PER_MS:
            raise TMC2209ConfigError(
                f"accel_steps_per_ms out of range: {accel_steps_per_ms!r}"
            )
        self._log.info("Set acceleration request: accel_steps_per_ms=%s", accel_steps_per_ms)
        response = self._transact("acceleration", [str(accel_steps_per_ms)])
        applied_acceleration = _parse_float(_require_value(response.values, "accel"), "accel")
        self._log.info("Set acceleration applied: accel_steps_per_ms=%s", applied_acceleration)
        return applied_acceleration

    def set_target(self, target: int) -> tuple[int, bool]:
        self._log.info("Set target request: target=%s", target)
        response = self._transact("target", [str(target)])
        target_value = _parse_int(_require_value(response.values, "target"), "target")
        target_set = _parse_bool(_require_value(response.values, "target_set"), "target_set")
        self._log.info("Set target applied: target=%s target_set=%s", target_value, target_set)
        return target_value, target_set

    def run(self) -> bool:
        self._log.info("Run request")
        response = self._transact("run")
        is_running = _parse_bool(_require_value(response.values, "running"), "running")
        self._log.info("Run applied: running=%s", is_running)
        return is_running

    def halt(self) -> bool:
        self._log.info("Halt request")
        response = self._transact("stop")
        is_stopping = _parse_bool(_require_value(response.values, "stopping"), "stopping")
        self._log.info("Halt applied: stopping=%s", is_stopping)
        return is_stopping

    def _transact(self, command: str, args: list[str] | None = None) -> TMC2209Response:
        payload = command
        if args:
            payload = f"{payload} {' '.join(args)}"
        payload = f"{payload}{COMMAND_TERMINATOR}"

        self._log.debug("tmc2209 tx command=%s", payload.strip())
        raw = self._serial.query(payload)
        self._log.debug("tmc2209 rx response=%r", raw)

        response = TMC2209Response.from_line(raw)
        if not response.ok:
            raise TMC2209CommandError(response.error or "tmc2209 error")
        return response

    def _validate_serial(self) -> None:
        terminator = getattr(self._serial, "terminator", None)
        if not terminator:
            raise TMC2209ConfigError("serial terminator is required")
        if b"\n" not in terminator:
            raise TMC2209ConfigError("serial terminator must include \\n")


def _normalize_param_name(name: str) -> str:
    if not name:
        raise TMC2209ConfigError("param name is required")
    return name.lstrip(":")


def _split_key_value(token: str) -> tuple[str, str]:
    if KEY_VALUE_SEPARATOR not in token:
        raise TMC2209ResponseError(f"missing key-value separator: {token!r}")
    key, value = token.split(KEY_VALUE_SEPARATOR, 1)
    if not key:
        raise TMC2209ResponseError(f"empty key in response: {token!r}")
    if value == "":
        raise TMC2209ResponseError(f"empty value for key {key!r}")
    return key, value


def _require_value(values: dict[str, str], key: str) -> str:
    if key not in values:
        raise TMC2209ResponseError(f"missing key in response: {key!r}")
    return values[key]


def _parse_bool(value: str, label: str) -> bool:
    if value not in BOOL_VALUES:
        raise TMC2209ProtocolError(f"{label} must be 0 or 1: {value!r}")
    return value == "1"


def _parse_int(value: str, label: str) -> int:
    try:
        return int(value, 10)
    except ValueError as exc:
        raise TMC2209ProtocolError(f"{label} must be integer: {value!r}") from exc


def _parse_float(value: str, label: str) -> float:
    try:
        return float(value)
    except ValueError as exc:
        raise TMC2209ProtocolError(f"{label} must be float: {value!r}") from exc


def _parse_hex(value: str, label: str) -> int:
    if not value.startswith(HEX_PREFIX):
        raise TMC2209ProtocolError(f"{label} must be hex with {HEX_PREFIX} prefix: {value!r}")
    try:
        return int(value, 16)
    except ValueError as exc:
        raise TMC2209ProtocolError(f"{label} must be hex: {value!r}") from exc


def _format_param_value(value: str | int | float | bool) -> str:
    if value is True:
        return "1"
    if value is False:
        return "0"
    return str(value)


def _validate_microsteps(microsteps: int) -> None:
    if microsteps not in MICROSTEPS_ALLOWED:
        raise TMC2209ConfigError(f"microsteps not allowed: {microsteps!r}")
