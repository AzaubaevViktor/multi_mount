from __future__ import annotations

from dataclasses import dataclass
from threading import RLock
from time import monotonic, sleep

from serial_wrapper.wrapper import SerialLine, SerialTransportError
from sky.motor import Motor, MotorDirection, MotorMotionMode, MotorPhase, MotorStatus
from sky.physics import Dec, DecPerSecond


@dataclass(frozen=True, slots=True)
class TMC2209Response:
    ok: bool
    fields: dict[str, str]
    raw: str


@dataclass(slots=True)
class TMC2209Config:
    port: str | None = None
    search_pattern: str | None = None
    baudrate: int = 115200
    timeout: float = 1.0
    steps_per_degree: float = 200.0
    microsteps: int = 16
    default_acceleration_sps2: float = 1000.0
    max_goto_speed_sps: float = 4000.0


def parse_response_line(line: str | bytes) -> TMC2209Response:
    text = line.decode("ascii") if isinstance(line, bytes) else line
    text = text.strip()
    if not text:
        raise ValueError("Empty TMC2209 response")

    parts = [part for part in text.split(";") if part]
    ok_flag = parts[0]
    if ok_flag not in {"0", "1"}:
        raise ValueError(f"Invalid TMC2209 status prefix: {text!r}")

    fields: dict[str, str] = {}
    for part in parts[1:]:
        if "=" in part:
            key, value = part.split("=", 1)
            fields[key] = value
        else:
            fields[part] = ""
    return TMC2209Response(ok=ok_flag == "1", fields=fields, raw=text)


class TMC2209Motor(Motor):
    def __init__(self, config: TMC2209Config | None = None, transport: SerialLine | None = None) -> None:
        self.config = config or TMC2209Config()
        self.transport = transport or SerialLine(
            port=self.config.port,
            search_pattern=self.config.search_pattern,
            baudrate=self.config.baudrate,
            timeout=self.config.timeout,
            terminator=b"\n",
        )
        self._lock = RLock()
        self._connected = False
        self._direction = MotorDirection.POSITIVE
        self._motion_mode = MotorMotionMode.STOPPED
        self._speed_sps = 0.0
        self._acceleration_sps2 = self.config.default_acceleration_sps2
        self._target_steps: int | None = None
        self._position_steps = 0
        self._microsteps = self.config.microsteps

    def connect(self) -> None:
        with self._lock:
            self.transport.connect()
            self._connected = True
            self.set_microsteps(self.config.microsteps)
            self.set_acceleration(self.config.default_acceleration_sps2)
            self.status()

    def disconnect(self) -> None:
        with self._lock:
            self.transport.close()
            self._connected = False

    def status(self) -> MotorStatus:
        with self._lock:
            response = self._command("status")
            self._position_steps = int(response.fields.get("position", self._position_steps))
            phase = self._phase_from_text(response.fields.get("phase", "idle"))
            mode = self._mode_from_text(response.fields.get("mode", self._motion_mode.value))
            direction = self._direction_from_value(response.fields.get("direction"))
            speed_sps = float(response.fields.get("speed", self._speed_sps))
            actual_speed = float(response.fields.get("actual_speed", speed_sps if phase != MotorPhase.IDLE else 0.0))
            target_value = response.fields.get("target")
            target_steps = int(target_value) if target_value is not None else self._target_steps
            return MotorStatus(
                connected=self._connected,
                position_steps=self._position_steps,
                mode=mode,
                phase=phase,
                direction=direction,
                speed_sps=speed_sps,
                actual_speed_sps=actual_speed,
                target_steps=target_steps,
                target_set=response.fields.get("target_set", "0") == "1",
                enabled=response.fields.get("enabled", "1") == "1",
                raw=response.fields,
            )

    def set_steps(self, steps: int) -> None:
        with self._lock:
            self._command(f"position {int(steps)}")
            self._position_steps = int(steps)

    def set_speed(self, steps_per_second: float) -> None:
        if steps_per_second < 0:
            raise ValueError("TMC2209 speed must be absolute; use direction separately")
        rounded = int(round(steps_per_second))
        with self._lock:
            self._command(f"speed {rounded}")
            self._speed_sps = float(rounded)

    def set_acceleration(self, steps_per_second_square: float) -> None:
        rounded = int(round(max(0.0, steps_per_second_square)))
        with self._lock:
            self._command(f"acceleration {rounded}")
            self._acceleration_sps2 = float(rounded)

    def set_direction(self, direction: MotorDirection) -> None:
        with self._lock:
            self._command(f"direction {1 if direction == MotorDirection.POSITIVE else 0}")
            self._direction = direction

    def set_delta(self, delta_steps: int) -> None:
        with self._lock:
            self._command(f"delta {int(delta_steps)}")
            self._target_steps = int(delta_steps)

    def get_speed_sps_by_delta(self, delta_steps: int) -> float:
        magnitude = abs(delta_steps)
        if magnitude == 0:
            return 0.0
        return min(max(50.0, magnitude / 4.0), self.config.max_goto_speed_sps)

    def get_speed_by_speed_sps(self, steps_per_second: float) -> DecPerSecond:
        return DecPerSecond(steps_per_second / self.config.steps_per_degree)

    def set_motion_mode(self, mode: MotorMotionMode) -> None:
        mapped = "target" if mode == MotorMotionMode.TARGET else "free_ride"
        with self._lock:
            self._command(f"mode {mapped}")
            self._motion_mode = mode

    def set_microsteps(self, microsteps: int) -> None:
        with self._lock:
            self._command(f"set microsteps={int(microsteps)}")
            self._microsteps = int(microsteps)

    def convert_position_to_steps(self, position: Dec) -> int:
        return int(round(position.degrees * self.config.steps_per_degree))

    def convert_steps_to_position(self, steps: int) -> Dec:
        return Dec(steps / self.config.steps_per_degree)

    def convert_speed_to_steps_per_second(self, speed: DecPerSecond) -> float:
        if speed.degrees_per_second < 0:
            raise ValueError("TMC2209 speed conversion expects non-negative speed")
        return speed.degrees_per_second * self.config.steps_per_degree

    def run(self) -> None:
        with self._lock:
            self._command("run")

    def stop(self) -> None:
        with self._lock:
            self._command("stop")
            self._motion_mode = MotorMotionMode.STOPPED
            self._target_steps = None

    def wait_till_stop(self, timeout: float | None = None) -> MotorStatus:
        deadline = monotonic() + (timeout or 5.0)
        while monotonic() < deadline:
            status = self.status()
            if not status.running:
                return status
            sleep(0.05)
        raise TimeoutError("TMC2209 motor did not stop in time")

    def reset(self) -> None:
        with self._lock:
            self.transport.reconnect()
            self._connected = False
        self.connect()

    def _command(self, payload: str) -> TMC2209Response:
        raw = self.transport.query_text(f"{payload}\n", terminator=b"\n")
        response = parse_response_line(raw)
        if not response.ok:
            raise SerialTransportError(f"TMC2209 command failed: {payload} -> {response.raw}")
        return response

    @staticmethod
    def _phase_from_text(text: str) -> MotorPhase:
        mapping = {
            "idle": MotorPhase.IDLE,
            "accel": MotorPhase.ACCEL,
            "cruise": MotorPhase.CRUISE,
            "decel": MotorPhase.DECEL,
            "target_reached": MotorPhase.TARGET_REACHED,
        }
        return mapping.get(text, MotorPhase.ERROR)

    @staticmethod
    def _mode_from_text(text: str) -> MotorMotionMode:
        if text == "target":
            return MotorMotionMode.TARGET
        if text == "free_ride":
            return MotorMotionMode.FREE_RIDE
        return MotorMotionMode.STOPPED

    def _direction_from_value(self, value: str | None) -> MotorDirection:
        if value is None:
            return self._direction
        return MotorDirection.POSITIVE if value == "1" else MotorDirection.NEGATIVE
