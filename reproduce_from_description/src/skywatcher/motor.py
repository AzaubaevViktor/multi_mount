from __future__ import annotations

from dataclasses import dataclass
from threading import RLock
from time import monotonic, sleep

from serial_wrapper.wrapper import SerialLine, SerialTransportError
from sky.constants import SIDEREAL_RATE_HOURS_PER_SECOND
from sky.motor import Motor, MotorDirection, MotorMotionMode, MotorPhase, MotorStatus
from sky.physics import Ha, HaPerSecond

from .protocol import (
    FRAME_TERMINATOR,
    SkyWatcherCommand,
    build_command,
    decode_s24,
    decode_u24,
    encode_s24,
    encode_u24,
    parse_response,
    parse_status_bits,
)


@dataclass(slots=True)
class SkyWatcherConfig:
    port: str | None = None
    search_pattern: str | None = None
    baudrate: int = 9600
    timeout: float = 1.0
    steps_per_revolution: int | None = None
    timer_frequency_hz: int | None = None
    high_speed_ratio: int | None = None
    high_speed_threshold_sps: float = 256.0
    max_goto_speed_sps: float = 8192.0


class SkyWatcherMotor(Motor):
    def __init__(self, config: SkyWatcherConfig | None = None, transport: SerialLine | None = None) -> None:
        self.config = config or SkyWatcherConfig()
        self.transport = transport or SerialLine(
            port=self.config.port,
            search_pattern=self.config.search_pattern,
            baudrate=self.config.baudrate,
            timeout=self.config.timeout,
            terminator=FRAME_TERMINATOR,
        )
        self._lock = RLock()
        self._steps_per_revolution = self.config.steps_per_revolution or 0
        self._timer_frequency_hz = self.config.timer_frequency_hz or 0
        self._high_speed_ratio = self.config.high_speed_ratio or 1
        self._microsteps = 1
        self._direction = MotorDirection.POSITIVE
        self._motion_mode = MotorMotionMode.STOPPED
        self._speed_sps = 0.0
        self._acceleration_sps2 = 0.0
        self._target_steps: int | None = None
        self._position_steps = 0
        self._connected = False

    def connect(self) -> None:
        with self._lock:
            self.transport.connect()
            self._query(SkyWatcherCommand.INITIALIZE)
            if self._steps_per_revolution == 0:
                self._steps_per_revolution = decode_u24(self._query(SkyWatcherCommand.INQUIRE_GRID_PER_REVOLUTION))
            if self._timer_frequency_hz == 0:
                self._timer_frequency_hz = decode_u24(self._query(SkyWatcherCommand.INQUIRE_TIMER_FREQUENCY))
            if self._high_speed_ratio <= 1:
                ratio = decode_u24(self._query(SkyWatcherCommand.INQUIRE_HIGHSPEED_RATIO))
                self._high_speed_ratio = ratio or 1
            self._position_steps = decode_s24(self._query(SkyWatcherCommand.INQUIRE_POSITION))
            self._connected = True

    def disconnect(self) -> None:
        with self._lock:
            self.transport.close()
            self._connected = False

    def status(self) -> MotorStatus:
        with self._lock:
            bits = parse_status_bits(self._query(SkyWatcherCommand.INQUIRE_STATUS))
            self._position_steps = decode_s24(self._query(SkyWatcherCommand.INQUIRE_POSITION))
            if bits.running:
                phase = MotorPhase.CRUISE
            elif bits.target_mode and self._target_steps is not None:
                phase = MotorPhase.TARGET_REACHED
            else:
                phase = MotorPhase.IDLE

            return MotorStatus(
                connected=self._connected,
                position_steps=self._position_steps,
                mode=MotorMotionMode.TARGET if bits.target_mode else self._motion_mode,
                phase=phase,
                direction=MotorDirection.POSITIVE if bits.direction_positive else MotorDirection.NEGATIVE,
                speed_sps=self._speed_sps,
                actual_speed_sps=self._speed_sps if bits.running else 0.0,
                target_steps=self._target_steps,
                target_set=self._target_steps is not None,
                raw={"status_bits": bits.raw},
            )

    def set_steps(self, steps: int) -> None:
        with self._lock:
            self._query(SkyWatcherCommand.SET_AXIS_POSITION, encode_s24(steps))
            self._position_steps = steps

    def set_speed(self, steps_per_second: float) -> None:
        if steps_per_second < 0:
            raise ValueError("SkyWatcher controller expects absolute speed with separate direction")
        with self._lock:
            self._speed_sps = round(steps_per_second, 3)
            if self._speed_sps == 0.0:
                return

            use_high_speed = self._speed_sps >= self.config.high_speed_threshold_sps
            effective_speed = self._speed_sps / self._high_speed_ratio if use_high_speed else self._speed_sps
            period = max(1, round(self._timer_frequency_hz / effective_speed))
            self._query(SkyWatcherCommand.SET_STEP_PERIOD, encode_u24(period))
            self._query(SkyWatcherCommand.SET_MOTION_MODE, self._mode_bits(high_speed=use_high_speed))

    def set_acceleration(self, steps_per_second_square: float) -> None:
        with self._lock:
            self._acceleration_sps2 = max(0.0, float(steps_per_second_square))

    def set_direction(self, direction: MotorDirection) -> None:
        with self._lock:
            self._direction = direction
            self._query(SkyWatcherCommand.SET_MOTION_MODE, self._mode_bits())

    def set_delta(self, delta_steps: int) -> None:
        if delta_steps < 0:
            raise ValueError("SkyWatcher delta must be positive; use direction separately")
        with self._lock:
            self._target_steps = delta_steps if self._direction == MotorDirection.POSITIVE else -delta_steps
            payload = encode_u24(delta_steps)
            self._query(SkyWatcherCommand.SET_GOTO_TARGET_INCREMENT, payload)
            self._query(SkyWatcherCommand.SET_BREAK_POINT_INCREMENT, payload)

    def get_speed_sps_by_delta(self, delta_steps: int) -> float:
        magnitude = abs(delta_steps)
        if magnitude == 0:
            return 0.0
        return min(max(32.0, magnitude / 8.0), self.config.max_goto_speed_sps)

    def get_speed_by_speed_sps(self, steps_per_second: float) -> HaPerSecond:
        return HaPerSecond(steps_per_second * 24.0 / self._steps_per_revolution)

    def set_motion_mode(self, mode: MotorMotionMode) -> None:
        with self._lock:
            self._motion_mode = mode
            self._query(SkyWatcherCommand.SET_MOTION_MODE, self._mode_bits())

    def set_microsteps(self, microsteps: int) -> None:
        with self._lock:
            self._microsteps = max(1, int(microsteps))

    def convert_position_to_steps(self, position: Ha) -> int:
        return round(position.hours / 24.0 * self._steps_per_revolution)

    def convert_steps_to_position(self, steps: int) -> Ha:
        return Ha(steps * 24.0 / self._steps_per_revolution)

    def convert_speed_to_steps_per_second(self, speed: HaPerSecond) -> float:
        if speed.hours_per_second < 0:
            raise ValueError("SkyWatcher speed conversion expects non-negative speed")
        return round(speed.hours_per_second * self._steps_per_revolution / 24.0, 3)

    def run(self) -> None:
        with self._lock:
            self._query(SkyWatcherCommand.START_MOTION)

    def stop(self) -> None:
        with self._lock:
            self._query(SkyWatcherCommand.STOP_MOTION)
            self._motion_mode = MotorMotionMode.STOPPED
            self._target_steps = None

    def wait_till_stop(self, timeout: float | None = None) -> MotorStatus:
        deadline = monotonic() + (timeout or 5.0)
        while monotonic() < deadline:
            status = self.status()
            if not status.running:
                return status
            sleep(0.05)
        raise TimeoutError("SkyWatcher motor did not stop in time")

    def reset(self) -> None:
        with self._lock:
            self.transport.reconnect()
            self._connected = False
        self.connect()

    def _query(self, command: SkyWatcherCommand, payload: str = "") -> str:
        raw = self.transport.query(build_command(command, payload=payload), terminator=FRAME_TERMINATOR)
        response = parse_response(raw)
        if not response.ok:
            raise SerialTransportError(f"SkyWatcher command failed: {command.value} -> {response.payload}")
        return response.payload

    def _mode_bits(self, high_speed: bool | None = None) -> str:
        mode = 0
        if self._motion_mode == MotorMotionMode.TARGET:
            mode |= 0x08
        if (high_speed if high_speed is not None else self._speed_sps >= self.config.high_speed_threshold_sps):
            mode |= 0x02
        if self._direction == MotorDirection.POSITIVE:
            mode |= 0x04
        return f"{mode:02X}"
