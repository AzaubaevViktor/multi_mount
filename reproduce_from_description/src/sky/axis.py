from __future__ import annotations

import queue
from dataclasses import dataclass, field
from enum import Enum
from threading import Event, RLock, Thread
from time import monotonic
from typing import Any

from .constants import AXIS_COMMAND_TIMEOUT, AXIS_GOTO_POLL_INTERVAL, AXIS_IDLE_POLL_INTERVAL, SIDEREAL_RATE_HOURS_PER_SECOND
from .motor import Motor, MotorDirection, MotorMotionMode, MotorPhase, MotorStatus
from .physics import Dec, DecPerSecond, Ha, HaPerSecond


class AxisMotionMode(str, Enum):
    STOPPED = "stopped"
    TRACK = "track"
    SLEW = "slew"
    GUIDE = "guide"
    GOTO = "goto"


@dataclass(slots=True)
class AxisCommand:
    name: str
    payload: dict[str, Any] = field(default_factory=dict)
    done: Event = field(default_factory=Event)
    result: Any = None
    error: BaseException | None = None


@dataclass(slots=True)
class AxisSnapshot:
    position: Any
    mode: AxisMotionMode
    tracking_speed: Any
    active_speed: Any
    motor_status: MotorStatus | None


class Axis:
    def __init__(self, motor: Motor, initial_position: Any, tracking_speed: Any) -> None:
        self.motor = motor
        self._lock = RLock()
        self._commands: queue.Queue[AxisCommand] = queue.Queue()
        self._stop_event = Event()
        self._worker: Thread | None = None
        self._position = initial_position
        self._tracking_speed = tracking_speed
        self._active_speed = tracking_speed
        self._motion_mode = AxisMotionMode.STOPPED
        self._last_status: MotorStatus | None = None
        self._last_sample_at: float | None = None

    def connect(self) -> None:
        with self._lock:
            if self._worker and self._worker.is_alive():
                return

            self.motor.connect()
            self._last_status = self.motor.status()
            self._position = self.motor.convert_steps_to_position(self._last_status.position_steps)
            self._last_sample_at = monotonic()
            self._stop_event.clear()
            self._worker = Thread(target=self._motion_convertor, name=f"{self.__class__.__name__}-motion", daemon=True)
            self._worker.start()
            if self._speed_value(self._tracking_speed) != 0.0:
                self.change_speed(self._tracking_speed)

    def disconnect(self) -> None:
        self._stop_event.set()
        worker = self._worker
        if worker is not None:
            worker.join(timeout=1.0)
        with self._lock:
            self.motor.disconnect()
            self._worker = None
            self._motion_mode = AxisMotionMode.STOPPED

    def sync_to(self, position: Any) -> None:
        self._submit("sync", position=position)

    def goto_to(self, position: Any) -> None:
        self._submit("goto", position=position)

    def change_speed(self, speed: Any) -> None:
        self._submit("set_sky_speed", speed=speed)

    def move(self, speed: Any, mode: AxisMotionMode = AxisMotionMode.SLEW) -> None:
        self._submit("move", speed=speed, mode=mode)

    def halt(self) -> None:
        self._submit("halt")

    def stop(self) -> None:
        self._submit("stop")

    def status(self) -> AxisSnapshot:
        with self._lock:
            return AxisSnapshot(
                position=self._position,
                mode=self._motion_mode,
                tracking_speed=self._tracking_speed,
                active_speed=self._active_speed,
                motor_status=self._last_status,
            )

    def wait_until_idle(self, timeout: float = 5.0) -> AxisSnapshot:
        deadline = monotonic() + timeout
        while monotonic() < deadline:
            snapshot = self.status()
            if snapshot.mode != AxisMotionMode.GOTO:
                return snapshot
        raise TimeoutError(f"{self.__class__.__name__} did not leave GOTO mode")

    def monitor_groups(self) -> list[dict[str, Any]]:
        snapshot = self.status()
        return [
            {
                "name": self.__class__.__name__,
                "fields": [
                    {"name": "mode", "value": snapshot.mode.value},
                    {"name": "position", "value": str(snapshot.position)},
                    {"name": "tracking_speed", "value": str(snapshot.tracking_speed)},
                ],
            }
        ]

    def _submit(self, name: str, **payload: Any) -> Any:
        command = AxisCommand(name=name, payload=payload)
        self._commands.put(command)
        if not command.done.wait(timeout=AXIS_COMMAND_TIMEOUT):
            raise TimeoutError(f"Axis command timed out: {name}")
        if command.error is not None:
            raise RuntimeError(f"Axis command failed: {name}") from command.error
        return command.result

    def _motion_convertor(self) -> None:
        while not self._stop_event.is_set():
            try:
                command = self._commands.get(timeout=AXIS_IDLE_POLL_INTERVAL)
            except queue.Empty:
                self._poll_motor()
                continue

            try:
                with self._lock:
                    if command.name == "sync":
                        position = command.payload["position"]
                        self._position = position
                        self.motor.set_steps(self.motor.convert_position_to_steps(position))
                        self._last_status = self.motor.status()
                        self._last_sample_at = monotonic()
                    elif command.name == "goto":
                        target = command.payload["position"]
                        delta_steps = self._delta_to_steps(target)
                        self.motor.set_motion_mode(MotorMotionMode.TARGET)
                        self.motor.set_direction(MotorDirection.POSITIVE if delta_steps >= 0 else MotorDirection.NEGATIVE)
                        self.motor.set_speed(self.motor.get_speed_sps_by_delta(delta_steps))
                        self.motor.set_delta(abs(delta_steps))
                        self.motor.run()
                        self._motion_mode = AxisMotionMode.GOTO
                        self._active_speed = self._tracking_speed
                    elif command.name == "set_sky_speed":
                        self._tracking_speed = command.payload["speed"]
                        if self._motion_mode != AxisMotionMode.GOTO:
                            self._apply_speed(self._tracking_speed, AxisMotionMode.TRACK if self._speed_value(self._tracking_speed) != 0.0 else AxisMotionMode.STOPPED)
                    elif command.name == "move":
                        speed = command.payload["speed"]
                        mode = command.payload["mode"]
                        self._apply_speed(speed, mode)
                    elif command.name == "halt":
                        fallback_mode = AxisMotionMode.TRACK if self._speed_value(self._tracking_speed) != 0.0 else AxisMotionMode.STOPPED
                        self._apply_speed(self._tracking_speed, fallback_mode)
                    elif command.name == "stop":
                        self.motor.stop()
                        self._motion_mode = AxisMotionMode.STOPPED
                        self._active_speed = self._zero_speed()
                    else:
                        raise ValueError(f"Unknown axis command: {command.name}")
            except BaseException as exc:
                command.error = exc
            finally:
                command.done.set()

    def _poll_motor(self) -> None:
        with self._lock:
            status = self.motor.status()
            if self._last_status is not None:
                delta_steps = status.position_steps - self._last_status.position_steps
                if delta_steps:
                    delta_position = self._delta_from_steps(delta_steps)
                    self._position = self._apply_position_delta(self._position, delta_position)
            self._last_status = status
            self._last_sample_at = monotonic()
            if self._motion_mode == AxisMotionMode.GOTO and not status.running:
                fallback_mode = AxisMotionMode.TRACK if self._speed_value(self._tracking_speed) != 0.0 else AxisMotionMode.STOPPED
                self._apply_speed(self._tracking_speed, fallback_mode)

    def _apply_speed(self, speed: Any, mode: AxisMotionMode) -> None:
        signed = self._speed_value(speed)
        if signed == 0.0:
            self.motor.stop()
            self.motor.set_motion_mode(MotorMotionMode.STOPPED)
            self._motion_mode = AxisMotionMode.STOPPED if mode == AxisMotionMode.STOPPED else mode
            self._active_speed = self._zero_speed()
            return

        self.motor.set_motion_mode(MotorMotionMode.FREE_RIDE)
        self.motor.set_direction(MotorDirection.POSITIVE if signed >= 0 else MotorDirection.NEGATIVE)
        self.motor.set_speed(self.motor.convert_speed_to_steps_per_second(self._abs_speed(speed)))
        self.motor.run()
        self._motion_mode = mode
        self._active_speed = speed

    def _position_delta(self, target: Any) -> Any:
        raise NotImplementedError

    def _delta_to_steps(self, target: Any) -> int:
        raise NotImplementedError

    def _delta_from_steps(self, steps: int) -> Any:
        raise NotImplementedError

    def _apply_position_delta(self, position: Any, delta: Any) -> Any:
        raise NotImplementedError

    def _speed_value(self, speed: Any) -> float:
        raise NotImplementedError

    def _abs_speed(self, speed: Any) -> Any:
        raise NotImplementedError

    def _zero_speed(self) -> Any:
        raise NotImplementedError


class AxisRA(Axis):
    def __init__(self, motor: Motor, initial_position: Ha | None = None) -> None:
        super().__init__(motor=motor, initial_position=initial_position or Ha(0.0), tracking_speed=HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND))

    def move(self, speed: HaPerSecond, mode: AxisMotionMode = AxisMotionMode.SLEW) -> None:
        combined = HaPerSecond(self._tracking_speed.hours_per_second + speed.hours_per_second)
        super().move(combined, mode=mode)

    def _position_delta(self, target: Ha) -> Ha:
        return Ha(self._position.shortest_delta_to(target))

    def _delta_to_steps(self, target: Ha) -> int:
        return self.motor.convert_position_to_steps(self._position_delta(target))

    def _delta_from_steps(self, steps: int) -> Ha:
        return self.motor.convert_steps_to_position(steps)

    def _apply_position_delta(self, position: Ha, delta: Ha) -> Ha:
        return position.shift(delta.hours)

    def _speed_value(self, speed: HaPerSecond) -> float:
        return speed.hours_per_second

    def _abs_speed(self, speed: HaPerSecond) -> HaPerSecond:
        return speed.absolute()

    def _zero_speed(self) -> HaPerSecond:
        return HaPerSecond(0.0)


class AxisDEC(Axis):
    def __init__(self, motor: Motor, initial_position: Dec | None = None) -> None:
        super().__init__(motor=motor, initial_position=initial_position or Dec(0.0), tracking_speed=DecPerSecond(0.0))

    def _position_delta(self, target: Dec) -> Dec:
        return Dec(self._position.delta_to(target))

    def _delta_to_steps(self, target: Dec) -> int:
        return self.motor.convert_position_to_steps(target) - self.motor.convert_position_to_steps(self._position)

    def _delta_from_steps(self, steps: int) -> Dec:
        return Dec(steps / self.motor.convert_position_to_steps(Dec(1.0)))

    def _apply_position_delta(self, position: Dec, delta: Dec) -> Dec:
        return position.shift(delta.degrees)

    def _speed_value(self, speed: DecPerSecond) -> float:
        return speed.degrees_per_second

    def _abs_speed(self, speed: DecPerSecond) -> DecPerSecond:
        return speed.absolute()

    def _zero_speed(self) -> DecPerSecond:
        return DecPerSecond(0.0)
