
import dataclasses
from enum import IntEnum, StrEnum
import logging
import threading
import time
from typing import Callable, Self
from serial_wrapper.wrapper import SerialLine
from sky.constants import STELLAR_DAY, STELLAR_SPEED
from sky.physics import HaPerSecond, Ha
from web_control.web import MonitorMixin, MonitorRenderer, monitor_action, monitor_field, monitor_group


class SkyWatcherWrongResponce(Exception):
    pass


class SkyWatcherCommandError(Exception):
    pass


class SkyWatcherRevu24Error(Exception):
    pass


class SkyWatcherTimeoutError(TimeoutError):
    pass


class SkyWatcherCommand(StrEnum):
    INQUIRE_TIMER_FREQ = "b"
    INQUIRE_CPR = "a"
    INQUIRE_POSITION = "j"
    INQUIRE_STATUS = "f"
    INQUIRE_HIGHSPEED_RATIO = "g"
    SET_STEP_PERIOD = "I"
    SET_GOTO_TARGET = "S"
    SET_GOTO_TARGET_INCREMENT = "H"
    SET_BREAK_POINT_INCREMENT = "M"
    SET_BREAK_POINT = "M"
    SET_AXIS_POSITION = "E"
    SET_MOTION_MODE = "G"
    START_MOTION = "J"
    STOP_MOTION = "K"
    INSTANT_STOP = "L"
    INITIALIZE = "F"



class Direction(IntEnum):
    BACKWARD = 0
    FORWARD = 1


class SlewMode(IntEnum):
    SLEW = 0
    GOTO = 1


class SpeedMode(IntEnum):
    LOWSPEED = 0
    HIGHSPEED = 1


@dataclasses.dataclass
class SkyWatcherStatus:
    raw: int
    running: bool
    initialized: bool
    slew_mode: SlewMode
    direction: Direction
    speed_mode: SpeedMode

    @classmethod
    def from_bytes(cls, data: bytes) -> "SkyWatcherStatus":
        # LOGGER.info("status_data data=%r", data)
        b1 = data[0] if len(data) > 0 else 0
        b2 = data[1] if len(data) > 1 else 0
        b3 = data[2] if len(data) > 2 else 0
        raw = b2 | (b1 << 8) | (b3 << 16)
        running = bool(b2 & 0x01)
        initialized = bool(b3 & 0x01)
        slew_mode = SlewMode.SLEW if (b1 & 0x01) else SlewMode.GOTO
        direction = Direction.BACKWARD if (b1 & 0x02) else Direction.FORWARD
        speed_mode = SpeedMode.HIGHSPEED if (b1 & 0x04) else SpeedMode.LOWSPEED
        return cls(
            raw=raw,
            running=running,
            initialized=initialized,
            slew_mode=slew_mode,
            direction=direction,
            speed_mode=speed_mode,
        )

    def to_command(self) -> str:
        return SkyWatcherMotionStatus.from_status(self).to_command()


@dataclasses.dataclass(frozen=True)
class SkyWatcherMotionStatus:
    slew_mode: SlewMode
    direction: Direction
    speed_mode: SpeedMode

    @classmethod
    def from_status(cls, status: SkyWatcherStatus) -> "SkyWatcherMotionStatus":
        return cls(
            slew_mode=status.slew_mode,
            direction=status.direction,
            speed_mode=status.speed_mode,
        )

    def to_command(self) -> str:
        slew_mode = self.slew_mode
        speed_mode = self.speed_mode
        if slew_mode == SlewMode.SLEW:
            motion_mode = "1" if speed_mode == SpeedMode.LOWSPEED else "3"
        elif slew_mode == SlewMode.GOTO:
            motion_mode = "2" if speed_mode == SpeedMode.LOWSPEED else "0"
        else:
            motion_mode = "1"

        direction_mode = "0" if self.direction == Direction.FORWARD else "1"
        return f"{motion_mode}{direction_mode}"


class Axis(StrEnum):
    RA = "1"
    DEC = "2"


@dataclasses.dataclass
class SkyWatcherAxisState:
    cpr: int


class Revu24:
    @staticmethod
    def from_mount(data: str) -> int:
        if len(data) == 2:
            data = f"{data}0000"
        if len(data) < 6:
            raise SkyWatcherRevu24Error(f"Expected at least 6 hex chars, got {len(data)}")

        reordered = data[4:6] + data[2:4] + data[0:2]
        try:
            return int(reordered, 16)
        except ValueError as exc:
            raise SkyWatcherRevu24Error(f"Invalid hex data: {data!r}") from exc

    @staticmethod
    def from_int(value: int) -> str:
        try:
            if value < 0 or value > 0xFFFFFF:
                raise SkyWatcherRevu24Error(
                    f"Expected value in range 0..{0xFFFFFF}, got {value}"
                )
        except TypeError as exc:
            raise SkyWatcherRevu24Error(f"Invalid value: {value!r}") from exc

        return value.to_bytes(3, "little").hex().upper()


class SkyWatcherWebMixin(MonitorMixin):
    monitor_name = "SkyWatcher Mount"
    monitor_groups = (
        monitor_group("main", "Mount"),
        monitor_group("status", "Status"),
        monitor_group("config", "Config"),
    )
    monitor_fields = (
        monitor_field("connected", "Connected", "is_connected"),
        monitor_field("tracking_rate", "Tracking rate", lambda self: round(self._last_tracking_speed / STELLAR_SPEED, 6), mode="rw", setter="set_monitor_tracking_rate"),
        monitor_field("ha", "Hour angle", "monitor_hour_angle"),
        monitor_field("status", "Motor status", "monitor_status_payload", renderer=MonitorRenderer.JSON, group="status"),
        monitor_field("steps_360", "Steps 360", lambda self: getattr(self, "ra_steps_360", None), group="config"),
        monitor_field("steps_worm", "Steps worm", lambda self: getattr(self, "ra_steps_worm", None), group="config"),
        monitor_field("highspeed_ratio", "Highspeed ratio", lambda self: getattr(self, "ra_highspeed_ratio", None), group="config"),
    )
    monitor_actions = (
        monitor_action("stop", "Stop motor", "monitor_stop_motor"),
        monitor_action("refresh", "Refresh structure", "monitor_refresh_layout"),
    )

    def __init__(self) -> None:
        super().__init__()

    def monitor_status_payload(self: SkyWatcherMount) -> dict[str, int | bool | str]:
        if not self.is_connected:
            return {
                "raw": 0,
                "running": False,
                "initialized": False,
                "slew_mode": "DISCONNECTED",
                "direction": "DISCONNECTED",
                "speed_mode": "DISCONNECTED",
            }
        status = self.get_status()
        return {
            "raw": status.raw,
            "running": status.running,
            "initialized": status.initialized,
            "slew_mode": status.slew_mode.name,
            "direction": status.direction.name,
            "speed_mode": status.speed_mode.name,
        }

    def monitor_hour_angle(self: SkyWatcherMount) -> str | None:
        if not self.is_connected:
            return None
        return str(self.get_telescope_ha())

    def set_monitor_tracking_rate(self: SkyWatcherMount, value: str) -> float:
        if not self.is_connected:
            raise RuntimeError("SkyWatcher mount is not connected")
        rate = float(value)
        self.start_tracking(HaPerSecond(STELLAR_SPEED * rate))
        return rate

    def monitor_stop_motor(self: SkyWatcherMount) -> bool:
        self.gracefully_stop_motor()
        return True

    def monitor_refresh_layout(self) -> str:
        self.monitor_force_refresh()
        return "ok"


class SkyWatcherMount(SkyWatcherWebMixin):
    _LEADING = ":"
    _TRAILING = "\r"
    _COMMAND_ERROR_PREFIX = "!"
    _RESPONCE_PREFIX = "="

    _LOWSPEED_SPEED = STELLAR_SPEED * 128
    _LOWSPEED_MARGIN_S = Ha(10 * 60)
    _HIGHSPEED_SPEED = STELLAR_SPEED * 800

    _POSITION_OFFSET = 0x800000

    MIN_RATE = 0.05
    MAX_RATE = 800
    MIN_SPEED = STELLAR_SPEED * MIN_RATE
    MAX_SPEED = STELLAR_SPEED * MAX_RATE

    def __init__(self, serial: SerialLine) -> None:
        super().__init__()
        self.logger = logging.getLogger("skywatcher")
        self._serial = serial
        
        if self._serial.terminator != bytes(self._TRAILING, self._serial.encoding):
            raise RuntimeError(f"Should be terminator: {self._TRAILING} instead {self._serial.terminator}")

        self.timeout_s = 5
        self.poll_interval_s = .5

        self.ra_steps_360: int
        self.ra_steps_worm: int
        self.ra_highspeed_ratio: int
        self._last_tracking_speed = STELLAR_SPEED

        self._last_status_snapshot: tuple[int, bool, bool, SlewMode, Direction, SpeedMode] | None = None

        self.is_connected = False

        self._mount_seconds_cache = Ha(0)
        self._mount_seconds_cache_update = 0
        self._mount_seconds_cache_lock = threading.Lock()

    def _transact(self, cmd: SkyWatcherCommand, arg: str | None = None, axis: Axis = Axis.RA) -> str:
        """ All transactions works only with RA """
        self.logger.debug("Send %s(%s) ...", cmd.name, arg if arg is not None else "")

        payload = [
            self._LEADING,
            cmd.value,
            str(axis)
        ]

        if arg is not None:
            payload.append(arg)
        
        payload.append(self._TRAILING)

        payload_raw = ''.join(payload)

        count = 3
        while True:
            try:
                self.logger.debug("TX %r", payload_raw)
                response = self._serial.query(payload_raw)
                self.logger.debug("RX %r", response)

                if not response.endswith(self._TRAILING):
                    raise SkyWatcherWrongResponce(response)
                

                if response[0] == self._COMMAND_ERROR_PREFIX:
                    # TODO: Add error names from doc
                    raise SkyWatcherCommandError(response)
                
                if response[0] != self._RESPONCE_PREFIX:
                    raise SkyWatcherWrongResponce(response)
                
                responce_clean = response.removesuffix(self._TRAILING)
                break
            except SkyWatcherWrongResponce:
                if not count:
                    raise 

                self.logger.exception("While quering, %d last", count)

                count -= 1

                time.sleep(.1)
        
        responce_clean = responce_clean.removeprefix(self._RESPONCE_PREFIX)
        
        self.logger.debug(
            "Receive: %s(%s) -> %s",
            cmd.name,
            arg if arg is not None else "",
            responce_clean,
        )
        
        return responce_clean
    
    def _do_initialise(self):
        self.logger.info("Wait until mount is initialized...")

        if not self.get_status().initialized:
            self._transact(SkyWatcherCommand.INITIALIZE)
            start = time.monotonic()
            while not self.get_status().initialized:
                if (wait_time := time.monotonic() - start) > self.timeout_s:
                    raise TimeoutError(f"SkyWatcher mount can't initialize for {wait_time:.2f}s")
                time.sleep(self.poll_interval_s)
        else:
            self.logger.info("Mount initialized")

        self.logger.info("Get rate values")
        self.ra_steps_360 = Revu24.from_mount(self._transact(SkyWatcherCommand.INQUIRE_CPR))
        self.ra_steps_worm = Revu24.from_mount(self._transact(SkyWatcherCommand.INQUIRE_TIMER_FREQ))
        self.ra_highspeed_ratio = Revu24.from_mount(self._transact(SkyWatcherCommand.INQUIRE_HIGHSPEED_RATIO))

    def get_status(self) -> SkyWatcherStatus:
        status = SkyWatcherStatus.from_bytes(
            self._transact(SkyWatcherCommand.INQUIRE_STATUS).encode('ascii')
        )

        current_snapshot = (
            status.raw,
            status.running,
            status.initialized,
            status.slew_mode,
            status.direction,
            status.speed_mode,
        )
        if self._last_status_snapshot is None:
            self.logger.info("Mount status initial: %s", status)
        elif current_snapshot != self._last_status_snapshot:
            previous = self._last_status_snapshot
            self.logger.info(
                "Mount status changed: raw %06X -> %06X running %s -> %s initialized %s -> %s "
                "slew_mode %s -> %s direction %s -> %s speed_mode %s -> %s",
                previous[0],
                status.raw,
                previous[1],
                status.running,
                previous[2],
                status.initialized,
                previous[3],
                status.slew_mode,
                previous[4],
                status.direction,
                previous[5],
                status.speed_mode,
            )
        else:
            self.logger.debug("Mount status poll: %s", status)

        self._last_status_snapshot = current_snapshot
        return status

    def connect(self):
        self._serial.connect()
        self._do_initialise()
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False
        self._serial.close()

    def _ticks_to_seconds(self, ticks: int) -> float:
        return ticks / self.ra_steps_360 * 24 * 60 * 60
    
    def _ha_to_ticks(self, ha: Ha) -> int:
        return int(round(float(ha) / (24 * 60 * 60) * self.ra_steps_360))

    _MOUNT_SECONDS_CACHE_TTL_S = .25
    def get_telescope_ha(self) -> Ha:
        if time.monotonic() - self._mount_seconds_cache_update <= self._MOUNT_SECONDS_CACHE_TTL_S:
            return self._mount_seconds_cache

        data = self._transact(SkyWatcherCommand.INQUIRE_POSITION)
        ticks = (Revu24.from_mount(data) - self._POSITION_OFFSET) % self.ra_steps_360
        # Ticks / Full circle / (24h) -> hours
        seconds = Ha(self._ticks_to_seconds(ticks))
        with self._mount_seconds_cache_lock:
            self._mount_seconds_cache = seconds
            self._mount_seconds_cache_update = time.monotonic()

        return seconds.wrap()
    
    def set_telescope_ha(self, position: Ha) -> bool:
        ticks = (self._ha_to_ticks(position) + self._POSITION_OFFSET) % self.ra_steps_360

        with self._mount_seconds_cache_lock:
            self._transact(SkyWatcherCommand.SET_AXIS_POSITION, Revu24.from_int(int(ticks)))
            self._mount_seconds_cache_update = 0  # Drop cache

        return True
    
    def gracefully_stop_motor(self):
        self.logger.info("Stop motor")
        self._transact(SkyWatcherCommand.STOP_MOTION)

    def wait_till_stop(
            self, 
            timeout_s: float | None = None, 
            do_stop: bool = False, 
            func: Callable[[Self], None] | None = None
        ) -> None:
        if timeout_s is not None and timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        
        if do_stop:
            self.gracefully_stop_motor()

        if timeout_s is not None:
            deadline = time.monotonic() + timeout_s
        else:
            deadline = None

        poll_interval_s = min(self.poll_interval_s, 0.2)

        while True:
            if not self.get_status().running:
                return
            if deadline is not None:
                if time.monotonic() > deadline:
                    raise SkyWatcherTimeoutError(f"Slew did not finish within {timeout_s}s")
            
            poll_start = time.monotonic()
            if func is not None:
                try:
                    func(self)
                except Exception:
                    self.logger.exception("While executing %r", func)
            if (delta := poll_interval_s - (time.monotonic() - poll_start)) > 0:
                time.sleep(delta)
    
    def _set_motion(
            self,
            target_status: SkyWatcherMotionStatus,
            current_status: SkyWatcherStatus | None = None,
        ) -> SkyWatcherStatus:
        if current_status is None:
            current_status = self.get_status()

        current_motion_status = SkyWatcherMotionStatus.from_status(current_status)

        slew_mode_changed = target_status.slew_mode != current_motion_status.slew_mode
        direction_changed = target_status.direction != current_motion_status.direction
        speed_mode_changed = target_status.speed_mode != current_motion_status.speed_mode

        status_changed = (
            slew_mode_changed or direction_changed or speed_mode_changed
        )
        if not status_changed:
            return current_status

        if current_status.running:
            self.wait_till_stop(do_stop=True)
            current_status = dataclasses.replace(current_status, running=False)

        self.logger.info("Set motion mode: %s", target_status)
        self._transact(SkyWatcherCommand.SET_MOTION_MODE, target_status.to_command())
        return current_status

    def _set_speed(self, period: int):
        self._transact(SkyWatcherCommand.SET_STEP_PERIOD, Revu24.from_int(period))
    
    def _set_target(self, ticks: int):
        self._transact(SkyWatcherCommand.SET_GOTO_TARGET_INCREMENT, Revu24.from_int(ticks))

    def _set_target_breaks(self, ticks: int):
        self._transact(SkyWatcherCommand.SET_BREAK_POINT_INCREMENT, Revu24.from_int(ticks))

    def _start_motor(self):
        self.logger.info("Start motor")
        self._transact(SkyWatcherCommand.START_MOTION)

    def _do_wrap_delta_move(self, delta: Ha) -> tuple[Ha, HaPerSecond]:
        wrapped_delta = delta.moving_wrap()
        delta_abs_seconds = abs(wrapped_delta)
        
        is_highspeed = delta_abs_seconds > self._LOWSPEED_MARGIN_S

        real_speed = self._HIGHSPEED_SPEED if is_highspeed else self._LOWSPEED_SPEED

        if wrapped_delta < Ha(0):
            real_speed *= -1

        return delta_abs_seconds, real_speed

    def get_slew_real_speed(self, delta_seconds: Ha) -> HaPerSecond:
        return self._do_wrap_delta_move(delta_seconds)[1]

    def slew_delta(self, delta: Ha) -> bool:
        delta_seconds, real_speed = self._do_wrap_delta_move(delta)

        self.set_ra_speed(real_speed, SlewMode.GOTO)

        delta_ticks = self._ha_to_ticks(delta_seconds) % self.ra_steps_360

        self.logger.info(
            "Start slewing for %ds (%d ticks) speed %s (rate %.2f)",
            delta_seconds,
            delta_ticks,
            real_speed,
            real_speed / STELLAR_SPEED,
        )

        self._set_target(delta_ticks)
        self._set_target_breaks(min(200, delta_ticks))  # TODO: Check highspeed
        self._start_motor()
        return True

    def move_ra(self, speed: HaPerSecond) -> bool:
        self.logger.info("Move RA request: speed=%s", speed)
        status = self.get_status()
        if status.running and status.slew_mode == SlewMode.GOTO:
            self.logger.warning("Can not slew while GOTO in progress")
            return False
        
        self.set_ra_speed(speed)
        self.logger.info("Move RA started: speed=%s", speed)

        return True

    def set_ra_speed(self, speed: HaPerSecond, mode: SlewMode = SlewMode.SLEW):
        """Speed is HA-seconds per second. Internal rate is relative to STELLAR_SPEED."""
        if speed == HaPerSecond(.0):
            self.gracefully_stop_motor()
            return True

        rate = speed / STELLAR_SPEED
        rate_abs = abs(rate)

        if not (self.MIN_SPEED < abs(speed) < self.MAX_SPEED):
            self.logger.warning(
                "Speed out of limits: speed=%s min=%s max=%s (rate=%.3f)",
                speed,
                self.MIN_SPEED,
                self.MAX_SPEED,
                rate_abs,
            )

        target_direction = Direction.FORWARD if speed > HaPerSecond(.0) else Direction.BACKWARD

        is_highspeed = False
        if abs(speed) > self._LOWSPEED_SPEED:
            rate_abs /= self.ra_highspeed_ratio
            is_highspeed = True

        target_speed_mode = SpeedMode.HIGHSPEED if is_highspeed else SpeedMode.LOWSPEED

        target_status = SkyWatcherMotionStatus(
            slew_mode=mode,
            direction=target_direction,
            speed_mode=target_speed_mode,
        )

        # TODO: SIDEREAL_DAY or STELLAR_DAY?
        period = STELLAR_DAY * self.ra_steps_worm / self.ra_steps_360 / rate_abs

        self.logger.info(
            "Set RA speed: %s (rate=%.3f * %d highspeed=%s mode=%s) // period=%d",
            speed,
            rate_abs,
            self.ra_highspeed_ratio if is_highspeed else 1,
            is_highspeed,
            mode.name,
            int(period),
        )

        current_status = self._set_motion(target_status)
        self._set_speed(int(period))
        if mode == SlewMode.SLEW and not current_status.running:
            self._start_motor()
        
        return True
    
    def start_tracking(self, speed: HaPerSecond) -> bool:
        self.logger.info("Start tracking request: speed=%s", speed)
        if speed == HaPerSecond(.0):
            self.gracefully_stop_motor()
            return True

        self._last_tracking_speed = speed
        self.set_ra_speed(speed)
        self.logger.info("Start tracking applied: speed=%s", speed)
        return True
