from dataclasses import dataclass
from enum import StrEnum
import logging
import queue
import threading
import time
from typing import Any

from .protocol import AlignmentMode
from lx200.protocols import Ha, Dec, AxisPos


class LX200Commands(StrEnum):
    GET_TELECOPE_RA = "GR"
    SET_TELESCOPE_RA = "Sr"

    GET_TELESCOPE_DEC = "GD"
    SET_TELESCOPE_DEC = "Sd"

    SYNC = "CM"
    SLEW = "MS"
    GET_DISTANCE = "D"

    MOVE_EAST = "Me"
    MOVE_NORTH = "Mn"
    MOVE_SOUTH = "Ms"
    MOVE_WEST = "Mw"

    HALT_ALL = "Q"

    HALT_EAST = "Qe"
    HALT_NORTH = "Qn"
    HALT_SOUTH = "Qs"
    HALT_WEST = "Qw"

    SET_TARGET_DEC = "Sd"
    SET_TARGET_RA = "Sr"
    
    GET_CALENDAR_FORMAT = "Gc"
    GET_SITE1_NAME = "GM"

    GET_TRACKING_RATE = "GT"
    GET_CURRENT_SITE_LATITUDE= "Gt"
    GET_CURRENT_SITE_LONGITUDE = "Gg"
    GET_UTC_OFFSET_TIME = "GG"
    GET_LOCAL_TIME = "GL"
    GET_CURRENT_DATE = "GC"

    SET_CURRENT_SITE_LONGTITUDE = "Sg"
    SET_CURRENT_SITE_LATITUDE = "St"
    SET_UTC = "SG"

    SET_LOCAL_TIME = "SL"
    SET_LOCAL_DATE = "SC"

    SET_MINIMUM_ELEVATION = "Sh"
    SET_HIGHEST_ELEVATION = "So"

    SET_SLEW_TO_GUIDE = "RG"
    SET_SLEW_TO_CENTER = "RC"
    SET_SLEW_TO_FIND = "RM"
    SET_SLEW_TO_MAX = "RS"

    GUIDE = "Mg"


class _LX200NotImplementedCommand:
    def __init__(self, cmd: str) -> None:
        self.cmd = cmd


class MoveDirection(StrEnum):
    EAST = "east"
    NORTH = "north"
    SOUTH = "south"
    WEST = "west"


_logger = logging.getLogger("lx200")


@dataclass
class GuideTask:
    direction: MoveDirection
    ms: int


class AxisCommandType(StrEnum):
    SET_TRACKING_RATE = "set_tracking_rate"
    HALT_MOTION = "halt_motion"
    HALT_DIRECTION = "halt_direction"


@dataclass
class AxisCommand:
    type: AxisCommandType
    rate: float | None = None
    update_sky_rate: bool = False
    direction: MoveDirection | None = None


class LX200Base:
    def connect(self) -> None:
        raise NotImplementedError()
    
    def stop(self) -> None:
        raise NotImplementedError()
    
    def __del__(self):
        self.stop()
    
    # Auxilary
    def handle_alignment(self, data: bytes) -> AlignmentMode:
        raise NotImplementedError()
    
    def get_calendar_format(self) -> str:
        return "24"
    
    def get_site1_name(self) -> str:
        return "base_lx200"
    
    def get_tracking_rate(self) -> str:
        return "60.0"
    
    # Telecope control
    def get_telescope_ra(self) -> Ha:
        raise NotImplementedError()
    
    def motor_position(self) -> tuple[float, float]:
        raise NotImplementedError()
    
    def sync_telescope_ra(self, position: Ha) -> bool:
        raise NotImplementedError()
    
    def get_telescope_dec(self) -> Dec:
        raise NotImplementedError()
    
    def sync_telescope_dec(self, position: Dec) -> bool:
        raise NotImplementedError()
    
    def slew_to_ra(self, position: Ha) -> bool:
        raise NotImplementedError()
    
    def slew_to_dec(self, position: Dec) -> bool:
        raise NotImplementedError()

    def set_slew_to_find(self) -> bool:
        raise NotImplementedError()

    def set_slew_to_guide(self) -> bool:
        return self.set_slew_to_find()

    def set_slew_to_center(self) -> bool:
        return self.set_slew_to_find()

    def set_slew_to_max(self) -> bool:
        return self.set_slew_to_find()

    def get_distance(self) -> str:
        raise NotImplementedError()
    
    # Manual moving
    def move_east(self) -> bool:
        raise NotImplementedError()
    
    def move_north(self) -> bool:
        raise NotImplementedError()
    
    def move_south(self) -> bool:
        raise NotImplementedError()
    
    def move_west(self) -> bool:
        raise NotImplementedError()

    # Halt movements
    def halt_all(self) -> bool:
        raise NotImplementedError()
    
    def halt_east(self) -> bool:
        raise NotImplementedError()
    
    def halt_north(self) -> bool:
        raise NotImplementedError()
    
    def halt_south(self) -> bool:
        raise NotImplementedError()
    
    def halt_west(self) -> bool:
        raise NotImplementedError()
    
    # Guiding: increase / decrease current tracking rate and returns it when its ok
    def guide_east(self, ms: int) -> None:
        raise NotImplementedError()
    
    def guide_north(self, ms: int) -> None:
        raise NotImplementedError()
    
    def guide_south(self, ms: int) -> None:
        raise NotImplementedError()
    
    def guide_west(self, ms: int) -> None:
        raise NotImplementedError()


class LX200AxisHandler[_POS_CLS: AxisPos](LX200Base):
    AXIS_NAME: str
    DIRECTIONS: tuple[MoveDirection, ...]
    POS_CLS: type[_POS_CLS]

    _TELEMETRY_INTERVAL_S = 1.0
    _RATE_COMPENSATE_INTERVAL_S = .5
    _GUIDE_QUEUE_POLL_INTERVAL_S = .1
    COMPENSATE_MOTOR_SIGN: int  # Is this really the best way for RA/DEC?

    MIN_TRACKING_RATE: float
    DEFAULT_TRACKING_RATE: float
    MAX_TRACKING_RATE: float
    DEFAULT_GUIDE_INTERVAL_MS: int = 4000

    _POSITION_DELTA_ACCEPTED_RATE_S = .1

    def __init__(self) -> None:
        self.logger = logging.getLogger(type(self).__name__)
        self._working = True

        self._position_update_lock = threading.RLock()
        self._mount_position_raw: float = 0
        self._motor_position_raw: float = 0
        self._last_update_s: float = 0

        self._sky_track_rate: float = self.DEFAULT_TRACKING_RATE
        self._current_track_rate: float = self.DEFAULT_TRACKING_RATE
        self._guide_interval: int = self.DEFAULT_GUIDE_INTERVAL_MS
        self._guide_queue: queue.Queue[GuideTask] = queue.Queue()
        self._axis_command_queue: queue.Queue[AxisCommand] = queue.Queue()

        self._telemetry_thread = threading.Thread(target=self._do_log_telemetry, name=f"{type(self).__name__}_telemetry")
        self._telemetry_thread.start()

        self._compensate_thread = threading.Thread(target=self._compensate_tracking_rate, name=f"{type(self).__name__}_compensate")
        self._compensate_thread.start()

        self._goto_to: Any

    # TODO: extract NotImplemented _X methods to Motor interface (in different file), unite statuses, motion type and other stuff to base classes, use it for skywatcher and tmc2209adapter

    def _is_motor_connected(self) -> bool:
        raise NotImplementedError()

    def _get_motor_status(self) -> Any:
        raise NotImplementedError()
    
    def _get_motor_raw_position(self) -> float:
        raise NotImplementedError()
    
    def _get_default_tracking_speed(self) -> float:
        raise NotImplementedError()
    
    def _wrap_mount_position(self, mount_position: float) -> float:
        raise NotImplementedError()

    def _wrap_motor_position(self, motor_position: float) -> float:
        raise NotImplementedError()
    
    def _set_tracking_rate(self, rate: float) -> float | None:
        raise NotImplementedError()

    def set_tracking_rate(self, rate: float, update_sky_rate: bool = False) -> None:
        self._axis_command_queue.put(
            AxisCommand(
                type=AxisCommandType.SET_TRACKING_RATE,
                rate=rate,
                update_sky_rate=update_sky_rate,
            )
        )
    
    def resume_tracking(self) -> None:
        self.set_tracking_rate(self._sky_track_rate)

    def _halt_motion(self) -> None:
        raise NotImplementedError()

    def guide_east(self, ms: int) -> None:
        self._guide_queue.put(GuideTask(
            direction=MoveDirection.EAST, ms=ms,
        ))

    def guide_north(self, ms: int) -> None:
        self._guide_queue.put(GuideTask(
            direction=MoveDirection.NORTH, ms=ms,
        ))

    def guide_south(self, ms: int) -> None:
        self._guide_queue.put(GuideTask(
            direction=MoveDirection.SOUTH, ms=ms,
        ))

    def guide_west(self, ms: int) -> None:
        self._guide_queue.put(GuideTask(
            direction=MoveDirection.WEST, ms=ms,
        ))

    def halt_all(self):
        self.halt_motion()
        return True

    def halt_motion(self) -> None:
        self._axis_command_queue.put(
            AxisCommand(type=AxisCommandType.HALT_MOTION)
        )

    def _halt_direction(self, direction: MoveDirection) -> bool:
        if direction not in self.DIRECTIONS:
            return False

        self._axis_command_queue.put(
            AxisCommand(type=AxisCommandType.HALT_DIRECTION, direction=direction)
        )
        return True

    def halt_east(self) -> bool:
        return self._halt_direction(MoveDirection.EAST)

    def halt_north(self) -> bool:
        return self._halt_direction(MoveDirection.NORTH)

    def halt_south(self) -> bool:
        return self._halt_direction(MoveDirection.SOUTH)

    def halt_west(self) -> bool:
        return self._halt_direction(MoveDirection.WEST)

    def _apply_tracking_rate_now(self, rate: float, update_sky_rate: bool) -> None:
        rounded_rate = self._set_tracking_rate(rate)
        applied_rate = rounded_rate if rounded_rate is not None else rate

        if update_sky_rate:
            self._sky_track_rate = applied_rate
        self._current_track_rate = applied_rate

        try:
            motor_position = self._get_motor_raw_position()
            self._motor_position_raw = self._wrap_motor_position(motor_position)
            self.logger.debug(
                "Update motor position snapshot: %.3f",
                self._motor_position_raw,
            )
        except Exception:
            self.logger.exception("While updating motor position snapshot")

        self._last_update_s = time.monotonic()

    def _apply_axis_command(self, cmd: AxisCommand) -> None:
        self.logger.debug("Apply axis command: %s", cmd)
        match cmd.type:
            # TODO: Rewrite to set values and _apply_tracking_rate_now down + logs
            case AxisCommandType.SET_TRACKING_RATE:
                if cmd.rate is None:
                    self.logger.warning("Skip empty tracking rate command: %s", cmd)
                    return
                with self._position_update_lock:
                    self._apply_tracking_rate_now(cmd.rate, update_sky_rate=cmd.update_sky_rate)
                self.logger.info(
                    "Applied tracking rate command: rate=%.3f update_sky_rate=%s current=%.3f sky=%.3f",
                    cmd.rate,
                    cmd.update_sky_rate,
                    self._current_track_rate,
                    self._sky_track_rate,
                )
            case AxisCommandType.HALT_MOTION:
                self.logger.info("Apply halt motion command: drop sky tracking to default")
                self._halt_motion()
                with self._position_update_lock:
                    self._sky_track_rate = self.DEFAULT_TRACKING_RATE
                    self._apply_tracking_rate_now(self._sky_track_rate, update_sky_rate=False)
                self.logger.info(
                    "Applied halt motion command: current=%.3f sky=%.3f",
                    self._current_track_rate,
                    self._sky_track_rate,
                )
            case AxisCommandType.HALT_DIRECTION:
                if cmd.direction not in self.DIRECTIONS:
                    self.logger.warning("Ignore wrong halt direction for %s: %s", self.AXIS_NAME, cmd.direction)
                    return
                self.logger.info("Apply halt direction command: %s", cmd.direction)
                self._halt_motion()
                with self._position_update_lock:
                    self._apply_tracking_rate_now(self._sky_track_rate, update_sky_rate=False)
                self.logger.info(
                    "Applied halt direction command: direction=%s current=%.3f sky=%.3f",
                    cmd.direction,
                    self._current_track_rate,
                    self._sky_track_rate,
                )
            case _:
                self.logger.warning("Unknown axis command: %s", cmd)

    def _do_apply_guide(self) -> None:
        _logger = self.logger.getChild("guide")
        while self._working:
            try:
                guide_task = self._guide_queue.get(timeout=self._GUIDE_QUEUE_POLL_INTERVAL_S)
            except queue.Empty:
                continue

            if guide_task.direction not in self.DIRECTIONS:
                _logger.warning("Wrong direction: %s", guide_task.direction)
                continue
            
            # TODO: Move this to polar_compensator and rewrite to set rates instead of guide commands
            match guide_task.direction:
                case MoveDirection.EAST | MoveDirection.SOUTH:
                    guide_rate = (self.MAX_TRACKING_RATE - self.DEFAULT_TRACKING_RATE) * guide_task.ms / self._guide_interval + self.DEFAULT_TRACKING_RATE
                case MoveDirection.WEST | MoveDirection.NORTH:
                    guide_rate = self.DEFAULT_TRACKING_RATE - (
                        (self.DEFAULT_TRACKING_RATE - self.MIN_TRACKING_RATE) * guide_task.ms / self._guide_interval
                    )
                case _:
                    _logger.warning("Wrong direction: %s", guide_task.direction)
                    continue

            _logger.debug("Found applyable guide task: %s -> rate: %.3f", guide_task, guide_rate)
            try:
                self.set_tracking_rate(guide_rate, update_sky_rate=True)
            except Exception:
                _logger.exception("While queueing guide task: %s -> rate=%.3f", guide_task, guide_rate)

    def _do_log_telemetry(self):
        _logger = self.logger.getChild("telemetry")
        while self._working:
            if not self._is_motor_connected():
                time.sleep(self._TELEMETRY_INTERVAL_S)
                continue

            try:
                status = self._get_motor_status()
            except Exception:
                _logger.exception("While polling telemetry")
                time.sleep(self._TELEMETRY_INTERVAL_S)
                continue

            consistent = self._position_update_lock.acquire(timeout=.1)

            current_mount_position = self._mount_position_raw
            current_motor_position = self._motor_position_raw

            if consistent:
                self._position_update_lock.release()

            _logger.info(
                "%s: MNT(%s raw=%d) MTR(%s raw=%s) status=%s goto_active=%s consistent=%s",
                self.AXIS_NAME,
                self.POS_CLS.from_raw(current_mount_position),  
                current_mount_position,
                self.POS_CLS.from_raw(current_motor_position),
                current_motor_position,
                str(status),
                getattr(self, "_goto_to", None) is not None,
                consistent,
            )

            time.sleep(self._TELEMETRY_INTERVAL_S)

    def _compensate_tracking_rate(self):
        _logger = self.logger.getChild("compensate")
        self._last_update_s = time.monotonic()

        while self._working:
            if not self._is_motor_connected():
                time.sleep(self._RATE_COMPENSATE_INTERVAL_S)
                continue

            now = time.monotonic()
            wait_timeout_s = max(0., self._RATE_COMPENSATE_INTERVAL_S - (now - self._last_update_s))

            try:
                command = self._axis_command_queue.get(timeout=wait_timeout_s)
            except queue.Empty:
                command = None
            if command:
                commands = [command]
                while True:
                    try:
                        commands.append(self._axis_command_queue.get_nowait())
                    except queue.Empty:
                        break

                for queued_command in commands:
                    try:
                        self._apply_axis_command(queued_command)
                    except Exception:
                        _logger.exception("While applying axis command: %s", queued_command)
                continue

            with self._position_update_lock:
                try:
                    motor_position = self._wrap_motor_position(self._get_motor_raw_position())
                    self.logger.debug("Motor raw position: %f", motor_position)
                    now = time.monotonic()
                except Exception as e:
                    _logger.warning("While get raw position: %s", e)
                    continue

                elapsed_s = now - self._last_update_s

                expected_delta_seconds = elapsed_s * self._get_default_tracking_speed() * self._current_track_rate

                actual_delta_seconds = motor_position - self._motor_position_raw

                actual_delta_seconds *= self.COMPENSATE_MOTOR_SIGN

                delta = expected_delta_seconds - actual_delta_seconds

                self.logger.debug(
                    "Calculated delta by %.3fs: %.3f = exp=%.3f = (%.3fs * (%.3fs/s x%.3f)) - act=%.3f = (%.3f - %.3f); MNT: %.3f",
                    elapsed_s,
                    delta, 
                    expected_delta_seconds, elapsed_s, self._get_default_tracking_speed(), self._current_track_rate,
                    actual_delta_seconds, motor_position,  self._motor_position_raw,
                    self._mount_position_raw,
                )
                
                if abs(delta) < self._POSITION_DELTA_ACCEPTED_RATE_S * elapsed_s:
                    delta = 0

                new_mount_position = self._wrap_mount_position(self._mount_position_raw + delta)

                try:
                    self.logger.info(
                        "Update mount position: %s -> %s (%.2f -> %.2f)",
                        self.POS_CLS.from_raw(self._mount_position_raw),
                        self.POS_CLS.from_raw(new_mount_position),
                        self._mount_position_raw, 
                        new_mount_position,
                    )
                except:
                    _logger.exception("While log")

                self._mount_position_raw = new_mount_position

                self._motor_position_raw = motor_position
                self._last_update_s = now

    def stop(self):
        if not self._working:
            return 

        self._working = False
        if self._telemetry_thread and self._telemetry_thread.is_alive():
            try:
                self._telemetry_thread.join(timeout=self._TELEMETRY_INTERVAL_S * 5)
            except Exception:
                self.logger.exception("While finishing telemetry thread")
        if self._compensate_thread and self._compensate_thread.is_alive():
            self._compensate_thread.join(timeout=self._RATE_COMPENSATE_INTERVAL_S * 5)
        if self._guide_thread and self._guide_thread.is_alive():
            self._guide_thread.join(timeout=self._GUIDE_QUEUE_POLL_INTERVAL_S * 5)

    def __del__(self):
        self.stop()


class LX200RAHandler(LX200AxisHandler[Ha]):
    POS_CLS = Ha
    AXIS_NAME = "Ra"
    DIRECTIONS = (MoveDirection.EAST, MoveDirection.WEST)
    COMPENSATE_MOTOR_SIGN = 1

    MIN_TRACKING_RATE = 0
    DEFAULT_TRACKING_RATE = 1
    MAX_TRACKING_RATE = 2
    DEFAULT_GUIDE_INTERVAL_MS = 4000


class LX200DECHandler(LX200AxisHandler[Dec]):
    POS_CLS = Dec
    AXIS_NAME = "Dec"
    DIRECTIONS = (MoveDirection.NORTH, MoveDirection.SOUTH)
    COMPENSATE_MOTOR_SIGN = -1

    MIN_TRACKING_RATE = -1
    DEFAULT_TRACKING_RATE = 0
    MAX_TRACKING_RATE = 1
    DEFAULT_GUIDE_INTERVAL_MS = 4000


class LX200Handler(LX200Base):
    def __init__(self) -> None:
        self.logger = logging.getLogger(type(self).__name__)
        self._target_ra: Ha = Ha.from_hours(0)
        self._target_dec: Dec = Dec.from_degrees(0)

        self._manual_move_directions: list[MoveDirection] = []

        self._is_connected = False

    def connect(self) -> None:
        self._is_connected = True
    
    def _do_handle(self, cmd: LX200Commands, argument: Any) -> Any:
        result = None

        match (cmd, argument):
            case LX200Commands.GET_TELECOPE_RA, _:
                result = self.get_telescope_ra()
            case LX200Commands.SET_TELESCOPE_RA, position:
                self._target_ra = Ha.from_string(position)
                result = True

            case LX200Commands.GET_TELESCOPE_DEC, _:
                result = self.get_telescope_dec()
            case LX200Commands.SET_TELESCOPE_DEC, position:
                self._target_dec = Dec.from_string(position)
                result = True

            case LX200Commands.SYNC, _:
                result = self.sync_telescope_ra(self._target_ra)
                result &= self.sync_telescope_dec(self._target_dec)
                result = "OK"
            case LX200Commands.SLEW, _:
                result = self.slew_to_ra(self._target_ra)
                result = self.slew_to_dec(self._target_dec)
                result = False
                # But 1<below horison>#
                # But 2<below higher>#

            case LX200Commands.MOVE_EAST, _:
                if self.move_east():
                    if MoveDirection.EAST not in self._manual_move_directions:
                        self._manual_move_directions.append(MoveDirection.EAST)
                result = None
            case LX200Commands.MOVE_NORTH, _:
                if self.move_north():
                    if MoveDirection.NORTH not in self._manual_move_directions:
                        self._manual_move_directions.append(MoveDirection.NORTH)
                result = None
            case LX200Commands.MOVE_SOUTH, _:
                if self.move_south():
                    if MoveDirection.SOUTH not in self._manual_move_directions:
                        self._manual_move_directions.append(MoveDirection.SOUTH)
                result = None
            case LX200Commands.MOVE_WEST, _:
                if self.move_west():
                    if MoveDirection.WEST not in self._manual_move_directions:
                        self._manual_move_directions.append(MoveDirection.WEST)
                result = None

            case LX200Commands.HALT_ALL, _:
                self.halt_all()
                self._manual_move_directions.clear()
                result = None
            case LX200Commands.HALT_EAST, _:
                if MoveDirection.EAST in self._manual_move_directions:
                    self.halt_east()
                    self._manual_move_directions.remove(MoveDirection.EAST)
                result = None
            case LX200Commands.HALT_NORTH, _:
                if MoveDirection.NORTH in self._manual_move_directions:
                    self.halt_north()
                    self._manual_move_directions.remove(MoveDirection.NORTH)
                result = None
            case LX200Commands.HALT_SOUTH, _:
                if MoveDirection.SOUTH in self._manual_move_directions:
                    self.halt_south()
                    self._manual_move_directions.remove(MoveDirection.SOUTH)
                result = None
            case LX200Commands.HALT_WEST, _:
                if MoveDirection.WEST in self._manual_move_directions:
                    self.halt_west()
                    self._manual_move_directions.remove(MoveDirection.WEST)
                result = None

            case LX200Commands.GET_CALENDAR_FORMAT, _:
                result = self.get_calendar_format()
            case LX200Commands.GET_SITE1_NAME, _:
                result = self.get_site1_name()
            case LX200Commands.GET_TRACKING_RATE, _:
                result = self.get_tracking_rate()
            case LX200Commands.GET_CURRENT_SITE_LATITUDE, _:
                result = "+00*00"
            case LX200Commands.GET_CURRENT_SITE_LONGITUDE, _:
                result = "+000*00"
            case LX200Commands.SET_CURRENT_SITE_LONGTITUDE, _:
                result = True
            case LX200Commands.SET_CURRENT_SITE_LATITUDE, _:
                result = False

            case LX200Commands.GET_UTC_OFFSET_TIME, _:
                result = "+0"
            case LX200Commands.SET_UTC, _:
                result = True

            case LX200Commands.GET_LOCAL_TIME, _:
                result = "00:00:00"
            case LX200Commands.SET_LOCAL_TIME, _:
                result = True
            case LX200Commands.GET_CURRENT_DATE, _:
                result = "01/01/26"
            case LX200Commands.SET_LOCAL_DATE, _:
                result = True

            case LX200Commands.SET_SLEW_TO_GUIDE, _:
                self.set_slew_to_guide()
                result = None
            case LX200Commands.SET_SLEW_TO_CENTER, _:
                self.set_slew_to_center()
                result = None
            case LX200Commands.SET_SLEW_TO_FIND, _:
                self.set_slew_to_find()
                result = None
            case LX200Commands.SET_SLEW_TO_MAX, _:
                self.set_slew_to_max()
                result = None
            case LX200Commands.GUIDE, data:
                direction = data[0].lower()
                ms = int(data[1:])

                match direction:
                    case 'w': 
                        self.guide_west(ms)
                    case 'e':
                        self.guide_east(ms)
                    case 'n':
                        self.guide_north(ms)
                    case 's':
                        self.guide_south(ms)
                    case _:
                        raise RuntimeError(f"Wrong guide direction: {direction}")
                
                result = None
            case LX200Commands.GET_DISTANCE, _:
                result = self.get_distance()
            case _:
                _logger.warning("Not implemented command: %s(%s)", cmd, argument)
                result = _LX200NotImplementedCommand(cmd)
        
        return result

    def handle(self, full_command: str) -> Any:
        _cmd, argument = full_command[:2], full_command[2:]
        try:
            cmd = LX200Commands(_cmd)
        except ValueError as e:
            raise RuntimeError(f"Unknown LX200 command: {_cmd}({argument})") from e
        _logger.info("Get command %s %s(%s)", cmd, cmd.name, argument)

        result = self._do_handle(cmd, argument)
        
        if isinstance(result, _LX200NotImplementedCommand):
            raise RuntimeError(f"Not implemented LX200 command: {cmd} {cmd.name}({argument})")
        elif result is not None:
            _logger.info("Answer command %s %s(%s) -> %s", cmd, cmd.name, argument, result)
        else:
            _logger.warning("Empty responce: %s %s(%s) -> ∅", cmd, cmd.name, argument)

        return result

    def stop(self):
        pass
