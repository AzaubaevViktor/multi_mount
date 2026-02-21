from dataclasses import dataclass
from enum import StrEnum
import logging
import queue
from re import A
import threading
import time
from typing import Any, Callable


from .protocol import AlignmentMode
from lx200.protocols import LX200Ha, LX200Dec, LX200PositionBase


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

    SET_SLEW_TO_FIND = "RM"
    # TODO: Add RG, RS, RC

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
    direction: str
    ms: int


class GuideStatus(StrEnum):
    NONE = "none"
    ACTIVE = "active"
    STOPPED = "stopped"



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
    def get_telescope_ra(self) -> LX200Ha:
        raise NotImplementedError()
    
    def motor_position(self) -> tuple[float, float]:
        raise NotImplementedError()
    
    def sync_telescope_ra(self, position: LX200Ha) -> bool:
        raise NotImplementedError()
    
    def get_telescope_dec(self) -> LX200Dec:
        raise NotImplementedError()
    
    def sync_telescope_dec(self, position: LX200Dec) -> bool:
        raise NotImplementedError()
    
    def slew_to_ra(self, position: LX200Ha) -> bool:
        raise NotImplementedError()
    
    def slew_to_dec(self, position: LX200Dec) -> bool:
        raise NotImplementedError()

    def set_slew_to_find(self) -> bool:
        raise NotImplementedError()

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
    def guide_east(self) -> bool:
        raise NotImplementedError()
    
    def guide_north(self) -> bool:
        raise NotImplementedError()
    
    def guide_south(self) -> bool:
        raise NotImplementedError()
    
    def guide_west(self) -> bool:
        raise NotImplementedError()
    
    def guide_reset(self) -> bool:
        raise NotImplementedError()


class LX200AxisHandler[_POS_CLS: LX200PositionBase](LX200Base):
    AXIS_NAME: str
    POS_CLS: type[_POS_CLS]

    _TELEMETRY_INTERVAL_S = 1.0
    _RATE_COMPENSATE_INTERVAL_S = .5
    COMPENSATE_MOTOR_SIGN: int  # Is this really the best way for RA/DEC?

    _DEFAULT_TRACKING_RATE: float
    _POSITION_DELTA_ACCEPTED_RATE_S = .1

    def __init__(self) -> None:
        self.logger = logging.getLogger(type(self).__name__)
        self._working = True

        self._position_update_lock = threading.RLock()
        self._mount_position_raw: float = 0
        self._motor_position_raw: float = 0
        self._last_update_s: float = 0

        self._current_track_rate_coef = self._DEFAULT_TRACKING_RATE
        self._guide_status = GuideStatus.NONE

        self._telemetry_thread = threading.Thread(target=self._do_log_telemetry, name=f"{type(self).__name__}_telemetry")
        self._telemetry_thread.start()

        self._compensate_thread = threading.Thread(target=self._compensate_tracking_rate, name=f"{type(self).__name__}_compensate")
        self._compensate_thread.start()

        self._goto_to: Any

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
    
    def _guide_east(self) -> bool:
        raise NotImplementedError()
    
    def _guide_north(self) -> bool:
        raise NotImplementedError()
    
    def _guide_south(self) -> bool:
        raise NotImplementedError()
    
    def _guide_west(self) -> bool:
        raise NotImplementedError()
    
    def _guide_reset(self) -> bool:
        raise NotImplementedError()
    
    def _run_guide_with_status(
        self,
        action: str,
        target_status: GuideStatus,
        callback: Callable[[], bool],
    ) -> bool:
        with self._position_update_lock:
            previous_status = self._guide_status
            self._guide_status = target_status
            self.logger.info(
                "Guide status %s -> %s (%s)",
                previous_status,
                self._guide_status,
                action,
            )
            result = callback()
            if not result:
                self._guide_status = previous_status
                self.logger.warning(
                    "Guide action %s failed, rollback status to %s",
                    action,
                    previous_status,
                )
            return result
    
    def guide_east(self) -> bool:
        return self._run_guide_with_status(
            action="guide_east",
            target_status=GuideStatus.ACTIVE,
            callback=self._guide_east,
        )
    
    def guide_north(self) -> bool:
        return self._run_guide_with_status(
            action="guide_north",
            target_status=GuideStatus.ACTIVE,
            callback=self._guide_north,
        )
    
    def guide_south(self) -> bool:
        return self._run_guide_with_status(
            action="guide_south",
            target_status=GuideStatus.ACTIVE,
            callback=self._guide_south,
        )
    
    def guide_west(self) -> bool:
        return self._run_guide_with_status(
            action="guide_west",
            target_status=GuideStatus.ACTIVE,
            callback=self._guide_west,
        )
    
    def guide_reset(self) -> bool:
        return self._run_guide_with_status(
            action="guide_reset",
            target_status=GuideStatus.STOPPED,
            callback=self._guide_reset,
        )

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
        while self._working:
            if not self._is_motor_connected():
                time.sleep(self._RATE_COMPENSATE_INTERVAL_S)
                continue

            now = time.monotonic()
            sleep_time = self._RATE_COMPENSATE_INTERVAL_S - (now - self._last_update_s)

            if sleep_time > 0:
                time.sleep(sleep_time)

            with self._position_update_lock:
                try:
                    motor_position = self._get_motor_raw_position()
                    self.logger.debug("Motor raw position: %f", motor_position)
                    now = time.monotonic()
                except Exception as e:
                    _logger.warning("While get raw position: %s", e)
                    continue
                
                if self._guide_status == GuideStatus.ACTIVE:
                    self.logger.debug(
                        "Skip tracking compensation: guide_status=%s",
                        self._guide_status,
                    )
                    self._last_update_s = now
                    continue
                
                if self._guide_status == GuideStatus.STOPPED:
                    self.logger.debug(
                        "Sync tracking state after guide stop: motor_raw %.2f -> %.2f",
                        self._motor_position_raw,
                        motor_position,
                    )
                    self._motor_position_raw = motor_position
                    self._last_update_s = now
                    self._guide_status = GuideStatus.NONE
                    continue
                
                self.logger.debug(
                    "Compensate tracking rate: guide_status=%s",
                    self._guide_status,
                )

                elapsed_s = now - self._last_update_s

                # TODO: hide Current_track_rate_coef under lock in other places
                expected_delta_seconds = elapsed_s * self._get_default_tracking_speed() * self._current_track_rate_coef

                # TODO: Add _wrap_motor_position
                actual_delta_seconds = motor_position - self._motor_position_raw

                actual_delta_seconds *= self.COMPENSATE_MOTOR_SIGN

                delta = expected_delta_seconds - actual_delta_seconds

                self.logger.debug(
                    "Calculated delta by %.3fs: %.3f = ((%f = (x%.2f)) - %.2f = (%.2f - %.2f)); MNT:%.2f",
                    elapsed_s,
                    delta, 
                    expected_delta_seconds, 
                    self._current_track_rate_coef,
                    actual_delta_seconds, 
                    motor_position,
                    self._motor_position_raw,
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
        self._working = False
        if self._telemetry_thread and self._telemetry_thread.is_alive():
            try:
                self._telemetry_thread.join(timeout=self._TELEMETRY_INTERVAL_S * 5)
            except Exception:
                self.logger.exception("While finishing telemetry thread")
        if self._compensate_thread and self._compensate_thread.is_alive():
            self._compensate_thread.join(timeout=self._RATE_COMPENSATE_INTERVAL_S * 5)

    def __del__(self):
        self.stop()


class LX200RAHandler(LX200AxisHandler[LX200Ha]):
    POS_CLS = LX200Ha
    AXIS_NAME = "Ra"
    COMPENSATE_MOTOR_SIGN = 1


class LX200DECHandler(LX200AxisHandler[LX200Dec]):
    POS_CLS = LX200Dec
    AXIS_NAME = "Dec"
    COMPENSATE_MOTOR_SIGN = -1


class LX200Handler(LX200Base):
    def __init__(self) -> None:
        self._target_ra: LX200Ha = LX200Ha.from_hours(0)
        self._target_dec: LX200Dec = LX200Dec.from_degrees(0)

        self._manual_move_directions: list[MoveDirection] = []

        self._is_connected = False

        self._guide_queue: queue.Queue[GuideTask] = queue.Queue()
        self._guide_thread = threading.Thread(target=self._do_guide, name="GuideHelper")
        self._thread_work = True
        self._guide_thread.start()

    def _do_guide(self):
        # TODO: Guide RA and DEC separately
        DEFAULT_WAIT_TIMEOUT = 1
        stop_guide = time.monotonic() + DEFAULT_WAIT_TIMEOUT
        current_guide_direction: str | None = None

        while self._thread_work:
            if not self._is_connected:
                time.sleep(.1)
                continue

            timeout = stop_guide - time.monotonic()
            if timeout < 0:
                timeout = 0
            
            try:
                guide_task = self._guide_queue.get(timeout=timeout)
            except queue.Empty:
                if current_guide_direction:
                    # TODO: Show real guide time
                    self.guide_reset()
                    stop_guide = time.monotonic() + DEFAULT_WAIT_TIMEOUT
                    current_guide_direction = None
                continue

            if current_guide_direction is not None and \
                current_guide_direction != guide_task.direction:
                self.guide_reset()

            match guide_task.direction:
                case 'w':
                    self.guide_west()
                case 'e':
                    self.guide_east()
                case 'n':
                    self.guide_north()
                case 's':
                    self.guide_south()
                case _:
                    raise RuntimeError(f"Wrong guide direction: {guide_task.direction}")
                
            stop_guide = time.monotonic() + guide_task.ms / 1000.
            current_guide_direction = guide_task.direction
        
    def connect(self) -> None:
        self._is_connected = True
    
    def _do_handle(self, cmd: LX200Commands, argument: Any) -> Any:
        result = None

        match (cmd, argument):
            case LX200Commands.GET_TELECOPE_RA, _:
                result = self.get_telescope_ra()
            case LX200Commands.SET_TELESCOPE_RA, position:
                self._target_ra = LX200Ha.from_string(position)
                result = True

            case LX200Commands.GET_TELESCOPE_DEC, _:
                result = self.get_telescope_dec()
            case LX200Commands.SET_TELESCOPE_DEC, position:
                self._target_dec = LX200Dec.from_string(position)
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

            case LX200Commands.SET_SLEW_TO_FIND, _:
                self.set_slew_to_find()
                result = None
            case LX200Commands.GUIDE, data:
                direction = data[0]
                ms = int(data[1:])

                guide_task = GuideTask(direction, ms)

                match direction:
                    case 'w' | 'e' | 'n' | 's':
                        self._guide_queue.put(guide_task)
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
        cmd = LX200Commands(_cmd)
        _logger.info("Get command %s %s(%s)", cmd, cmd.name, argument)

        result = self._do_handle(cmd, argument)
        
        if isinstance(result, _LX200NotImplementedCommand):
            _logger.warning("Empty responce: %s %s(%s) -> ∅", cmd, cmd.name, argument)

            result = None
        elif result is not None:
            _logger.info("Answer command %s %s(%s) -> %s", cmd, cmd.name, argument, result)
        else:
            _logger.warning("Empty responce: %s %s(%s) -> ∅", cmd, cmd.name, argument)

        return result

    def stop(self):
        self._thread_work = False
        if self._guide_thread and self._guide_thread.is_alive():
            self._guide_thread.join()
