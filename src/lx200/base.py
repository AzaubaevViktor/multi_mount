
from dataclasses import dataclass
from enum import StrEnum
import logging
import queue
import threading
import time
from typing import Any


from .protocol import AlignmentMode
from lx200.protocols import LX200Ha, LX200Dec


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
    
    def get_telescope_raw_position(self) -> tuple[float, float]:
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


class LX200Handler(LX200Base):
    def __init__(self) -> None:
        self._target_ra: LX200Ha = LX200Ha.from_hours(0)
        self._target_dec: LX200Dec = LX200Dec.from_degrees(0)

        self._manual_move_direction: MoveDirection | None = None

        self._is_connected = False

        self._guide_queue: queue.Queue[GuideTask] = queue.Queue()
        self._guide_thread = threading.Thread(target=self._do_guide, name="GuideHelper")
        self._thread_work = True
        self._guide_thread.start()

    def _do_guide(self):
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
                    self._manual_move_direction = MoveDirection.EAST
                result = None
            case LX200Commands.MOVE_NORTH, _:
                if self.move_north():
                    self._manual_move_direction = MoveDirection.NORTH
                result = None
            case LX200Commands.MOVE_SOUTH, _:
                if self.move_south():
                    self._manual_move_direction = MoveDirection.SOUTH
                result = None
            case LX200Commands.MOVE_WEST, _:
                if self.move_west():
                    self._manual_move_direction = MoveDirection.WEST
                result = None

            case LX200Commands.HALT_ALL, _:
                self.halt_all()
                self._manual_move_direction = None
                result = None
            case LX200Commands.HALT_EAST, _:
                if self._manual_move_direction == MoveDirection.EAST:
                    self.halt_east()
                    self._manual_move_direction = None
                result = None
            case LX200Commands.HALT_NORTH, _:
                if self._manual_move_direction == MoveDirection.NORTH:
                    self.halt_north()
                    self._manual_move_direction = None
                result = None
            case LX200Commands.HALT_SOUTH, _:
                if self._manual_move_direction == MoveDirection.SOUTH:
                    self.halt_south()
                    self._manual_move_direction = None
                result = None
            case LX200Commands.HALT_WEST, _:
                if self._manual_move_direction == MoveDirection.WEST:
                    self.halt_west()
                    self._manual_move_direction = None
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
