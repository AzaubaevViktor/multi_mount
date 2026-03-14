from collections import deque
from enum import StrEnum
import logging
import re
import threading
from typing import Any

from sky.physics import Dec, Ha, Second, SkyDirection

from .protocol import AlignmentMode


class LX200Commands(StrEnum):
    # Telescope coordinates
    GET_TELECOPE_RA = "GR"
    SET_TELESCOPE_RA = "Sr"

    GET_TELESCOPE_DEC = "GD"
    SET_TELESCOPE_DEC = "Sd"

    # Telescope actions
    SYNC = "CM"
    SLEW = "MS"
    GET_DISTANCE = "D"

    # Manual motion
    MOVE_EAST = "Me"
    MOVE_NORTH = "Mn"
    MOVE_SOUTH = "Ms"
    MOVE_WEST = "Mw"

    # Halt motion
    HALT_ALL = "Q"
    HALT_EAST = "Qe"
    HALT_NORTH = "Qn"
    HALT_SOUTH = "Qs"
    HALT_WEST = "Qw"

    # LX200 info and site settings
    GET_CALENDAR_FORMAT = "Gc"
    GET_SITE1_NAME = "GM"

    GET_TRACKING_RATE = "GT"
    GET_CURRENT_SITE_LATITUDE = "Gt"
    GET_CURRENT_SITE_LONGITUDE = "Gg"
    GET_UTC_OFFSET_TIME = "GG"
    GET_LOCAL_TIME = "GL"
    GET_CURRENT_DATE = "GC"

    SET_CURRENT_SITE_LONGTITUDE = "Sg"
    SET_CURRENT_SITE_LATITUDE = "St"
    SET_UTC = "SG"

    SET_LOCAL_TIME = "SL"
    SET_LOCAL_DATE = "SC"

    # Elevation limits
    SET_MINIMUM_ELEVATION = "Sh"
    SET_HIGHEST_ELEVATION = "So"

    # Slew rates
    SET_SLEW_TO_GUIDE = "RG"
    SET_SLEW_TO_CENTER = "RC"
    SET_SLEW_TO_FIND = "RM"
    SET_SLEW_TO_MAX = "RS"

    # Guide pulse
    GUIDE = "Mg"


class _LX200NotImplementedCommand:
    def __init__(self, cmd: str) -> None:
        self.cmd = cmd


_logger = logging.getLogger("lx200")


class LX200Base:
    def connect(self) -> None:
        raise NotImplementedError()

    def stop(self) -> None:
        raise NotImplementedError()

    def __del__(self) -> None:
        self.stop()

    def handle_alignment(self, data: bytes) -> AlignmentMode:
        raise NotImplementedError()

    def get_calendar_format(self) -> str:
        return "24"

    def get_site1_name(self) -> str:
        return "base_lx200"

    def get_tracking_rate(self) -> str:
        return "60.0"

    def set_minimum_elevation(self, position: Dec) -> bool:
        return True

    def set_highest_elevation(self, position: Dec) -> bool:
        return True

    def get_telescope_ra(self) -> Ha:
        raise NotImplementedError()

    def sync_telescope(self, ra: Ha, dec: Dec) -> bool:
        result = self.sync_telescope_ra(ra)
        result &= self.sync_telescope_dec(dec)
        return result

    def sync_telescope_ra(self, position: Ha) -> bool:
        raise NotImplementedError()

    def get_telescope_dec(self) -> Dec:
        raise NotImplementedError()

    def sync_telescope_dec(self, position: Dec) -> bool:
        raise NotImplementedError()

    def slew_to(self, ra: Ha, dec: Dec) -> bool:
        result = self.slew_to_ra(ra)
        result &= self.slew_to_dec(dec)
        return result

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

    def move_east(self) -> bool:
        raise NotImplementedError()

    def move_north(self) -> bool:
        raise NotImplementedError()

    def move_south(self) -> bool:
        raise NotImplementedError()

    def move_west(self) -> bool:
        raise NotImplementedError()

    def halt_all(self) -> bool:
        raise NotImplementedError()

    def stop_all(self) -> bool:
        return self.halt_all()

    def halt_east(self) -> bool:
        raise NotImplementedError()

    def halt_north(self) -> bool:
        raise NotImplementedError()

    def halt_south(self) -> bool:
        raise NotImplementedError()

    def halt_west(self) -> bool:
        raise NotImplementedError()

    def guide_east(self, ms: int) -> None:
        raise NotImplementedError()

    def guide_north(self, ms: int) -> None:
        raise NotImplementedError()

    def guide_south(self, ms: int) -> None:
        raise NotImplementedError()

    def guide_west(self, ms: int) -> None:
        raise NotImplementedError()


class LX200Handler(LX200Base):
    """
    All methods should be fast and non-blocking.
    """
    DOUBLE_STOP_WINDOW_S = Second(1.0)

    def __init__(self) -> None:
        self.logger = logging.getLogger(type(self).__name__)
        self._target_ra = Ha(0)
        self._target_dec = Dec(0)
        self._minimum_elevation = Dec(0)
        self._highest_elevation = Dec(90 * 60 * 60)
        self._manual_move_directions: list[SkyDirection] = []
        self._last_halt_all: Second | None = None
        self._monitor_lock = threading.RLock()
        self._recent_commands: deque[tuple[Second, str]] = deque(maxlen=8)
        self._last_guide_command: tuple[Second, str] | None = None

        self._is_connected = False

    def connect(self) -> None:
        self._is_connected = True
    
    def _do_handle(self, cmd: LX200Commands, argument: Any, now: Second | None = None) -> Any:
        result = None
        if now is None:
            now = Second.monotonic()

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
                self.sync_telescope(self._target_ra, self._target_dec)
                result = "OK"
            case LX200Commands.SLEW, _:
                self._last_halt_all = None
                self.slew_to(self._target_ra, self._target_dec)
                result = False
                # LX200 also allows non-zero result codes for below-horizon / above-limit failures.

            case LX200Commands.MOVE_EAST, _:
                self._last_halt_all = None
                if self.move_east() and SkyDirection.EAST not in self._manual_move_directions:
                    self._manual_move_directions.append(SkyDirection.EAST)
            case LX200Commands.MOVE_NORTH, _:
                self._last_halt_all = None
                if self.move_north() and SkyDirection.NORTH not in self._manual_move_directions:
                    self._manual_move_directions.append(SkyDirection.NORTH)
            case LX200Commands.MOVE_SOUTH, _:
                self._last_halt_all = None
                if self.move_south() and SkyDirection.SOUTH not in self._manual_move_directions:
                    self._manual_move_directions.append(SkyDirection.SOUTH)
            case LX200Commands.MOVE_WEST, _:
                self._last_halt_all = None
                if self.move_west() and SkyDirection.WEST not in self._manual_move_directions:
                    self._manual_move_directions.append(SkyDirection.WEST)
            case LX200Commands.HALT_ALL, _:
                if self._last_halt_all is not None and now - self._last_halt_all <= self.DOUBLE_STOP_WINDOW_S:
                    self.stop_all()
                    self._last_halt_all = None
                else:
                    self.halt_all()
                    self._last_halt_all = now

                self._manual_move_directions.clear()
            case LX200Commands.HALT_EAST, _:
                if SkyDirection.EAST in self._manual_move_directions:
                    self.halt_east()
                    self._manual_move_directions.remove(SkyDirection.EAST)
            case LX200Commands.HALT_NORTH, _:
                if SkyDirection.NORTH in self._manual_move_directions:
                    self.halt_north()
                    self._manual_move_directions.remove(SkyDirection.NORTH)
            case LX200Commands.HALT_SOUTH, _:
                if SkyDirection.SOUTH in self._manual_move_directions:
                    self.halt_south()
                    self._manual_move_directions.remove(SkyDirection.SOUTH)
            case LX200Commands.HALT_WEST, _:
                if SkyDirection.WEST in self._manual_move_directions:
                    self.halt_west()
                    self._manual_move_directions.remove(SkyDirection.WEST)
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

            case LX200Commands.SET_MINIMUM_ELEVATION, position:
                match = re.fullmatch(r"([+-]?)(\d{2})\*", position)
                if not match:
                    result = False
                else:
                    sign = -1 if match.group(1) == "-" else 1
                    result = self.set_minimum_elevation(Dec(sign * int(match.group(2)) * 60 * 60))
            case LX200Commands.SET_HIGHEST_ELEVATION, position:
                match = re.fullmatch(r"([+-]?)(\d{2})\*", position)
                if not match:
                    result = False
                else:
                    sign = -1 if match.group(1) == "-" else 1
                    result = self.set_highest_elevation(Dec(sign * int(match.group(2)) * 60 * 60))

            case LX200Commands.SET_SLEW_TO_GUIDE, _:
                self.set_slew_to_guide()
            case LX200Commands.SET_SLEW_TO_CENTER, _:
                self.set_slew_to_center()
            case LX200Commands.SET_SLEW_TO_FIND, _:
                self.set_slew_to_find()
            case LX200Commands.SET_SLEW_TO_MAX, _:
                self.set_slew_to_max()
            case LX200Commands.GUIDE, data:
                self._last_halt_all = None
                direction = data[0].lower()
                ms = int(data[1:])

                match direction:
                    case "w":
                        self.guide_west(ms)
                    case "e":
                        self.guide_east(ms)
                    case "n":
                        self.guide_north(ms)
                    case "s":
                        self.guide_south(ms)
                    case _:
                        raise RuntimeError(f"Wrong guide direction: {direction}")
            
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

        now = Second.monotonic()
        with self._monitor_lock:
            if cmd == LX200Commands.GUIDE:
                self._last_guide_command = (now, full_command)
            elif cmd not in {LX200Commands.GET_TELECOPE_RA, LX200Commands.GET_TELESCOPE_DEC}:
                self._recent_commands.append((now, full_command))

            result = self._do_handle(cmd, argument, now)

        _logger.info("Get command %s %s(%s)", cmd, cmd.name, argument)

        if isinstance(result, _LX200NotImplementedCommand):
            raise RuntimeError(f"Not implemented LX200 command: {cmd} {cmd.name}({argument})")
        if result is not None:
            _logger.info("Answer command %s %s(%s) -> %s", cmd, cmd.name, argument, result)
        else:
            _logger.warning("Empty responce: %s %s(%s) -> ∅", cmd, cmd.name, argument)

        return result

    def set_minimum_elevation(self, position: Dec) -> bool:
        self._minimum_elevation = position
        return True

    def set_highest_elevation(self, position: Dec) -> bool:
        self._highest_elevation = position
        return True

    def stop(self) -> None:
        pass

    def command_monitor(self) -> dict[str, object]:
        with self._monitor_lock:
            return {
                "recent": list(self._recent_commands),
                "guide": self._last_guide_command,
            }
