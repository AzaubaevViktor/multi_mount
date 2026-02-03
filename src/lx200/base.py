
from enum import Enum, StrEnum
import logging
from typing import Any, Callable


from .protocol import AlignmentMode
from lx200.protocols import LX200Hours, LX200Dec


class LX200Commands(StrEnum):
    GET_TELECOPE_RA = "GR"
    SET_TELESCOPE_RA = "Sr"

    GET_TELESCOPE_DEC = "GD"
    SET_TELESCOPE_DEC = "Sd"

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
    SET_CURR_SITE_LATITUDE = "St"
    SET_UTC = "SG"

    SET_LOCAL_TIME = "SL"
    SET_LOCAL_DATE = "SC"

    SET_MINIMUM_ELEVATION = "Sh"
    SET_HIGHEST_ELEVATION = "So"

    SET_SLEW_TO_FIND = "RM"


class LX200UnknownCommand(Exception):
    pass


_logger = logging.getLogger("lx200")


class LX200Base:
    def connect(self):
        raise NotImplementedError()
    
    def handle_alignment(self, data: bytes) -> AlignmentMode:
        raise NotImplementedError()

    def handle(self, full_command: str) -> str | None:
        cmd, argument = full_command[:2], full_command[2:]
        _logger.info("Get command %s(%s)", cmd, argument)

        match (cmd, argument):
            case LX200Commands.GET_TELECOPE_RA, _:
                result = self.get_telescope_ra()
            case LX200Commands.SET_TELESCOPE_RA, position:
                result = self.set_telescope_ra(LX200Hours.from_string(position))
            case LX200Commands.GET_TELESCOPE_DEC, _:
                result = self.get_telescope_dec()
            case LX200Commands.SET_TELESCOPE_DEC, position:
                result = self.set_telescope_dec(LX200Dec.from_string(position))
            case LX200Commands.GET_CALENDAR_FORMAT, _:
                result = self.get_calendar_format()
            case LX200Commands.GET_SITE1_NAME, _:
                result = self.get_site1_name()
            case LX200Commands.GET_TRACKING_RATE, _:
                result = self.get_tracking_rate()
            case LX200Commands.GET_CURRENT_SITE_LATITUDE, _:
                result = "+10*10"
            case LX200Commands.GET_CURRENT_SITE_LONGITUDE, _:
                result = "+011*11"
            case LX200Commands.GET_UTC_OFFSET_TIME, _:
                result = "-4"
            case LX200Commands.GET_LOCAL_TIME, _:
                result = "00:00:00"
            case LX200Commands.GET_CURRENT_DATE, _:
                result = "01/01/2000"
            case _:
                _logger.warning("Wrong command: %s", full_command)
                result = None
                # raise LX200UnknownCommand(full_command)
        
        if result is not None:
            if isinstance(result, bool):
                str_result = "1" if result else "0"
            else:
                str_result = str(result)
            _logger.debug("Convert %r -> %s", result, str_result)
            _logger.info("Answer command %s(%s) -> %s", cmd, argument, str_result)

            return str_result
        else:
            return None

    def get_telescope_ra(self) -> LX200Hours:
        raise NotImplementedError()
    
    def set_telescope_ra(self, position: LX200Hours) -> bool:
        raise NotImplementedError()
    
    def get_telescope_dec(self) -> LX200Dec:
        raise NotImplementedError()
    
    def set_telescope_dec(self, position: LX200Dec) -> bool:
        raise NotImplementedError()
    
    def get_calendar_format(self) -> str:
        return "24"
    
    def get_site1_name(self) -> str:
        return "base_lx200"
    
    def get_tracking_rate(self) -> str:
        return "60.0"
