
from enum import Enum, StrEnum
import logging
from typing import Any, Callable

from .protocol import AlignmentMode
from lx200.protocols import LX200Hours


class LX200Commands(StrEnum):
    GET_TELECOPE_RA = "GR"
    SET_TELESCOPE_RA = "Sr"


class LX200UnknownCommand(Exception):
    pass


_logger = logging.getLogger("lx200")


class LX200Base:
    def connect(self):
        raise NotImplementedError()
    
    def handle_alignment(self, data: bytes) -> AlignmentMode:
        raise NotImplementedError()

    def handle(self, full_command: str) -> str:
        cmd, argument = full_command[:2], full_command[2:]
        match (cmd, argument):
            case LX200Commands.GET_TELECOPE_RA, _:
                result = self.get_telescope_ra()
            case LX200Commands.SET_TELESCOPE_RA, position:
                result = self.set_telescope_ra(LX200Hours.from_string(position))
            case _:
                raise LX200UnknownCommand(full_command)
            
        if result:
            str_result = str(result)
            _logger.debug("Convert %r -> %s", result, str_result)
            return str(result)
        else:
            raise Exception("Wrong return")

    def get_telescope_ra(self) -> LX200Hours:
        raise NotImplementedError()
    
    def set_telescope_ra(self, position: LX200Hours) -> str:
        raise NotImplementedError()


