
from enum import Enum, StrEnum
from typing import Any, Callable

from lx200.protocols import LX200Hours


class LX200Commands(StrEnum):
    GET_TELECOPE_RA = "GR"


class LX200UnknownCommand(Exception):
    pass


class LX200Base:
    def do_handle(self, full_command: str) -> str:
        cmd, argument = full_command[:2], full_command[2:]
        match (cmd, argument):
            case LX200Commands.GET_TELECOPE_RA:
                result = self.get_telescope_ra()
            case _:
                raise LX200UnknownCommand(full_command)
            
        if result:
            return str(result)
        else:
            raise Exception("Wrong return")

    def get_telescope_ra(self) -> LX200Hours:
        raise NotImplementedError()


