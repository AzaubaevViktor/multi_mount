from __future__ import annotations

from typing import Any, Optional, Protocol, Tuple

from ..protocol import LX200Command, LX200Constants, LX200ParseError
from ..server import CommandSpec


def parse_no_arg(arg: Optional[str]) -> Tuple[()]:
    if arg:
        raise LX200ParseError(f"unexpected arg: {arg!r}")
    return ()


def parse_object_size_arg(arg: Optional[str]) -> Tuple[str]:
    if arg is None:
        raise LX200ParseError("missing object size argument")
    return (arg,)


def format_ok(accepted: bool) -> str:
    return LX200Constants.RESPONSE_OK if accepted else LX200Constants.RESPONSE_ERR


def format_distance(value: str) -> str:
    return f"{value}{LX200Constants.TERMINATOR}"


class LX200ObjectBackend(Protocol):
    def set_object_size(self, value: str) -> bool:
        raise NotImplementedError

    def get_distance(self) -> str:
        raise NotImplementedError


class LX200ObjectPlugin:
    def __init__(self, backend: LX200ObjectBackend) -> None:
        self._backend = backend

    def specs(self) -> list[CommandSpec[Any]]:
        return [
            CommandSpec(LX200Command.SET_OBJECT_SIZE, parse_object_size_arg, self.set_object_size, format_ok),
            CommandSpec(LX200Command.GET_DISTANCE, parse_no_arg, self.get_distance, format_distance),
        ]

    def set_object_size(self, value: str) -> bool:
        return self._backend.set_object_size(value)

    def get_distance(self) -> str:
        return self._backend.get_distance()
