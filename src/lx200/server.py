from __future__ import annotations

import dataclasses
import logging
from typing import Any, Callable, Generic, Optional, Protocol, TypeVar

from .protocol import (
    LX200Command,
    LX200ParseError,
    LX200UnsupportedCommandError,
    LX200ValueError,
    parse_request,
)

TResult = TypeVar("TResult")


@dataclasses.dataclass(frozen=True)
class CommandSpec(Generic[TResult]):
    command: LX200Command
    parse: Callable[[Optional[str]], tuple[Any, ...]]
    handler: Callable[..., TResult]
    format: Callable[[TResult], str]


class LX200Plugin(Protocol):
    def specs(self) -> list[CommandSpec[Any]]:
        raise NotImplementedError


class LX200CommandHandler(Protocol):
    def handle_command(self, raw: str) -> str:
        raise NotImplementedError


class LX200Server:
    def __init__(self, plugins: list[LX200Plugin], logger: Optional[logging.Logger] = None) -> None:
        self.log = logger or logging.getLogger("lx200.server")
        self._specs: dict[LX200Command, CommandSpec[Any]] = {}
        for plugin in plugins:
            for spec in plugin.specs():
                if spec.command in self._specs:
                    raise LX200ValueError(f"duplicate handler: {spec.command!r}")
                self._specs[spec.command] = spec

    def handle_command(self, raw: str) -> str:
        request = parse_request(raw)
        self.log.debug("lx200 rx command=%s arg=%r", request.command.name, request.arg)
        spec = self._specs.get(request.command)
        if spec is None:
            raise LX200UnsupportedCommandError(f"unsupported command {request.command!r}")
        try:
            args = spec.parse(request.arg)
        except LX200ParseError:
            raise
        result = spec.handler(*args)
        return spec.format(result)
