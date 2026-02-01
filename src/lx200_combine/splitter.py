from __future__ import annotations

import logging
from enum import StrEnum
from typing import Optional

from lx200.protocol import (
    LX200Command,
    LX200MoveDirection,
    LX200ParseError,
    LX200CommandRequest,
    parse_request,
)
from lx200.server import LX200CommandHandler


class LX200CombineConstants:
    LOGGER_NAME = "lx200.combine"
    RA_ONLY_COMMANDS = {
        LX200Command.GET_RA,
        LX200Command.SET_RA,
        LX200Command.MOVE_EAST,
        LX200Command.MOVE_WEST,
    }
    DEC_ONLY_COMMANDS = {
        LX200Command.GET_DEC,
        LX200Command.SET_DEC,
        LX200Command.MOVE_NORTH,
        LX200Command.MOVE_SOUTH,
    }
    BOTH_COMMANDS = {
        LX200Command.GOTO,
        LX200Command.SYNC,
        LX200Command.RATE_GUIDE,
        LX200Command.RATE_CENTER,
        LX200Command.RATE_FIND,
        LX200Command.RATE_SLEW,
    }
    PRIMARY_COMMANDS = {  # send only to "primary" mount set in __init__
        LX200Command.SET_LOCAL_TIME,
        LX200Command.SET_DATE,
        LX200Command.SET_UTC_OFFSET,
        LX200Command.SET_LATITUDE,
        LX200Command.SET_LONGITUDE,
        LX200Command.GET_LOCAL_TIME,
        LX200Command.GET_DATE,
        LX200Command.GET_DATE_ALT,
        LX200Command.GET_UTC_OFFSET,
        LX200Command.GET_LONGITUDE,
        LX200Command.GET_LATITUDE,
        LX200Command.SET_OBJECT_SIZE,
        LX200Command.GET_DISTANCE,
    }
    COMBINE_COMMANDS = {  # Combine values from both mounts
        LX200Command.GET_TRACKING_RATE,
        LX200Command.GET_SITE_NAME,
    }
    RA_ONLY_DIRECTIONS = {
        LX200MoveDirection.EAST,
        LX200MoveDirection.WEST,
    }
    DEC_ONLY_DIRECTIONS = {
        LX200MoveDirection.NORTH,
        LX200MoveDirection.SOUTH,
    }


class LX200CombineError(Exception):
    pass


class LX200CombineConfigurationError(LX200CombineError):
    pass


class LX200CombineResponseMismatchError(LX200CombineError):
    def __init__(self, command: LX200Command, ra_response: str, dec_response: str) -> None:
        self.command = command
        self.ra_response = ra_response
        self.dec_response = dec_response
        message = (
            "mismatched responses for command "
            f"{command!r}: ra={ra_response!r} dec={dec_response!r}"
        )
        super().__init__(message)


class LX200Route(StrEnum):
    RA = "ra"
    DEC = "dec"
    BOTH = "both"


class LX200Splitter(LX200CommandHandler):
    def __init__(
        self,
        ra_handler: LX200CommandHandler,
        dec_handler: LX200CommandHandler,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        if ra_handler is None or dec_handler is None:
            raise LX200CombineConfigurationError("Both RA and DEC handlers are required.")
        self._ra_handler = ra_handler
        self._dec_handler = dec_handler
        self._log = logger or logging.getLogger(LX200CombineConstants.LOGGER_NAME)
        self._validate_routes()

    def handle_command(self, raw: str) -> str:
        request = parse_request(raw)
        route = self._route_for_request(request)
        self._log.info("splitter route=%s command=%s raw=%r", route.value, request.command, raw)
        if route == LX200Route.RA:
            return self._ra_handler.handle_command(raw)
        if route == LX200Route.DEC:
            return self._dec_handler.handle_command(raw)
        return self._handle_both(raw, request.command)

    def _validate_routes(self) -> None:
        overlap = LX200CombineConstants.RA_ONLY_COMMANDS & LX200CombineConstants.DEC_ONLY_COMMANDS
        if overlap:
            command_list = ",".join(cmd.value for cmd in sorted(overlap, key=lambda cmd: cmd.value))
            raise LX200CombineConfigurationError(
                f"Overlapping RA/DEC routes are not allowed: {command_list}"
            )
        both_overlap = (
            LX200CombineConstants.RA_ONLY_COMMANDS & LX200CombineConstants.BOTH_COMMANDS
        ) | (LX200CombineConstants.DEC_ONLY_COMMANDS & LX200CombineConstants.BOTH_COMMANDS)
        if both_overlap:
            command_list = ",".join(
                cmd.value for cmd in sorted(both_overlap, key=lambda cmd: cmd.value)
            )
            raise LX200CombineConfigurationError(
                f"Commands cannot be both axis-specific and shared: {command_list}"
            )
        if LX200Command.STOP in LX200CombineConstants.RA_ONLY_COMMANDS:
            raise LX200CombineConfigurationError("STOP cannot be a RA-only command.")
        if LX200Command.STOP in LX200CombineConstants.DEC_ONLY_COMMANDS:
            raise LX200CombineConfigurationError("STOP cannot be a DEC-only command.")
        if LX200Command.STOP in LX200CombineConstants.BOTH_COMMANDS:
            raise LX200CombineConfigurationError("STOP cannot be a shared command.")
        union = (
            LX200CombineConstants.RA_ONLY_COMMANDS
            | LX200CombineConstants.DEC_ONLY_COMMANDS
            | LX200CombineConstants.BOTH_COMMANDS
            | {LX200Command.STOP}
        )
        if union != set(LX200Command):
            missing = set(LX200Command) - union
            extras = union - set(LX200Command)
            if missing:
                missing_list = ",".join(
                    cmd.value for cmd in sorted(missing, key=lambda cmd: cmd.value)
                )
                raise LX200CombineConfigurationError(
                    f"Missing route definitions for commands: {missing_list}"
                )
            if extras:
                extra_list = ",".join(cmd.value for cmd in sorted(extras, key=lambda cmd: cmd.value))
                raise LX200CombineConfigurationError(
                    f"Unknown commands configured in routes: {extra_list}"
                )
        direction_overlap = (
            LX200CombineConstants.RA_ONLY_DIRECTIONS & LX200CombineConstants.DEC_ONLY_DIRECTIONS
        )
        if direction_overlap:
            direction_list = ",".join(
                direction.value for direction in sorted(direction_overlap, key=lambda d: d.value)
            )
            raise LX200CombineConfigurationError(
                f"Overlapping stop directions are not allowed: {direction_list}"
            )

    def _route_for_request(self, request: LX200CommandRequest) -> LX200Route:
        if request.command == LX200Command.STOP:
            return self._route_for_stop(request.arg)
        if request.command in LX200CombineConstants.RA_ONLY_COMMANDS:
            return LX200Route.RA
        if request.command in LX200CombineConstants.DEC_ONLY_COMMANDS:
            return LX200Route.DEC
        if request.command in LX200CombineConstants.BOTH_COMMANDS:
            return LX200Route.BOTH
        raise LX200CombineConfigurationError(f"Unhandled command route: {request.command!r}")

    def _route_for_stop(self, arg: Optional[str]) -> LX200Route:
        if arg is None:
            return LX200Route.BOTH
        direction = self._parse_stop_direction(arg)
        if direction in LX200CombineConstants.RA_ONLY_DIRECTIONS:
            return LX200Route.RA
        if direction in LX200CombineConstants.DEC_ONLY_DIRECTIONS:
            return LX200Route.DEC
        raise LX200ParseError(f"unsupported direction: {arg!r}")

    def _parse_stop_direction(self, arg: str) -> LX200MoveDirection:
        try:
            return LX200MoveDirection(arg.lower())
        except ValueError as exc:
            raise LX200ParseError(f"invalid direction: {arg!r}") from exc

    def _handle_both(self, raw: str, command: LX200Command) -> str:
        ra_response = self._ra_handler.handle_command(raw)
        dec_response = self._dec_handler.handle_command(raw)
        if ra_response != dec_response:
            raise LX200CombineResponseMismatchError(command, ra_response, dec_response)
        return ra_response
