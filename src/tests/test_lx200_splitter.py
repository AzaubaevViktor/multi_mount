from __future__ import annotations

import dataclasses

import pytest

from lx200.protocol import LX200ParseError
from lx200_combine import (
    LX200CombineResponseMismatchError,
    LX200Splitter,
)


class SplitterTestConstants:
    CMD_GET_RA = ":GR#"
    CMD_GET_DEC = ":GD#"
    CMD_GOTO = ":MS#"
    CMD_GET_SITE_NAME = ":GM#"
    CMD_STOP_ALL = ":Q#"
    CMD_STOP_EAST = ":Qe#"
    CMD_STOP_NORTH = ":Qn#"
    CMD_STOP_INVALID = ":Qx#"
    RESP_RA = "RA"
    RESP_DEC = "DEC"
    RESP_OK = "0"
    RESP_DEFAULT = ""
    RESP_SITE_NAME_RA = "RA-MOUNT#"
    RESP_SITE_NAME_DEC = "DEC-MOUNT#"
    RESP_SITE_NAME_COMBINED = "Combine mount RA:RA-MOUNT DEC:DEC-MOUNT#"
    EMPTY_CALLS = []


@dataclasses.dataclass
class SplitterHandlerStub:
    name: str
    responses: dict[str, str]
    default_response: str
    calls: list[str] = dataclasses.field(default_factory=list)

    def handle_command(self, raw: str) -> str:
        self.calls.append(raw)
        return self.responses.get(raw, self.default_response)


def test_routes_ra_command_to_ra_handler() -> None:
    ra_handler = SplitterHandlerStub(
        name="ra",
        responses={SplitterTestConstants.CMD_GET_RA: SplitterTestConstants.RESP_RA},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    dec_handler = SplitterHandlerStub(
        name="dec",
        responses={},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    splitter = LX200Splitter(ra_handler, dec_handler)

    result = splitter.handle_command(SplitterTestConstants.CMD_GET_RA)

    assert result == SplitterTestConstants.RESP_RA
    assert ra_handler.calls == [SplitterTestConstants.CMD_GET_RA]
    assert dec_handler.calls == SplitterTestConstants.EMPTY_CALLS


def test_routes_dec_command_to_dec_handler() -> None:
    ra_handler = SplitterHandlerStub(
        name="ra",
        responses={},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    dec_handler = SplitterHandlerStub(
        name="dec",
        responses={SplitterTestConstants.CMD_GET_DEC: SplitterTestConstants.RESP_DEC},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    splitter = LX200Splitter(ra_handler, dec_handler)

    result = splitter.handle_command(SplitterTestConstants.CMD_GET_DEC)

    assert result == SplitterTestConstants.RESP_DEC
    assert ra_handler.calls == SplitterTestConstants.EMPTY_CALLS
    assert dec_handler.calls == [SplitterTestConstants.CMD_GET_DEC]


def test_routes_both_commands_and_returns_shared_response() -> None:
    ra_handler = SplitterHandlerStub(
        name="ra",
        responses={SplitterTestConstants.CMD_GOTO: SplitterTestConstants.RESP_OK},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    dec_handler = SplitterHandlerStub(
        name="dec",
        responses={SplitterTestConstants.CMD_GOTO: SplitterTestConstants.RESP_OK},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    splitter = LX200Splitter(ra_handler, dec_handler)

    result = splitter.handle_command(SplitterTestConstants.CMD_GOTO)

    assert result == SplitterTestConstants.RESP_OK
    assert ra_handler.calls == [SplitterTestConstants.CMD_GOTO]
    assert dec_handler.calls == [SplitterTestConstants.CMD_GOTO]


def test_routes_stop_command_to_both_when_no_direction() -> None:
    ra_handler = SplitterHandlerStub(
        name="ra",
        responses={SplitterTestConstants.CMD_STOP_ALL: SplitterTestConstants.RESP_DEFAULT},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    dec_handler = SplitterHandlerStub(
        name="dec",
        responses={SplitterTestConstants.CMD_STOP_ALL: SplitterTestConstants.RESP_DEFAULT},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    splitter = LX200Splitter(ra_handler, dec_handler)

    result = splitter.handle_command(SplitterTestConstants.CMD_STOP_ALL)

    assert result == SplitterTestConstants.RESP_DEFAULT
    assert ra_handler.calls == [SplitterTestConstants.CMD_STOP_ALL]
    assert dec_handler.calls == [SplitterTestConstants.CMD_STOP_ALL]


def test_routes_stop_command_by_direction() -> None:
    ra_handler = SplitterHandlerStub(
        name="ra",
        responses={SplitterTestConstants.CMD_STOP_EAST: SplitterTestConstants.RESP_DEFAULT},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    dec_handler = SplitterHandlerStub(
        name="dec",
        responses={SplitterTestConstants.CMD_STOP_NORTH: SplitterTestConstants.RESP_DEFAULT},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    splitter = LX200Splitter(ra_handler, dec_handler)

    result_east = splitter.handle_command(SplitterTestConstants.CMD_STOP_EAST)
    result_north = splitter.handle_command(SplitterTestConstants.CMD_STOP_NORTH)

    assert result_east == SplitterTestConstants.RESP_DEFAULT
    assert result_north == SplitterTestConstants.RESP_DEFAULT
    assert ra_handler.calls == [SplitterTestConstants.CMD_STOP_EAST]
    assert dec_handler.calls == [SplitterTestConstants.CMD_STOP_NORTH]


def test_raises_on_mismatched_responses() -> None:
    ra_handler = SplitterHandlerStub(
        name="ra",
        responses={SplitterTestConstants.CMD_GOTO: SplitterTestConstants.RESP_OK},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    dec_handler = SplitterHandlerStub(
        name="dec",
        responses={SplitterTestConstants.CMD_GOTO: SplitterTestConstants.RESP_DEC},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    splitter = LX200Splitter(ra_handler, dec_handler)

    with pytest.raises(LX200CombineResponseMismatchError):
        splitter.handle_command(SplitterTestConstants.CMD_GOTO)

    assert ra_handler.calls == [SplitterTestConstants.CMD_GOTO]
    assert dec_handler.calls == [SplitterTestConstants.CMD_GOTO]


def test_stop_with_invalid_direction_raises_parse_error() -> None:
    ra_handler = SplitterHandlerStub(
        name="ra",
        responses={},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    dec_handler = SplitterHandlerStub(
        name="dec",
        responses={},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    splitter = LX200Splitter(ra_handler, dec_handler)

    with pytest.raises(LX200ParseError):
        splitter.handle_command(SplitterTestConstants.CMD_STOP_INVALID)

    assert ra_handler.calls == SplitterTestConstants.EMPTY_CALLS
    assert dec_handler.calls == SplitterTestConstants.EMPTY_CALLS


def test_combines_site_name_from_both_mounts() -> None:
    ra_handler = SplitterHandlerStub(
        name="ra",
        responses={SplitterTestConstants.CMD_GET_SITE_NAME: SplitterTestConstants.RESP_SITE_NAME_RA},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    dec_handler = SplitterHandlerStub(
        name="dec",
        responses={SplitterTestConstants.CMD_GET_SITE_NAME: SplitterTestConstants.RESP_SITE_NAME_DEC},
        default_response=SplitterTestConstants.RESP_DEFAULT,
    )
    splitter = LX200Splitter(ra_handler, dec_handler)

    result = splitter.handle_command(SplitterTestConstants.CMD_GET_SITE_NAME)

    assert result == SplitterTestConstants.RESP_SITE_NAME_COMBINED
    assert ra_handler.calls == [SplitterTestConstants.CMD_GET_SITE_NAME]
    assert dec_handler.calls == [SplitterTestConstants.CMD_GET_SITE_NAME]
