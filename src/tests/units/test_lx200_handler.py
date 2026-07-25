import pytest

from lx200.base import (
    LX200BadCommandError,
    LX200Commands,
    LX200Handler,
    LX200SlewResult,
    LX200UnknownCommandError,
)
from sky.physics import Dec, Ha, SkyDirection


class _RecordingLX200(LX200Handler):
    def __init__(self, slew_accepted: bool = True) -> None:
        super().__init__()
        self.calls: list[str] = []
        self.slew_accepted = slew_accepted
        self.synced: tuple[Ha, Dec] | None = None
        self.slewed_to: tuple[Ha, Dec] | None = None
        self.guides: list[tuple[str, int]] = []

    def get_telescope_ra(self) -> Ha:
        return Ha(0)

    def sync_telescope(self, ra: Ha, dec: Dec) -> bool:
        self.synced = (ra, dec)
        return True

    def get_telescope_dec(self) -> Dec:
        return Dec(0)

    def slew_to(self, ra: Ha, dec: Dec) -> bool:
        self.calls.append("slew_to")
        self.slewed_to = (ra, dec)
        return self.slew_accepted

    def move_east(self) -> bool:
        self.calls.append("move_east")
        return True

    def move_north(self) -> bool:
        self.calls.append("move_north")
        return True

    def move_south(self) -> bool:
        self.calls.append("move_south")
        return True

    def move_west(self) -> bool:
        self.calls.append("move_west")
        return True

    def halt_all(self) -> bool:
        self.calls.append("halt_all")
        return True

    def stop_all(self) -> bool:
        self.calls.append("stop_all")
        return True

    def halt_east(self) -> bool:
        self.calls.append("halt_east")
        return True

    def halt_north(self) -> bool:
        self.calls.append("halt_north")
        return True

    def halt_south(self) -> bool:
        self.calls.append("halt_south")
        return True

    def halt_west(self) -> bool:
        self.calls.append("halt_west")
        return True

    def guide_east(self, ms: int) -> None:
        self.guides.append(("east", ms))

    def guide_north(self, ms: int) -> None:
        self.guides.append(("north", ms))

    def guide_south(self, ms: int) -> None:
        self.guides.append(("south", ms))

    def guide_west(self, ms: int) -> None:
        self.guides.append(("west", ms))


def test_repeated_halt_all_never_stops_tracking() -> None:
    handler = _RecordingLX200()

    handler.handle("Q")
    handler.handle("Q")

    assert handler.calls == ["halt_all", "halt_all"]


def test_motion_does_not_change_halt_all_behavior() -> None:
    handler = _RecordingLX200()

    handler.handle("Q")
    handler.handle("Me")
    handler.handle("Q")

    assert handler.calls == ["halt_all", "move_east", "halt_all"]


def test_directional_halt_without_manual_move_falls_back_to_halt_all() -> None:
    handler = _RecordingLX200()

    handler.handle("Qw")
    handler.handle("Qs")

    assert handler.calls == ["halt_all", "halt_all"]


def test_ra_halt_commands_support_lx200_client_mapping() -> None:
    handler = _RecordingLX200()

    handler.handle("Me")
    handler.handle("Qw")
    handler.handle("Mw")
    handler.handle("Qe")

    assert handler.calls == [
        "move_east",
        "halt_east",
        "move_west",
        "halt_west",
    ]


def test_ra_halt_commands_keep_legacy_same_direction_mapping() -> None:
    handler = _RecordingLX200()

    handler.handle("Me")
    handler.handle("Qe")
    handler.handle("Mw")
    handler.handle("Qw")

    assert handler.calls == [
        "move_east",
        "halt_east",
        "move_west",
        "halt_west",
    ]


def test_new_ra_manual_move_replaces_stale_opposite_direction() -> None:
    handler = _RecordingLX200()

    handler.handle("Mw")
    handler.handle("Me")
    handler.handle("Qw")

    assert handler.calls == [
        "move_west",
        "move_east",
        "halt_east",
    ]


def test_slew_clears_stale_manual_directions_before_directional_halt() -> None:
    handler = _RecordingLX200()

    handler.handle("Me")
    handler.handle("MS")
    handler.handle("Qw")

    assert handler.calls == [
        "move_east",
        "slew_to",
        "halt_all",
    ]


class TestSlewResult:
    """`:MS#` is a status, not an acknowledgement (ARCHITECTURE.md, "Ограничения").

    The handler used to answer a hardcoded `False` and throw away whatever
    `slew_to()` returned. `False` serialises to `"0"`, and in LX200 `0` means
    *the slew was accepted* -- so a mount that refused the slew still reported
    success to the client.
    """

    def test_an_accepted_slew_is_a_zero_with_no_terminator(self) -> None:
        handler = _RecordingLX200(slew_accepted=True)

        result = handler.handle("MS")

        assert isinstance(result, LX200SlewResult)
        assert result.accepted is True
        assert result.to_wire() == "0"

    def test_a_refused_slew_is_a_non_zero_status_with_a_reason(self) -> None:
        handler = _RecordingLX200(slew_accepted=False)

        result = handler.handle("MS")

        assert isinstance(result, LX200SlewResult)
        assert result.accepted is False
        assert result.code != 0
        assert result.to_wire().startswith("1")
        assert result.to_wire().endswith("#")

    def test_the_slew_uses_the_target_the_client_set(self) -> None:
        handler = _RecordingLX200()

        handler.handle("Sr12:34:56")
        handler.handle("Sd+41*22:33")
        handler.handle("MS")

        assert handler.slewed_to == (Ha.from_string("12:34:56"), Dec.from_string("+41*22:33"))

    def test_the_codes_the_protocol_reserves_are_expressible(self) -> None:
        assert LX200SlewResult.accept().to_wire() == "0"
        assert LX200SlewResult.reject("below horizon").to_wire() == "1below horizon#"
        assert LX200SlewResult.above_limit("above limit").to_wire() == "2above limit#"


class TestMalformedArgumentsDoNotEscapeAsRawErrors:
    """PLAN.md #30: a malformed `Mg` used to kill the LX200 server's client thread.

    Every payload below is reachable from the network, so it has to come back as
    an `LX200BadCommandError` the socket loop knows how to answer -- not as a raw
    IndexError/ValueError/RuntimeError out of the connection thread.
    """

    @pytest.mark.parametrize(
        "command",
        [
            "Mg",  # IndexError: data[0] on an empty payload
            "Mgw",  # ValueError: int("")
            "Mgx1000",  # RuntimeError: "Wrong guide direction"
            "Mgw12.5",
            "Mgw 1000",
        ],
    )
    def test_broken_guide_pulses_raise_a_typed_error(self, command: str) -> None:
        handler = _RecordingLX200()

        with pytest.raises(LX200BadCommandError) as raised:
            handler.handle(command)

        # `Mg` is unanswered in LX200, so there is nothing to send back.
        assert raised.value.answer is None
        assert handler.guides == []

    @pytest.mark.parametrize("command", ["Sr", "Sr99", "Sr12:34", "Srhh:mm:ss"])
    def test_broken_target_ra_is_rejected_with_a_zero(self, command: str) -> None:
        handler = _RecordingLX200()

        with pytest.raises(LX200BadCommandError) as raised:
            handler.handle(command)

        assert raised.value.answer is False

    @pytest.mark.parametrize("command", ["Sd", "Sd+41", "Sd41*22:33"])
    def test_broken_target_dec_is_rejected_with_a_zero(self, command: str) -> None:
        handler = _RecordingLX200()

        with pytest.raises(LX200BadCommandError) as raised:
            handler.handle(command)

        assert raised.value.answer is False

    @pytest.mark.parametrize("command", ["Sh", "Sh1*", "Shxx*", "So+9*"])
    def test_broken_elevation_limits_are_rejected_with_a_zero(self, command: str) -> None:
        handler = _RecordingLX200()

        with pytest.raises(LX200BadCommandError) as raised:
            handler.handle(command)

        assert raised.value.answer is False

    def test_an_unknown_command_raises_the_typed_error_too(self) -> None:
        handler = _RecordingLX200()

        with pytest.raises(LX200UnknownCommandError) as raised:
            handler.handle("ZZ")

        assert isinstance(raised.value, LX200BadCommandError)
        assert raised.value.answer is None

    @pytest.mark.parametrize(
        "command,expected",
        [
            ("MgW1000", ("west", 1000)),
            ("Mge0500", ("east", 500)),
            ("Mgn1", ("north", 1)),
            ("Mgs99999", ("south", 99999)),
        ],
    )
    def test_well_formed_guide_pulses_still_work(self, command: str, expected) -> None:
        handler = _RecordingLX200()

        handler.handle(command)

        assert handler.guides == [expected]


class TestSiteCoordinatesAnswerTheSameWay:
    """PLAN.md #30: `Sg` answered 1 and `St` answered 0 for the same kind of payload.

    Neither stored anything either, so `Gt`/`Gg` contradicted whatever the client
    had just set.
    """

    def test_both_setters_acknowledge_a_valid_value(self) -> None:
        handler = _RecordingLX200()

        assert handler.handle("St+55*45") is True
        assert handler.handle("Sg+037*36") is True

    def test_what_was_set_is_what_is_read_back(self) -> None:
        handler = _RecordingLX200()

        handler.handle("St-33*56")
        handler.handle("Sg+018*28")

        assert handler.handle("Gt") == "-33*56"
        assert handler.handle("Gg") == "+018*28"

    def test_defaults_are_unchanged_until_a_client_sets_them(self) -> None:
        handler = _RecordingLX200()

        assert handler.handle("Gt") == "+00*00"
        assert handler.handle("Gg") == "+000*00"

    @pytest.mark.parametrize("command", ["St", "St+5*45", "St+55*4", "Sg+37*36", "Sgxxx*yy", "Sg"])
    def test_both_setters_reject_a_broken_value_the_same_way(self, command: str) -> None:
        handler = _RecordingLX200()

        with pytest.raises(LX200BadCommandError) as raised:
            handler.handle(command)

        assert raised.value.answer is False


class TestClientSessionState:
    """Target RA/DEC and the manual-move list belong to one conversation, not to the process."""

    def test_a_new_session_forgets_the_previous_target(self) -> None:
        handler = _RecordingLX200()
        handler.handle("Sr12:34:56")
        handler.handle("Sd+41*22:33")

        handler.begin_client_session("second client")
        handler.handle("MS")

        assert handler.slewed_to == (Ha(0), Dec(0))

    def test_a_new_session_forgets_the_previous_manual_move(self) -> None:
        handler = _RecordingLX200()
        handler.handle("Me")

        handler.begin_client_session()
        handler.handle("Qw")

        # Without the reset, `Qw` would map onto the *previous* client's east move.
        assert handler.calls == ["move_east", "halt_all"]

    def test_a_new_session_keeps_the_site_and_limits(self) -> None:
        """Site and elevation limits describe the mount's location, not the conversation."""
        handler = _RecordingLX200()
        handler.handle("St+55*45")
        handler.handle("So+80*")

        handler.begin_client_session()

        assert handler.handle("Gt") == "+55*45"
        assert handler._highest_elevation == Dec(80 * 60 * 60)


def test_parsing_a_command_has_no_side_effects() -> None:
    """The split that makes the rest of this file possible: naming a command != running it."""
    handler = _RecordingLX200()

    assert handler.parse("Sr12:34:56") == (LX200Commands.SET_TELESCOPE_RA, "12:34:56")
    assert handler.parse("Q") == (LX200Commands.HALT_ALL, "")
    assert handler.calls == []
    assert handler._target_ra == Ha(0)

    with pytest.raises(LX200UnknownCommandError):
        handler.parse("ZZ")


def test_halt_direction_still_reaches_the_right_axis_after_a_guide_pulse() -> None:
    handler = _RecordingLX200()

    handler.handle("Mn")
    handler.handle("Mgs0500")
    handler.handle("Qn")

    assert handler.calls == ["move_north", "halt_north"]
    assert handler.guides == [("south", 500)]
    assert SkyDirection.NORTH not in handler._manual_move_directions


def test_the_slew_status_reads_as_words_for_humans_and_as_lx200_on_the_wire() -> None:
    """The manual console and the log print `str(...)`; only the socket sees `to_wire()`."""
    assert str(LX200SlewResult.accept()) == "accepted"
    assert "below horizon" in str(LX200SlewResult.reject("below horizon"))
    assert LX200SlewResult.reject("below horizon").to_wire() == "1below horizon#"
