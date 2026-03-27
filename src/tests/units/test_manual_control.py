from manual_control import ManualControlConsole


class _FakeLX200:
    def __init__(self) -> None:
        self.commands: list[str] = []

    def handle(self, command: str):
        self.commands.append(command)
        return {
            "GR": "12:34:56",
            "Me": None,
            "Qe": None,
        }.get(command, True)


class _FakeServer:
    def __init__(self) -> None:
        self.host = "localhost"
        self.port = 7624
        self._running = False
        self.last_error = None
        self.start_calls = 0
        self.stop_calls = 0

    def start_background(self) -> bool:
        self.start_calls += 1
        self._running = True
        return True

    def stop(self, join_timeout_s: float = 1.0) -> None:
        del join_timeout_s
        self.stop_calls += 1
        self._running = False

    def is_running(self) -> bool:
        return self._running


def test_manual_console_help_includes_single_axis_examples() -> None:
    console = ManualControlConsole(
        _FakeLX200(),  # type: ignore[arg-type]
        _FakeServer(),  # type: ignore[arg-type]
        lambda: {"ra": True, "dec": False},
    )

    help_text = console.help_text()

    assert "lx200 on" in help_text
    assert ":Me#" in help_text
    assert ":Mw#" in help_text
    assert ":Mn#" not in help_text


def test_manual_console_handles_lx200_toggle_and_status() -> None:
    server = _FakeServer()
    console = ManualControlConsole(
        _FakeLX200(),  # type: ignore[arg-type]
        server,  # type: ignore[arg-type]
        lambda: {"ra": True, "dec": False},
    )

    should_continue, message = console.handle_line("lx200 on")

    assert should_continue is True
    assert server.start_calls == 1
    assert "LX200 server: on" in message

    should_continue, message = console.handle_line("status")

    assert should_continue is True
    assert "RA=up" in message
    assert "DEC=wait" in message

    should_continue, message = console.handle_line("lx200 off")

    assert should_continue is True
    assert server.stop_calls == 1
    assert "LX200 server: off" in message


def test_manual_console_sends_raw_lx200_commands() -> None:
    lx200 = _FakeLX200()
    console = ManualControlConsole(
        lx200,  # type: ignore[arg-type]
        _FakeServer(),  # type: ignore[arg-type]
        lambda: {"ra": True, "dec": True},
    )

    should_continue, message = console.handle_line(":GR#")

    assert should_continue is True
    assert lx200.commands == ["GR"]
    assert message == "GR -> 12:34:56"

    should_continue, message = console.handle_line("Me")

    assert should_continue is True
    assert lx200.commands == ["GR", "Me"]
    assert message == "Me -> ∅"
