import contextlib
import socket as socket_module
import threading as threading_module
import time

import pytest

import lx200.base_server as base_server_module
from lx200.base import LX200Handler
from lx200.base_server import LX200SimpleServer
from sky.physics import Dec, Ha


class _RecordingLX200(LX200Handler):
    def __init__(self) -> None:
        super().__init__()
        self.connect_calls = 0

    def connect(self) -> None:
        self.connect_calls += 1
        super().connect()

    def get_telescope_ra(self) -> Ha:
        return Ha(0)

    def sync_telescope(self, ra: Ha, dec: Dec) -> bool:
        return True

    def get_telescope_dec(self) -> Dec:
        return Dec(0)

    def slew_to(self, ra: Ha, dec: Dec) -> bool:
        return True

    def move_east(self) -> bool:
        return True

    def move_north(self) -> bool:
        return True

    def move_south(self) -> bool:
        return True

    def move_west(self) -> bool:
        return True

    def halt_all(self) -> bool:
        return True

    def halt_east(self) -> bool:
        return True

    def halt_north(self) -> bool:
        return True

    def halt_south(self) -> bool:
        return True

    def halt_west(self) -> bool:
        return True

    def guide_east(self, ms: int) -> None:
        return None

    def guide_north(self, ms: int) -> None:
        return None

    def guide_south(self, ms: int) -> None:
        return None

    def guide_west(self, ms: int) -> None:
        return None


class _FakeServerSocket:
    def __init__(self) -> None:
        self.closed = False

    def __enter__(self) -> "_FakeServerSocket":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        return None

    def setsockopt(self, level: int, optname: int, value: int) -> None:
        return None

    def bind(self, address: tuple[str, int]) -> None:
        self.address = address

    def listen(self, backlog: int) -> None:
        self.backlog = backlog

    def accept(self):
        if self.closed:
            raise OSError("socket closed")
        raise KeyboardInterrupt()

    def close(self) -> None:
        self.closed = True


def test_server_connects_lx200_when_disconnected(monkeypatch) -> None:
    handler = _RecordingLX200()
    server = LX200SimpleServer(handler)

    monkeypatch.setattr(base_server_module.socket, "socket", lambda *args, **kwargs: _FakeServerSocket())

    with pytest.raises(KeyboardInterrupt):
        server.serve_forever()

    assert handler.connect_calls == 1
    assert handler.is_connected() is True


def test_server_skips_connect_when_lx200_already_connected(monkeypatch) -> None:
    handler = _RecordingLX200()
    handler.connect()
    server = LX200SimpleServer(handler)

    monkeypatch.setattr(base_server_module.socket, "socket", lambda *args, **kwargs: _FakeServerSocket())

    with pytest.raises(KeyboardInterrupt):
        server.serve_forever()

    assert handler.connect_calls == 1


class _BlockingFakeServerSocket(_FakeServerSocket):
    def accept(self):
        while not self.closed:
            time.sleep(0.01)
        raise OSError("socket closed")


def test_server_can_run_in_background_and_stop(monkeypatch) -> None:
    handler = _RecordingLX200()
    server = LX200SimpleServer(handler)
    fake_socket = _BlockingFakeServerSocket()

    monkeypatch.setattr(base_server_module.socket, "socket", lambda *args, **kwargs: fake_socket)

    assert server.start_background() is True

    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        if server.is_running():
            break
        time.sleep(0.01)
    else:
        raise AssertionError("server did not start in background")

    server.stop(join_timeout_s=0.5)

    assert server.is_running() is False
    assert fake_socket.closed is True


# ---------------------------------------------------------------------------
# Connection-level behaviour, over a real socket pair. `_handle_client` is the
# unit under test: it is what a client actually talks to, and every defect below
# is only visible at that level.
# ---------------------------------------------------------------------------


class _SlewRefusingLX200(_RecordingLX200):
    def slew_to(self, ra: Ha, dec: Dec) -> bool:
        return False


@contextlib.contextmanager
def _client_of(server: LX200SimpleServer):
    """A live client socket wired to `server._handle_client` in its own thread."""
    client, server_side = socket_module.socketpair()
    client.settimeout(2.0)
    thread = threading_module.Thread(target=server._handle_client, args=(server_side,), daemon=True)
    thread.start()
    try:
        yield client, thread
    finally:
        client.close()
        thread.join(2.0)


def _send(client: socket_module.socket, text: str) -> None:
    client.sendall(text.encode("ascii"))


def _read(client: socket_module.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        data = client.recv(size - len(chunks))
        if not data:
            break
        chunks.extend(data)
    return bytes(chunks)


def test_a_malformed_guide_pulse_does_not_kill_the_connection() -> None:
    """PLAN.md #30: `:Mg#` raised IndexError straight out of the socket loop."""
    server = LX200SimpleServer(_RecordingLX200())

    with _client_of(server) as (client, thread):
        _send(client, ":Mg#")
        _send(client, ":GR#")

        assert _read(client, 9) == b"00:00:00#"
        assert thread.is_alive()


@pytest.mark.parametrize("bad_command", [":Mg#", ":Mgw#", ":Mgx1000#", ":ZZ#", ":#"])
def test_no_unanswered_bad_command_ends_the_session(bad_command: str) -> None:
    """These carry no LX200 answer at all, so the session must simply carry on."""
    server = LX200SimpleServer(_RecordingLX200())

    with _client_of(server) as (client, thread):
        _send(client, bad_command)
        _send(client, ":GR#")

        assert _read(client, 9) == b"00:00:00#"
        assert thread.is_alive()


@pytest.mark.parametrize("bad_command", [":Sr99#", ":Sdxx#", ":Sh1*#", ":St+5*45#"])
def test_a_rejected_set_command_answers_zero_and_the_session_carries_on(bad_command: str) -> None:
    server = LX200SimpleServer(_RecordingLX200())

    with _client_of(server) as (client, thread):
        _send(client, bad_command)
        assert _read(client, 1) == b"0"

        _send(client, ":Sr12:34:56#")
        assert _read(client, 1) == b"1"
        assert thread.is_alive()


def test_an_accepted_slew_answers_a_bare_zero() -> None:
    server = LX200SimpleServer(_RecordingLX200())

    with _client_of(server) as (client, _thread):
        _send(client, ":MS#")
        _send(client, ":GR#")

        # `0` for the slew, then GR's own answer: no terminator sneaked in after the 0.
        assert _read(client, 10) == b"0" + b"00:00:00#"


def test_a_refused_slew_answers_a_non_zero_status_with_a_reason() -> None:
    """The defect: `MS` used to answer a hardcoded `0` -- "slew accepted" -- either way."""
    server = LX200SimpleServer(_SlewRefusingLX200())

    with _client_of(server) as (client, _thread):
        _send(client, ":MS#")

        answer = _read(client, 1)
        assert answer == b"1"
        rest = bytearray()
        while not rest.endswith(b"#"):
            rest.extend(client.recv(64))
        assert b"refused" in bytes(rest)


def test_a_second_client_is_refused_while_the_first_holds_the_mount() -> None:
    """Two clients on one handler overwrite each other's target and halt each other's axes."""
    server = LX200SimpleServer(_RecordingLX200())

    with _client_of(server) as (first, first_thread):
        _send(first, ":GR#")
        assert _read(first, 9) == b"00:00:00#"

        with _client_of(server) as (second, second_thread):
            second_thread.join(2.0)
            assert second_thread.is_alive() is False
            assert second.recv(16) == b""  # refused, socket closed
            assert server.refused_clients == 1

        # ... and the first client is untouched.
        _send(first, ":GR#")
        assert _read(first, 9) == b"00:00:00#"
        assert first_thread.is_alive()


def test_the_slot_is_released_so_the_next_client_gets_in() -> None:
    server = LX200SimpleServer(_RecordingLX200())

    with _client_of(server) as (first, _thread):
        _send(first, ":GR#")
        assert _read(first, 9) == b"00:00:00#"

    with _client_of(server) as (second, _thread):
        _send(second, ":GR#")
        assert _read(second, 9) == b"00:00:00#"

    assert server.refused_clients == 0


def test_monitor_reports_tcp_peer_and_clears_it_on_disconnect() -> None:
    server = LX200SimpleServer(_RecordingLX200())
    assert server.monitor()["client_connected"] is False
    assert server.monitor()["client_address"] is None

    with socket_module.socket(socket_module.AF_INET, socket_module.SOCK_STREAM) as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        listener.settimeout(2.0)
        for _ in range(2):
            with socket_module.create_connection(listener.getsockname(), timeout=2.0) as client:
                accepted, peer = listener.accept()
                thread = threading_module.Thread(target=server._handle_client, args=(accepted,), daemon=True)
                thread.start()
                try:
                    _send(client, ":GR#")
                    assert _read(client, 9) == b"00:00:00#"
                    snapshot = server.monitor()
                    assert snapshot["client_connected"] is True
                    assert snapshot["client_address"] == {"host": peer[0], "port": peer[1]}

                    with _client_of(server) as (second, second_thread):
                        second_thread.join(2.0)
                        assert not second_thread.is_alive()
                        assert second.recv(16) == b""
                    assert server.monitor()["client_address"] == snapshot["client_address"]
                finally:
                    client.shutdown(socket_module.SHUT_RDWR)
                    thread.join(2.0)
                    assert not thread.is_alive()

            assert server.monitor()["client_connected"] is False
            assert server.monitor()["client_address"] is None


def test_a_new_client_does_not_inherit_the_previous_client_target() -> None:
    handler = _RecordingLX200()
    server = LX200SimpleServer(handler)

    with _client_of(server) as (first, _thread):
        _send(first, ":Sr12:34:56#")
        assert _read(first, 1) == b"1"

    with _client_of(server) as (second, _thread):
        _send(second, ":GR#")
        assert _read(second, 9) == b"00:00:00#"

    assert handler._target_ra == Ha(0)


def test_a_garbled_frame_does_not_strand_the_commands_behind_it() -> None:
    """`break` used to abandon the rest of the buffer, and its client's answers with it."""
    server = LX200SimpleServer(_RecordingLX200(), buffer_size=64)

    with _client_of(server) as (client, _thread):
        client.sendall(b"garbage#:GR#")

        assert _read(client, 9) == b"00:00:00#"
