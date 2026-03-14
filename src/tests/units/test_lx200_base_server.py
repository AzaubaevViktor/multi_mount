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
        raise KeyboardInterrupt()


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
