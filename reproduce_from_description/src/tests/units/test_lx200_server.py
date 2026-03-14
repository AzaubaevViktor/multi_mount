from __future__ import annotations

import socket
import threading
from types import SimpleNamespace

from lx200.base import ClockSiteState, LX200Base, LX200Handler
from lx200.base_server import _LX200RequestHandler
from sky.combiner import GuideDirection
from sky.physics import Dec, Ha, PointCoordinates


class FakeLX200(LX200Base):
    def __init__(self) -> None:
        self.connected = False
        self.coordinates = PointCoordinates(ra=Ha(1.5), dec=Dec(12.0))

    def connect(self) -> None:
        self.connected = True

    def stop(self) -> None:
        self.connected = False

    def handle_alignment(self) -> str:
        return "P"

    def current_coordinates(self) -> PointCoordinates:
        return self.coordinates

    def sync_telescope(self, coordinates: PointCoordinates) -> None:
        self.coordinates = coordinates

    def slew_to(self, coordinates: PointCoordinates) -> bool:
        self.coordinates = coordinates
        return False

    def set_rate_preset(self, preset: str) -> None:
        return None

    def move(self, direction: GuideDirection) -> None:
        return None

    def halt(self, direction: GuideDirection | None = None) -> None:
        return None

    def guide(self, direction: GuideDirection, milliseconds: int) -> None:
        return None

    def clock_site_state(self) -> ClockSiteState:
        return ClockSiteState()

    def update_clock_site_state(self, **values: str) -> None:
        return None

    def set_highest_elevation(self, value: Dec) -> None:
        return None

    def set_minimum_elevation(self, value: Dec) -> None:
        return None


def test_server_handles_alignment_and_command_framing() -> None:
    fake = FakeLX200()
    fake.connect()
    client, server_socket = socket.socketpair()
    server = SimpleNamespace(handler=LX200Handler(fake), lx200=fake)
    thread = threading.Thread(target=_LX200RequestHandler, args=(server_socket, ("local", 0), server), daemon=True)
    thread.start()

    try:
        client.sendall(b"\x06")
        assert client.recv(16) == b"P"

        client.sendall(b":GR#")
        assert client.recv(32) == b"01:30:00#"
    finally:
        client.close()
        server_socket.close()
        fake.stop()
        thread.join(timeout=1.0)
