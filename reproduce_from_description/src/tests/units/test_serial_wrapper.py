from __future__ import annotations

from collections import deque

from serial_wrapper import wrapper


class FakeSerial:
    def __init__(self, **_: object) -> None:
        self.read_chunks = deque([b"x", b"x", b"=", b"O", b"K", b"\n"])
        self.writes: list[bytes] = []
        self.is_open = True

    @property
    def in_waiting(self) -> int:
        return len(self.read_chunks)

    def read(self, _: int) -> bytes:
        if not self.read_chunks:
            return b""
        return self.read_chunks.popleft()

    def write(self, data: bytes) -> None:
        self.writes.append(data)

    def flush(self) -> None:
        return None

    def close(self) -> None:
        self.is_open = False


def test_search_matches_device_and_description(monkeypatch) -> None:
    monkeypatch.setattr(
        wrapper,
        "list_serial_ports",
        lambda: [
            wrapper.SerialDeviceInfo(device="/dev/cu.usbmodem111", description="TMC2209 controller", hwid="abc"),
            wrapper.SerialDeviceInfo(device="/dev/cu.fake", description="Other", hwid="def"),
        ],
    )

    matches = wrapper.SerialLine.search("TMC2209|usbmodem111")

    assert [match.device for match in matches] == ["/dev/cu.usbmodem111"]


def test_query_discards_noise_before_prefix() -> None:
    line = wrapper.SerialLine(port="/dev/fake", serial_factory=FakeSerial)
    line.connect()

    reply = line.query(b"status\n", response_prefix=b"=", terminator=b"\n")

    assert reply == b"=OK\n"
