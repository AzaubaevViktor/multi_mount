"""Transport-level chaos for the mount simulator (PLAN.md §3, stage 3b).

The simulator already routes every host<->device byte through
:class:`sim.fake_serial.Transport` and can make the device vanish with
:meth:`sim.fake_serial.FakeSerial.unplug`. This module is the only thing built
on top of those seams: a seeded generator of the three failures a USB-serial
line really produces, wired into one ``Transport`` that a test hands to
``SimSerialLine``.

What is modelled
----------------
- **byte loss** in either direction (``lose_write_byte`` / ``lose_read_byte``),
  per byte;
- **short transfer** (``short_write`` / ``short_read``): only a prefix of the
  chunk makes it. On the write side this is a real pyserial partial write —
  ``FakeSerial.write`` then reports the shorter count, which nobody upstream
  reads. On the read side the tail is *lost*, not deferred: the fake port has no
  redelivery queue, and a lost tail is exactly the truncated ``'=1'`` answer
  seen in the March logs (PLAN.md §2.4);
- **unplug** (``unplug``): the device disappears mid-transfer, the bytes of that
  very transfer included. ``is_open`` stays true, so the host only learns about
  it on the next syscall — the P2 failure mode.

What is deliberately *not* modelled: bit flips inside a byte. USB already
CRC-checks every packet, so on this hardware corruption shows up as missing
bytes (buffer overrun, timeout, unplug), not as altered ones. Neither device
protocol carries a checksum, so injecting flipped bytes would only prove the
already-known fact that a mangled-but-well-formed frame is undetectable.

Reproducibility
---------------
Every decision comes from one ``random.Random(seed)`` owned by :class:`Chaos`,
consumed in a fixed order (unplug -> per-byte loss -> short transfer). The same
seed and profile therefore replay byte for byte, which :attr:`Chaos.trace`
records for comparison, and :attr:`Chaos.stats` counts what actually bit — a
chaos run that damaged nothing is a green test that proves nothing.
"""

import random
from dataclasses import dataclass

from sim.fake_serial import SimSerialLine, Transport


@dataclass(frozen=True)
class ChaosProfile:
    """Fault probabilities. All-zero (the default) is a perfectly clean line."""

    lose_write_byte: float = 0.0
    """Probability per byte that a host->device byte never arrives."""

    lose_read_byte: float = 0.0
    """Probability per byte that a device->host byte never arrives."""

    short_write: float = 0.0
    """Probability per ``write()`` that only a prefix of it is delivered."""

    short_read: float = 0.0
    """Probability per read chunk that only a prefix of it reaches the host."""

    unplug: float = 0.0
    """Probability per transfer that the device vanishes (requires :meth:`Chaos.bind`)."""


@dataclass
class ChaosStats:
    """What the chaos actually did, as opposed to what it was allowed to do."""

    writes: int = 0
    reads: int = 0
    written_bytes: int = 0
    read_bytes: int = 0
    lost_write_bytes: int = 0
    lost_read_bytes: int = 0
    short_writes: int = 0
    short_reads: int = 0
    unplugs: int = 0

    @property
    def bites(self) -> int:
        """Number of faults injected; ``0`` means the run proved nothing."""
        return (
            self.lost_write_bytes
            + self.lost_read_bytes
            + self.short_writes
            + self.short_reads
            + self.unplugs
        )


class Chaos:
    """Seeded fault generator for one simulated serial line.

    Usage::

        chaos = Chaos(seed, ChaosProfile(lose_read_byte=0.02, unplug=0.001))
        line = SimSerialLine(device, clock, transport=chaos.transport, ...)
        chaos.bind(line)

    The transport is built once and stays valid across reconnects, because
    ``SimSerialLine.connect`` re-installs it into every new port; :meth:`bind`
    keeps the line itself, which is where the current port (the one an unplug
    has to kill) lives.
    """

    def __init__(self, seed: int, profile: ChaosProfile) -> None:
        self.seed = seed
        self.profile = profile
        self.stats = ChaosStats()
        self.trace: list[tuple[str, bytes]] = []

        self._random = random.Random(seed)
        self._line: SimSerialLine | None = None
        self.transport = Transport(on_write=self._on_write, on_read=self._on_read)

    def bind(self, line: SimSerialLine) -> "Chaos":
        self._line = line
        return self

    def _on_write(self, data: bytes) -> bytes:
        self.stats.writes += 1
        self.stats.written_bytes += len(data)

        if self._roll(self.profile.unplug):
            self._unplug()
            payload = b""
        else:
            payload, lost, shortened = self._damage(data, self.profile.lose_write_byte, self.profile.short_write)
            self.stats.lost_write_bytes += lost
            self.stats.short_writes += int(shortened)

        self.trace.append(("write", payload))
        return payload

    def _on_read(self, data: bytes) -> bytes:
        self.stats.reads += 1
        self.stats.read_bytes += len(data)

        if self._roll(self.profile.unplug):
            self._unplug()
            payload = b""
        else:
            payload, lost, shortened = self._damage(data, self.profile.lose_read_byte, self.profile.short_read)
            self.stats.lost_read_bytes += lost
            self.stats.short_reads += int(shortened)

        self.trace.append(("read", payload))
        return payload

    def _damage(self, data: bytes, lose_probability: float, short_probability: float) -> tuple[bytes, int, bool]:
        kept = bytearray()
        lost = 0
        for byte in data:
            if self._roll(lose_probability):
                lost += 1
                continue
            kept.append(byte)

        shortened = False
        if kept and self._roll(short_probability):
            # randrange(len) keeps 0..len-1 bytes, so a short transfer always
            # drops at least one byte and may drop the whole chunk.
            del kept[self._random.randrange(len(kept)):]
            shortened = True

        return bytes(kept), lost, shortened

    def _roll(self, probability: float) -> bool:
        # A zero probability must not consume the generator: profiles that
        # disable a fault stay comparable with profiles that never had it.
        return probability > 0.0 and self._random.random() < probability

    def _unplug(self) -> None:
        if self._line is None:
            raise RuntimeError("Chaos.bind(line) is required before an unplug fault can fire")
        port = self._line.serial
        if port is None:
            return
        port.unplug()
        self.stats.unplugs += 1
