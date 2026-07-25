"""Injectable time source for the transport and driver layers.

``SerialLine`` and the two motor drivers are the only production objects that
have to wait (connect backoff, reset pulse, retry pauses) or to measure elapsed
time (position/voltage TTL caches, stop timeouts). They take a :class:`Clock`
in their constructor and default to :data:`REAL_CLOCK`, so runtime assembly in
``src/__main__.py`` keeps behaving exactly as before, while tests can drive the
whole stack -- transport included -- from the simulator's virtual clock
(``src/sim/clock.py``), which implements this same protocol.

The module intentionally has no project imports: it is a leaf that both
``serial_wrapper`` (below the drivers) and ``sky`` (above them) may depend on
without inverting the layering.
"""

import time
from typing import Protocol, runtime_checkable


@runtime_checkable
class Clock(Protocol):
    """Minimal time source: read the current instant, wait for a while."""

    def monotonic(self) -> float:
        """Seconds since an arbitrary fixed point; never goes backwards."""
        ...

    def sleep(self, seconds: float) -> None:
        """Let ``seconds`` pass: block on a real clock, advance a virtual one."""
        ...


class RealClock:
    """Wall-clock implementation, the default for every production object.

    Both methods resolve the ``time`` attribute at call time, so the existing
    tests that monkeypatch ``time.monotonic`` / ``time.sleep`` globally keep
    working through it.
    """

    def monotonic(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)


REAL_CLOCK = RealClock()

# "Never happened" marker for cached ``monotonic()`` timestamps. Plain ``0.0``
# only reads as "long ago" against a wall clock; a virtual clock starts at 0.0
# and would make a never-filled cache look fresh.
NEVER = float("-inf")


class TtlCache[T]:
    """One value that is worth re-reading only every so often, on an injected clock.

    Both drivers ask their board things that change far slower than the caller
    asks for them -- the supply voltage on either axis costs eight commands on
    the RA board and a whole status frame on the DEC one -- and both had grown
    their own "value + timestamp + TTL" trio for it. This is that trio, named
    once, so the arithmetic and the "never filled" sentinel have one
    implementation and one test instead of one per driver.

    What it deliberately does *not* do is decide anything: refreshing, giving up,
    backing off after a failure and what to answer while the value is unknown are
    policy, and the two drivers really do differ there (the RA session backs off
    for a minute after a board refuses the query; the DEC driver keeps reporting
    the last value it had). The cache answers exactly two questions -- what was
    stored, and is it still young -- and leaves the rest to the caller.

    ``None`` is a value like any other: a controller whose firmware does not
    report the voltage answers ``None``, and that answer is worth caching too, or
    the driver would re-ask every single time.
    """

    def __init__(self, ttl_s: float, clock: Clock = REAL_CLOCK) -> None:
        self._ttl_s = ttl_s
        self._clock = clock
        self._value: T | None = None
        self._updated_s = NEVER

    @property
    def value(self) -> T | None:
        """What was stored last, fresh or stale; ``None`` until the first store."""
        return self._value

    def is_fresh(self) -> bool:
        """Strictly younger than the TTL. Never true before the first store."""
        return self._clock.monotonic() - self._updated_s < self._ttl_s

    def store(self, value: T) -> None:
        self._value = value
        self._updated_s = self._clock.monotonic()

    def forget(self) -> None:
        """Back to "never read": what is remembered is no longer about this board."""
        self._value = None
        self._updated_s = NEVER
