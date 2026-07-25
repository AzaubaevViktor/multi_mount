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
