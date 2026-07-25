"""`clock.TtlCache`: the "value + timestamp + TTL" trio both drivers grew.

Everything here used to be implicit in three separate copies (the RA position
cache, the RA voltage pair, the DEC voltage), and the two properties that
actually bite -- "never fresh before the first read" and "a stale value is still
readable" -- were never asserted anywhere.
"""

from clock import TtlCache
from sim.clock import Clock as VirtualClock


def test_nothing_is_fresh_before_the_first_store():
    # The whole reason the "never" marker is -inf and not 0.0: a virtual clock
    # starts at 0.0, so an empty cache timestamped 0.0 would read as brand new
    # and the driver would answer with a voltage it has never read.
    clock = VirtualClock()
    assert clock.monotonic() == 0.0

    cache: TtlCache[int] = TtlCache(5.0, clock)

    assert cache.is_fresh() is False
    assert cache.value is None


def test_a_stored_value_is_fresh_until_the_ttl_runs_out():
    clock = VirtualClock()
    cache: TtlCache[int] = TtlCache(5.0, clock)

    cache.store(42)
    assert cache.is_fresh() is True

    clock.advance(4.9)
    assert cache.is_fresh() is True

    clock.advance(0.2)
    assert cache.is_fresh() is False


def test_a_stale_value_is_still_readable():
    # What the DEC driver leans on: a voltage that could not be re-read is
    # reported as the last known one, not as "unknown".
    clock = VirtualClock()
    cache: TtlCache[int] = TtlCache(1.0, clock)
    cache.store(12)

    clock.advance(60)

    assert cache.is_fresh() is False
    assert cache.value == 12


def test_none_is_a_value_and_is_cached_like_one():
    # A controller whose firmware does not report the voltage answers None every
    # time; caching that answer is what keeps the driver from spending a status
    # frame per dashboard tick to learn the same nothing.
    clock = VirtualClock()
    cache: TtlCache[float | None] = TtlCache(2.0, clock)

    cache.store(None)

    assert cache.is_fresh() is True
    assert cache.value is None


def test_forget_puts_it_back_to_never_read():
    clock = VirtualClock()
    cache: TtlCache[int] = TtlCache(5.0, clock)
    cache.store(7)

    cache.forget()

    assert cache.is_fresh() is False
    assert cache.value is None


def test_the_clock_is_the_injected_one():
    """Two caches on two clocks do not age each other."""
    ticking = VirtualClock()
    frozen = VirtualClock()
    aging: TtlCache[int] = TtlCache(1.0, ticking)
    kept: TtlCache[int] = TtlCache(1.0, frozen)
    aging.store(1)
    kept.store(1)

    ticking.advance(10)

    assert aging.is_fresh() is False
    assert kept.is_fresh() is True
