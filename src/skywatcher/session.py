"""One conversation with the RA board: strict serialization, state, quirks.

Everything this layer does is a consequence of something written down in
``docs/protocol/RA_PROTOCOL.md``:

* §8.1 the board does not buffer — two commands in one write cost the second
  one, so the line is strictly request/response and every command goes through
  :meth:`SkyWatcherSession.transact`;
* §7 `:J1` latches its target when it *arrives*, so it is the one command that
  may not be blindly re-sent after a lost answer;
* `:E` (Set Axis Position) is **never sent**. Not "not while moving", not
  "carefully": writing the axis counter is a mechanically destructive action on
  some mounts, and the owner has ruled it out on this one. The driver therefore
  never tells the board where it is — it *recomputes* where the board is, by
  keeping a software offset between the board's own counter and the logical
  coordinate. A sync moves the offset; a reboot moves the offset; the board's
  register is read and never written. Everything §11 says about the `:E` quirk
  is consequently moot here, and the quirk is only kept in the simulator and in
  the conformance set, where nothing turns an axis;
* §10.3 `:K1` starts a braking ramp, so "stopped" is a fact to be waited for,
  not an acknowledgement to be trusted;
* §12 a rebooted board is indistinguishable from a legal sync unless the driver
  remembers what it wrote — which is why the expected position, period and
  initialization flag are kept here.

The layer knows nothing about hour angles or sidereal rates: it speaks in
counts, periods and flags. :class:`skywatcher.motor.SkyWatcherMotor` is the one
that turns those into the axis API.
"""

import logging

from clock import NEVER, REAL_CLOCK, Clock, TtlCache
from serial_wrapper.wrapper import SerialLine
from sky.motor import MotorStopRequire
from skywatcher.board import SkyWatcherBoard, TimerPeriod, probe_board
from skywatcher.codec import (
    Command,
    SkyWatcherCodec,
    SkyWatcherMotorError,
    SkyWatcherMotorProtocolError,
    SkyWatcherMotorRebootError,
    SkyWatcherMotorTimeoutError,
    Status,
)
from skywatcher.protocol import Protocol

POSITION_OFFSET = 0x800000

# §6.8: both supply voltages sit in the `:C`/`:n` window as int16 little-endian in
# hundredths of a volt.
BATTERY_VOLT_ADDRESS = 0x0004
USB_VOLT_ADDRESS = 0x001C
VOLT_PER_COUNT = 0.01


class SkyWatcherSession:
    """Transport, expected state and firmware quirks of one RA controller."""

    _CONNECT_ATTEMPTS = 3
    _CONNECT_RETRY_DELAY_S = 0.25

    REPEATS = 3
    # The one command in this protocol that is not safe to repeat. The board latches
    # `position + increment` at the moment it *receives* `:J1` (§7), so a resend a
    # couple of tenths of a second later does not repeat the start — it re-arms the
    # GOTO from wherever the axis has crawled to meanwhile, and the axis overshoots by
    # exactly the distance it covered during the retry. Everything else this driver
    # sends is a query or a set-value command, i.e. free to repeat, so "may be
    # repeated" is a property of the command and lives here rather than as a special
    # case in `run()`: the next non-idempotent command joins this set and inherits the
    # whole handling.
    _NON_IDEMPOTENT = frozenset({Command.START_MOTION})
    # Below this the axis has not started: the position answers of two reads taken
    # around one round trip may legitimately differ by a step of rounding.
    _MOTION_CONFIRM_STEPS = 2

    _STOP_POLL_S = 0.05
    # §11: the initialization flag drops "asynchronously, hundreds of milliseconds" after
    # an `:E`. The board was seen taking ~0.3s; the check waits longer than that on purpose.
    _INIT_FLAG_SETTLE_S = 0.5

    _POSITION_CACHE_TTL_S = 0.25

    # How far from its power-up value the board's counter may sit and still
    # count as "this board has just restarted". Derived and defended in
    # :meth:`_near_power_up_counter`.
    _REBOOT_POSITION_WINDOW_TICKS = 3_200

    # One reading is eight commands (`:C1` + `:n1` per byte, two bytes per channel,
    # two channels), and the dashboard asks on every tick. A supply that sags fast
    # enough to matter (§12: motor load -> brownout -> reboot) still sags over
    # seconds, so a few seconds of staleness costs nothing and keeps the line free.
    _POWER_CACHE_TTL_S = 5.0
    # A board that answers `!0` or garbage will keep doing so. Retrying every tick would
    # be the `:fL#` storm all over again, only eight commands wide instead of one.
    _POWER_FAILURE_BACKOFF_S = 60.0

    def __init__(self, serial: SerialLine, clock: Clock = REAL_CLOCK) -> None:
        self._serial = serial
        self._clock = clock
        self._logger = logging.getLogger(type(self).__name__)
        self._board: SkyWatcherBoard | None = None

        self._last_status: Status | None = None
        # The board's own counter, as last read, in counts around its power-up
        # value of 0x800000. Never written.
        self._raw_ticks: TtlCache[int] = TtlCache(self._POSITION_CACHE_TTL_S, clock)
        # logical position = (board counter + this) mod CPR. The only thing a
        # sync changes, and the only thing a reboot recovery changes.
        self._offset_ticks = 0

        # §12.4: what the driver last saw on the board. Without this the sign
        # "the counter is back at its power-up value" is uninterpretable — a
        # board that has simply never moved reads the same.
        self._expected_raw_ticks: int | None = None
        self._expected_period: TimerPeriod | None = None
        self._expected_initialized = False
        self._reboot_check_busy = False
        self.reboots_detected = 0

        # Both rails together: they are read as a pair and are only meaningful as
        # a pair (§6.8), so one cache holds the pair and cannot go half-stale.
        self._rails: TtlCache[tuple[float, float]] = TtlCache(self._POWER_CACHE_TTL_S, clock)
        self._power_v_retry_at = NEVER

    # ------------------------------------------------------------------
    # connect / disconnect
    # ------------------------------------------------------------------

    @property
    def board(self) -> SkyWatcherBoard | None:
        return self._board

    def require_board(self) -> SkyWatcherBoard:
        if self._board is None:
            raise SkyWatcherMotorProtocolError("motor geometry is not initialized")
        return self._board

    def open(self) -> SkyWatcherBoard:
        if self._serial.terminator != Protocol.ANSWER_END_BYTE:
            raise SkyWatcherMotorProtocolError(
                f"invalid SerialLine terminator: expected {Protocol.ANSWER_END_BYTE!r}, got {self._serial.terminator!r}"
            )
        self._serial.connect()
        # A reconnect is a different session (possibly a different power source, and
        # certainly a board that may have been off in between): nothing measured or
        # expected before it may be carried over.
        self.forget_power_v()
        self._forget_expectations()

        # A mount that stays silent right after the port opens is usually transient
        # (PLAN.md §1 П3): retry the handshake a bounded number of times before
        # declaring the mount unreachable.
        for attempt in range(self._CONNECT_ATTEMPTS):
            try:
                self.transact(Command.INITIALIZE)
                break
            except SkyWatcherMotorError as error:
                if attempt + 1 >= self._CONNECT_ATTEMPTS:
                    raise SkyWatcherMotorProtocolError(
                        f"mount is not responding on {self._serial.port}: no answer to INITIALIZE after {self._CONNECT_ATTEMPTS} attempts"
                    ) from error

                self._logger.warning(
                    "Mount is not responding on %s, attempt %d/%d: %s",
                    self._serial.port, attempt + 1, self._CONNECT_ATTEMPTS, error,
                )
                self._clock.sleep(self._CONNECT_RETRY_DELAY_S * 2 ** attempt)

        self._expected_initialized = True
        board, loaded_period = probe_board(self.transact, self._logger)
        self._board = board
        # The probe put back the period it found, so that value — not an assumption
        # about what a board powers up with — is what the driver expects to read.
        self._expected_period = loaded_period
        return board

    def close(self) -> None:
        self.forget_power_v()
        self._serial.close()

    def _forget_expectations(self) -> None:
        self._last_status = None
        self._raw_ticks.forget()
        self._expected_raw_ticks = None
        self._expected_period = None
        self._expected_initialized = False
        self._offset_ticks = 0

    # ------------------------------------------------------------------
    # transport
    # ------------------------------------------------------------------

    def transact(self, command: Command, arg: str | None = None) -> str:
        payload = SkyWatcherCodec.frame(command, arg)
        # Where the axis stood before the first attempt is the only evidence there will
        # be afterwards, and it has to be read *fresh*: a cached position from before
        # the axis stopped would look like movement caused by this very command.
        position_before = None
        if command in self._NON_IDEMPOTENT:
            reboots_before = self.reboots_detected
            position_before = self.position_ticks(fresh=True)
            if self.reboots_detected != reboots_before:
                # §12.4.2: the board rebooted between the caller's decision and this
                # command. Its target registers went with it, so starting now would
                # run the axis to a target nobody asked for. The state has been put
                # back; the motion has not been started, and the caller must know.
                raise SkyWatcherMotorRebootError(
                    f"RA board rebooted before {command.name}; state restored, motion not started"
                )

        count = self.REPEATS
        response = None
        while count > 0:
            try:
                response = self._serial.query(payload, response_prefixes=SkyWatcherCodec.RESPONSE_PREFIXES)
                return SkyWatcherCodec.parse_reply(response)

            except SkyWatcherMotorProtocolError:
                # An in-transaction retry is a recoverable transient: the caller decides whether the
                # final failure is worth an error record, so keep the per-attempt trace on DEBUG.
                self._logger.debug(
                    "While quering %s(%s) `%s` -> `%s`, %d last", command.name, arg, payload, response, count, exc_info=True
                )
                self._serial.drain_after_error(self._logger, "a protocol error")
                # The line has had the same recovery an ordinary retry gives it; only then
                # is the board asked what happened. A command that must not be repeated is
                # repeated only after the board has been shown *not* to have executed it —
                # silence alone does not distinguish "never arrived" from "answered, and
                # the answer was eaten".
                if position_before is not None and self._motion_started(position_before):
                    self._logger.info(
                        "%s reached the board and only its answer was lost, not re-sending it", command.name
                    )
                    # `:J1` is acknowledged with a bare `=`: there is no body to restore.
                    return ""
                count -= 1
                if count == 0:
                    raise
                self._clock.sleep(0.1)
        raise SkyWatcherMotorProtocolError(f"failed to execute command {command} with payload {payload}")

    def _motion_started(self, position_before: int) -> bool:
        """Did the axis really start, or was only the acknowledgement lost?

        Both questions asked here are queries, so asking them costs nothing but time
        even if the guess is wrong. The Running bit answers most of the cases; the
        position answers the rest, because a GOTO short enough to finish inside one
        retry window (the drain alone is half a second) is over by the time this runs
        and reports itself as stopped — indistinguishable from never having started
        except by the distance travelled.
        """
        if self.status().running:
            return True
        return self._ticks_distance(self.position_ticks(fresh=True), position_before) > self._MOTION_CONFIRM_STEPS

    def _ticks_distance(self, one: int, other: int) -> int:
        cpr = self.require_board().cpr
        return min((one - other) % cpr, (other - one) % cpr)

    # ------------------------------------------------------------------
    # board state
    # ------------------------------------------------------------------

    @property
    def last_status(self) -> Status | None:
        return self._last_status

    def status(self) -> Status:
        status = SkyWatcherCodec.parse_status(self.transact(Command.INQUIRE_STATUS))
        self._last_status = status
        if self._suspects_reboot(status) and self._handle_suspected_reboot():
            return self._last_status
        return status

    def position_ticks(self, fresh: bool = False) -> int:
        """Where the axis is in the driver's own frame, in board counts.

        This is *not* the board's register. It is that register plus the offset
        the driver keeps (:attr:`_offset_ticks`), because the register is never
        written — see the module docstring. The two coincide only until the
        first sync or the first reboot.
        """
        cpr = self.require_board().cpr
        cached = self._raw_ticks.value
        if not fresh and cached is not None and self._raw_ticks.is_fresh():
            return (cached + self._offset_ticks) % cpr
        raw = self._read_raw_ticks()
        if self._observe_position(raw):
            # A reboot was found and the offset re-derived: the counter read
            # above belongs to a board that has just restarted, and the answer
            # is what the new offset makes of it.
            recovered = self._raw_ticks.value
            raw = recovered if recovered is not None else raw
            return (raw + self._offset_ticks) % cpr
        self._remember_raw(raw)
        return (raw + self._offset_ticks) % cpr

    def _read_raw_ticks(self) -> int:
        cpr = self.require_board().cpr
        return (SkyWatcherCodec.decode_revu24(self.transact(Command.INQUIRE_POSITION)) - POSITION_OFFSET) % cpr

    def _remember_raw(self, raw: int) -> None:
        self._raw_ticks.store(raw)
        # Not the same thing as the cache above, and deliberately not folded into
        # it: the cache expires, this does not. It is what §12.2 compares the next
        # counter reading against, and a memory that quietly went stale would make
        # a reboot uninterpretable.
        self._expected_raw_ticks = raw

    def read_period(self) -> TimerPeriod:
        return TimerPeriod(SkyWatcherCodec.decode_revu24(self.transact(Command.INQUIRE_STEP_PERIOD)))

    def set_period(self, period: TimerPeriod) -> None:
        self.transact(Command.SET_STEP_PERIOD, SkyWatcherCodec.encode_revu24(int(period)))
        self._expected_period = period

    def initialize(self) -> None:
        self.transact(Command.INITIALIZE)
        self._expected_initialized = True

    def set_position_ticks(self, ticks: int) -> None:
        """Sync: move the driver's offset, leave the board's counter alone.

        **`:E` is not sent, here or anywhere else.** Writing the axis position
        register is a mechanically destructive operation on some mounts and the
        owner has forbidden it on this one, so a sync cannot be "tell the board
        it is at X". It is "remember that the board's X reads as ours" — one
        subtraction, no command on the wire, and nothing for §11's quirk to
        happen to.

        Whoever reads this next: do not "restore" the `:E`. It is not missing,
        it is removed, and the offset below is the whole replacement.
        """
        cpr = self.require_board().cpr
        if self.status().running:
            # Not a protocol rule any more — an arithmetic one. The offset is
            # derived from a counter read, and a counter read of a moving axis
            # is stale by the round trip, so the sync would be wrong by however
            # far the axis travelled meanwhile.
            raise MotorStopRequire("cannot change steps while motor is moving")
        raw = self._read_raw_ticks()
        self._remember_raw(raw)
        self._offset_ticks = (ticks - raw) % cpr

    # ------------------------------------------------------------------
    # stopping
    # ------------------------------------------------------------------

    def request_stop(self) -> None:
        self.transact(Command.STOP_MOTION)

    def wait_till_running_clears(self, timeout_s: float | None) -> None:
        """§10.3: `K` starts a braking *ramp*, so wait for the fact, not the `=`.

        The board answers immediately, but at the top speed it allows the axis
        keeps running for another 0.6s and coasts ~1148 counts. Returning on the
        acknowledgement alone made `Axis.disconnect` (reset -> stop -> disconnect)
        close the port on a physically moving axis.
        """
        deadline = None if timeout_s is None else self._clock.monotonic() + timeout_s
        while self.status().running:
            if deadline is not None and self._clock.monotonic() > deadline:
                raise SkyWatcherMotorTimeoutError(f"motor did not stop within {timeout_s}s")
            self._clock.sleep(self._STOP_POLL_S)

    # ------------------------------------------------------------------
    # §12: reboot detection and recovery
    # ------------------------------------------------------------------

    def _suspects_reboot(self, status: Status) -> bool:
        """The cheap half of §12.2, paid on every status read.

        The initialization flag is the only sign that comes for free with a `:f1`
        the driver was going to send anyway, so it is the trigger — and only the
        trigger. On its own it means nothing (§11 drops it without any reboot),
        which is why a suspicion costs two more reads before it becomes a verdict.
        """
        return (
            self._board is not None
            and not self._reboot_check_busy
            and self._expected_initialized
            and not status.initialized
            # §12.1: a board that has just rebooted is stopped. A moving axis
            # with a dropped flag has some other explanation, and an offset
            # derived from a counter read taken mid-motion would be wrong by
            # however far the axis moved during the round trip.
            and not status.running
        )

    def _handle_suspected_reboot(self, raw_ticks: int | None = None) -> bool:
        self._reboot_check_busy = True
        try:
            if not self._confirms_reboot(raw_ticks):
                return False
            self.reboots_detected += 1
            self._logger.error(
                "RA board rebooted (RA_PROTOCOL.md §12.2): counter back within %d counts of 0x%06X, "
                "initialization flag down, step period back at %s. Re-deriving the position offset so the "
                "logical position stays %s, and putting the period %s back.",
                self._REBOOT_POSITION_WINDOW_TICKS,
                POSITION_OFFSET,
                self.require_board().power_up_period,
                self._logical_ticks(),
                self._expected_period,
            )
            self._recover_from_reboot()
            return True
        finally:
            self._reboot_check_busy = False

    def _logical_ticks(self) -> int | None:
        if self._expected_raw_ticks is None:
            return None
        return (self._expected_raw_ticks + self._offset_ticks) % self.require_board().cpr

    def _near_power_up_counter(self, raw_ticks: int) -> bool:
        """Is the board's counter back where a restart leaves it (§26)?

        Not "exactly at the power-up value" — that rule missed every real
        reboot there is. Six measured restarts put the counter at 0, -7, +216,
        +448, +645 and -1241 counts of its power-up value: the axis keeps
        coasting while the controller is down, and the faster it was going the
        further it gets.

        The window is the coast the board's own numbers allow. Its speed
        ceiling is 9 000 steps/s (§2.6) and its measured brake ramp is
        13 000 steps/s², so a free stop from the fastest the board can go is
        9 000^2/(2*13 000) = 3 115 counts. Rounded to 3 200, that is an upper
        bound by construction: an axis that is not being driven cannot stop
        *later* than its own driven ramp would, and indeed every measured coast
        came in well under it — the worst, 1 241, by a factor of 2.6.

        What it costs: a false "the board rebooted" is now possible anywhere
        within 3 200 counts (5.5 arcminutes of axis rotation, 22 s of hour
        angle) of the power-up value, where before it took an exact hit. What
        that false positive can do is bounded, and deliberately so — the
        recovery no longer writes anything to the board (see
        :meth:`set_position_ticks`), it only re-derives the offset, and the
        offset is re-derived from the very reading that triggered the
        suspicion. Six points is not many; §26 says what to measure to do
        better, and the honest summary is that the *shape* of the rule is
        established and its width is an upper bound rather than a fit.
        """
        cpr = self.require_board().cpr
        distance = min(raw_ticks % cpr, (-raw_ticks) % cpr)
        return distance <= self._REBOOT_POSITION_WINDOW_TICKS

    def _confirms_reboot(self, raw_ticks: int | None) -> bool:
        """The other two signs of §12.2, and the memory that makes them readable.

        Every sign has a legal explanation on its own (§12.2 lists them), so all
        three have to hold *and* none of them may be the driver's own doing. With
        nothing written down to compare against, the state of a board that answers
        normally is simply not interpretable (§12.4.4) — and saying so in the log
        is more use than a guess.
        """
        board = self.require_board()
        if self._expected_raw_ticks is None or self._expected_period is None:
            self._logger.warning(
                "Initialization flag is down and there is nothing to compare the board's state with "
                "(RA_PROTOCOL.md §12.4): counter=%s period=%s",
                self._expected_raw_ticks, self._expected_period,
            )
            return False
        if self._near_power_up_counter(self._expected_raw_ticks):
            # The board's counter was already inside the window, so finding it
            # there now says nothing at all.
            return False
        if raw_ticks is None:
            raw_ticks = self._read_raw_ticks()
        if not self._near_power_up_counter(raw_ticks):
            return False
        if self._expected_period == board.power_up_period:
            # Same reasoning as above for the period: the driver put the board on
            # the power-up period itself, so reading it back proves nothing.
            return False
        return self.read_period() == board.power_up_period

    def _recover_from_reboot(self) -> None:
        """§12.4.2, without writing a thing to the axis: `:F1`, offset, period.

        The board's counter has restarted from its own zero, so the driver's
        logical coordinate is preserved by **moving the offset**, not by telling
        the board where it was. The new offset is the last logical position the
        driver knew, which makes the counter the board is now keeping — the
        coast it did while the controller was down, and everything after it —
        add on top of that position instead of being thrown away. `:E` would
        have thrown it away, and `:E` is forbidden regardless.

        Verified against the simulator only (``SkyWatcherSim.reboot()`` and its
        ``reboot_on_goto_arrival`` mode): the live half is still stage Э5 of
        ``docs/RA_REWRITE_PLAN.md``.
        """
        logical = self._logical_ticks()
        expected_period = self._expected_period
        status = SkyWatcherCodec.parse_status(self.transact(Command.INQUIRE_STATUS))
        self._last_status = status
        if status.running:
            # Not a state a rebooted board can be in (§12.1). Report and leave
            # the board alone rather than re-deriving an offset from a counter
            # that is moving under the read.
            self._logger.error("RA board looks rebooted but reports a running axis; not restoring anything")
            return

        self.initialize()
        if logical is not None:
            self._offset_ticks = logical % self.require_board().cpr
            self._remember_raw(self._read_raw_ticks())
        if expected_period is not None:
            self.set_period(expected_period)
        self._last_status = SkyWatcherCodec.parse_status(self.transact(Command.INQUIRE_STATUS))

    def _observe_position(self, raw_ticks: int) -> bool:
        """Second entry point into §12.2, for a position read that came first.

        A reboot parks the counter near its power-up value, and a caller that
        asks for the position before it asks for the status would otherwise
        overwrite the one memory that makes that reading interpretable.
        """
        if self._reboot_check_busy or self._board is None:
            return False
        if not self._near_power_up_counter(raw_ticks):
            return False
        if self._expected_raw_ticks is None or self._near_power_up_counter(self._expected_raw_ticks):
            return False
        status = SkyWatcherCodec.parse_status(self.transact(Command.INQUIRE_STATUS))
        self._last_status = status
        if not self._suspects_reboot(status):
            return False
        return self._handle_suspected_reboot(raw_ticks=raw_ticks)

    # ------------------------------------------------------------------
    # §6.8: the supply voltage window
    # ------------------------------------------------------------------

    def power_v(self) -> float | None:
        now = self._clock.monotonic()
        if self._rails.is_fresh() or now < self._power_v_retry_at:
            return self.cached_power_v()

        try:
            battery_v = self._read_voltage(BATTERY_VOLT_ADDRESS)
            usb_v = self._read_voltage(USB_VOLT_ADDRESS)
        except SkyWatcherMotorError as error:
            # A board that cannot answer this will not answer it in 40ms either, and
            # the caller is a dashboard tick. Back off, report "unknown", stay quiet.
            self._rails.forget()
            self._power_v_retry_at = now + self._POWER_FAILURE_BACKOFF_S
            self._logger.warning(
                "Could not read the RA supply voltage, next try in %.0fs: %s", self._POWER_FAILURE_BACKOFF_S, error
            )
            return None

        self._rails.store((battery_v, usb_v))
        self._power_v_retry_at = NEVER
        return self.cached_power_v()

    def cached_power_v(self) -> float | None:
        rails = self._rails.value
        return max(rails) if rails is not None else None

    @property
    def battery_v(self) -> float | None:
        return self._rails.value[0] if self._rails.value is not None else None

    @property
    def usb_v(self) -> float | None:
        return self._rails.value[1] if self._rails.value is not None else None

    def forget_power_v(self) -> None:
        self._rails.forget()
        self._power_v_retry_at = NEVER

    def _read_voltage(self, address: int) -> float:
        # int16 little-endian: the low byte is at `address`, the high byte one above.
        # Swapping them turns 6.04 V into 236.44 V, which is why both halves are read
        # here and not by two independent callers.
        low = self._read_memory_byte(address)
        high = self._read_memory_byte(address + 1)
        return ((high << 8) | low) * VOLT_PER_COUNT

    def _read_memory_byte(self, address: int) -> int:
        # `:C1<lo><hi>\r` then `:n1\r` -> `=XX\r`. The window does not auto increment:
        # it has to be set again before every single byte.
        self.transact(Command.SET_MEMORY_ADDRESS, SkyWatcherCodec.encode_memory_address(address))
        return SkyWatcherCodec.decode_memory_byte(self.transact(Command.INQUIRE_MEMORY_BYTE))
