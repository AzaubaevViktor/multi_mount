"""One conversation with the RA board: strict serialization, state, quirks.

Everything this layer does is a consequence of something written down in
``docs/protocol/RA_PROTOCOL.md``:

* §8.1 the board does not buffer — two commands in one write cost the second
  one, so the line is strictly request/response and every command goes through
  :meth:`SkyWatcherSession.transact`;
* §7 `:J1` latches its target when it *arrives*, so it is the one command that
  may not be blindly re-sent after a lost answer;
* §11 `:E` sent while the axis moves is accepted with `=` and clears the
  initialization flag a few hundred milliseconds later — the guard and the
  repair both have to live on the host;
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

from clock import NEVER, REAL_CLOCK, Clock
from serial_wrapper.wrapper import SerialLine
from sky.motor import MotorStopRequire
from skywatcher.board import SkyWatcherBoard, probe_board
from skywatcher.codec import (
    Command,
    SkyWatcherCodec,
    SkyWatcherMotorError,
    SkyWatcherMotorProtocolError,
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
        self._position_ticks: int | None = None
        self._position_updated = NEVER

        # §12.4: what the driver itself put on the board. Without this the sign
        # "position is exactly 0x800000" is uninterpretable, and a reboot cannot
        # be told apart from a legal `:E1000080`.
        self._expected_position_ticks: int | None = None
        self._expected_period: int | None = None
        self._expected_initialized = False
        self._reboot_check_busy = False
        self.reboots_detected = 0

        self._battery_v: float | None = None
        self._usb_v: float | None = None
        self._power_v_updated = NEVER
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
        self._position_ticks = None
        self._position_updated = NEVER
        self._expected_position_ticks = None
        self._expected_period = None
        self._expected_initialized = False

    # ------------------------------------------------------------------
    # transport
    # ------------------------------------------------------------------

    def transact(self, command: Command, arg: str | None = None) -> str:
        payload = SkyWatcherCodec.frame(command, arg)
        # Where the axis stood before the first attempt is the only evidence there will
        # be afterwards, and it has to be read *fresh*: a cached position from before
        # the axis stopped would look like movement caused by this very command.
        position_before = self.position_ticks(fresh=True) if command in self._NON_IDEMPOTENT else None

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
                # Read the leftovers *before* dropping them. The old order was the reverse —
                # `drop_buffers()` and then a read that could only come back empty — which is
                # why the March logs hold 20 601 records of `['']` and not one byte of the
                # garbage that actually confused the parser.
                data = self._serial.read_all_data(timeout=.5)
                if data:
                    self._logger.info("Discarding %d leftover byte-groups after a protocol error: %s", len(data), data)
                self._serial.drop_buffers()
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
        """Axis counter in board counts, zero at the logical zero of §12.1."""
        cpr = self.require_board().cpr
        if not fresh and self._position_ticks is not None and self._clock.monotonic() - self._position_updated <= self._POSITION_CACHE_TTL_S:
            return self._position_ticks
        ticks = (SkyWatcherCodec.decode_revu24(self.transact(Command.INQUIRE_POSITION)) - POSITION_OFFSET) % cpr
        if self._observe_position(ticks):
            # A reboot was found and undone: the counter read above belongs to the
            # board that no longer exists, so the restored position is the answer.
            return self._position_ticks if self._position_ticks is not None else ticks
        self._remember_position(ticks)
        return ticks

    def _remember_position(self, ticks: int) -> None:
        self._position_ticks = ticks
        self._position_updated = self._clock.monotonic()
        self._expected_position_ticks = ticks

    def read_period(self) -> int:
        return SkyWatcherCodec.decode_revu24(self.transact(Command.INQUIRE_STEP_PERIOD))

    def set_period(self, period: int) -> None:
        self.transact(Command.SET_STEP_PERIOD, SkyWatcherCodec.encode_revu24(period))
        self._expected_period = period

    def initialize(self) -> None:
        self.transact(Command.INITIALIZE)
        self._expected_initialized = True

    def set_position_ticks(self, ticks: int) -> None:
        """`:E` with the whole §11 protocol around it.

        The guard is the point: the board accepts `:E` on the move with `=` (the
        spec's `!2` never comes) and then clears the initialization flag
        asynchronously, 5 runs out of 5. The board offers no protection, so this
        host-side check is the whole of it.
        """
        cpr = self.require_board().cpr
        if self.status().running:
            raise MotorStopRequire("cannot change steps while motor is moving")
        self.transact(Command.SET_AXIS_POSITION, SkyWatcherCodec.encode_revu24((ticks + POSITION_OFFSET) % cpr))
        self._remember_position(ticks % cpr)
        # §11 again: from here until the flag has been re-read, a cleared flag is
        # this command's doing and not a sign of a reboot.
        self._expected_initialized = False
        self._restore_initialization_after_set_position()

    def _restore_initialization_after_set_position(self) -> None:
        # §11 requirements 3 and 4: after *any* `:E` re-read `:f1` and re-initialize if the
        # flag went down — and do the re-read late, because the flag drops hundreds of
        # milliseconds after the command was acknowledged. An immediate check would pass
        # while the flag is still up and miss the very thing it is there to catch.
        self._clock.sleep(self._INIT_FLAG_SETTLE_S)
        if self.status().initialized:
            self._expected_initialized = True
            return
        self._logger.warning(
            "Initialization flag went down after SET_AXIS_POSITION (RA_PROTOCOL.md §11), re-initializing the axis"
        )
        self.initialize()
        if not self.status().initialized:
            raise SkyWatcherMotorProtocolError("axis stays uninitialized after re-sending INITIALIZE")

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
            # §12.1: a board that has just rebooted is stopped. A moving axis with a
            # dropped flag is the §11 quirk, and restoring a position there would be
            # the very thing §11 forbids.
            and not status.running
        )

    def _handle_suspected_reboot(self, position_ticks: int | None = None) -> bool:
        self._reboot_check_busy = True
        try:
            if not self._confirms_reboot(position_ticks):
                return False
            self.reboots_detected += 1
            self._logger.error(
                "RA board rebooted (RA_PROTOCOL.md §12.2): position back at 0x%06X, initialization flag down, "
                "step period back at %d. Restoring position %s and period %s.",
                POSITION_OFFSET,
                self.require_board().power_up_period,
                self._expected_position_ticks,
                self._expected_period,
            )
            self._recover_from_reboot()
            return True
        finally:
            self._reboot_check_busy = False

    def _confirms_reboot(self, position_ticks: int | None) -> bool:
        """The other two signs of §12.2, and the memory that makes them readable.

        Every sign has a legal explanation on its own (§12.2 lists them), so all
        three have to hold *and* none of them may be the driver's own doing. With
        nothing written down to compare against, the state of a board that answers
        normally is simply not interpretable (§12.4.4) — and saying so in the log
        is more use than a guess.
        """
        board = self.require_board()
        if self._expected_position_ticks is None or self._expected_period is None:
            self._logger.warning(
                "Initialization flag is down and there is nothing to compare the board's state with "
                "(RA_PROTOCOL.md §12.4): position=%s period=%s",
                self._expected_position_ticks, self._expected_period,
            )
            return False
        if self._expected_position_ticks == 0:
            # The driver believes the axis is at the logical zero anyway, so the
            # position sign carries no information at all.
            return False
        if position_ticks is None:
            position_ticks = (SkyWatcherCodec.decode_revu24(self.transact(Command.INQUIRE_POSITION)) - POSITION_OFFSET) % board.cpr
        if position_ticks != 0:
            return False
        if self._expected_period == board.power_up_period:
            # Same reasoning as above for the period: the driver put the board on
            # the power-up period itself, so reading it back proves nothing.
            return False
        return self.read_period() == board.power_up_period

    def _recover_from_reboot(self) -> None:
        """§12.4.2: `:F1` again, position back at a *stopped* axis, period again.

        Verified against the simulator only (``SkyWatcherSim.reboot()``): the
        board was disconnected when this was written, so the live-hardware half of
        it is stage Э5 of ``docs/RA_REWRITE_PLAN.md``, not a checked fact.
        """
        expected_position = self._expected_position_ticks
        expected_period = self._expected_period
        status = SkyWatcherCodec.parse_status(self.transact(Command.INQUIRE_STATUS))
        self._last_status = status
        if status.running:
            # Not a state a rebooted board can be in (§12.1), and `:E` on a moving
            # axis is exactly the quirk of §11. Report and leave the board alone.
            self._logger.error("RA board looks rebooted but reports a running axis; not restoring anything")
            return

        self.initialize()
        if expected_position is not None:
            self.set_position_ticks(expected_position)
        if expected_period is not None:
            self.set_period(expected_period)
        self._last_status = SkyWatcherCodec.parse_status(self.transact(Command.INQUIRE_STATUS))

    def _observe_position(self, ticks: int) -> bool:
        """Second entry point into §12.2, for a position read that came first.

        A reboot parks the counter at the logical zero, and a caller that asks for
        the position before it asks for the status would otherwise overwrite the
        one memory that makes the zero readable.
        """
        if self._reboot_check_busy or self._board is None:
            return False
        if ticks != 0 or not self._expected_position_ticks:
            return False
        status = SkyWatcherCodec.parse_status(self.transact(Command.INQUIRE_STATUS))
        self._last_status = status
        if not self._suspects_reboot(status):
            return False
        return self._handle_suspected_reboot(position_ticks=ticks)

    # ------------------------------------------------------------------
    # §6.8: the supply voltage window
    # ------------------------------------------------------------------

    def power_v(self) -> float | None:
        now = self._clock.monotonic()
        if now - self._power_v_updated < self._POWER_CACHE_TTL_S or now < self._power_v_retry_at:
            return self.cached_power_v()

        try:
            battery_v = self._read_voltage(BATTERY_VOLT_ADDRESS)
            usb_v = self._read_voltage(USB_VOLT_ADDRESS)
        except SkyWatcherMotorError as error:
            # A board that cannot answer this will not answer it in 40ms either, and
            # the caller is a dashboard tick. Back off, report "unknown", stay quiet.
            self._battery_v = None
            self._usb_v = None
            self._power_v_retry_at = now + self._POWER_FAILURE_BACKOFF_S
            self._logger.warning(
                "Could not read the RA supply voltage, next try in %.0fs: %s", self._POWER_FAILURE_BACKOFF_S, error
            )
            return None

        self._battery_v = battery_v
        self._usb_v = usb_v
        self._power_v_updated = now
        self._power_v_retry_at = NEVER
        return self.cached_power_v()

    def cached_power_v(self) -> float | None:
        voltages = [v for v in (self._battery_v, self._usb_v) if v is not None]
        return max(voltages) if voltages else None

    @property
    def battery_v(self) -> float | None:
        return self._battery_v

    @property
    def usb_v(self) -> float | None:
        return self._usb_v

    def forget_power_v(self) -> None:
        self._battery_v = None
        self._usb_v = None
        self._power_v_updated = NEVER
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
