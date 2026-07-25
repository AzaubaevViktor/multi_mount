"""SkyWatcher motor-controller endpoint simulator (RA axis, channel 1).

The reference for every behaviour here is ``docs/protocol/RA_PROTOCOL.md`` — a
transcript of the live board, which is what this simulator must imitate. The
command set reference
(``references/skywatcher_motor_controller_command_set_extracted.md``) is only
consulted where the live board was not probed:

- command ``:<cmd>1<data>\\r`` -> ``=<data>\\r`` on success, ``!<code>\\r`` on error;
- 24-bit values travel as Revu24 (byte-reversed hex), positions offset by 0x800000;
- status body of ``:f1`` is 3 ASCII hex chars whose low bits the driver reads as
  flags: char0 = mode nibble (bit0 1=tracking/slew 0=goto, bit1 1=backward,
  bit2 1=highspeed), char1 bit0 = running, char2 bit0 = initialized.

Firmware quirks reproduced on purpose, because the driver has to survive them
and there is no other way to test that it does (protocol §§3.2, 7, 8, 10, 11):

- ``:E`` and ``:S`` are accepted **while the axis runs** (no ``!2``), and an
  ``:E`` sent on the move asynchronously clears the initialization flag a few
  hundred milliseconds later;
- ``:K`` brakes along a ramp: the Running bit stays up and the axis keeps
  travelling after the board answered ``=``;
- ``:I`` is clamped from below at 100x the sidereal period (1103 on this board),
  whatever was written; ``:i`` reads the stored value back;
- error code ``4 Not Initialized`` does not exist: the board never checks the
  flag, ``:J1`` without ``:F1`` is accepted;
- ``:C1<lo><hi>`` + ``:n1`` is a byte window into live memory, **without auto
  increment**: the address has to be set again before every read (§6 of the
  protocol document, and how the vendor's app does it);
- the Fast bit of the status is raised by the board itself from the *actual*
  step period and dropped when the axis stops — it is not an echo of the ``G``
  command (§10.2);
- any hex digit is accepted as a channel and answers like a second, inert axis;
  only a non-hex channel is an error (§3.2);
- two complete commands written **in one go** cost the second one: the board
  drops whatever arrives while it is answering (§8.1);
- the parser recognizes the command letter first and only then checks the
  argument, so a wrong length is ``!1`` while a missing channel is ``!0``
  (§8.2), and lower-case hex is ``!3`` (§7).

Kinematics are integrated over the injected virtual :class:`sim.clock.Clock`:
position is the integral of speed, speed in steps/s is
``timer_freq / step_period`` (times ``highspeed_ratio`` in highspeed mode), and
a GOTO run stops exactly on the target. No real time is ever consulted.

Deliberately **not** modelled, because the live board was never asked: the write
commands ``:W`` (Extended Setting), ``:N`` (Set EEPROM Value), ``:R`` (Set
Register Value) and ``:Q`` (bootloader). They answer ``!0`` here; on hardware
they most likely answer ``=``. Nothing in this project sends them.
"""

import math

from sim.clock import Clock
from sim.faults import FaultScript

ERR_UNKNOWN_COMMAND = 0
ERR_COMMAND_LENGTH = 1
ERR_NOT_STOPPED = 2
ERR_INVALID_CHARACTER = 3

_POSITION_OFFSET = 0x800000

# Constants of the RA board this project drives, measured in protocol §2:
# `:a1` -> CPR, `:b1` -> timer frequency, `:g1` -> highspeed ratio,
# `:e1` -> `=03110A` (firmware 0x0311, mount code 0x0A), `:c1` -> brake steps.
RA_CPR = 12_492_146
RA_TIMER_FREQ = 16_000_000
RA_HIGHSPEED_RATIO = 1
RA_MOUNT_CODE = 0x0A
RA_BOARD_VERSION = 0x0311
RA_BRAKE_STEPS = 16_980

# Supply voltages as `int16 little-endian, hundredths of a volt` in the `:C`/`:n`
# window — the format the vendor's app reads (RA_SA_CONSOLE_PROTOCOL.md §2) and
# the numbers our board answered with on 2026-07-25 (RA_PROTOCOL.md §6.8):
# battery 604 -> 6.04 V (multimeter 6.08 V at the same moment), USB 470 -> 4.70 V.
RA_BATTERY_VOLT_HUNDREDTHS = 604
RA_USB_VOLT_HUNDREDTHS = 470

# Addresses of the two 16-bit values inside that window, low byte first.
BATTERY_VOLT_ADDRESS = 0x0004
USB_VOLT_ADDRESS = 0x001C

# A sidereal day in seconds, the divisor of the 1x tracking period (§2: the
# board's own `:D1` = 110359 is reproduced by this expression to within 1).
_STELLAR_DAY_S = 86164.1

# Acceleration and braking, measured in step 2 §2.6 by sampling `:j1` every
# 50 ms through a whole 5-second run (`tools.ra_step2 ramp`). The board ramps
# the *commanded* rate linearly and steps at `min(commanded, ceiling)`:
#
#   * 1724 (64x): 2 000 -> ~9 000 steps/s in ~0.7 s, i.e. ~12 000 steps/s²;
#     after `:K1` the rate falls to zero in 0.62 s over 3 101 counts;
#   * 1103 (100x): the same ~9 000 steps/s ceiling, but after `:K1` the axis
#     keeps its rate for ~0.44 s before slowing — exactly the time the
#     commanded 14 506 needs to fall to the 9 000 the axis was really doing —
#     and then stops in 0.6 s, 6 886 counts past the `:K1`;
#   * 6897 (16x): no ramp at all. The first 66 ms sample already reads the full
#     2 320 steps/s, and `:K1` stops the axis inside one 50 ms poll.
#
# So the ramp belongs to the Fast bit, not to motion as such: below the Fast
# threshold the board starts and stops instantly. §10.3's "≈1148 counts of
# overshoot" was measured at a speed the board never actually reached.
# 13 000 steps/s² is the value that reproduces *both* stops: from period 1724
# it predicts 3 309 counts against the 3 101 measured, and from 1103 — 6 927
# against 6 886. The acceleration side agrees too (0 -> 9 000 in 0.69 s against
# the ~0.7 s sampled). Poll granularity of the measurement is ~55 ms, so the
# remaining few per cent is the measurement, not the model.
_RAMP_SPS_PER_S = 13_000.0
_BRAKE_STOP_SPS = 6.0

# Delay of the firmware quirk in §11: `:E` sent while the axis runs clears the
# initialization flag "asynchronously, hundreds of milliseconds later", which is
# why the same command sequence loses the flag in one run and keeps it in the
# next. A driver that re-reads `:f1` immediately after `:E` sees it still set.
_INIT_FLAG_RESET_DELAY_S = 0.3

# Step 2 §2.6: the board's own ceiling on step rate. Period 1103 (nominally
# 100x sidereal, 14 506 steps/s) drives the live axis at the same ~9 000
# steps/s as period 1724 does: 9 015 measured in the `ramp` phase, ~8 600 in
# `load`, and 8 847 at 1724. The period clamp of §10.5 therefore never reaches
# the real limit — the limit is lower and is not expressible as a period.
_SPEED_CEILING_SPS = 9_000.0

# Step 2 §11, §13, §15: **GOTO ignores the step period.** `:I1` is accepted in
# goto mode, reads back unchanged and is not obeyed — the controller picks the
# rate itself, exactly as the specification §3.3 says it does ("for GOTO mode,
# the motor controller handles this automatically"). Measured: a 20 000-count
# run commanded with `:I1` = 110 359 (145 steps/s) accelerated over ~1.1 s to
# ~8 600 steps/s, i.e. to the board's own ceiling, and a 1 000-count run
# commanded the same way peaked at ~7 300 steps/s. So the goto target rate is
# the ceiling, whatever period is loaded.
_GOTO_CRUISE_SPS = _SPEED_CEILING_SPS

# ...and the board slows down by itself on the approach, without being asked and
# without regard to `:m1`/`:M1`: the rate comes off the cruise about 10 000…16 000
# counts before the target and settles on a ~4 650 steps/s plateau it then holds
# all the way in (§13: 4 640 measured; §16: 4 611…4 667 on a 1 000-count run;
# §23: 4 547…4 824 over the last 4 000 counts of a 200 000-count run).
_GOTO_APPROACH_SPS = 4_650.0
_GOTO_APPROACH_TICKS = 10_000

# And then, in the last ~200 counts, it drops off that plateau **in one step** to
# the rate of the period that was loaded with `:I1` — 4 547 -> 145 steps/s
# between 121 and 100 counts remaining, and 145 steps/s is exactly
# `timer_freq / 110 359`, the period that run had written (step 2 §23). So `:I1`
# is not ignored after all: it is ignored for the cruise and obeyed for the
# final approach, which is what the specification's "the motor controller
# handles this automatically" (§3.3) turns out to mean in practice.
#
# **This step is where the board dies.** All five recorded crashes happened
# there: 150, 208 and 223 counts short of the target, at 4 600…5 200 steps/s,
# on a rate change of ~4 500 steps/s taken in one control period. The one run
# that got through the step to the crawl arrived cleanly (§23).
#
# 200 is the middle of the measured window: the transition was seen at ~110
# counts on the surviving run and at 147…223 on the crashes.
_GOTO_FINAL_TICKS = 200

# §10.2: the Fast bit of `:f1` follows the *loaded period*, not the `G` letter.
# Measured bracket: at period 6897 (16x sidereal) the bit stays down even while
# running, at 1724 (64x) it goes up — including when `G` asked for slow. The
# exact threshold is somewhere in between; the simulator puts it at 32x, the
# power of two in the middle of the measured interval. This is an assumption,
# the two endpoints are facts.
_FAST_SPEED_MULTIPLE = 32

# Motion mode char of the `G` command as the driver emits it
# (_MotionStatus.to_command): (slew_is_tracking, highspeed).
_MOTION_MODE_CHARS = {
    "0": (False, True),
    "1": (True, False),
    "2": (False, False),
    "3": (True, True),
}

_HEX_DIGITS = frozenset("0123456789ABCDEF")

# How many hex chars of argument each command letter takes. The parser checks
# the letter first and the length second (§8.2), so an unknown letter is `!0`
# whatever follows it, while a known letter with a wrong argument is `!1`.
# `k` is deliberately absent: it is documented in the spec but answers `!0` on
# this board (§4).
_DATA_LENGTHS = {
    # inquiries
    "a": 0, "b": 0, "c": 0, "d": 0, "D": 0, "e": 0, "f": 0, "g": 0, "h": 0,
    "i": 0, "j": 0, "m": 0, "n": 0, "r": 0, "s": 0,
    "q": 6,
    # setters
    "A": 2, "B": 1, "C": 4, "E": 6, "F": 0, "G": 2, "H": 6, "I": 6, "J": 0,
    "K": 0, "L": 0, "M": 6, "O": 1, "P": 1, "S": 6, "T": 6, "U": 6, "V": 2,
    "z": 0,
}

# Commands that are accepted verbatim on the live board (§3, "команды-сеттеры,
# проверенные при неподвижной оси") but whose effect was never observed through
# the protocol, so the simulator answers `=` and changes nothing.
_ACCEPTED_NO_EFFECT = frozenset("ABOPTUVz")

# Commands that act on one axis. On a phantom channel (§3.2) they are swallowed:
# the board answers `=` and the real axis 1 does not move a step.
_AXIS_SETTERS = frozenset("EFGHIJKLMS")

# Extended inquire IDs, §5. Only these five answer; everything else is `!0`.
# ID 3 is the USB voltage of the window with a constant high byte on top —
# that identity was verified byte for byte on the wire (§6.8), so it is
# assembled here rather than hard-coded.
_EXTENDED_INQUIRE = {
    0x000001: 0x008000,
    0x000002: 0x000010,
    0x000004: 0xCAC9A3,
    0x000005: 0x48DB49,
}
_EXTENDED_INQUIRE_USB_ID = 0x000003
_EXTENDED_INQUIRE_USB_HIGH_BYTE = 0x02

# §3.2: the phantom axis behind every hex channel that is not `1`. Measured:
# `:f2` = `=000`, `:j2` = `=000080`, plus the board-wide `:e2` and `:a2`, which
# answer exactly as on channel 1.
_PHANTOM_STATUS = "000"


def encode_revu24(value: int) -> str:
    if value < 0 or value > 0xFFFFFF:
        raise ValueError(f"expected value in range 0..{0xFFFFFF}, got {value}")
    return value.to_bytes(3, "little").hex().upper()


def decode_revu24(data: str) -> int:
    if len(data) == 2:
        data = f"{data}0000"
    if len(data) != 6:
        raise ValueError(f"expected 6 hex chars, got {data!r}")
    return int(data[4:6] + data[2:4] + data[0:2], 16)


def _capped_area(v_start: float, v_end: float, duration: float) -> float:
    """Distance covered while the rate slides from ``v_start`` to ``v_end``.

    Everything above :data:`_SPEED_CEILING_SPS` is flattened to the ceiling,
    because that is what the axis does: the commanded rate keeps rising past
    9 000 steps/s, the stepping does not. Without the flattening the simulator
    would credit the axis with distance it never travels — and the visible
    consequence, the ~0.44 s of unchanged speed after `:K1` from period 1103,
    would disappear.
    """
    if duration <= 0:
        return 0.0
    cap = _SPEED_CEILING_SPS
    low, high = (v_start, v_end) if v_start <= v_end else (v_end, v_start)
    if high <= cap:
        return (v_start + v_end) / 2 * duration
    if low >= cap:
        return cap * duration
    # One crossing: a flat stretch at the ceiling plus a trapezoid below it.
    above = (high - cap) / (high - low) * duration
    return cap * above + (cap + low) / 2 * (duration - above)


class SkyWatcherSim:
    """Simulated SkyWatcher motor board behind a :class:`sim.fake_serial.FakeSerial`.

    Out of the box this is *our* RA board: the constants default to the ones
    read off the live controller (§2), so a test that passes nothing at all
    exercises the hardware the project actually drives. Tests that need round
    arithmetic instead of the real 12 492 146 counts per revolution override
    ``cpr``/``timer_freq``/``highspeed_ratio`` explicitly.

    ``mount_code`` is reported by ``:e1`` and nothing else: the driver no longer
    keeps a min-period table keyed by it — it measures the clamp with
    ``:I1``/``:i1`` instead (§10.5). Scripted faults (empty/truncated reply,
    dead motor) go through :attr:`faults`.
    """

    def __init__(
        self,
        clock: Clock,
        *,
        cpr: int = RA_CPR,
        timer_freq: int = RA_TIMER_FREQ,
        highspeed_ratio: int = RA_HIGHSPEED_RATIO,
        mount_code: int = RA_MOUNT_CODE,
        board_version: int = RA_BOARD_VERSION,
        position_steps: int = 0,
        battery_volt_hundredths: int = RA_BATTERY_VOLT_HUNDREDTHS,
        usb_volt_hundredths: int = RA_USB_VOLT_HUNDREDTHS,
        brake_steps: int = RA_BRAKE_STEPS,
        reboot_on_goto_arrival: bool = False,
    ) -> None:
        self._clock = clock
        self.faults = FaultScript()
        # Off by default, because it is not something *every* SkyWatcher board
        # does — it is what **this** controller did four times out of four
        # (step 2 §13, §16). See :meth:`_reboot_on_arrival`.
        self.reboot_on_goto_arrival = reboot_on_goto_arrival

        self.cpr = cpr
        self.timer_freq = timer_freq
        self.highspeed_ratio = highspeed_ratio
        self.mount_code = mount_code
        self.board_version = board_version
        self.brake_steps = brake_steps

        self.initialized = False
        self.running = False
        self.tracking_mode = True
        self.backward = False
        self.highspeed = False
        # §10.5: whatever is written with `:I`, the board stores at least
        # `tracking_period_1x // 100` — it refuses to run faster than 100x
        # sidereal. On this board that is exactly the measured 1103.
        self.tracking_period_1x = max(1, int(round(_STELLAR_DAY_S * timer_freq / cpr)))
        self.min_period = max(1, self.tracking_period_1x // 100)
        self.step_period = self.tracking_period_1x
        self.target_increment = 0
        self.brake_increment = 0
        self.goto_target: float | None = None
        self.goto_target_position = 0.0
        self._target_from_increment = False
        self.position = float(position_steps)
        self.braking = False

        # `:C`/`:n` window. The address survives a read (no auto increment), so a
        # driver that reads two bytes without setting the address twice gets the
        # same byte twice — exactly what the live board does.
        self.battery_volt_hundredths = battery_volt_hundredths
        self.usb_volt_hundredths = usb_volt_hundredths
        self.memory_address = 0
        self.memory: dict[int, int] = {}

        self._commanded_sps = 0.0
        self._init_reset_at: float | None = None
        self._integrated_at = clock.now
        self._rx = bytearray()
        self._tx = bytearray()

    def reboot(self) -> None:
        """Power-cycle the controller: §12.1, the state the board comes up in.

        Every observable the driver has goes back to what was measured on the
        live board at the start of the session — position exactly `0x800000`,
        initialization flag down, step period at the 1x tracking value, axis
        stopped in tracking mode — and nothing else changes, because a reboot
        does not alter the board's constants or the memory window.

        This is the only way to produce the three signs of §12.2 at once, and
        thus the only test bench the reboot detection has: the live board was
        disconnected when it was written (Э5 of ``docs/RA_REWRITE_PLAN.md``).
        """
        self.initialized = False
        self.running = False
        self.braking = False
        self.tracking_mode = True
        self.backward = False
        self.highspeed = False
        self.position = 0.0
        self.step_period = self.tracking_period_1x
        self.target_increment = 0
        self.brake_increment = 0
        self.goto_target = None
        self.goto_target_position = 0.0
        self._target_from_increment = False
        self._commanded_sps = 0.0
        self._init_reset_at = None
        self.memory_address = 0
        # The RX buffer is not carried across a reset either: a command half sent
        # when the power dipped is not completed by the board that comes back.
        self._rx.clear()

    def feed(self, data: bytes) -> None:
        for index, byte in enumerate(data):
            if byte == ord(":"):
                # Reset-on-second-colon per the reference: a new ':' abandons
                # the previous partial command.
                self._rx = bytearray(b":")
                continue
            if not self._rx:
                continue
            if byte == ord("\r"):
                command = self._rx[1:].decode("ascii", errors="replace")
                self._rx.clear()
                self._handle(command)
                # §8.1: the board does not buffer. Whatever else the host put on
                # the wire in the same write is lost while the board is busy
                # answering — reproduced four times on hardware, and the reason
                # every driver here has to be strictly request/response.
                if index + 1 < len(data):
                    return
                continue
            # Only `\r` terminates a command (§8): `#` and everything else is
            # just another data byte, so `:fL#` leaves the board silent.
            self._rx.append(byte)

    def drain(self) -> bytes:
        data = bytes(self._tx)
        self._tx.clear()
        return data

    def on_dtr(self, level: bool) -> None:
        # SkyWatcher boards do not use the DTR line.
        return None

    def requested_sps(self) -> float:
        """Rate the loaded period asks for, before the board's own ceiling.

        This is what the *commanded* velocity ramps towards; what the axis
        actually steps at is :meth:`speed_sps`.
        """
        sps = self.timer_freq / self.step_period
        if self.highspeed:
            sps *= self.highspeed_ratio
        return sps

    def speed_sps(self) -> float:
        """Steady-state rate of the loaded period, in steps/s.

        The board's own ceiling (step 2 §2.6) is applied here, so this is what
        the axis really does once the ramp is over — not what the period asks
        for. During the ramp the instantaneous rate is lower; while braking it
        decays from here to zero.
        """
        return min(self.requested_sps(), _SPEED_CEILING_SPS)

    def ramps(self) -> bool:
        """Whether this speed is ramped at all (step 2 §2.6, §13).

        In *tracking* mode the threshold is the Fast bit's, and for the same
        reason: on the live board 6897 starts and stops instantly, 1724 and 1103
        do not. Using one predicate for both keeps the simulator from claiming a
        combination that was never seen — a ramped run with the Fast bit down.

        In *goto* mode there is no threshold, because there is no period to take
        it from: the board ignores `:I1` and drives to its own rate (§11), and
        the one fully sampled goto took ~1.1 s to get there from a standstill
        (§13) even though the loaded period asked for 145 steps/s.
        """
        if not self.tracking_mode:
            return True
        return self.step_period * _FAST_SPEED_MULTIPLE <= self.tracking_period_1x

    def fast_bit(self) -> bool:
        """Whether `:f1` reports Fast, the way the board decides it (§10.2).

        Not what `G` asked for: the board looks at the period it is actually
        stepping with, and it drops the bit the moment the axis stops.
        """
        return self.running and self.step_period * _FAST_SPEED_MULTIPLE <= self.tracking_period_1x

    def _memory_byte(self, address: int) -> int:
        """One byte of the `:C`/`:n` window.

        The two voltages live there as int16 little-endian in hundredths of a
        volt, so they are assembled from the current attributes on every read —
        a test that changes :attr:`battery_volt_hundredths` mid-session sees the
        new value, as it would on a discharging battery.
        """
        for base, value in (
            (BATTERY_VOLT_ADDRESS, self.battery_volt_hundredths),
            (USB_VOLT_ADDRESS, self.usb_volt_hundredths),
        ):
            if address == base:
                return value & 0xFF
            if address == base + 1:
                return (value >> 8) & 0xFF
        return self.memory.get(address, 0)

    def _integrate(self) -> None:
        now = self._clock.now
        dt = now - self._integrated_at
        self._integrated_at = now
        if dt > 0:
            self._advance(dt)

        # §11: the initialization flag drops on its own timer, so how a driver
        # sees it depends on how long after the `:E` it asked.
        if self._init_reset_at is not None and now >= self._init_reset_at:
            self.initialized = False
            self._init_reset_at = None

    def _advance(self, dt: float) -> None:
        if not self.running:
            return
        sign = -1.0 if self.backward else 1.0

        if self.braking:
            # §10.3: `K` starts a ramp. The board answers `=` at once but the
            # Running bit stays up and the axis keeps coasting. The ramp is
            # linear in the *commanded* rate and the axis steps at the capped
            # one, which is why a stop from 1103 begins with almost half a
            # second at unchanged speed (step 2 §2.6).
            travelled = self._ramp_commanded(0.0, dt)
            self.position += sign * travelled
            if self._commanded_sps < _BRAKE_STOP_SPS:
                self._commanded_sps = 0.0
                self.braking = False
                self.running = False
                # A channel that actually *moved* in goto comes back in tracking
                # mode, exactly as the reference's note *4 says (step 2 §14: two
                # GOTO runs cut short by `:K1` both ended `:f1` = `=101`). §2.4
                # is not contradicted — it stopped a goto channel that had never
                # been started, and such a channel has no run to come back from,
                # which is why the flip lives here and not in the `K` handler.
                self.tracking_mode = True
            return

        if not self.tracking_mode and self.goto_target is not None:
            # Distance left *in the direction of travel*: a target that lies
            # behind is reached the long way round, exactly as a counter that
            # wraps at CPR would.
            remaining = (sign * (self.goto_target - self.position)) % self.cpr
            travel = self._ramp_commanded(self._goto_commanded_sps(remaining), dt)
            if self.reboot_on_goto_arrival and travel >= remaining - _GOTO_FINAL_TICKS:
                # The board never gets to make the step down onto the final
                # crawl: this is the instant all five recorded crashes happened.
                # Tested against the *crossing*, not against the position at the
                # top of the step — a caller that polls once a second would
                # otherwise step straight over the window and the board would
                # look immortal.
                self.position += sign * max(0.0, remaining - _GOTO_FINAL_TICKS)
                self._reboot_on_arrival(sign, min(self._commanded_sps, _SPEED_CEILING_SPS))
                return
            if travel >= remaining:
                self.position = self.goto_target
                self.goto_target = None
                self.running = False
                # The ramp state has to go with the run: without this the next
                # `:J1` would start at the rate this one ended on, and the
                # acceleration of step 2 §2.6 would silently disappear.
                self._commanded_sps = 0.0
                # Auto-return to tracking (speed) mode after the motor stops.
                self.tracking_mode = True
                return
            self.position += sign * travel
            return

        self.position += sign * self._ramp_commanded(self.requested_sps(), dt)

    def _goto_commanded_sps(self, remaining: float) -> float:
        """Rate a goto run is heading for, given how far it still has to go.

        Neither half of this comes from the host: `:I1` is ignored in goto mode
        (§11) and `:m1`/`:M1` have no effect on the deceleration (§2.2, §11).
        The board cruises at its own ceiling and comes off it on its own about
        :data:`_GOTO_APPROACH_TICKS` before the target.
        """
        if remaining <= _GOTO_APPROACH_TICKS:
            return _GOTO_APPROACH_SPS
        return _GOTO_CRUISE_SPS

    def _reboot_on_arrival(self, sign: float, arrival_sps: float) -> None:
        """The failure of step 2 §13/§16, reproduced: the board dies on arrival.

        Four crashes out of four landed within 150…208 counts of the target,
        after seconds of healthy running, with the axis going at a steady
        ~4 650 steps/s — no ramp, no acceleration, no sag on either supply
        channel. What the board does at that instant is a reset, and the
        counter comes back at the logical zero **plus the distance the axis
        coasted while the controller was down**: +645, -1241, +448 and +216
        were measured at 4 600…8 700 steps/s, and -7 at 145 steps/s (§3.3,
        §18, §26).

        That coast is modelled as a free stop under the board's own measured
        brake ramp, 13 000 steps/s² (§2.6): from 4 650 steps/s it gives 831
        counts, which sits inside the measured 448…1 241 band. The mechanism
        behind the reset is *not* modelled and is not known — the supply spike
        the owner suspects lasts milliseconds and the `:C`/`:n` window updates
        once a second, so it could not have been caught (§16).

        Not modelled either: the ~0.5 s of silence the live board goes through
        before it answers again. The simulator comes back instantly, so a driver
        that relies on the timeout to notice a reset would pass here and fail on
        hardware — which is why the detection keys on state, not on timing.
        """
        coast = arrival_sps ** 2 / (2 * _RAMP_SPS_PER_S)
        self.reboot()
        self.position = sign * coast

    def _ramp_commanded(self, target_sps: float, dt: float) -> float:
        """Move the commanded rate towards ``target_sps`` and return the travel.

        Below the Fast threshold the commanded rate jumps: the live board at
        6897 was already at full speed 66 ms after `:J1` and stopped inside one
        50 ms poll. Above it the rate slides at :data:`_RAMP_SPS_PER_S`, and the
        distance is integrated over the *capped* rate, because that is what the
        axis actually steps at.
        """
        if not self.ramps():
            self._commanded_sps = target_sps
            return min(target_sps, _SPEED_CEILING_SPS) * dt

        started_at = self._commanded_sps
        # The ramp may end inside this step. Integrating the whole step as one
        # trapezoid would then keep "spending" a speed the axis no longer has —
        # a 1 s poll over a 0.6 s stop overshot by 40% before this split.
        ramp_s = min(dt, abs(target_sps - started_at) / _RAMP_SPS_PER_S)
        rate = _RAMP_SPS_PER_S if target_sps > started_at else -_RAMP_SPS_PER_S
        self._commanded_sps = started_at + rate * ramp_s
        if ramp_s >= dt:
            return _capped_area(started_at, self._commanded_sps, ramp_s)
        self._commanded_sps = target_sps
        return _capped_area(started_at, target_sps, ramp_s) + min(target_sps, _SPEED_CEILING_SPS) * (dt - ramp_s)

    def _reply(self, command: str, body: str = "", error: int | None = None) -> None:
        if error is not None:
            line = f"!{error}\r".encode("ascii")
        else:
            line = f"={body}\r".encode("ascii")
        self._tx.extend(self.faults.apply(line, b"\r", command))

    def _raw_position(self, steps: float) -> int:
        """The 24-bit counter, the way the live board reports it.

        The reduction is *centred* on purpose: a position a little below the
        logical zero has to read a little below `0x800000`, because that is what
        the board did. After the reboot of §3.3 it answered `=F9FF7F`, i.e.
        `0x7FFFF9` = zero minus 7, and after the one in §26 `0x8000D8` = zero
        plus 216. Folding into `0..cpr` first instead would turn that -7 into
        12 492 139 and put the counter on the far side of the axis — which is
        exactly the reading the reboot detection has to recognize.
        """
        steps_i = int(round(steps)) % self.cpr
        if steps_i > self.cpr // 2:
            steps_i -= self.cpr
        return (steps_i + _POSITION_OFFSET) % 0x1000000

    def _position_body(self, steps: float) -> str:
        return encode_revu24(self._raw_position(steps))

    def _goto_target_steps(self) -> float:
        """The target the `:h1` register holds and `:J1` would run to.

        One register, two ways to write it: `:S` puts an absolute position in
        it, `:H` an increment from wherever the axis is when the run starts.
        Whichever came last wins.
        """
        if self._target_from_increment:
            sign = -1.0 if self.backward else 1.0
            return self.position + sign * self.target_increment
        return self.goto_target_position

    def _handle(self, command: str) -> None:
        self._integrate()

        # §7/§8.2: an empty command and a command without a channel byte are
        # both "Unknown Command", not a length error — the board decides that a
        # packet this short is not a command at all.
        if not command:
            self._reply(command, error=ERR_UNKNOWN_COMMAND)
            return
        cmd = command[0]
        if cmd not in _DATA_LENGTHS:
            self._reply(cmd, error=ERR_UNKNOWN_COMMAND)
            return
        if len(command) < 2:
            self._reply(cmd, error=ERR_UNKNOWN_COMMAND)
            return
        channel = command[1]
        if channel not in _HEX_DIGITS:
            self._reply(cmd, error=ERR_INVALID_CHARACTER)
            return
        data = command[2:]
        if len(data) != _DATA_LENGTHS[cmd]:
            self._reply(cmd, error=ERR_COMMAND_LENGTH)
            return
        if not set(data) <= _HEX_DIGITS:
            # Lower case included: `:I1abcdef` is `!3` on this board (§7).
            self._reply(cmd, error=ERR_INVALID_CHARACTER)
            return

        if channel != "1":
            self._handle_phantom(cmd, data)
            return
        self._handle_axis(cmd, data)

    def _handle_phantom(self, cmd: str, data: str) -> None:
        """§3.2: every hex channel answers, but only channel 1 is a real axis.

        Measured on the live board: `:f2`/`:f3`/`:f0`/`:f4`/`:f9` all answer
        `=000`, `:j2` answers `=000080`, and the board-wide `:e2` and `:a2`
        answer exactly what channel 1 does. The rest of this mapping is an
        assumption of the same shape — board constants are board-wide, axis
        state reads as a stopped, uninitialized axis parked at logical zero,
        and setters are swallowed.

        The property that matters to the driver is the last one: a command sent
        to the wrong channel is lost silently, so mistyping a channel can never
        be caught by watching for an error (which is why the driver pins it).
        """
        if cmd == "f":
            self._reply(cmd, _PHANTOM_STATUS)
        elif cmd in ("j", "d", "h", "m"):
            self._reply(cmd, encode_revu24(_POSITION_OFFSET))
        elif cmd == "i":
            self._reply(cmd, encode_revu24(self.tracking_period_1x))
        elif cmd in _AXIS_SETTERS:
            self._reply(cmd)
        else:
            self._handle_board_wide(cmd, data)

    def _handle_board_wide(self, cmd: str, data: str) -> None:
        """Answers that do not depend on which axis is being addressed."""
        if cmd == "e":
            self._reply(cmd, f"{(self.board_version << 8) | self.mount_code:06X}")
        elif cmd == "a":
            self._reply(cmd, encode_revu24(self.cpr))
        elif cmd == "b":
            self._reply(cmd, encode_revu24(self.timer_freq))
        elif cmd == "g":
            self._reply(cmd, f"{self.highspeed_ratio:02X}")
        elif cmd == "D":
            self._reply(cmd, encode_revu24(self.tracking_period_1x))
        elif cmd == "c":
            self._reply(cmd, encode_revu24(self.brake_steps))
        elif cmd == "s":
            # §2: PEC period reads zero — this board has no PEC at all (§5.1).
            self._reply(cmd, encode_revu24(0))
        elif cmd == "r":
            # §13 #23: the `:A`/`:r` register file is a stub, all 256 addresses
            # read back `=00`.
            self._reply(cmd, "00")
        elif cmd == "n":
            # One byte at the current address, two hex chars, and the address is
            # left where it was: the vendor re-sends `:C` before every `:n`.
            self._reply(cmd, f"{self._memory_byte(self.memory_address):02X}")
        elif cmd == "C":
            # Set the window address: four hex chars, low byte first — `:C11C00`
            # is address 0x001C. The reply carries no data (`=\r` on the wire).
            self.memory_address = int(data[2:4] + data[0:2], 16)
            self._reply(cmd)
        elif cmd == "q":
            self._handle_extended_inquire(cmd, data)
        elif cmd in _ACCEPTED_NO_EFFECT:
            self._reply(cmd)
        else:  # pragma: no cover - every letter of _DATA_LENGTHS is handled above
            self._reply(cmd, error=ERR_UNKNOWN_COMMAND)

    def _handle_extended_inquire(self, cmd: str, data: str) -> None:
        identifier = decode_revu24(data)
        if identifier == _EXTENDED_INQUIRE_USB_ID:
            value = (_EXTENDED_INQUIRE_USB_HIGH_BYTE << 16) | (self.usb_volt_hundredths & 0xFFFF)
            self._reply(cmd, encode_revu24(value))
            return
        answer = _EXTENDED_INQUIRE.get(identifier)
        if answer is None:
            # §5: only IDs 1..5 exist; ID 0 (axis indexer) included, this board
            # answers `!0` to everything else.
            self._reply(cmd, error=ERR_UNKNOWN_COMMAND)
            return
        self._reply(cmd, encode_revu24(answer))

    def _handle_axis(self, cmd: str, data: str) -> None:
        if cmd == "F":
            self.initialized = True
            self._reply(cmd)
        elif cmd == "j":
            self._reply(cmd, self._position_body(self.position))
        elif cmd == "d":
            # Step 2 §2.3: `:d1` is not a position. It answers `=000080` with
            # the axis parked at 0x81A172, after an `:E1` that set something
            # else, and after hundreds of thousands of counts travelled. §3 read
            # it equal to `:j1` only because the axis stood at the logical zero.
            self._reply(cmd, encode_revu24(_POSITION_OFFSET))
        elif cmd == "h":
            self._reply(cmd, self._position_body(self._goto_target_steps()))
        elif cmd == "m":
            # Step 2 §2.2: the brake point is derived from the **target**
            # (`:h1`), not from the position, with the sign taken from the
            # direction. §3.1 could not tell the two apart because position and
            # target both sat at `0x800000` in that session; step 2 separated
            # them on the live board — `:H1` of +50 000 moved `:h1` and `:m1`
            # together while the position never changed.
            # The subtraction is done on the raw 24-bit counter (`0x800000 -
            # 0x7FBDAC = 0x4254` exactly), i.e. not reduced modulo CPR.
            sign = 1 if self.backward else -1
            raw = self._raw_position(self._goto_target_steps())
            self._reply(cmd, encode_revu24((raw + sign * self.brake_steps) % 0x1000000))
        elif cmd == "f":
            mode_nibble = (
                (1 if self.tracking_mode else 0)
                | (2 if self.backward else 0)
                | (4 if self.fast_bit() else 0)
            )
            self._reply(cmd, f"{mode_nibble}{1 if self.running else 0}{1 if self.initialized else 0}")
        elif cmd == "I":
            self.step_period = max(self.min_period, decode_revu24(data))
            self._reply(cmd)
        elif cmd == "i":
            self._reply(cmd, encode_revu24(self.step_period))
        elif cmd == "H":
            self.target_increment = decode_revu24(data)
            self._target_from_increment = True
            self._reply(cmd)
        elif cmd == "M":
            # Step 2 §2.2: the increment is stored and used **nowhere** — that
            # is the board's behaviour, not a simplification. The vendor app
            # sends `:M1AC0D00` (3 500) before every `:J1` and `:m1` does not
            # move afterwards: the braking distance on this board is hard-wired
            # to `:c1` = 16 980.
            self.brake_increment = decode_revu24(data)
            self._reply(cmd)
        elif cmd == "E":
            # §11/§13 #7: the spec demands a stopped motor, the board does not
            # enforce it — it answers `=` and, if the axis was running, drops
            # the initialization flag a few hundred milliseconds later. The
            # protection has to live in the host, so the simulator must not
            # provide it here.
            self.position = float((decode_revu24(data) - _POSITION_OFFSET) % self.cpr)
            if self.running:
                self._init_reset_at = self._clock.now + _INIT_FLAG_RESET_DELAY_S
            self._reply(cmd)
        elif cmd == "S":
            # §13 #8: `S` is likewise accepted on the move. It writes the same
            # target register `:H` does, only absolutely, and a `:J` in goto
            # mode runs to it.
            self.goto_target_position = float((decode_revu24(data) - _POSITION_OFFSET) % self.cpr)
            self._target_from_increment = False
            self._reply(cmd)
        elif cmd == "G":
            if data[0] not in _MOTION_MODE_CHARS or data[1] not in ("0", "1"):
                self._reply(cmd, error=ERR_INVALID_CHARACTER)
                return
            tracking_mode, highspeed = _MOTION_MODE_CHARS[data[0]]
            backward = data[1] == "1"
            if self.running and (tracking_mode, highspeed, backward) != (self.tracking_mode, self.highspeed, self.backward):
                self._reply(cmd, error=ERR_NOT_STOPPED)
                return
            self.tracking_mode = tracking_mode
            self.highspeed = highspeed
            self.backward = backward
            self._reply(cmd)
        elif cmd == "J":
            # §7/§13 #9: there is no `!4 Not Initialized` on this board — `:J1`
            # without a preceding `:F1` starts the axis just the same. The
            # initialization flag is an indicator, not a guard.
            self.running = True
            self.braking = False
            if not self.tracking_mode:
                self.goto_target = self._goto_target_steps()
            self._reply(cmd)
        elif cmd in ("K", "L"):
            # §10.3: braking is a ramp, not a switch. The channel returns to
            # tracking mode immediately (reference §5.1 note *4, confirmed on
            # hardware), but Running stays up until the ramp has run out.
            # `:L` (Instant Stop) is answered `=` by the board (§3) and is
            # modelled as `:K`: its ramp, if any, was never measured.
            if self.running and not self.braking:
                self.braking = True
            self.goto_target = None
            # `K` by itself does NOT change the mode (step 2 §2.4): on a goto
            # channel that was never started, `:G120` -> `:K1` -> `:f1` answers
            # `=001`. The mode comes back to tracking only when a channel that
            # was *running* comes to a stop, which `_advance` does at the end of
            # the ramp — see the note there and step 2 §14.
            self._reply(cmd)
        else:
            self._handle_board_wide(cmd, data)
