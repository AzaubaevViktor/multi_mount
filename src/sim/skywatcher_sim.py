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

# Braking model of `K`, fitted to the two numbers measured in §10.3: at 14506
# steps/s (the board's own top speed) the axis overshoots ~1148 counts, and
# 0.6s after `K` the Running bit is still up — while at 64x sidereal (9280
# steps/s) it is already down. A speed decaying as `v0 * exp(-t / tau)` matches
# both: total travel is `v0 * tau` = 1148 counts, and the time to fall under
# `_BRAKE_STOP_SPS` is 0.616s from 14506 steps/s but 0.581s from 9280.
_BRAKE_TAU_S = 0.0791
_BRAKE_STOP_SPS = 6.0

# Delay of the firmware quirk in §11: `:E` sent while the axis runs clears the
# initialization flag "asynchronously, hundreds of milliseconds later", which is
# why the same command sequence loses the flag in one run and keeps it in the
# next. A driver that re-reads `:f1` immediately after `:E` sees it still set.
_INIT_FLAG_RESET_DELAY_S = 0.3

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
    ) -> None:
        self._clock = clock
        self.faults = FaultScript()

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

        self._brake_speed_sps = 0.0
        self._init_reset_at: float | None = None
        self._integrated_at = clock.now
        self._rx = bytearray()
        self._tx = bytearray()

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

    def speed_sps(self) -> float:
        """Speed the currently loaded step period asks for, in steps/s.

        This is the commanded speed, not the instantaneous one: while the axis
        brakes (:attr:`braking`) it decays from here down to zero.
        """
        sps = self.timer_freq / self.step_period
        if self.highspeed:
            sps *= self.highspeed_ratio
        return sps

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
            # Running bit stays up and the axis keeps coasting.
            decay = math.exp(-dt / _BRAKE_TAU_S)
            self.position += sign * self._brake_speed_sps * _BRAKE_TAU_S * (1.0 - decay)
            self._brake_speed_sps *= decay
            if self._brake_speed_sps < _BRAKE_STOP_SPS:
                self._brake_speed_sps = 0.0
                self.braking = False
                self.running = False
            return

        travel = self.speed_sps() * dt
        if not self.tracking_mode and self.goto_target is not None:
            # Distance left *in the direction of travel*: a target that lies
            # behind is reached the long way round, exactly as a counter that
            # wraps at CPR would.
            remaining = (sign * (self.goto_target - self.position)) % self.cpr
            if travel >= remaining:
                self.position = self.goto_target
                self.goto_target = None
                self.running = False
                # Auto-return to tracking (speed) mode after the motor stops.
                self.tracking_mode = True
                return
        self.position += sign * travel

    def _reply(self, command: str, body: str = "", error: int | None = None) -> None:
        if error is not None:
            line = f"!{error}\r".encode("ascii")
        else:
            line = f"={body}\r".encode("ascii")
        self._tx.extend(self.faults.apply(line, b"\r", command))

    def _raw_position(self, steps: float) -> int:
        return (int(round(steps)) % self.cpr + _POSITION_OFFSET) % 0x1000000

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
            # §3: `:d1` answered exactly what `:j1` did in every reading.
            self._reply(cmd, self._position_body(self.position))
        elif cmd == "h":
            self._reply(cmd, self._position_body(self._goto_target_steps()))
        elif cmd == "m":
            # §3.1: the brake point is `position ± brake steps`, the sign taken
            # from the direction — a derived value, not a constant. The board
            # does the subtraction on the raw 24-bit counter (`0x800000 -
            # 0x7FBDAC = 0x4254` exactly), so it is not reduced modulo CPR.
            sign = 1 if self.backward else -1
            raw = self._raw_position(self.position)
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
            self._brake_speed_sps = 0.0
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
                self._brake_speed_sps = self.speed_sps()
            self.goto_target = None
            self.tracking_mode = True
            self._reply(cmd)
        else:
            self._handle_board_wide(cmd, data)
