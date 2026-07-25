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
and there is no other way to test that it does (protocol §§7, 10.3, 10.5, 11):

- ``:E`` and ``:S`` are accepted **while the axis runs** (no ``!2``), and an
  ``:E`` sent on the move asynchronously clears the initialization flag a few
  hundred milliseconds later;
- ``:K`` brakes along a ramp: the Running bit stays up and the axis keeps
  travelling after the board answered ``=``;
- ``:I`` is clamped from below at 100x the sidereal period (1103 on this board),
  whatever was written; ``:i`` reads the stored value back;
- error code ``4 Not Initialized`` does not exist: the board never checks the
  flag, ``:J1`` without ``:F1`` is accepted.

Kinematics are integrated over the injected virtual :class:`sim.clock.Clock`:
position is the integral of speed, speed in steps/s is
``timer_freq / step_period`` (times ``highspeed_ratio`` in highspeed mode), and
a GOTO run stops exactly on the target. No real time is ever consulted.
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
# `:e1` -> `=03110A` (firmware 0x0311, mount code 0x0A).
RA_CPR = 12_492_146
RA_TIMER_FREQ = 16_000_000
RA_HIGHSPEED_RATIO = 1
RA_MOUNT_CODE = 0x0A
RA_BOARD_VERSION = 0x0311

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

# Motion mode char of the `G` command as the driver emits it
# (_MotionStatus.to_command): (slew_is_tracking, highspeed).
_MOTION_MODE_CHARS = {
    "0": (False, True),
    "1": (True, False),
    "2": (False, False),
    "3": (True, True),
}


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
    ) -> None:
        self._clock = clock
        self.faults = FaultScript()

        self.cpr = cpr
        self.timer_freq = timer_freq
        self.highspeed_ratio = highspeed_ratio
        self.mount_code = mount_code
        self.board_version = board_version

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
        self.goto_target_position: float | None = None
        self.position = float(position_steps)
        self.braking = False

        self._brake_speed_sps = 0.0
        self._init_reset_at: float | None = None
        self._integrated_at = clock.now
        self._rx = bytearray()
        self._tx = bytearray()

    def feed(self, data: bytes) -> None:
        for byte in data:
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
            remaining = abs(self.goto_target - self.position)
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

    def _handle(self, command: str) -> None:
        self._integrate()

        if not command:
            return
        cmd = command[0]
        if len(command) < 2:
            self._reply(cmd, error=ERR_COMMAND_LENGTH)
            return
        if command[1] != "1":
            self._reply(cmd, error=ERR_INVALID_CHARACTER)
            return
        data = command[2:]

        if cmd == "F":
            self.initialized = True
            self._reply(cmd)
        elif cmd == "e":
            self._reply(cmd, f"{(self.board_version << 8) | self.mount_code:06X}")
        elif cmd == "a":
            self._reply(cmd, encode_revu24(self.cpr))
        elif cmd == "b":
            self._reply(cmd, encode_revu24(self.timer_freq))
        elif cmd == "g":
            self._reply(cmd, f"{self.highspeed_ratio:02X}")
        elif cmd == "j":
            raw = (int(round(self.position)) % self.cpr + _POSITION_OFFSET) % 0x1000000
            self._reply(cmd, encode_revu24(raw))
        elif cmd == "f":
            mode_nibble = (
                (1 if self.tracking_mode else 0)
                | (2 if self.backward else 0)
                | (4 if self.highspeed else 0)
            )
            self._reply(cmd, f"{mode_nibble}{1 if self.running else 0}{1 if self.initialized else 0}")
        elif cmd == "I":
            try:
                period = decode_revu24(data)
            except ValueError:
                self._reply(cmd, error=ERR_INVALID_CHARACTER)
                return
            self.step_period = max(self.min_period, period)
            self._reply(cmd)
        elif cmd == "i":
            self._reply(cmd, encode_revu24(self.step_period))
        elif cmd == "H":
            try:
                self.target_increment = decode_revu24(data)
            except ValueError:
                self._reply(cmd, error=ERR_INVALID_CHARACTER)
                return
            self._reply(cmd)
        elif cmd == "M":
            try:
                self.brake_increment = decode_revu24(data)
            except ValueError:
                self._reply(cmd, error=ERR_INVALID_CHARACTER)
                return
            self._reply(cmd)
        elif cmd == "E":
            # §11/§13 #7: the spec demands a stopped motor, the board does not
            # enforce it — it answers `=` and, if the axis was running, drops
            # the initialization flag a few hundred milliseconds later. The
            # protection has to live in the host, so the simulator must not
            # provide it here.
            try:
                raw = decode_revu24(data)
            except ValueError:
                self._reply(cmd, error=ERR_INVALID_CHARACTER)
                return
            self.position = float((raw - _POSITION_OFFSET) % self.cpr)
            if self.running:
                self._init_reset_at = self._clock.now + _INIT_FLAG_RESET_DELAY_S
            self._reply(cmd)
        elif cmd == "S":
            # §13 #8: `S` is likewise accepted on the move. The target it sets
            # is recorded but unused: GOTO runs are driven by the `H` increment
            # the driver sends, and absolute-target GOTO was never probed on
            # the live board (§14).
            try:
                raw = decode_revu24(data)
            except ValueError:
                self._reply(cmd, error=ERR_INVALID_CHARACTER)
                return
            self.goto_target_position = float((raw - _POSITION_OFFSET) % self.cpr)
            self._reply(cmd)
        elif cmd == "G":
            if len(data) != 2:
                self._reply(cmd, error=ERR_COMMAND_LENGTH)
                return
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
                sign = -1.0 if self.backward else 1.0
                self.goto_target = self.position + sign * self.target_increment
            self._reply(cmd)
        elif cmd == "K":
            # §10.3: braking is a ramp, not a switch. The channel returns to
            # tracking mode immediately (reference §5.1 note *4, confirmed on
            # hardware), but Running stays up until the ramp has run out.
            if self.running and not self.braking:
                self.braking = True
                self._brake_speed_sps = self.speed_sps()
            self.goto_target = None
            self.tracking_mode = True
            self._reply(cmd)
        else:
            self._reply(cmd, error=ERR_UNKNOWN_COMMAND)
