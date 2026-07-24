"""SkyWatcher motor-controller endpoint simulator (RA axis, channel 1).

Protocol per ``src/skywatcher/motor.py`` (ground truth for what the driver
sends/expects) and ``references/skywatcher_motor_controller_command_set_extracted.md``:

- command ``:<cmd>1<data>\\r`` -> ``=<data>\\r`` on success, ``!<code>\\r`` on error;
- 24-bit values travel as Revu24 (byte-reversed hex), positions offset by 0x800000;
- the non-standard voltage inquiry ``:fL#`` answers ``<2 hex>#`` (terminator ``#``);
- status body of ``:f1`` is 3 ASCII hex chars whose low bits the driver reads as
  flags: char0 = mode nibble (bit0 1=tracking/slew 0=goto, bit1 1=backward,
  bit2 1=highspeed), char1 bit0 = running, char2 bit0 = initialized.

Kinematics are integrated over the injected virtual :class:`sim.clock.Clock`:
position is the integral of speed, speed in steps/s is
``timer_freq / step_period`` (times ``highspeed_ratio`` in highspeed mode), and
a GOTO run stops exactly on the target. No real time is ever consulted.
"""

from sim.clock import Clock
from sim.faults import FaultScript

ERR_UNKNOWN_COMMAND = 0
ERR_COMMAND_LENGTH = 1
ERR_NOT_STOPPED = 2
ERR_INVALID_CHARACTER = 3
ERR_NOT_INITIALIZED = 4

_POSITION_OFFSET = 0x800000

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

    Defaults describe a healthy board: CPR/timer freq/highspeed ratio are valid
    so the driver's ``invalid mount config`` check passes, and ``mount_code``
    feeds the driver's min-period table (0x0A -> 0x0600, 0xF0 -> 12, else 6).
    ``voltage_supported=False`` reproduces the P1 board that never answers
    ``:fL#``. Scripted faults (empty/truncated reply, dead motor) go through
    :attr:`faults`.
    """

    def __init__(
        self,
        clock: Clock,
        *,
        cpr: int = 8_000_000,
        timer_freq: int = 64935,
        highspeed_ratio: int = 16,
        mount_code: int = 0x00,
        board_version: int = 0x0311,
        voltage_v: float = 12.6,
        voltage_supported: bool = True,
        position_steps: int = 0,
    ) -> None:
        self._clock = clock
        self.faults = FaultScript()

        self.cpr = cpr
        self.timer_freq = timer_freq
        self.highspeed_ratio = highspeed_ratio
        self.mount_code = mount_code
        self.board_version = board_version
        self.voltage_v = voltage_v
        self.voltage_supported = voltage_supported

        self.initialized = False
        self.running = False
        self.tracking_mode = True
        self.backward = False
        self.highspeed = False
        self.step_period = max(1, int(round(86164.1 * timer_freq / cpr)))
        self.target_increment = 0
        self.brake_increment = 0
        self.goto_target: float | None = None
        self.position = float(position_steps)

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
            self._rx.append(byte)
            if self._rx == b":fL#":
                self._rx.clear()
                self._handle_voltage()

    def drain(self) -> bytes:
        data = bytes(self._tx)
        self._tx.clear()
        return data

    def on_dtr(self, level: bool) -> None:
        # SkyWatcher boards do not use the DTR line.
        return None

    def speed_sps(self) -> float:
        sps = self.timer_freq / self.step_period
        if self.highspeed:
            sps *= self.highspeed_ratio
        return sps

    def _integrate(self) -> None:
        now = self._clock.now
        dt = now - self._integrated_at
        self._integrated_at = now
        if dt <= 0 or not self.running:
            return

        travel = self.speed_sps() * dt
        sign = -1.0 if self.backward else 1.0
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

    def _handle_voltage(self) -> None:
        self._integrate()
        if not self.voltage_supported:
            return
        line = f"{int(round(self.voltage_v * 10)):02X}#".encode("ascii")
        self._tx.extend(self.faults.apply(line, b"#", "fL"))

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
            self.step_period = max(1, period)
            self._reply(cmd)
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
            if self.running:
                self._reply(cmd, error=ERR_NOT_STOPPED)
                return
            try:
                raw = decode_revu24(data)
            except ValueError:
                self._reply(cmd, error=ERR_INVALID_CHARACTER)
                return
            self.position = float((raw - _POSITION_OFFSET) % self.cpr)
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
            if not self.initialized:
                self._reply(cmd, error=ERR_NOT_INITIALIZED)
                return
            self.running = True
            if not self.tracking_mode:
                sign = -1.0 if self.backward else 1.0
                self.goto_target = self.position + sign * self.target_increment
            self._reply(cmd)
        elif cmd == "K":
            self.running = False
            self.goto_target = None
            self.tracking_mode = True
            self._reply(cmd)
        else:
            self._reply(cmd, error=ERR_UNKNOWN_COMMAND)
