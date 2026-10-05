"""TMC2209/Arduino DEC-controller endpoint simulator.

Ground truth is the firmware ``telescope_dec/src/main.cpp`` (the Python driver
``src/tmc2209/motor.py`` is its client). The board speaks two dialects and picks
one per line:

- a line **containing** ``#`` anywhere is a **framed** request (protocol v3, length
  + CRC + sequence number — see ``src/tmc2209/protocol.py``) and is answered with a
  frame. Anywhere, not only at position 0: the RX FIFO leaves a stump in front of
  the next write, and recognising the frame behind that stump is exactly what turns
  §6.2 from a wrecked command into a resynchronisation;
- anything else is a v2 **line** command, answered with ``1;key=value;...\\n`` /
  ``0;error=...;\\n`` (``outFlushLineV2`` appends a bare LF), floats formatted with
  2 decimals — byte for byte what ``docs/protocol/DEC_PROTOCOL.md`` recorded from
  the board.

The two dialects differ in how they treat a **NUL** in the line, and the difference
is deliberate. The frame path is driven by the received *length* (``memchr``, and a
parser that is handed ``lineLenV2``), so a NUL is just another non-hex byte: a frame
behind one is answered normally, a NUL inside the hex is refused as damage. The line
path still runs on a C string, so a leading NUL empties the command and the board
stays silent (§5.8). Before the length-driven parse landed, the frame path behaved
like the line path and a frame behind a NUL was lost without a trace — 0 replies out
of 10 on the bench, against 10 out of 10 after (§12.8).

``tx_overflow`` is reported in **both** dialects, but only by the reflashed board:
``framed=False`` models the firmware still in the field, which has neither the
counter nor the frame parser (both confirmed on the bench, before and after the
flash of task #19).

``ready\\r\\n`` is printed by ``Serial.println(F("ready"))`` at the end of
``setup()`` after the board resets (host DTR toggle) and is the same in both.

Failure modes
-------------
The three ways this board loses data are modelled here rather than in the chaos
transport, because they are properties of *this* firmware and not of the cable
(``DEC_PROTOCOL.md`` §6):

- ``tx_ring_capacity`` — the 255-byte TX ring. A reply that does not fit is either
  cut on an arbitrary byte boundary and glued to the next one
  (``tx_overflow_truncates=True``, firmware ``300a439``, still in the field) or
  dropped whole and replaced by a ``tx_overflow`` marker (the current firmware).
- ``rx_capacity`` — the 64-byte hardware RX FIFO. Bytes past it in a single write
  are lost while the board is busy, which both swallows whole commands and leaves
  a stump in the line buffer that the next command glues itself onto.
- ``framed=False`` — the board that has not been reflashed yet: a framed request
  is just an unknown command to it.

Motion model mirrors ``updateMotionStateV2``/``serviceStepperv2``:

- ``desired`` speed is 0 when stopping/not running, else ``speed``;
- in target mode the direction is forced towards the target and ``desired``
  drops to 0 once within the stopping distance ``actual^2 / (2*accel)``;
- ``actual`` ramps towards ``desired`` at ``accel`` steps/s^2 (jumps instantly
  when accel is 0), position integrates ``actual`` over the virtual clock, and
  crossing the target snaps to it (``completeTargetV2``).

Phases replicate ``getPhaseCodeV2``: idle (disabled), hold (actual<=0),
deceleration (desired<=0 or actual>desired), acceleration (actual<desired),
running otherwise. The virtual :class:`sim.clock.Clock` is the only time
source; integration substeps are pure computation.

Boot delay: after a DTR rising edge the firmware takes a couple of real
seconds to boot, so the host's ``read_all_data()`` right after
``SerialLine.reset()`` sees nothing and ``ready`` arrives on the following
poll. The sim models that as "skip one drain after reset" instead of real time.
"""

from sim.clock import Clock
from sim.faults import FaultScript
from tmc2209.protocol import (
    CRC_SIZE,
    FRAME_START,
    HEADER_SIZE,
    MODE_NAMES,
    PROTOCOL_VERSION,
    ErrorCode,
    Op,
    crc16,
    encode_frame,
    encode_status_payload,
    to_centi,
)

MICROSTEPS_ALLOWED = {1, 2, 4, 8, 16, 32, 64, 128, 256}

_MAX_SPEED_SPS = 40000
_MAX_ACCEL_SPS2 = 100000
_DERATED_SPEED_LIMIT_SPS = 1000
_FINE_APPROACH_FULL_STEPS = 32

_DRIVER_FLAG_OTPW = 0x01
_DRIVER_FLAG_OT = 0x02
_DRIVER_FLAG_SHORT = 0x04
_DRIVER_FLAG_STALL = 0x10

_STALL_POLL_INTERVAL_S = 0.2
_STALL_ARM_TIME_S = 0.8
_STALL_MIN_SPEED_SPS = 500
_STALL_STRONG_SG = 8
_STALL_LOADED_SG = 20
_STALL_CLEAR_SG = 40
_STALL_SCORE_LIMIT = 6

_INTEGRATION_SUBSTEP_S = 0.01

# Capacities of the real board, offered as ready-made arguments so that a test
# says `tx_ring_capacity=TX_RING_CAPACITY` instead of repeating a bare 255.
TX_RING_CAPACITY = 255
RX_FIFO_CAPACITY = 64

_FIRMWARE_BUILD = 2

# Command names, kept as the single vocabulary of the fault script: a test targets
# `command="status"` and gets the same fault whichever dialect asked for it.
_OP_NAMES: dict[int, str] = {
    Op.HELLO: "hello",
    Op.STATUS: "status",
    Op.POSITION: "position",
    Op.SPEED: "speed",
    Op.ACCELERATION: "acceleration",
    Op.DIRECTION: "direction",
    Op.DELTA: "delta",
    Op.MODE: "mode",
    Op.ENABLED: "enabled",
    Op.MICROSTEPS: "set",
    Op.RUN: "run",
    Op.STOP: "stop",
}

_ERROR_NAMES: dict[int, str] = {
    ErrorCode.UNKNOWN_CMD: "unknown_cmd",
    ErrorCode.BAD_VALUE: "bad_value",
    ErrorCode.RANGE: "range",
    ErrorCode.MISSING_PARAM: "missing_param",
    ErrorCode.UNKNOWN_PARAM: "unknown_param",
    ErrorCode.BAD_PARAM: "bad_param",
    ErrorCode.SINGLE_PARAM: "single_param",
    ErrorCode.INVALID_MICROSTEPS: "invalid_microsteps",
    ErrorCode.INVALID_BOOL: "invalid_bool",
    ErrorCode.TX_OVERFLOW: "tx_overflow",
    ErrorCode.BAD_CRC: "bad_crc",
    ErrorCode.BAD_FRAME: "bad_frame",
    ErrorCode.DRIVER_FAULT: "driver_fault",
}


class _Reply:
    """One answer, rendered for both dialects.

    The command semantics above build this once; only the two renderings differ,
    so a behaviour change cannot land in one dialect and miss the other.
    """

    def __init__(self, pairs: list[tuple[str, str]], payload: bytes, error: int | None = None) -> None:
        self.pairs = pairs
        self.payload = payload
        self.error = error

    @classmethod
    def failure(cls, code: int) -> "_Reply":
        return cls([("error", _ERROR_NAMES[code])], bytes((code,)), error=code)


class TMC2209Sim:
    """Simulated DEC controller behind a :class:`sim.fake_serial.FakeSerial`."""

    def __init__(
        self,
        clock: Clock,
        *,
        power_v: float = 12.0,
        ready_on_reset: bool = True,
        framed: bool = True,
        tx_ring_capacity: int | None = None,
        tx_overflow_truncates: bool = False,
        rx_capacity: int | None = None,
        uart_to_driver_dead: bool = False,
    ) -> None:
        # This flag preserves the historical broken-UART/failure mode. The repaired
        # board normally talks to the driver and confirms each microstep write.
        self.uart_to_driver_dead = uart_to_driver_dead
        self._clock = clock
        self.faults = FaultScript()
        self.power_v = power_v
        self.ready_on_reset = ready_on_reset
        self.framed = framed
        self.tx_ring_capacity = tx_ring_capacity
        self.tx_overflow_truncates = tx_overflow_truncates
        self.rx_capacity = rx_capacity

        self._rx = bytearray()
        self._tx = bytearray()
        self._skip_drains = 0
        self._boot()

    def _boot(self) -> None:
        self.enabled = False
        self.dir_negative = False
        self.running = False
        self.stop_requested = False
        self.has_target = False
        self.free_ride = False
        self.target = 0
        self.speed_sps = 500.0
        self.desired_sps = 0.0
        self.actual_sps = 0.0
        self.accel_sps2 = 1000.0
        self.position = 0.0
        self.microsteps = 16
        self.active_microsteps = 16
        self.driver_flags = 0
        self.sg_result = 100
        self.stall_latched = False
        self._stall_eligible_s = 0.0
        self._stall_poll_s = 0.0
        self._stall_score = 0
        self.safety = "normal"
        self.safety_events = 0
        self.speed_limit_sps = _MAX_SPEED_SPS
        self._safety_clear_s = 0.0
        self.tx_overflow = 0
        self.rx_dropped = 0
        self._seq = 0

        self._integrated_at = self._clock.now
        if self.ready_on_reset:
            # `Serial.println(F("ready"))` (main.cpp:452) — Arduino's println
            # appends CR LF, unlike the plain LF of `outFlushLineV2` used by
            # every command reply.
            self._tx.extend(b"ready\r\n")

    def feed(self, data: bytes) -> None:
        if self.rx_capacity is not None and len(data) > self.rx_capacity:
            # The board is busy with the command it is running, the 64-byte
            # hardware FIFO fills up and everything past it is gone. The cut
            # normally lands mid-line, so the stump stays in `_rx` and the next
            # write glues itself onto it — both effects of DEC_PROTOCOL.md §6.2.
            self.rx_dropped += len(data) - self.rx_capacity
            data = data[: self.rx_capacity]
        for byte in data:
            if byte == ord("\r"):
                continue
            if byte == ord("\n"):
                line = self._rx.decode("ascii", errors="replace")
                self._rx.clear()
                self._handle(line)
                continue
            self._rx.append(byte)

    def drain(self) -> bytes:
        if self._skip_drains > 0:
            self._skip_drains -= 1
            return b""
        data = bytes(self._tx)
        self._tx.clear()
        return data

    def on_dtr(self, level: bool) -> None:
        if level:
            # Rising edge = board reboot; one skipped drain models boot time.
            self._rx.clear()
            self._tx.clear()
            self._boot()
            self._skip_drains = 1

    def _integrate(self) -> None:
        now = self._clock.now
        dt = now - self._integrated_at
        self._integrated_at = now
        while dt > 1e-9:
            step = min(_INTEGRATION_SUBSTEP_S, dt)
            dt -= step
            self._step_motion(step)

    def _step_motion(self, dt: float) -> None:
        if self.driver_flags & (_DRIVER_FLAG_OT | _DRIVER_FLAG_SHORT):
            if self.safety != "shutdown":
                self.safety_events += 1
            self.safety = "shutdown"
            self.speed_limit_sps = 0
            self.enabled = False
            self.active_microsteps = self.microsteps
            self._safety_clear_s = 0.0
        elif self.driver_flags & _DRIVER_FLAG_OTPW:
            if self.safety == "normal":
                self.safety_events += 1
            self.safety = "derated"
            self.speed_limit_sps = _DERATED_SPEED_LIMIT_SPS
            self._safety_clear_s = 0.0
        elif self.safety != "normal" and not self.stall_latched:
            self._safety_clear_s += dt
            if self._safety_clear_s >= 5.0:
                self.safety = "normal"
                self.speed_limit_sps = _MAX_SPEED_SPS
                self._safety_clear_s = 0.0

        if not self.enabled:
            self.actual_sps = 0.0
            self.desired_sps = 0.0
            self.running = False
            self.stop_requested = False
            self.active_microsteps = self.microsteps
            return

        if self.running and self.has_target and not self.free_ride:
            delta = self.target - int(round(self.position))
            if delta == 0:
                self._complete_target()
                return
            self.dir_negative = delta < 0

            stopping_distance = (
                (self.actual_sps * self.actual_sps) / (2.0 * self.accel_sps2)
                if self.accel_sps2 > 0.0
                else 0.0
            )
            fine_approach = stopping_distance + self.microsteps * _FINE_APPROACH_FULL_STEPS
            if self.active_microsteps == 1 and abs(delta) <= fine_approach:
                self.active_microsteps = self.microsteps
            elif self.active_microsteps != 1 and self.actual_sps <= 0.0 and abs(delta) > fine_approach:
                self.active_microsteps = 1
        else:
            self.active_microsteps = self.microsteps

        desired = 0.0
        if not self.stop_requested and self.running:
            desired = min(self.speed_sps, self.speed_limit_sps)

        if self.has_target and not self.free_ride and desired > 0.0 and self.accel_sps2 > 0.0:
            remaining = self.target - self.position
            stopping_distance = (self.actual_sps * self.actual_sps) / (2.0 * self.accel_sps2)
            if abs(remaining) <= stopping_distance:
                desired = 0.0

        self.desired_sps = desired
        if self.accel_sps2 <= 0.0:
            self.actual_sps = desired
        else:
            delta_speed = self.accel_sps2 * dt
            if self.actual_sps < desired:
                self.actual_sps = min(self.actual_sps + delta_speed, desired)
            elif self.actual_sps > desired:
                self.actual_sps = max(self.actual_sps - delta_speed, desired)

        if self.stop_requested and self.actual_sps <= 0.0:
            self.stop_requested = False
            self.running = False
            self.actual_sps = 0.0
            self.desired_sps = 0.0

        stall_eligible = (
            self.enabled
            and self.running
            and not self.stop_requested
            and self.safety in {"normal", "derated"}
            and self._phase() == "running"
            and self.actual_sps >= _STALL_MIN_SPEED_SPS
        )
        if not stall_eligible:
            self._stall_eligible_s = 0.0
            self._stall_poll_s = 0.0
            self._stall_score = 0
        elif not self.stall_latched:
            self._stall_eligible_s += dt
            if self._stall_eligible_s >= _STALL_ARM_TIME_S:
                self._stall_poll_s += dt
                if self._stall_poll_s >= _STALL_POLL_INTERVAL_S:
                    self._stall_poll_s -= _STALL_POLL_INTERVAL_S
                    if self.sg_result <= _STALL_STRONG_SG:
                        self._stall_score = min(self._stall_score + 3, _STALL_SCORE_LIMIT)
                    elif self.sg_result <= _STALL_LOADED_SG:
                        self._stall_score = min(self._stall_score + 1, _STALL_SCORE_LIMIT)
                    elif self.sg_result >= _STALL_CLEAR_SG:
                        self._stall_score = max(self._stall_score - 1, 0)

                    if self._stall_score >= _STALL_SCORE_LIMIT:
                        self.stall_latched = True
                        self.driver_flags |= _DRIVER_FLAG_STALL
                        self.safety_events += 1
                        self.safety = "shutdown"
                        self.speed_limit_sps = 0
                        self.enabled = False
                        self.running = False
                        self.stop_requested = False
                        self.actual_sps = 0.0
                        self.desired_sps = 0.0
                        self.active_microsteps = self.microsteps
                        return

        if self.actual_sps > 0.0:
            travel = self.actual_sps * dt
            self.position += -travel if self.dir_negative else travel
            if self.running and self.has_target and not self.free_ride:
                crossed = self.position <= self.target if self.dir_negative else self.position >= self.target
                if crossed:
                    self.position = float(self.target)
                    self._complete_target()

    def _complete_target(self) -> None:
        self.has_target = False
        self.running = False
        self.stop_requested = False
        self.actual_sps = 0.0
        self.desired_sps = 0.0
        self.active_microsteps = self.microsteps

    def _phase(self) -> str:
        if not self.enabled:
            return "idle"
        if self.actual_sps <= 0.0:
            return "hold"
        if self.desired_sps <= 0.0:
            return "deceleration"
        if self.actual_sps < self.desired_sps:
            return "acceleration"
        if self.actual_sps > self.desired_sps:
            return "deceleration"
        return "running"

    # -- wire -----------------------------------------------------------------

    def _emit(self, data: bytes, command: str, marker: bytes) -> None:
        """Push one rendered reply into the TX ring, modelling its capacity.

        `outAppendCharV2` fills a 255-byte ring that only empties as the UART
        drains it, and the whole input buffer is parsed before a single byte
        leaves — which is why two commands in one write overflow it
        (DEC_PROTOCOL.md §6.1). `drain()` is the moment the ring frees here.
        """
        data = self.faults.apply(data, b"\n", command)
        if not data:
            return
        if self.tx_ring_capacity is None:
            self._tx.extend(data)
            return

        free = self.tx_ring_capacity - len(self._tx)
        if len(data) <= free:
            self._tx.extend(data)
            return

        self.tx_overflow = min(self.tx_overflow + 1, 0xFFFF)
        if self.tx_overflow_truncates:
            # Firmware 300a439, the one still on the board: the reply is cut on
            # whatever byte the ring ran out at — terminator included — and the
            # next reply is appended straight onto the stump.
            self._tx.extend(data[:free])
            return
        if len(marker) <= free:
            self._tx.extend(marker)

    def _emit_legacy(self, command: str, reply: _Reply) -> None:
        body = "".join(f"{key}={value};" for key, value in reply.pairs if key != "events")
        line = f"{'0' if reply.error is not None else '1'};{body}\n".encode("ascii")
        self._emit(line, command, b"0;error=tx_overflow;\n")

    def _emit_frame(self, op: int, seq: int, reply: _Reply, command: str) -> None:
        answer_op = Op.ERROR if reply.error is not None else op
        frame = encode_frame(answer_op, seq, reply.payload).encode("ascii")
        overflow = encode_frame(Op.ERROR, seq, bytes((ErrorCode.TX_OVERFLOW,))).encode("ascii")
        self._emit(frame, command, overflow)

    # -- dispatch ---------------------------------------------------------------

    def _handle(self, line: str) -> None:
        self._integrate()

        # A marker anywhere selects the framed dialect, and the search runs over the
        # whole line: the firmware uses `memchr` with the received length, not
        # `strchr`, so an embedded NUL no longer ends it early. That distinction was
        # not free -- with `strchr` a frame behind a NUL was invisible and answered
        # with silence (0 replies out of 10 on the bench; 10 out of 10 after the fix).
        #
        # The board that has not been reflashed has no marker rule at all: the frame
        # falls through to the line parser below, where it is a word nobody knows.
        if self.framed and FRAME_START in line:
            self._handle_frame(line)
            return

        # The v2 branch still takes a C string, and that is left as it is: the line
        # protocol has no framing to salvage and its NUL behaviour is documented
        # (DEC_PROTOCOL.md 5.8) -- a leading NUL empties the command and the board
        # answers nothing at all.
        line = line.split("\x00", 1)[0]

        tokens = line.split()
        if not tokens:
            return
        parsed = self._parse_legacy(tokens)
        reply = parsed if isinstance(parsed, _Reply) else self._apply(parsed[0], parsed[1])
        self._emit_legacy(tokens[0], reply)

    def _handle_frame(self, line: str) -> None:
        # Resynchronise on the newest marker: a line that starts with `#` but
        # carries two of them is a stump left by the RX FIFO with a fresh command
        # appended to it. The board obeys the newest one — nobody is waiting for
        # the answer to the older, half-eaten command any more. (The host resolves
        # the mirror-image case differently, see `decode_response`: an answer that
        # arrived late may still be the one it asked for.)
        body = line.rsplit(FRAME_START, 1)[-1].strip()
        try:
            raw = b"" if len(body) % 2 else bytes.fromhex(body)
        except ValueError:
            raw = b""
        if len(raw) < HEADER_SIZE + CRC_SIZE:
            self._emit_frame(Op.ERROR, 0, _Reply.failure(ErrorCode.BAD_FRAME), "frame")
            return

        declared, op, seq = raw[0], raw[1], raw[2]
        # The sequence number is echoed as received even though it may itself be
        # the damaged byte: either the host recognises its own number and reads
        # the error, or the number is wrong and the host rejects the frame as
        # stale. Both outcomes are safe; silence would not be.
        if len(raw) != HEADER_SIZE + declared + CRC_SIZE:
            # Structural: the frame does not carry what it says it carries. Kept
            # apart from a CRC mismatch so that "the length is a lie" and "a byte
            # changed" stay two distinguishable answers.
            self._emit_frame(Op.ERROR, seq, _Reply.failure(ErrorCode.BAD_FRAME), "frame")
            return
        if crc16(raw[:-CRC_SIZE]) != int.from_bytes(raw[-CRC_SIZE:], "big"):
            self._emit_frame(Op.ERROR, seq, _Reply.failure(ErrorCode.BAD_CRC), "frame")
            return

        payload = raw[HEADER_SIZE:-CRC_SIZE]
        command = _OP_NAMES.get(op, "frame")
        if op not in _OP_NAMES:
            self._emit_frame(op, seq, _Reply.failure(ErrorCode.UNKNOWN_CMD), command)
            return

        sizes: dict[int, int] = {Op.POSITION: 4, Op.SPEED: 4, Op.ACCELERATION: 4, Op.DELTA: 4,
                                 Op.DIRECTION: 1, Op.ENABLED: 1, Op.MODE: 1}
        arg: int | None = None
        if op in sizes:
            if len(payload) != sizes[op]:
                self._emit_frame(op, seq, _Reply.failure(ErrorCode.BAD_VALUE), command)
                return
            arg = int.from_bytes(payload, "big", signed=sizes[op] == 4)
        elif op == Op.MICROSTEPS:
            # No payload = read the parameter, two bytes = write it.
            if len(payload) not in (0, 2):
                self._emit_frame(op, seq, _Reply.failure(ErrorCode.BAD_VALUE), command)
                return
            arg = int.from_bytes(payload, "big") if payload else None
        elif payload:
            self._emit_frame(op, seq, _Reply.failure(ErrorCode.BAD_VALUE), command)
            return

        self._seq = seq
        self._emit_frame(op, seq, self._apply(op, arg), command)

    def _parse_legacy(self, tokens: list[str]) -> "tuple[int, int | None] | _Reply":
        """Map a v2 line command onto the same opcode the framed dialect uses.

        Only the surface differs: everything the two dialects disagree about is
        parsing, and that disagreement ends here.
        """
        command, args = tokens[0], tokens[1:]

        if command == "status":
            return (Op.STATUS, None)
        if command == "run":
            return (Op.RUN, None)
        if command == "stop":
            return (Op.STOP, None)
        if command in ("position", "enabled", "direction", "speed", "acceleration", "delta"):
            value = self._parse_long(args)
            if value is None:
                return _Reply.failure(ErrorCode.BAD_VALUE)
            op = {
                "position": Op.POSITION,
                "enabled": Op.ENABLED,
                "direction": Op.DIRECTION,
                "speed": Op.SPEED,
                "acceleration": Op.ACCELERATION,
                "delta": Op.DELTA,
            }[command]
            return (op, value)
        if command == "mode":
            if not args:
                return _Reply.failure(ErrorCode.BAD_VALUE)
            if len(args) > 1:
                return _Reply.failure(ErrorCode.SINGLE_PARAM)
            if args[0] not in MODE_NAMES:
                return _Reply.failure(ErrorCode.BAD_VALUE)
            return (Op.MODE, MODE_NAMES.index(args[0]))
        if command in ("set", "get"):
            if not args:
                return _Reply.failure(ErrorCode.MISSING_PARAM)
            if len(args) > 1:
                return _Reply.failure(ErrorCode.SINGLE_PARAM)
            parameter = args[0].removeprefix(":")
            if command == "get":
                if parameter != "microsteps":
                    return _Reply.failure(ErrorCode.UNKNOWN_PARAM)
                return (Op.MICROSTEPS, None)
            name, separator, raw_value = parameter.partition("=")
            if not separator or not name or not raw_value:
                return _Reply.failure(ErrorCode.BAD_PARAM)
            if name != "microsteps":
                return _Reply.failure(ErrorCode.UNKNOWN_PARAM)
            try:
                return (Op.MICROSTEPS, int(raw_value))
            except ValueError:
                return _Reply.failure(ErrorCode.BAD_VALUE)
        return _Reply.failure(ErrorCode.UNKNOWN_CMD)

    def _apply(self, op: int, arg: int | None) -> _Reply:
        """Command semantics, written once for both dialects."""
        if op == Op.HELLO:
            return _Reply([("protocol", str(PROTOCOL_VERSION)), ("firmware", str(_FIRMWARE_BUILD))],
                          bytes((PROTOCOL_VERSION, _FIRMWARE_BUILD)))

        if op == Op.STATUS:
            pairs = [
                ("initialised", "1"),
                ("enabled", "1" if self.enabled else "0"),
                ("mode", "free_ride" if self.free_ride else "target"),
                ("position", str(int(round(self.position)))),
                ("phase", self._phase()),
                ("target", str(self.target)),
                ("target_set", "1" if self.has_target else "0"),
                ("speed", f"{self.speed_sps:.2f}"),
                ("actual_speed", f"{self.actual_sps:.2f}"),
                ("accel_per_s", f"{self.accel_sps2:.2f}"),
                ("power_v", f"{self.power_v:.2f}"),
            ]
            if self.framed:
                # Last field, after power_v (main.cpp handleLineV2). Only the
                # reflashed board has it: `framed` is what tells the two firmware
                # versions apart, and the counter arrived in the same commit as
                # the frame parser.
                pairs.append(("tx_overflow", str(self.tx_overflow)))
                pairs.extend([
                    ("drv_flags", str(self.driver_flags)),
                    ("safety", self.safety),
                    ("mres", str(self.active_microsteps)),
                    ("fine_mres", str(self.microsteps)),
                    ("limit", str(self.speed_limit_sps)),
                    ("events", str(self.safety_events)),
                ])
            return _Reply(pairs, encode_status_payload(
                initialised=True,
                enabled=self.enabled,
                free_ride=self.free_ride,
                target_set=self.has_target,
                phase=self._phase(),
                position=int(round(self.position)),
                target=self.target,
                speed_sps=self.speed_sps,
                actual_sps=self.actual_sps,
                accel_sps2=self.accel_sps2,
                power_v=self.power_v,
                tx_overflow=self.tx_overflow,
                driver_flags=self.driver_flags,
                safety=self.safety,
                active_microsteps=self.active_microsteps,
                fine_microsteps=self.microsteps,
                speed_limit_sps=self.speed_limit_sps,
                safety_events=self.safety_events,
            ))

        if op == Op.POSITION:
            self.position = float(arg or 0)
            value = int(round(self.position))
            return _Reply([("position", str(value))], value.to_bytes(4, "big", signed=True))

        if op == Op.ENABLED:
            if arg and self.safety == "shutdown":
                return _Reply.failure(ErrorCode.DRIVER_FAULT)
            self.enabled = bool(arg)
            if not self.enabled:
                if self.stall_latched:
                    self.stall_latched = False
                    self.driver_flags &= ~_DRIVER_FLAG_STALL
                    self._stall_eligible_s = 0.0
                    self._stall_poll_s = 0.0
                    self._stall_score = 0
                    if not self.driver_flags & (_DRIVER_FLAG_OT | _DRIVER_FLAG_SHORT):
                        if self.driver_flags & _DRIVER_FLAG_OTPW:
                            self.safety = "derated"
                            self.speed_limit_sps = _DERATED_SPEED_LIMIT_SPS
                        else:
                            self.safety = "normal"
                            self.speed_limit_sps = _MAX_SPEED_SPS
                self.running = False
                self.stop_requested = False
                self.actual_sps = 0.0
                self.desired_sps = 0.0
            return _Reply([("enabled", "1" if self.enabled else "0")], bytes((int(self.enabled),)))

        if op == Op.DIRECTION:
            self.dir_negative = bool(arg)
            return _Reply([("direction", "1" if self.dir_negative else "0")], bytes((int(self.dir_negative),)))

        if op == Op.SPEED:
            if arg is None or arg < 0 or arg > _MAX_SPEED_SPS:
                return _Reply.failure(ErrorCode.RANGE)
            self.speed_sps = float(arg)
            return _Reply([("speed", f"{self.speed_sps:.2f}")], to_centi(self.speed_sps).to_bytes(4, "big"))

        if op == Op.ACCELERATION:
            if arg is None or arg < 0 or arg > _MAX_ACCEL_SPS2:
                return _Reply.failure(ErrorCode.RANGE)
            self.accel_sps2 = float(arg)
            return _Reply([("accel_per_s", f"{self.accel_sps2:.2f}")], to_centi(self.accel_sps2).to_bytes(4, "big"))

        if op == Op.DELTA:
            delta = arg or 0
            self.target = int(round(self.position)) + delta
            self.has_target = True
            return _Reply(
                [("delta", str(delta)), ("target", str(self.target)), ("target_set", "1")],
                delta.to_bytes(4, "big", signed=True) + self.target.to_bytes(4, "big", signed=True) + b"\x01",
            )

        if op == Op.MODE:
            if arg is None or arg >= len(MODE_NAMES):
                return _Reply.failure(ErrorCode.BAD_VALUE)
            self.free_ride = arg == MODE_NAMES.index("free_ride")
            return _Reply([("mode", MODE_NAMES[arg])], bytes((arg,)))

        if op == Op.RUN:
            if self.safety == "shutdown":
                return _Reply.failure(ErrorCode.DRIVER_FAULT)
            self.running = True
            self.stop_requested = False
            self.enabled = True
            return _Reply([("running", "1")], b"\x01")

        if op == Op.STOP:
            self.stop_requested = True
            self.running = True
            return _Reply([("stopping", "1")], b"\x01")

        if op == Op.MICROSTEPS:
            if arg is not None:
                if arg < 1 or arg > 256:
                    return _Reply.failure(ErrorCode.RANGE)
                if arg not in MICROSTEPS_ALLOWED:
                    return _Reply.failure(ErrorCode.INVALID_MICROSTEPS)
                if not self.uart_to_driver_dead:
                    self.microsteps = arg
                    self.active_microsteps = arg
            return _Reply([("microsteps", str(self.microsteps))], self.microsteps.to_bytes(2, "big"))

        return _Reply.failure(ErrorCode.UNKNOWN_CMD)

    @staticmethod
    def _parse_long(args: list[str]) -> int | None:
        if len(args) != 1:
            return None
        try:
            return int(args[0], 10)
        except ValueError:
            return None
