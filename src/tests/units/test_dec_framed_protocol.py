"""The framed DEC protocol (v3): the codec, on its own.

Every defect this dialect exists to kill is a *silent* one — a frame that lost,
gained or changed a byte and still parsed. So the tests here are not "does a good
frame decode" (one case) but "is there any single-byte damage that survives"
(exhaustive, over every position of a real frame).

The golden vectors at the top are the contract the AVR firmware is written
against: it builds the same bytes with a hand-rolled hex emitter and a bitwise
CRC, and nothing but a fixed literal can catch the two implementations drifting
apart.
"""

import pytest
from tmc2209.protocol import (
    CRC_SIZE,
    HEADER_SIZE,
    Op,
    TMC2209MotorIntegrityError,
    TMC2209MotorLegacyResponseError,
    TMC2209MotorProtocolError,
    TMC2209MotorStaleResponseError,
    TMC2209MotorTimeoutError,
    TMC2209MotorTruncatedResponseError,
    build_request,
    crc16,
    decode_frame,
    decode_response,
    encode_frame,
    encode_status_payload,
    response_values,
)

# The board's two hard limits (DEC_PROTOCOL.md §5.7, §6.2): a command must fit in the
# 64-byte hardware RX FIFO, and replies share a 255-byte TX ring.
RX_FIFO = 64
TX_RING = 255

_STATUS_PAYLOAD = encode_status_payload(
    initialised=True,
    enabled=True,
    free_ride=False,
    target_set=True,
    phase="running",
    position=123456,
    target=200000,
    speed_sps=1000.0,
    actual_sps=980.0,
    accel_sps2=1000.0,
    power_v=12.34,
    tx_overflow=7,
)
_STATUS_REPLY = "#1A022A0B030001E24000030D40000186A000017ED0000186A004D200078986\n"


def test_crc16_matches_the_published_ccitt_false_check_value() -> None:
    """The one thing that cannot be verified against our own implementation.

    CRC-16/CCITT-FALSE is defined by its check value: 0x29B1 over "123456789".
    Without this the firmware and the driver could agree on the same wrong CRC.
    """
    assert crc16(b"123456789") == 0x29B1


def test_golden_frames_are_byte_for_byte_what_the_firmware_must_emit() -> None:
    assert encode_frame(Op.STATUS, 1) == "#000201BADF\n"
    assert encode_frame(Op.HELLO, 1) == "#000101EF8C\n"
    assert encode_frame(Op.SPEED, 7, (1000).to_bytes(4, "big", signed=True)) == "#041107000003E8218D\n"
    assert encode_frame(Op.STATUS, 0x2A, _STATUS_PAYLOAD) == _STATUS_REPLY


def test_golden_status_frame_decodes_into_the_line_protocol_field_names() -> None:
    # Same keys, same formatting as `1;initialised=1;...` — everything above the
    # codec is written once and works in either dialect.
    assert response_values(decode_frame(_STATUS_REPLY[1:])) == {
        "initialised": "1",
        "enabled": "1",
        "mode": "target",
        "position": "123456",
        "phase": "running",
        "target": "200000",
        "target_set": "1",
        "speed": "1000.00",
        "actual_speed": "980.00",
        "accel_per_s": "1000.00",
        "power_v": "12.34",
        "tx_overflow": "7",
    }


def test_a_status_reply_is_less_than_half_the_line_protocol_one() -> None:
    """The compactness claim, in numbers.

    The line protocol needs 147 bytes for this same snapshot (DEC_PROTOCOL.md §3),
    so only one reply fits in the TX ring with room for a second to be cut in half.
    """
    assert len(_STATUS_REPLY) == 64
    assert TX_RING // len(_STATUS_REPLY) == 3


@pytest.mark.parametrize(
    "command,args",
    [
        ("status", None),
        ("hello", None),
        ("run", None),
        ("stop", None),
        ("position", ["-2147483648"]),
        ("speed", ["40000"]),
        ("acceleration", ["100000"]),
        ("delta", ["2147483647"]),
        ("direction", ["1"]),
        ("enabled", ["0"]),
        ("mode", ["free_ride"]),
        ("set", ["microsteps=256"]),
        ("get", ["microsteps"]),
    ],
)
def test_every_command_fits_the_boards_receive_fifo(command: str, args: list[str] | None) -> None:
    op, payload = build_request(command, args)

    assert len(encode_frame(op, 0xFF, payload)) <= RX_FIFO


def test_frames_round_trip_through_the_codec() -> None:
    frame = decode_frame(encode_frame(Op.DELTA, 42, b"\x00\x01\x02\x03")[1:])

    assert (frame.op, frame.seq, frame.payload) == (Op.DELTA, 42, b"\x00\x01\x02\x03")


def test_no_single_altered_character_survives_the_crc() -> None:
    """Byte corruption: the failure the line protocol could not see at all."""
    hex_body = _STATUS_REPLY[1:].strip()
    survivors = []
    for index in range(len(hex_body)):
        for replacement in "0123456789ABCDEF":
            if replacement == hex_body[index]:
                continue
            damaged = hex_body[:index] + replacement + hex_body[index + 1 :]
            try:
                decode_frame(damaged)
            except TMC2209MotorProtocolError:
                continue
            survivors.append(damaged)

    assert not survivors, f"{len(survivors)} corrupted frames decoded as valid"


def test_no_lost_character_survives_the_length_check() -> None:
    """Byte loss: what the flaky cable and the RX FIFO actually do."""
    hex_body = _STATUS_REPLY[1:].strip()
    survivors = []
    for index in range(len(hex_body)):
        damaged = hex_body[:index] + hex_body[index + 1 :]
        try:
            decode_frame(damaged)
        except TMC2209MotorProtocolError:
            continue
        survivors.append(damaged)

    assert not survivors, f"{len(survivors)} truncated frames decoded as valid"


def test_a_frame_cut_on_a_field_boundary_is_rejected_not_shortened() -> None:
    """The TX-ring cut (§6.1), which used to yield a valid response with fields missing."""
    cut = _STATUS_REPLY[1:].strip()[: 2 * (HEADER_SIZE + 10)]

    with pytest.raises(TMC2209MotorTruncatedResponseError, match="declares 26 payload bytes"):
        decode_frame(cut)


def test_an_odd_number_of_hex_digits_is_caught_before_the_crc() -> None:
    with pytest.raises(TMC2209MotorTruncatedResponseError, match="odd number of hex digits"):
        decode_frame(_STATUS_REPLY[1:].strip()[:-1])


def test_a_non_hex_character_is_an_integrity_error() -> None:
    body = _STATUS_REPLY[1:].strip()

    with pytest.raises(TMC2209MotorIntegrityError, match="non-hex"):
        decode_frame(body[:10] + "Z" + body[11:])


def test_a_glued_stump_does_not_poison_the_frame_behind_it() -> None:
    """Resynchronisation: garbage before a frame is skipped, not folded into it."""
    stump = _STATUS_REPLY.strip()[:20]
    good = encode_frame(Op.RUN, 5, b"\x01")

    frame = decode_response(stump + good, op=Op.RUN, seq=5)

    assert frame.payload == b"\x01"


def test_a_frame_answering_the_previous_command_is_stale_not_current() -> None:
    """The sequence number: an answer that arrived late is not an answer."""
    late = encode_frame(Op.STATUS, 41, _STATUS_PAYLOAD)

    with pytest.raises(TMC2209MotorStaleResponseError, match="seq=41"):
        decode_response(late, op=Op.STATUS, seq=42)


def test_the_older_of_two_frames_is_used_when_the_newer_one_is_damaged() -> None:
    good = encode_frame(Op.STATUS, 42, _STATUS_PAYLOAD)
    damaged = encode_frame(Op.STATUS, 42, _STATUS_PAYLOAD).strip()[:-4]

    frame = decode_response(good.strip() + damaged, op=Op.STATUS, seq=42)

    assert frame.payload == _STATUS_PAYLOAD


def test_two_answers_to_the_same_sequence_number_resolve_to_the_newer_one() -> None:
    """The sequence number is one byte, so it wraps every 256 requests.

    A reply that sat in the ring that long and a fresh one are indistinguishable
    by header alone, and the older one describes an axis 256 commands ago. Newest
    wins — the frames are walked back to front for exactly this case.
    """
    stale_position = encode_status_payload(
        initialised=True, enabled=True, free_ride=False, target_set=False, phase="running",
        position=1, target=0, speed_sps=0.0, actual_sps=0.0, accel_sps2=0.0, power_v=12.0, tx_overflow=0,
    )
    fresh_position = encode_status_payload(
        initialised=True, enabled=True, free_ride=False, target_set=False, phase="running",
        position=9999, target=0, speed_sps=0.0, actual_sps=0.0, accel_sps2=0.0, power_v=12.0, tx_overflow=0,
    )
    both = encode_frame(Op.STATUS, 7, stale_position).strip() + encode_frame(Op.STATUS, 7, fresh_position)

    frame = decode_response(both, op=Op.STATUS, seq=7)

    assert response_values(frame)["position"] == "9999"


def test_an_error_frame_is_accepted_as_the_answer_to_any_opcode() -> None:
    frame = decode_response(encode_frame(Op.ERROR, 9, b"\x03"), op=Op.SPEED, seq=9)

    assert response_values(frame) == {"error": "range"}


def test_silence_is_a_timeout_not_a_frame() -> None:
    with pytest.raises(TMC2209MotorTimeoutError):
        decode_response("", op=Op.STATUS, seq=1)


def test_a_v2_line_where_a_frame_was_expected_names_the_old_firmware() -> None:
    """The one answer that proves the board has not been reflashed."""
    with pytest.raises(TMC2209MotorLegacyResponseError, match="v2 line protocol"):
        decode_response("0;error=unknown_cmd;\n", op=Op.HELLO, seq=1)


def test_a_reply_whose_payload_has_the_wrong_size_is_a_protocol_error() -> None:
    """No IndexError, no half-read integer: a short payload is reported as such."""
    frame = decode_frame(encode_frame(Op.STATUS, 1, b"\x00\x01")[1:])

    with pytest.raises(TMC2209MotorProtocolError, match="carries 2 payload bytes, expected 26"):
        response_values(frame)


def test_an_unknown_phase_code_does_not_become_a_valueerror() -> None:
    payload = bytearray(_STATUS_PAYLOAD)
    payload[1] = 9
    frame = decode_frame(encode_frame(Op.STATUS, 1, bytes(payload))[1:])

    with pytest.raises(TMC2209MotorProtocolError, match="unknown phase code 9"):
        response_values(frame)


def test_the_frame_overhead_is_five_bytes_whatever_the_payload() -> None:
    for size in (0, 1, 4, 26):
        encoded = encode_frame(Op.STATUS, 0, bytes(size))
        assert len(encoded) == 2 * (HEADER_SIZE + size + CRC_SIZE) + len("#\n")
