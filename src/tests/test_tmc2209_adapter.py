import pytest

from lx200.protocols import LX200Dec
from tmc2209.tmc2209_adapter import (
    DEGREES_PER_REV,
    GEAR_RATIO_1,
    GEAR_RATIO_2,
    MICROSTEPS_ALLOWED,
    STEPS_PER_REV,
    Phase,
    TMC2209Adapter,
    TMC2209CommandError,
    TMC2209ConfigError,
    TMC2209DriverStatus,
    TMC2209ProtocolError,
    TMC2209Response,
    TMC2209ResponseError,
    TMC2209Status,
    _parse_bool,
    _parse_float,
    _parse_hex,
    _parse_int,
    steps_from_dec,
)


class _FakeSerialLine:
    def __init__(self, responses: dict[str, str], terminator: bytes = b"\n") -> None:
        self.responses = responses
        self.terminator = terminator
        self.last_payload: str | None = None

    def query(self, payload: str) -> str:
        self.last_payload = payload
        key = payload.strip()
        return self.responses.get(key, "0;error=unknown_cmd;")


def test_response_from_line_ok():
    response = TMC2209Response.from_line("1;enabled=1;position=10;")
    assert response.ok is True
    assert response.values == {"enabled": "1", "position": "10"}
    assert response.error is None


def test_response_from_line_error():
    response = TMC2209Response.from_line("0;error=bad_value;")
    assert response.ok is False
    assert response.error == "bad_value"


def test_response_invalid_prefix():
    with pytest.raises(TMC2209ResponseError):
        TMC2209Response.from_line("2;enabled=1;")


def test_response_duplicate_key():
    with pytest.raises(TMC2209ResponseError):
        TMC2209Response.from_line("1;enabled=1;enabled=0;")


def test_response_empty_value():
    with pytest.raises(TMC2209ResponseError):
        TMC2209Response.from_line("1;enabled=;")


def test_status_from_response():
    response = TMC2209Response.from_line(
        "1;initialised=1;enabled=0;position=12;phase=hold;target=5;"
        "target_set=1;speed=120.50;actual_speed=118.25;accel=10.00;"
    )
    status = TMC2209Status.from_response(response)
    assert status.initialised is True
    assert status.enabled is False
    assert status.position == 12
    assert status.phase == Phase.HOLD
    assert status.target == 5
    assert status.target_set is True
    assert status.speed_sps == pytest.approx(120.5)
    assert status.actual_speed_sps == pytest.approx(118.25)
    assert status.accel_steps_per_s == pytest.approx(10.0)


def test_driver_status_from_response():
    response = TMC2209Response.from_line(
        "1;gconf=0x00000001;drv_status=0x0000000A;sg_result=42;"
    )
    status = TMC2209DriverStatus.from_response(response)
    assert status.gconf == 0x1
    assert status.drv_status == 0xA
    assert status.sg_result == 42


def test_parse_helpers_invalid():
    with pytest.raises(TMC2209ProtocolError):
        _parse_bool("2", "enabled")
    with pytest.raises(TMC2209ProtocolError):
        _parse_int("abc", "position")
    with pytest.raises(TMC2209ProtocolError):
        _parse_float("abc", "speed")
    with pytest.raises(TMC2209ProtocolError):
        _parse_hex("123", "gconf")


def test_adapter_requires_newline_terminator():
    serial = _FakeSerialLine({}, terminator=b"\r")
    with pytest.raises(TMC2209ConfigError):
        TMC2209Adapter(serial)


def test_adapter_status_ok():
    serial = _FakeSerialLine(
        {
            "status": (
                "1;initialised=1;enabled=1;position=0;phase=idle;target=0;"
                "target_set=0;speed=0.00;actual_speed=0.00;accel=0.00;"
            )
        }
    )
    adapter = TMC2209Adapter(serial)
    status = adapter.status()
    assert status.enabled is True
    assert status.phase == Phase.IDLE


def test_adapter_status_error():
    serial = _FakeSerialLine({"status": "0;error=bad_value;"})
    adapter = TMC2209Adapter(serial)
    with pytest.raises(TMC2209CommandError):
        adapter.status()


def test_get_param_normalizes_name():
    serial = _FakeSerialLine({"get microsteps": "1;microsteps=16;"})
    adapter = TMC2209Adapter(serial)
    assert adapter.get_param(":microsteps") == "16"


def test_set_param_formats_bool():
    serial = _FakeSerialLine({"set stealth=1": "1;stealth=1;"})
    adapter = TMC2209Adapter(serial)
    assert adapter.set_param("stealth", True) == "1"


def test_set_speed_sps_range():
    serial = _FakeSerialLine({})
    adapter = TMC2209Adapter(serial)
    with pytest.raises(TMC2209ConfigError):
        adapter.set_speed_sps(-1)
    with pytest.raises(TMC2209ConfigError):
        adapter.set_speed_sps(40001)


def test_set_acceleration_range():
    serial = _FakeSerialLine({})
    adapter = TMC2209Adapter(serial)
    with pytest.raises(TMC2209ConfigError):
        adapter.set_acceleration_steps_per_ms(-1)
    with pytest.raises(TMC2209ConfigError):
        adapter.set_acceleration_steps_per_ms(100001)


def test_steps_from_dec_roundtrip():
    dec = LX200Dec.from_degrees(10.0)
    microsteps = max(MICROSTEPS_ALLOWED)
    steps = steps_from_dec(dec, microsteps)
    expected = int(
        round(
            dec.to_degrees()
            * STEPS_PER_REV
            * microsteps
            * GEAR_RATIO_1
            * GEAR_RATIO_2
            / DEGREES_PER_REV
        )
    )
    assert steps == expected


def test_steps_from_dec_negative():
    dec = LX200Dec.from_degrees(-10.0)
    microsteps = min(MICROSTEPS_ALLOWED)
    steps = steps_from_dec(dec, microsteps)
    assert steps < 0


def test_steps_from_dec_invalid_microsteps():
    dec = LX200Dec.from_degrees(1.0)
    with pytest.raises(TMC2209ConfigError):
        steps_from_dec(dec, 3)


def test_steps_from_dec_invalid_steps_per_rev(monkeypatch):
    dec = LX200Dec.from_degrees(1.0)
    monkeypatch.setattr("tmc2209.tmc2209_adapter.STEPS_PER_REV", 0)
    with pytest.raises(TMC2209ConfigError):
        steps_from_dec(dec, min(MICROSTEPS_ALLOWED))
