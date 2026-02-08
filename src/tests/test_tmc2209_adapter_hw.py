import time

import pytest

from serial_wrapper.wrapper import SerialLine
from tmc2209.tmc2209_adapter import TMC2209Adapter, TMC2209Status


DEVICE_PATTERN = r"^tty\.usb.*$"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT_S = 0.2
SERIAL_NAME = "tmc2209"
SERIAL_TERMINATOR = "\n"

POLL_INTERVAL_S = 0.2
STOP_TIMEOUT_S = 8.0
TARGET_TIMEOUT_S = 20.0

TARGET_STEPS = 20000
RUN_SPEED_SPS = (600, 1200, 2400)
TARGET_SPEED_SPS = 2500
STOP_SPEED_SPS = 2000
POSITION_TOLERANCE = 200


@pytest.fixture
def adapter() -> TMC2209Adapter:
    port = SerialLine.search(DEVICE_PATTERN)
    serial_line = SerialLine(
        port,
        SERIAL_BAUD,
        SERIAL_TIMEOUT_S,
        SERIAL_NAME,
        terminator=SERIAL_TERMINATOR,
    )
    adapter = TMC2209Adapter(serial_line)
    adapter.connect()

    try:
        adapter.set_enabled(True)
        _prepare_adapter(adapter, 0)
        yield adapter
    finally:
        try:
            adapter.stop()
            _wait_for_stop(adapter, STOP_TIMEOUT_S, POLL_INTERVAL_S)
        except Exception:
            pass
        adapter.close()


def _prepare_adapter(adapter: TMC2209Adapter, position: int) -> None:
    adapter.stop()
    _wait_for_stop(adapter, STOP_TIMEOUT_S, POLL_INTERVAL_S)
    adapter.set_position(position)
    adapter.set_target(position)


def _wait_for_stop(
    adapter: TMC2209Adapter,
    timeout_s: float,
    poll_interval_s: float,
) -> TMC2209Status:
    deadline = time.monotonic() + timeout_s
    last_status = adapter.status()
    while time.monotonic() < deadline:
        if abs(last_status.actual_speed_sps) <= 0.1:
            return last_status
        time.sleep(poll_interval_s)
        last_status = adapter.status()
    pytest.fail("Motor did not stop in time")


def _wait_for_target(
    adapter: TMC2209Adapter,
    timeout_s: float,
    poll_interval_s: float,
) -> TMC2209Status:
    deadline = time.monotonic() + timeout_s
    last_status = adapter.status()
    while time.monotonic() < deadline:
        if not last_status.target_set:
            return last_status
        time.sleep(poll_interval_s)
        last_status = adapter.status()
    pytest.fail("Target move did not finish in time")


def _wait_for_motion(
    adapter: TMC2209Adapter,
    start_position: int,
    direction_sign: int,
    timeout_s: float,
    poll_interval_s: float,
) -> TMC2209Status:
    deadline = time.monotonic() + timeout_s
    last_status = adapter.status()
    while time.monotonic() < deadline:
        if direction_sign > 0 and last_status.position > start_position:
            return last_status
        if direction_sign < 0 and last_status.position < start_position:
            return last_status
        time.sleep(poll_interval_s)
        last_status = adapter.status()
    pytest.fail("Motor did not start moving in time")


def test_hw_set_position(adapter: TMC2209Adapter) -> None:
    position = TARGET_STEPS
    assert adapter.set_position(position) == position

    status = adapter.status()
    assert status.position == position


@pytest.mark.parametrize("target", [TARGET_STEPS, -TARGET_STEPS])
def test_hw_move_to_target_both_directions(adapter: TMC2209Adapter, target: int) -> None:
    _prepare_adapter(adapter, 0)

    adapter.set_speed_sps(TARGET_SPEED_SPS)
    returned_target, target_set = adapter.set_target(target)
    assert returned_target == target
    assert target_set is True
    assert adapter.run() is True

    direction_sign = 1 if target > 0 else -1
    _wait_for_motion(adapter, 0, direction_sign, 5.0, POLL_INTERVAL_S)

    final_status = _wait_for_target(adapter, TARGET_TIMEOUT_S, POLL_INTERVAL_S)
    assert abs(final_status.position - target) <= POSITION_TOLERANCE
    assert final_status.target_set is False


@pytest.mark.parametrize(
    ("speed_sps", "direction"),
    [
        (RUN_SPEED_SPS[0], False),
        (RUN_SPEED_SPS[0], True),
        (RUN_SPEED_SPS[1], False),
        (RUN_SPEED_SPS[1], True),
        (RUN_SPEED_SPS[2], False),
        (RUN_SPEED_SPS[2], True),
    ],
)
def test_hw_run_speed_and_direction(
    adapter: TMC2209Adapter, speed_sps: int, direction: bool
) -> None:
    _prepare_adapter(adapter, 0)

    adapter.set_speed_sps(speed_sps)
    adapter.set_direction(direction)
    assert adapter.run() is True

    direction_sign = -1 if direction else 1
    moving_status = _wait_for_motion(adapter, 0, direction_sign, 5.0, POLL_INTERVAL_S)

    time.sleep(0.6)
    later_status = adapter.status()

    assert later_status.actual_speed_sps > 0
    if direction:
        assert later_status.position < moving_status.position
    else:
        assert later_status.position > moving_status.position

    adapter.stop()
    _wait_for_stop(adapter, STOP_TIMEOUT_S, POLL_INTERVAL_S)


def test_hw_stop_during_run(adapter: TMC2209Adapter) -> None:
    _prepare_adapter(adapter, 0)

    adapter.set_speed_sps(STOP_SPEED_SPS)
    adapter.set_direction(False)
    assert adapter.run() is True

    moving_status = _wait_for_motion(adapter, 0, 1, 5.0, POLL_INTERVAL_S)
    assert moving_status.position != 0

    assert adapter.stop() is True
    stopped_status = _wait_for_stop(adapter, STOP_TIMEOUT_S, POLL_INTERVAL_S)

    after_status = adapter.status()
    assert stopped_status.actual_speed_sps <= 0.1
    assert after_status.position == stopped_status.position


def test_hw_stop_during_target_move(adapter: TMC2209Adapter) -> None:
    _prepare_adapter(adapter, 0)

    target = TARGET_STEPS + 10000
    adapter.set_speed_sps(STOP_SPEED_SPS)
    adapter.set_target(target)
    assert adapter.run() is True

    _wait_for_motion(adapter, 0, 1, 5.0, POLL_INTERVAL_S)

    assert adapter.stop() is True
    stopped_status = _wait_for_stop(adapter, STOP_TIMEOUT_S, POLL_INTERVAL_S)

    assert stopped_status.position != target
    assert stopped_status.target == target
    assert stopped_status.target_set is True
