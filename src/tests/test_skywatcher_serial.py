import os
import time
import pytest

from lib.skywatcher import SkyWatcherMC
from serial_prims import SerialLineDevice


@pytest.fixture(scope="module")
def sw_device():
    port = os.environ.get("SKYWATCHER_PORT", '/dev/tty.PL2303G-USBtoUART2120')
    if not port:
        pytest.skip("SKYWATCHER_PORT not set; skipping hardware tests")
    # 9600 baud as used by the C++ driver
    dev = SerialLineDevice(port=port, baud=115200, timeout_s=1.0, name="test-swmc")
    mc = SkyWatcherMC(dev)
    yield mc
    try:
        dev.close()
    except Exception:
        pass


def test_connection_and_basic_queries(sw_device):
    mc = sw_device
    # inquire timer freq (basic command) and ensure integer returned
    tf = mc.inquire_timer_freq()
    assert isinstance(tf, int)


def test_check_position_and_status(sw_device):
    mc = sw_device
    pos = mc.inquire_position("1")
    assert isinstance(pos, int)
    st = mc.inquire_status("1")
    assert isinstance(st, int)


def test_enable_target_mode_and_update_pos(sw_device):
    mc = sw_device
    start = mc.inquire_position("1")
    # set a modest positive target (absolute target)
    target_inc = 200
    mc.set_goto_target("1", start + target_inc)
    mc.start_motion("1")
    # wait until position changes or timeout
    deadline = time.time() + 10
    moved = False
    last = start
    while time.time() < deadline:
        cur = mc.inquire_position("1")
        if cur != last:
            moved = True
            break
        time.sleep(0.2)
    mc.stop_motion("1")
    assert moved, "mount did not move after goto target/start"


def test_do_goto_and_check_stop(sw_device):
    mc = sw_device
    start = mc.inquire_position("1")
    # small goto: target 300 steps ahead
    target = start + 300
    mc.set_goto_target("1", target)
    mc.start_motion("1")
    # wait for movement to complete (position to settle)
    prev = mc.inquire_position("1")
    deadline = time.time() + 20
    while time.time() < deadline:
        time.sleep(0.5)
        cur = mc.inquire_position("1")
        if cur == prev:
            # settled
            break
        prev = cur
    final = mc.inquire_position("1")
    # expect final to be different from start
    assert final != start


def test_do_goto_backwards(sw_device):
    mc = sw_device
    start = mc.inquire_position("1")
    target = start - 250
    mc.set_goto_target("1", target)
    mc.start_motion("1")
    # wait a short while for movement
    time.sleep(2)
    mc.stop_motion("1")
    cur = mc.inquire_position("1")
    assert cur != start


def test_move_left_and_right_ra(sw_device):
    mc = sw_device
    start = mc.inquire_position("1")
    # move right (ccw=False)
    mc.set_step_period("1", 1000)
    mc.set_motion_mode("1", tracking=False, ccw=False, fast=False)
    mc.start_motion("1")
    time.sleep(1.0)
    mc.stop_motion("1")
    right_pos = mc.inquire_position("1")
    # move left (ccw=True)
    mc.set_motion_mode("1", tracking=False, ccw=True, fast=False)
    mc.start_motion("1")
    time.sleep(1.0)
    mc.stop_motion("1")
    left_pos = mc.inquire_position("1")
    assert right_pos != start or left_pos != right_pos


def test_enable_modes_and_check(sw_device):
    mc = sw_device
    # enable tracking mode and check no exception
    mc.set_motion_mode("1", tracking=True, ccw=False, fast=False)
    # set high speed and medium flags
    mc.set_motion_mode("1", tracking=False, ccw=False, fast=True, medium=True)
    assert True
