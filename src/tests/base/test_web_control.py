from web_control.example import SkyWatcherMonitorExample
from web_control.web import MonitorRegistry


def test_example_monitor_exposes_structure_and_actions() -> None:
    monitor = SkyWatcherMonitorExample()
    structure = monitor.monitor_structure()

    assert structure["name"] == "SkyWatcher"
    assert any(field["id"] == "tracking_rate" for field in structure["fields"])
    assert any(action["id"] == "slew" for action in structure["actions"])


def test_registry_updates_after_field_write() -> None:
    monitor = SkyWatcherMonitorExample()
    registry = MonitorRegistry({"skywatcher": monitor}, poll_interval_s=0.01)

    registry.set_field("skywatcher", "tracking_rate", "1.5")
    snapshot = monitor.monitor_snapshot()

    assert snapshot["tracking_rate"] == 1.5
