from web_control.example import SkyWatcherMonitorExample
from web_control.web import MonitorRegistry


def test_example_monitor_exposes_structure_and_actions() -> None:
    monitor = SkyWatcherMonitorExample()
    structure = monitor.monitor_structure()

    # `monitor_structure()` returns `dict[str, JsonValue]`, so narrow the payload
    # containers before indexing them.
    fields = structure["fields"]
    actions = structure["actions"]
    assert isinstance(fields, list)
    assert isinstance(actions, list)

    assert structure["name"] == "SkyWatcher"
    assert any(isinstance(field, dict) and field["id"] == "tracking_rate" for field in fields)
    assert any(isinstance(action, dict) and action["id"] == "slew" for action in actions)


def test_registry_updates_after_field_write() -> None:
    monitor = SkyWatcherMonitorExample()
    registry = MonitorRegistry({"skywatcher": monitor}, poll_interval_s=0.01)

    registry.set_field("skywatcher", "tracking_rate", "1.5")
    snapshot = monitor.monitor_snapshot()

    assert snapshot["tracking_rate"] == 1.5
