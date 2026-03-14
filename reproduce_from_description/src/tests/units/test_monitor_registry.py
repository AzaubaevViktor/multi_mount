from __future__ import annotations

from web_control.web import MonitorField, MonitorGroup, MonitorRegistry


class FakeMonitor:
    def monitor_groups(self):
        return [MonitorGroup(name="axis", fields=[MonitorField(name="mode", value="track")])]


def test_registry_emits_snapshot_with_versioned_objects() -> None:
    registry = MonitorRegistry()
    registry.register(FakeMonitor(), "ra")

    snapshot = registry.poll_once()

    assert snapshot["version"] == 1
    assert snapshot["objects"]["ra"]["groups"][0]["fields"][0]["value"] == "track"
