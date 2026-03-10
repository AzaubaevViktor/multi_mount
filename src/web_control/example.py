from __future__ import annotations

import threading
import time

from web_control.web import MonitorMixin, MonitorRenderer, MonitorServer, monitor_action, monitor_field, monitor_group


class SkyWatcherMonitorExample(MonitorMixin):
    monitor_name = "SkyWatcher"
    monitor_groups = (
        monitor_group("main", "Mount"),
        monitor_group("logs", "Logs"),
        monitor_group("details", "Details"),
    )
    monitor_fields = (
        monitor_field("headline", "Headline", lambda self: "SkyWatcher monitoring online", renderer=MonitorRenderer.TEXT),
        monitor_field("connected", "Connected", "connected"),
        monitor_field("tracking_rate", "Tracking rate", "tracking_rate", mode="rw", setter="set_tracking_rate"),
        monitor_field("status", "Status", lambda self: {"ra": self.ra, "dec": self.dec}, renderer=MonitorRenderer.JSON, group="details"),
        monitor_field("jobs", "Recent events", lambda self: list(self.events), renderer=MonitorRenderer.LIST, group="details"),
        monitor_field("logs", "Logger", lambda self: self._monitor_log, renderer=MonitorRenderer.LOGGER, group="logs"),
    )
    monitor_actions = (
        monitor_action("park", "Park", "park"),
        monitor_action("slew", "Slew", "slew_to"),
        monitor_action("refresh", "Refresh structure", "refresh_layout"),
    )

    def __init__(self) -> None:
        super().__init__()
        self.connected = True
        self.tracking_rate = 1.0
        self.ra = "12:15:00"
        self.dec = "+45:00:00"
        self.events = ["boot complete"]
        self._thread = threading.Thread(target=self._tick, daemon=True)
        self._thread.start()

    def set_tracking_rate(self, value: str) -> str:
        self.tracking_rate = float(value)
        self.monitor_log_line(f"tracking rate set to {self.tracking_rate}")
        return f"{self.tracking_rate:.3f}"

    def park(self) -> str:
        self.connected = False
        self.events.append("parked")
        self.monitor_log_line("mount parked")
        return "parked"

    def slew_to(self, ra: str, dec: str) -> dict[str, str]:
        self.ra = ra
        self.dec = dec
        self.events.append(f"slew {ra} {dec}")
        self.monitor_log_line(f"slew_to ra={ra} dec={dec}")
        return {"ra": ra, "dec": dec}

    def refresh_layout(self) -> str:
        self.monitor_force_refresh()
        return "structure refresh requested"

    def _tick(self) -> None:
        while True:
            if self.connected:
                seconds = int(time.time()) % 60
                self.ra = f"12:15:{seconds:02d}"
                if len(self.events) > 20:
                    del self.events[:-20]
            time.sleep(1)


def create_demo_server(host: str = "127.0.0.1", port: int = 8765) -> MonitorServer:
    return MonitorServer({"skywatcher": SkyWatcherMonitorExample()}, host=host, port=port)
