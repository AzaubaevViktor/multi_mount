from __future__ import annotations

import json
import logging
import threading
from dataclasses import asdict, dataclass, field
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from time import sleep
from typing import Any
from urllib.parse import parse_qs, urlparse

from sky.constants import MONITOR_POLL_INTERVAL


LOGGER = logging.getLogger(__name__)
STATIC_ROOT = Path(__file__).with_name("static")


@dataclass(slots=True)
class MonitorField:
    name: str
    value: Any
    unit: str = ""


@dataclass(slots=True)
class MonitorAction:
    name: str
    label: str
    method: str = "POST"


@dataclass(slots=True)
class MonitorGroup:
    name: str
    fields: list[MonitorField]
    actions: list[MonitorAction] = field(default_factory=list)


class MonitorMixin:
    def monitor_name(self) -> str:
        return self.__class__.__name__

    def monitor_groups(self) -> list[MonitorGroup] | list[dict[str, Any]]:
        return []


class MonitorRegistry:
    def __init__(self, poll_interval: float = MONITOR_POLL_INTERVAL) -> None:
        self.poll_interval = poll_interval
        self._lock = threading.RLock()
        self._objects: dict[str, Any] = {}
        self._snapshot: dict[str, Any] = {"version": 0, "objects": {}}
        self._events: list[dict[str, Any]] = []
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def register(self, obj: Any, name: str | None = None) -> None:
        key = name or getattr(obj, "monitor_name", lambda: obj.__class__.__name__)()
        with self._lock:
            self._objects[key] = obj

    def unregister(self, name: str) -> None:
        with self._lock:
            self._objects.pop(name, None)

    def start(self) -> None:
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._poll_loop, name="MonitorRegistryPoll", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return json.loads(json.dumps(self._snapshot))

    def poll_once(self) -> dict[str, Any]:
        objects: dict[str, Any] = {}
        with self._lock:
            registered = list(self._objects.items())

        for name, obj in registered:
            groups = obj.monitor_groups() if hasattr(obj, "monitor_groups") else []
            serialised_groups: list[dict[str, Any]] = []
            for group in groups:
                if isinstance(group, MonitorGroup):
                    serialised_groups.append(
                        {
                            "name": group.name,
                            "fields": [asdict(field) for field in group.fields],
                            "actions": [asdict(action) for action in group.actions],
                        }
                    )
                else:
                    serialised_groups.append(group)
            objects[name] = {"groups": serialised_groups}

        with self._lock:
            if objects != self._snapshot["objects"]:
                version = self._snapshot["version"] + 1
                self._snapshot = {"version": version, "objects": objects}
                self._events.append({"id": version, "snapshot": self.snapshot()})
                self._events = self._events[-32:]
            return self.snapshot()

    def events_since(self, last_id: int) -> list[dict[str, Any]]:
        with self._lock:
            return [event for event in self._events if event["id"] > last_id]

    def _poll_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.poll_once()
            except Exception:  # pragma: no cover - defensive runtime path
                LOGGER.exception("Monitor polling failed")
            sleep(self.poll_interval)


class MonitorRequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args: Any, directory: str | None = None, **kwargs: Any) -> None:
        super().__init__(*args, directory=str(STATIC_ROOT), **kwargs)

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path == "/api/snapshot":
            payload = json.dumps(self.server.registry.snapshot()).encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        if parsed.path == "/events":
            last_id = int(parse_qs(parsed.query).get("last_id", ["0"])[0])
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            while not self.server.stop_event.is_set():
                events = self.server.registry.events_since(last_id)
                for event in events:
                    last_id = event["id"]
                    payload = json.dumps(event["snapshot"])
                    self.wfile.write(f"id: {last_id}\ndata: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                sleep(0.2)
            return

        super().do_GET()

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A003
        LOGGER.info("monitor %s", format % args)


class _HTTPServer(ThreadingHTTPServer):
    def __init__(self, server_address: tuple[str, int], registry: MonitorRegistry) -> None:
        super().__init__(server_address, MonitorRequestHandler)
        self.registry = registry
        self.stop_event = threading.Event()


class MonitorServer:
    def __init__(self, host: str, port: int, registry: MonitorRegistry | None = None) -> None:
        self.registry = registry or MonitorRegistry()
        self._server = _HTTPServer((host, port), self.registry)
        self._thread: threading.Thread | None = None

    @property
    def server_address(self) -> tuple[str, int]:
        return self._server.server_address

    def start(self) -> None:
        self.registry.start()
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._server.serve_forever, name="MonitorServer", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._server.stop_event.set()
        self._server.shutdown()
        self._server.server_close()
        self.registry.stop()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)
