"""
Progress note:
- Done: standard-library monitoring prototype with descriptors, monitor mixin, background diff polling, SSE live updates, editable fields, action calls, minimal HTML UI, and a SkyWatcher-style demo monitor.
- Not done: true WebSocket transport, auth, stronger type coercion/validation, richer nested layout handling, and production hardening.
- Architecture and remaining work are tracked in ARCHITECTURE.md.
"""

from __future__ import annotations

from dataclasses import dataclass, field as dataclass_field
from enum import StrEnum
import inspect
import json
import logging
from pathlib import Path
import queue
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Mapping, get_type_hints
from urllib.parse import parse_qs, urlparse


JsonValue = None | bool | int | float | str | list["JsonValue"] | dict[str, "JsonValue"]
STATIC_DIR = Path(__file__).with_name("static")


class MonitorRenderer(StrEnum):
    VALUE = "value"
    TEXT = "text"
    JSON = "json"
    LIST = "list"
    LOGGER = "logger"


@dataclass(frozen=True)
class MonitorField:
    id: str
    label: str
    source: str | Callable[[Any], Any]
    renderer: MonitorRenderer = MonitorRenderer.VALUE
    mode: str = "ro"
    setter: str | Callable[[Any, Any], Any] | None = None
    group: str = "main"
    metadata: dict[str, JsonValue] = dataclass_field(default_factory=dict)


@dataclass(frozen=True)
class MonitorAction:
    id: str
    label: str
    callback: str | Callable[..., Any]
    group: str = "main"


@dataclass(frozen=True)
class MonitorGroup:
    id: str
    label: str
    children: tuple["MonitorGroup", ...] = ()
    metadata: dict[str, JsonValue] = dataclass_field(default_factory=dict)


def monitor_field(
    id: str,
    label: str,
    source: str | Callable[[Any], Any],
    *,
    renderer: MonitorRenderer = MonitorRenderer.VALUE,
    mode: str = "ro",
    setter: str | Callable[[Any, Any], Any] | None = None,
    group: str = "main",
    **metadata: JsonValue,
) -> MonitorField:
    return MonitorField(id=id, label=label, source=source, renderer=renderer, mode=mode, setter=setter, group=group, metadata=metadata)


def monitor_action(id: str, label: str, callback: str | Callable[..., Any], *, group: str = "main") -> MonitorAction:
    return MonitorAction(id=id, label=label, callback=callback, group=group)


def monitor_group(id: str, label: str, *children: MonitorGroup, **metadata: JsonValue) -> MonitorGroup:
    return MonitorGroup(id=id, label=label, children=children, metadata=metadata)


class MemoryLog:
    def __init__(self, limit: int = 200) -> None:
        self._limit = limit
        self._lines: list[str] = []
        self._lock = threading.Lock()

    def add(self, line: str) -> None:
        with self._lock:
            self._lines.append(line)
            if len(self._lines) > self._limit:
                del self._lines[:-self._limit]

    def snapshot(self) -> list[str]:
        with self._lock:
            return list(self._lines)


class MonitorMixin:
    monitor_name = "Monitor"
    monitor_groups: tuple[MonitorGroup, ...] = (MonitorGroup(id="main", label="Main"),)
    monitor_fields: tuple[MonitorField, ...] = ()
    monitor_actions: tuple[MonitorAction, ...] = ()

    def __init__(self) -> None:
        self._monitor_refresh_version = 0
        self._monitor_log = MemoryLog()

    def monitor_structure(self) -> dict[str, JsonValue]:
        field_payload = []
        for item in self.monitor_fields:
            value = self._monitor_resolve_value(item.source)
            field_payload.append(
                {
                    "id": item.id,
                    "label": item.label,
                    "renderer": item.renderer.value,
                    "mode": item.mode,
                    "group": item.group,
                    "value": self._monitor_jsonify(value),
                    "metadata": item.metadata,
                }
            )

        action_payload = []
        for item in self.monitor_actions:
            callback = self._monitor_resolve_callback(item.callback)
            signature = inspect.signature(callback)
            hints = get_type_hints(callback)
            arguments = []
            for name, parameter in signature.parameters.items():
                if name == "self":
                    continue
                annotation = hints.get(name, parameter.annotation)
                arguments.append(
                    {
                        "name": name,
                        "kind": parameter.kind.name,
                        "type": self._monitor_type_name(annotation),
                        "required": parameter.default is inspect.Signature.empty,
                        "default": None if parameter.default is inspect.Signature.empty else self._monitor_jsonify(parameter.default),
                    }
                )
            action_payload.append({"id": item.id, "label": item.label, "group": item.group, "arguments": arguments})

        return {
            "name": self.monitor_name,
            "groups": [self._monitor_group_to_payload(item) for item in self.monitor_groups],
            "fields": field_payload,
            "actions": action_payload,
            "refresh_version": self._monitor_refresh_version,
        }

    def monitor_snapshot(self) -> dict[str, JsonValue]:
        snapshot: dict[str, JsonValue] = {}
        for item in self.monitor_fields:
            snapshot[item.id] = self._monitor_jsonify(self._monitor_resolve_value(item.source))
        return snapshot

    def monitor_set_value(self, field_id: str, value: Any) -> JsonValue:
        item = next((field for field in self.monitor_fields if field.id == field_id), None)
        if item is None:
            raise KeyError(f"Unknown field: {field_id}")
        if item.mode != "rw" or item.setter is None:
            raise PermissionError(f"Field is read-only: {field_id}")
        setter = self._monitor_resolve_callback(item.setter)
        result = setter(value)
        return self._monitor_jsonify(result)

    def monitor_invoke_action(self, action_id: str, arguments: Mapping[str, Any] | None = None) -> JsonValue:
        item = next((action for action in self.monitor_actions if action.id == action_id), None)
        if item is None:
            raise KeyError(f"Unknown action: {action_id}")
        callback = self._monitor_resolve_callback(item.callback)
        signature = inspect.signature(callback)
        bound = signature.bind_partial(**(dict(arguments or {})))
        bound.apply_defaults()
        result = callback(*bound.args, **bound.kwargs)
        return self._monitor_jsonify(result)

    def monitor_force_refresh(self) -> None:
        self._monitor_refresh_version += 1

    def monitor_log_line(self, line: str) -> None:
        self._monitor_log.add(line)

    def _monitor_group_to_payload(self, item: MonitorGroup) -> dict[str, JsonValue]:
        return {
            "id": item.id,
            "label": item.label,
            "metadata": item.metadata,
            "children": [self._monitor_group_to_payload(child) for child in item.children],
        }

    def _monitor_resolve_callback(self, callback: str | Callable[..., Any]) -> Callable[..., Any]:
        if isinstance(callback, str):
            return getattr(self, callback)
        return callback.__get__(self, type(self)) if hasattr(callback, "__get__") else callback

    def _monitor_resolve_value(self, source: str | Callable[[Any], Any]) -> Any:
        value = getattr(self, source) if isinstance(source, str) else source(self)
        if isinstance(value, MemoryLog):
            return value.snapshot()
        if callable(value):
            value = value()
        return value

    def _monitor_type_name(self, annotation: Any) -> str:
        if annotation is inspect.Signature.empty:
            return "any"
        if isinstance(annotation, type):
            return annotation.__name__
        return str(annotation)

    def _monitor_jsonify(self, value: Any) -> JsonValue:
        if isinstance(value, (type(None), bool, int, float, str)):
            return value
        if isinstance(value, Mapping):
            return {str(key): self._monitor_jsonify(item) for key, item in value.items()}
        if isinstance(value, (list, tuple, set)):
            return [self._monitor_jsonify(item) for item in value]
        if hasattr(value, "__dict__"):
            return {key: self._monitor_jsonify(item) for key, item in vars(value).items() if not key.startswith("_")}
        return str(value)


class MonitorRegistry:
    def __init__(self, monitors: Mapping[str, MonitorMixin], poll_interval_s: float = 0.25) -> None:
        self._monitors = dict(monitors)
        self._poll_interval_s = poll_interval_s
        self._subscribers: list[queue.Queue[dict[str, JsonValue]]] = []
        self._lock = threading.Lock()
        self._last_snapshots = {monitor_id: monitor.monitor_snapshot() for monitor_id, monitor in self._monitors.items()}
        self._last_refresh_versions = {monitor_id: monitor.monitor_structure()["refresh_version"] for monitor_id, monitor in self._monitors.items()}
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @property
    def monitors(self) -> dict[str, MonitorMixin]:
        return self._monitors

    def subscribe(self) -> queue.Queue[dict[str, JsonValue]]:
        subscriber: queue.Queue[dict[str, JsonValue]] = queue.Queue()
        with self._lock:
            self._subscribers.append(subscriber)
        return subscriber

    def unsubscribe(self, subscriber: queue.Queue[dict[str, JsonValue]]) -> None:
        with self._lock:
            if subscriber in self._subscribers:
                self._subscribers.remove(subscriber)

    def structure_payload(self) -> dict[str, JsonValue]:
        return {
            "monitors": {
                monitor_id: monitor.monitor_structure()
                for monitor_id, monitor in self._monitors.items()
            }
        }

    def set_field(self, monitor_id: str, field_id: str, value: Any) -> JsonValue:
        result = self._monitors[monitor_id].monitor_set_value(field_id, value)
        self._emit({"type": "ack", "monitor": monitor_id, "field": field_id, "result": result})
        return result

    def invoke_action(self, monitor_id: str, action_id: str, arguments: Mapping[str, Any]) -> JsonValue:
        result = self._monitors[monitor_id].monitor_invoke_action(action_id, arguments)
        self._emit({"type": "action_result", "monitor": monitor_id, "action": action_id, "result": result})
        return result

    def _emit(self, event: dict[str, JsonValue]) -> None:
        stale: list[queue.Queue[dict[str, JsonValue]]] = []
        with self._lock:
            for subscriber in self._subscribers:
                try:
                    subscriber.put_nowait(event)
                except queue.Full:
                    stale.append(subscriber)
            for subscriber in stale:
                self._subscribers.remove(subscriber)

    def _run(self) -> None:
        while True:
            for monitor_id, monitor in self._monitors.items():
                refresh_version = monitor.monitor_structure()["refresh_version"]
                if refresh_version != self._last_refresh_versions[monitor_id]:
                    self._last_refresh_versions[monitor_id] = refresh_version
                    self._emit({"type": "structure", "monitor": monitor_id, "payload": monitor.monitor_structure()})

                snapshot = monitor.monitor_snapshot()
                if snapshot != self._last_snapshots[monitor_id]:
                    self._last_snapshots[monitor_id] = snapshot
                    self._emit({"type": "update", "monitor": monitor_id, "fields": snapshot})
            time.sleep(self._poll_interval_s)


class MonitorRequestHandler(BaseHTTPRequestHandler):
    registry: MonitorRegistry

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self._send_static("index.html", "text/html; charset=utf-8")
            return
        if parsed.path.startswith("/static/"):
            self._send_static(parsed.path.removeprefix("/static/"))
            return
        if parsed.path == "/api/structure":
            self._send_json(self.registry.structure_payload())
            return
        if parsed.path == "/events":
            subscriber = self.registry.subscribe()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            try:
                while True:
                    event = subscriber.get(timeout=15)
                    self.wfile.write(f"data: {json.dumps(event)}\n\n".encode("utf-8"))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, queue.Empty):
                pass
            finally:
                self.registry.unsubscribe(subscriber)
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            payload = {key: values[0] for key, values in parse_qs(raw.decode("utf-8")).items()}

        try:
            if parsed.path == "/api/set":
                result = self.registry.set_field(payload["monitor"], payload["field"], payload.get("value"))
                self._send_json({"ok": True, "result": result})
                return
            if parsed.path == "/api/action":
                result = self.registry.invoke_action(payload["monitor"], payload["action"], payload.get("arguments", {}))
                self._send_json({"ok": True, "result": result})
                return
            self.send_error(HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self._send_json({"ok": False, "error": f"{type(exc).__name__}: {exc}"}, status=HTTPStatus.BAD_REQUEST)

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send_text(self, payload: str, content_type: str, status: HTTPStatus = HTTPStatus.OK) -> None:
        raw = payload.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _send_json(self, payload: Mapping[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        self._send_text(json.dumps(payload), "application/json; charset=utf-8", status=status)

    def _send_static(self, relative_path: str, content_type: str | None = None) -> None:
        path = (STATIC_DIR / relative_path).resolve()
        if STATIC_DIR.resolve() not in path.parents and path != STATIC_DIR.resolve():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        if not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        if content_type is None:
            if path.suffix == ".css":
                content_type = "text/css; charset=utf-8"
            elif path.suffix == ".js":
                content_type = "application/javascript; charset=utf-8"
            else:
                content_type = "text/plain; charset=utf-8"
        self._send_text(path.read_text(encoding="utf-8"), content_type)


class MonitorServer:
    def __init__(self, monitors: Mapping[str, MonitorMixin], host: str = "127.0.0.1", port: int = 8765, poll_interval_s: float = 0.25) -> None:
        self._log = logging.getLogger("monitor_server")
        self.registry = MonitorRegistry(monitors, poll_interval_s=poll_interval_s)
        handler = type("BoundMonitorRequestHandler", (MonitorRequestHandler,), {"registry": self.registry})
        self._server = ThreadingHTTPServer((host, port), handler)
        self.host = host
        self.port = port

    @property
    def url(self) -> str:
        return f"http://{self.host}:{self.port}/"

    def serve_forever(self) -> None:
        self._log.warning("MonitorServer available at %s", self.url)
        self._server.serve_forever()

    def server_close(self) -> None:
        self._server.server_close()
