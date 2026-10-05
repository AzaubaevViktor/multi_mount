"""Agent HTTP interface using the same SYNC and sensor services as LX200."""

from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from math import isfinite
from pathlib import Path
import threading
from typing import Any

from pointing.sensor import SensorReading, SensorState
from pointing.service import PointingService
from sim.orientation_sensor import OrientationSensorSim
from sky.lx200 import SkyLX200
from utils.polar_align_lib import Vec3


def finite_number(payload: dict[str, Any], key: str, default: float | None = None) -> float:
    value = payload.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
        raise ValueError(f"{key} must be a finite number")
    return float(value)


class AgentAPI:
    def __init__(self, sky: SkyLX200, pointing: PointingService, simulator: OrientationSensorSim | None = None) -> None:
        self._sky, self._pointing, self._simulator = sky, pointing, simulator

    def handle_request(self, method: str, path: str, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        if method == "GET" and path in ("/v1/status", "/v1/sensor", "/v1/calibration"):
            status = self._pointing.status()
            status["mount_connected"] = self._sky.is_connected()
            status["simulated"] = self._simulator is not None
            if path == "/v1/sensor":
                return 200, status["sensor"]
            if path == "/v1/calibration":
                return 200, status["calibration"]
            return 200, status
        if method != "POST":
            return 404, {"error": "unknown_endpoint"}
        try:
            if path == "/v1/site":
                self._pointing.set_site(finite_number(payload, "latitude_deg"), finite_number(payload, "longitude_deg"))
            elif path == "/v1/calibration/start":
                self._pointing.start_calibration()
            elif path == "/v1/calibration/stop":
                self._pointing.stop_calibration()
            elif path == "/v1/calibration/reset":
                self._pointing.reset_calibration()
            elif path == "/v1/sync":
                timestamp = payload.get("timestamp")
                if timestamp is not None and not isinstance(timestamp, str):
                    raise ValueError("timestamp must be an ISO 8601 string with timezone")
                measurement_id = payload.get("measurement_id")
                if measurement_id is not None and not isinstance(measurement_id, str):
                    raise ValueError("measurement_id must be a string")
                result = self._sky.sync_from_solve(
                    finite_number(payload, "ra_hours"),
                    finite_number(payload, "dec_deg"),
                    timestamp=datetime.fromisoformat(timestamp) if timestamp is not None else None,
                    measurement_id=measurement_id,
                )
                return 200, result
            elif path == "/v1/sim/sensor" and self._simulator is not None:
                if "state" in payload:
                    state = SensorState(payload["state"])
                    if state not in (SensorState.AVAILABLE, SensorState.DEVICE_NOT_FOUND, SensorState.NO_DATA):
                        raise ValueError("simulated wire state must be available, device_not_found or no_data")
                    gravity = payload.get("gravity_m_s2")
                    magnetic = payload.get("magnetic_uT")
                    if (gravity is not None and not isinstance(gravity, dict)) or (
                        magnetic is not None and not isinstance(magnetic, dict)
                    ):
                        raise ValueError("vectors must be objects with x,y,z components")
                    age = finite_number(payload, "age_ms", 0)
                    if not 0 <= age <= 65535 or age != int(age):
                        raise ValueError("age_ms must be an integer in [0,65535]")
                    frozen = payload.get("frozen", False)
                    if not isinstance(frozen, bool):
                        raise ValueError("frozen must be a boolean")
                    reading = SensorReading(
                        state,
                        0 if state is SensorState.AVAILABLE else None,
                        int(age) if state is SensorState.AVAILABLE else None,
                        Vec3(*(finite_number(gravity, k) for k in ("x", "y", "z"))) if gravity is not None else None,
                        Vec3(*(finite_number(magnetic, k) for k in ("x", "y", "z"))) if magnetic is not None else None,
                    )
                    for vector, scale in ((reading.gravity, 1000), (reading.magnetic, 100)):
                        if vector is not None and any(abs(v * scale) > 32767 for v in vector.as_tuple()):
                            raise ValueError("raw vector exceeds the sensor wire range")
                    self._simulator.set_raw(reading, frozen=frozen)
                else:
                    altitude, azimuth = finite_number(payload, "alt_deg"), finite_number(payload, "az_deg")
                    if not -90 <= altitude <= 90 or not 0 <= azimuth < 360:
                        raise ValueError("pose needs altitude [-90,90] and azimuth [0,360)")
                    self._simulator.set_pose(altitude, azimuth, mounting_deg=finite_number(payload, "mounting_deg", 17))
            else:
                return 404, {"error": "unknown_endpoint"}
        except (ValueError, TypeError, OverflowError) as error:
            return 400, {"error": str(error)}
        except OSError:
            logging.getLogger(__name__).exception("Cannot persist sensor calibration")
            return 503, {"error": "calibration_storage_unavailable"}
        return 200, self._pointing.status()


class AgentAPIServer:
    def __init__(self, api: AgentAPI, host: str = "127.0.0.1", port: int = 8080) -> None:
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                self.respond("GET")

            def do_POST(self) -> None:
                self.respond("POST")

            def respond(self, method: str) -> None:
                if method == "GET" and self.path == "/":
                    data = Path(__file__).with_name("dashboard.html").read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                    if not 0 <= size <= 65536:
                        raise ValueError("request body must fit in 65536 bytes")
                    payload = json.loads(self.rfile.read(size)) if size else {}
                    if not isinstance(payload, dict):
                        raise ValueError("JSON body must be an object")
                    code, result = api.handle_request(method, self.path, payload)
                except (ValueError, UnicodeError) as error:
                    code, result = 400, {"error": str(error)}
                data = json.dumps(result, allow_nan=False).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, format: str, *args: Any) -> None:
                logging.getLogger("agent_api").debug(format, *args)

        self._server = ThreadingHTTPServer((host, port), Handler)
        self._server.daemon_threads = True
        self._thread: threading.Thread | None = None

    @property
    def address(self) -> tuple[str, int]:
        host, port = self._server.server_address[:2]
        return str(host), int(port)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._server.serve_forever, name="agent_api", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._thread is not None:
            self._server.shutdown()
            self._thread.join()
            self._thread = None
        self._server.server_close()
