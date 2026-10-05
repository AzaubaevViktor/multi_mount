"""Operator telemetry through the mount's public monitoring boundary."""

from collections import deque
from datetime import UTC, datetime
import threading
from typing import Any, Protocol

from sky.physics import Dec, Ha
from utils.polar_align_lib import ObserverSite, radec_to_altaz


class MountMonitor(Protocol):
    def monitor(self) -> dict[str, Any]: ...


class MountTelemetry:
    def __init__(self, mount: MountMonitor) -> None:
        self._mount = mount
        self._sample_lock = threading.Lock()
        self._samples: dict[str, deque[tuple[float, float | None, float]]] = {}
        self._revision: tuple[int, int] | None = None

    def status(self, site: ObserverSite | None = None) -> dict[str, Any]:
        result = self._mount.monitor()
        revision = (result["ra"]["position_revision"], result["dec"]["position_revision"])
        for name, position_type in (("ra", Ha), ("dec", Dec)):
            axis = result[name]
            axis["rates"] = {"mount_native_s": None, "motor_native_s": None, "interval_s": None}
            axis["mount_position_text"] = None
            if not axis["available"] or axis["motor"] is None:
                with self._sample_lock:
                    self._samples.pop(name, None)
                continue
            mount_position = result["equatorial"]
            native = None if mount_position is None else mount_position["ra_hours" if name == "ra" else "dec_deg"] * 3600
            axis["mount_position_text"] = str(position_type(native)) if native is not None else None
            observed = axis["observed_s"]
            current = (observed, native, axis["motor"]["position_native"])
            with self._sample_lock:
                if self._revision is not None and any(old < new for old, new in zip(revision, self._revision, strict=True)):
                    continue
                if revision != self._revision:
                    self._samples.clear()
                    self._revision = revision
                samples = self._samples.setdefault(name, deque())
                if samples and observed <= samples[-1][0]:
                    continue
                samples.append(current)
                while len(samples) > 1 and observed - samples[1][0] >= 1:
                    samples.popleft()
                first = samples[0]
            elapsed = observed - first[0]
            if not 1 <= elapsed <= 5:
                continue
            mount_rate = None if native is None or first[1] is None else float(position_type(native - first[1]).moving_wrap()) / elapsed
            motor_rate = float(position_type(current[2] - first[2]).moving_wrap()) / elapsed
            axis["rates"] = {"mount_native_s": mount_rate, "motor_native_s": motor_rate, "interval_s": elapsed}
        now = datetime.now(UTC)
        result["timestamp"] = now.isoformat()
        result["clock"] = now.astimezone().strftime("%H:%M:%S %Z")
        result["altaz"] = None
        if site is not None and result["equatorial"] is not None:
            position = result["equatorial"]
            altaz = radec_to_altaz(position["ra_hours"], position["dec_deg"], now, site)
            result["altaz"] = {"alt_deg": altaz.alt_deg, "az_deg": altaz.az_deg}
        return result
