from __future__ import annotations

from lx200.plugins.object import LX200ObjectBackend

from .common import SkyWatcherBackendConstants


class SkyWatcherObjectBackend(LX200ObjectBackend):
    def __init__(
        self,
        distance: str = SkyWatcherBackendConstants.DEFAULT_DISTANCE,
        object_size: str = SkyWatcherBackendConstants.DEFAULT_OBJECT_SIZE,
    ) -> None:
        self._distance = distance
        self._object_size = str(object_size)

    def set_object_size(self, value: str) -> bool:
        self._object_size = value
        return True

    def get_distance(self) -> str:
        return self._distance
