from __future__ import annotations

from lx200.models import LX200Site
from lx200.plugins.site import LX200SiteBackend

from .common import SkyWatcherBackendConstants


class SkyWatcherSiteBackend(LX200SiteBackend):
    def __init__(
        self,
        site: LX200Site | None = None,
        site_name: str = SkyWatcherBackendConstants.DEFAULT_SITE_NAME,
    ) -> None:
        self._site = site or LX200Site(
            latitude_deg=SkyWatcherBackendConstants.ZERO_FLOAT,
            longitude_west_deg=SkyWatcherBackendConstants.ZERO_FLOAT,
        )
        self._site_name = site_name

    def set_latitude(self, latitude_deg: float) -> bool:
        self._site = LX200Site(latitude_deg=latitude_deg, longitude_west_deg=self._site.longitude_west_deg)
        return True

    def set_longitude(self, longitude_west_deg: float) -> bool:
        self._site = LX200Site(latitude_deg=self._site.latitude_deg, longitude_west_deg=longitude_west_deg)
        return True

    def get_latitude(self) -> float:
        return self._site.latitude_deg

    def get_longitude(self) -> float:
        return self._site.longitude_west_deg

    def get_site_name(self) -> str:
        return self._site_name
