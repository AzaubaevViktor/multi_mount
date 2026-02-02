from __future__ import annotations

from lx200.models import LX200Site
from lx200.plugins import LX200SitePlugin
from lx200.server import LX200Server
from skywatcher_lx200.site import SkyWatcherSiteBackend

CMD_GET_LONGITUDE = ":Gg#"
LATITUDE_DEG = 50.0
LONGITUDE_WEST_DEG = 40.0
EXPECTED_LONGITUDE = "+040*00#"


def test_get_longitude_returns_site_longitude() -> None:
    backend = SkyWatcherSiteBackend(
        site=LX200Site(
            latitude_deg=LATITUDE_DEG,
            longitude_west_deg=LONGITUDE_WEST_DEG,
        )
    )
    server = LX200Server([LX200SitePlugin(backend)])

    result = server.handle_command(CMD_GET_LONGITUDE)

    assert result == EXPECTED_LONGITUDE
