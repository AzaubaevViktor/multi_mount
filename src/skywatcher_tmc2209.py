from __future__ import annotations

import dataclasses
import logging
from typing import Optional

from lx200.plugins import (
    LX200ObjectPlugin,
    LX200PointingPlugin,
    LX200SitePlugin,
    LX200TimePlugin,
    LX200TrackingPlugin,
)
from lx200.protocol import LX200Command, LX200GotoResult
from lx200.server import LX200CommandHandler, LX200Server
from lx200_combine.splitter import (
    LX200CombineResponseMismatchError,
    LX200PrimaryAxis,
    LX200Splitter,
)
from skywatcher_lx200.common import SkyWatcherBackendConstants
from skywatcher_lx200.mount import SkyWatcherMount
from skywatcher_lx200.object import SkyWatcherObjectBackend
from skywatcher_lx200.pointing import SkyWatcherPointingBackend
from skywatcher_lx200.site import SkyWatcherSiteBackend
from skywatcher_lx200.time import SkyWatcherTimeBackend
from skywatcher_lx200.tracking import SkyWatcherTrackingBackend
from tmc2209_lx200.mount import TMC2209Mount
from tmc2209_lx200.pointing import TMC2209PointingBackend
from tmc2209_lx200.tracking import TMC2209TrackingBackend


class SkyWatcherTMC2209SplitterConstants:
    LOGGER_NAME = "lx200.combine.skywatcher_tmc2209"
    DEFAULT_PRIMARY = LX200PrimaryAxis.RA
    DEFAULT_TMC_SITE_NAME = "TMC2209"


class SkyWatcherTMC2209SplitterError(Exception):
    pass


class SkyWatcherTMC2209SplitterConfigError(SkyWatcherTMC2209SplitterError):
    pass


@dataclasses.dataclass(frozen=True)
class SkyWatcherTMC2209SplitterConfig:
    primary: LX200PrimaryAxis = SkyWatcherTMC2209SplitterConstants.DEFAULT_PRIMARY
    skywatcher_site_name: str = SkyWatcherBackendConstants.DEFAULT_SITE_NAME
    tmc_site_name: str = SkyWatcherTMC2209SplitterConstants.DEFAULT_TMC_SITE_NAME

    def __post_init__(self) -> None:
        if self.primary not in (LX200PrimaryAxis.RA, LX200PrimaryAxis.DEC):
            raise SkyWatcherTMC2209SplitterConfigError(
                f"unsupported primary axis: {self.primary!r}"
            )
        if not self.skywatcher_site_name:
            raise SkyWatcherTMC2209SplitterConfigError("skywatcher site name is required")
        if not self.tmc_site_name:
            raise SkyWatcherTMC2209SplitterConfigError("tmc2209 site name is required")


class SkyWatcherTMC2209Splitter(LX200CommandHandler):
    def __init__(
        self,
        skywatcher_mount: SkyWatcherMount,
        tmc_mount: TMC2209Mount,
        config: SkyWatcherTMC2209SplitterConfig | None = None,
        *,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        if skywatcher_mount is None or tmc_mount is None:
            raise SkyWatcherTMC2209SplitterConfigError("Both mounts are required")
        self._config = config or SkyWatcherTMC2209SplitterConfig()
        self._log = logger or logging.getLogger(SkyWatcherTMC2209SplitterConstants.LOGGER_NAME)
        self._skywatcher_mount = skywatcher_mount
        self._tmc_mount = tmc_mount
        self._ra_handler = self._build_skywatcher_handler(
            self._skywatcher_mount,
            site_name=self._config.skywatcher_site_name,
        )
        self._dec_handler = self._build_tmc_handler(
            self._tmc_mount,
            site_name=self._config.tmc_site_name,
        )
        self._splitter = LX200Splitter(
            self._ra_handler,
            self._dec_handler,
            primary=self._config.primary,
            logger=self._log,
        )
        self._log.info(
            "skywatcher+tmc2209 splitter init primary=%s", self._config.primary
        )

    def handle_command(self, raw: str) -> str:
        try:
            return self._splitter.handle_command(raw)
        except LX200CombineResponseMismatchError as exc:
            if exc.command != LX200Command.GOTO:
                raise
            return self._combine_goto_response(exc.ra_response, exc.dec_response)

    def close(self) -> None:
        self._log.info("skywatcher+tmc2209 splitter close")
        self._skywatcher_mount.close()
        self._tmc_mount.close()

    @staticmethod
    def _build_skywatcher_handler(
        mount: SkyWatcherMount,
        *,
        site_name: str,
    ) -> LX200CommandHandler:
        plugins = [
            LX200PointingPlugin(SkyWatcherPointingBackend(mount)),
            LX200TimePlugin(SkyWatcherTimeBackend()),
            LX200SitePlugin(SkyWatcherSiteBackend(site_name=site_name)),
            LX200TrackingPlugin(SkyWatcherTrackingBackend(mount)),
            LX200ObjectPlugin(SkyWatcherObjectBackend()),
        ]
        return LX200Server(plugins)

    @staticmethod
    def _build_tmc_handler(
        mount: TMC2209Mount,
        *,
        site_name: str,
    ) -> LX200CommandHandler:
        # TODO: extract shared time/site/object backends into a neutral lx200 module.
        plugins = [
            LX200PointingPlugin(TMC2209PointingBackend(mount)),
            LX200TimePlugin(SkyWatcherTimeBackend()),
            LX200SitePlugin(SkyWatcherSiteBackend(site_name=site_name)),
            LX200TrackingPlugin(TMC2209TrackingBackend(mount)),
            LX200ObjectPlugin(SkyWatcherObjectBackend()),
        ]
        return LX200Server(plugins)

    def _combine_goto_response(self, ra_response: str, dec_response: str) -> str:
        results = []
        for response in (ra_response, dec_response):
            try:
                results.append(LX200GotoResult(response))
            except ValueError as exc:
                raise SkyWatcherTMC2209SplitterError(
                    f"unsupported goto response: {response!r}"
                ) from exc
        if LX200GotoResult.BELOW_HORIZON in results:
            return LX200GotoResult.BELOW_HORIZON.value
        if LX200GotoResult.OK in results:
            return LX200GotoResult.OK.value
        return LX200GotoResult.ALREADY_THERE.value
