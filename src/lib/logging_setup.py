import logging


class LoggingConstants:
    NAME_WIDTH = 20
    INDENT = "  "
    FORMAT_TEMPLATE = (
        "[%(levelname)-7s] %(asctime)s  %(pathname)s:%(lineno)d %(funcName)-{width}s %(name)-{width}s\n"
        "{indent}%(message)s"
    )
    DEFAULT_LEVEL = logging.DEBUG


def setup_logging(level: int = LoggingConstants.DEFAULT_LEVEL) -> None:
    format_str = LoggingConstants.FORMAT_TEMPLATE.format(
        width=LoggingConstants.NAME_WIDTH,
        indent=LoggingConstants.INDENT,
    )
    logging.basicConfig(level=level, format=format_str)
