"""Red tests for П8 (log format breaks grep/parsing), PLAN.md §1 П8.

Observed in real sessions:
  - 50-73% of lines in error-heavy files are traceback continuation lines with
    no timestamp;
  - ``read_all_data`` writes ``\\n['']`` inside a record => 20.6k bare lines in
    serial.log;
  - ``[WARNING]`` breaks column alignment (level padded to 6 chars, WARNING is 7).

Desired contract for ``RelativeFormatter`` (src/logging_setup.py):
  (a) a record with exc_info formats into ONE physical line (newlines escaped or
      the traceback embedded without bare untimestamped lines);
  (b) one uniform line prefix parses records of ALL levels, including WARNING:
      the level field has the same width for every level;
  (c) a multi-line payload (like ``\\n['']`` from read_all_data) produces no
      output lines without the timestamped prefix.
"""

import logging
import re
import time

from logging_setup import RelativeFormatter


# Every physical log line must start with the timestamped prefix:
# "YYYY-MM-DD [LEVEL] ..." — continuation lines without it break grep/parsing.
_PREFIX_RE = re.compile(r"^\d{4}-\d{2}-\d{2} \[[A-Z ]+\] ")


def _make_formatter() -> RelativeFormatter:
    return RelativeFormatter(start_time=time.monotonic(), base_path="/tmp")


def _make_record(level: int, msg: str, exc_info=None) -> logging.LogRecord:
    return logging.LogRecord(
        name="serial.ra",
        level=level,
        pathname=__file__,
        lineno=42,
        msg=msg,
        args=(),
        exc_info=exc_info,
    )


def test_record_with_exc_info_formats_into_single_line() -> None:
    """П8(a): exc_info record = one physical line, no bare traceback lines.

    See PLAN.md §1 П8 / §3 item 3.
    """
    formatter = _make_formatter()

    try:
        raise ValueError("boom")
    except ValueError:
        import sys

        record = _make_record(logging.ERROR, "While quering INQUIRE_VOLTAGE", exc_info=sys.exc_info())

    formatted = formatter.format(record)
    lines = formatted.splitlines()

    assert len(lines) == 1, (
        f"exc_info record produced {len(lines)} physical lines (expected 1); "
        f"continuation lines have no timestamp and break grep: {lines[1:3]!r}..."
    )


def test_level_field_width_is_uniform_across_levels() -> None:
    """П8(b): one regex / fixed columns must parse every level incl. WARNING.

    See PLAN.md §1 П8.
    """
    formatter = _make_formatter()
    levels = (logging.DEBUG, logging.INFO, logging.WARNING, logging.ERROR, logging.CRITICAL)

    prefix_widths = {}
    for level in levels:
        formatted = formatter.format(_make_record(level, "message"))
        match = re.match(r"^\d{4}-\d{2}-\d{2} \[([A-Z ]+)\] ", formatted)
        assert match is not None, f"line does not match the uniform prefix: {formatted!r}"
        prefix_widths[logging.getLevelName(level)] = len(match.group(1))

    assert len(set(prefix_widths.values())) == 1, (
        f"level field width differs across levels, columns are broken for a "
        f"single parsing regex: {prefix_widths}"
    )


def test_multiline_payload_produces_no_bare_lines() -> None:
    """П8(c): payload with embedded newlines must not create untimestamped lines.

    See PLAN.md §1 П8. Mirrors ``read_all_data`` logging ``...input:\\n['']``.
    """
    formatter = _make_formatter()
    record = _make_record(logging.INFO, "Receive all data from input:\n['']")

    formatted = formatter.format(record)
    bare_lines = [line for line in formatted.splitlines() if not _PREFIX_RE.match(line)]

    assert not bare_lines, (
        f"multi-line payload produced {len(bare_lines)} bare line(s) without the "
        f"timestamped prefix: {bare_lines!r}"
    )
