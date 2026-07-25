"""RA protocol conformance suite: one case list, two transports.

The cases live in :mod:`ra_conformance.cases` as data transcribed from
``docs/protocol/RA_PROTOCOL.md``; :mod:`ra_conformance.runner` executes them
against anything that can send bytes and get bytes back. The simulator run is
part of the unit suite (``src/tests/units/test_ra_conformance.py``), the
hardware run is an explicit entry point (``python -m tools.ra_conformance
--port ...``, or ``pytest src/tests/hw/test_ra_conformance_hw.py``).
"""

from ra_conformance.cases import CASES, build_cases
from ra_conformance.model import (
    Case,
    CaseResult,
    Exchange,
    Expect,
    Pause,
    Safety,
    Status,
    StepResult,
    exact,
    matching,
    one_of,
    render_bytes,
    silence,
)
from ra_conformance.runner import (
    HARDWARE_DEFAULT_SAFETY,
    SIMULATOR_SAFETY,
    HardwareWire,
    SerialWire,
    Summary,
    UnsafeOnHardware,
    Wire,
    describe_cases,
    is_allowed_on_hardware,
    render_markdown,
    run_case,
    run_cases,
    stop_axis,
    summarize,
)

__all__ = [
    "CASES",
    "HARDWARE_DEFAULT_SAFETY",
    "SIMULATOR_SAFETY",
    "Case",
    "CaseResult",
    "Exchange",
    "Expect",
    "HardwareWire",
    "Pause",
    "Safety",
    "SerialWire",
    "Status",
    "StepResult",
    "Summary",
    "UnsafeOnHardware",
    "Wire",
    "build_cases",
    "describe_cases",
    "exact",
    "is_allowed_on_hardware",
    "matching",
    "one_of",
    "render_bytes",
    "render_markdown",
    "run_case",
    "run_cases",
    "silence",
    "stop_axis",
    "summarize",
]
