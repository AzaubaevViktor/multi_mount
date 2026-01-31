from __future__ import annotations
import datetime as dt
from typing import Tuple


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def wrap_deg(deg: float) -> float:
    deg = deg % 360.0
    if deg < 0:
        deg += 360.0
    return deg


def wrap_hours(h: float) -> float:
    h = h % 24.0
    if h < 0:
        h += 24.0
    return h


def hms_to_hours(h: int, m: int, s: int) -> float:
    return h + m / 60.0 + s / 3600.0
