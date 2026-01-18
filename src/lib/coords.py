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


def hours_to_hms(hours: float) -> Tuple[int, int, int]:
    hours = wrap_hours(hours)
    h = int(hours)
    rem = (hours - h) * 60.0
    m = int(rem)
    s = int(round((rem - m) * 60.0))
    if s == 60:
        s = 0
        m += 1
    if m == 60:
        m = 0
        h = (h + 1) % 24
    return h, m, s


def deg_to_dms(deg: float) -> Tuple[int, int, int, int]:
    sign = 1
    if deg < 0:
        sign = -1
        deg = -deg
    d = int(deg)
    rem = (deg - d) * 60.0
    m = int(rem)
    s = int(round((rem - m) * 60.0))
    if s == 60:
        s = 0
        m += 1
    if m == 60:
        m = 0
        d += 1
    return sign, d, m, s


def parse_ra_hms(s: str) -> float:
    parts = s.strip().split(":")
    if len(parts) != 3:
        raise ValueError(f"bad RA {s!r}")
    h, m, sec = (int(p) for p in parts)
    return wrap_hours(hms_to_hours(h, m, sec))


def parse_dec_dms(s: str) -> float:
    s = s.strip()
    sign = 1
    if s.startswith("-"):
        sign = -1
        s = s[1:]
    elif s.startswith("+"):
        s = s[1:]
    s = s.replace("°", "*")
    if "*" not in s:
        raise ValueError(f"bad DEC {s!r}")
    d_str, rest = s.split("*", 1)
    d = int(d_str)
    if ":" in rest:
        m_str, sec_str = rest.split(":", 1)
        m = int(m_str)
        sec = int(sec_str)
    else:
        m = int(rest)
        sec = 0
    deg = sign * (d + m / 60.0 + sec / 3600.0)
    return clamp(deg, -90.0, 90.0)


def fmt_ra(hours: float) -> str:
    h, m, s = hours_to_hms(hours)
    return f"{h:02d}:{m:02d}:{s:02d}#"


def fmt_dec(deg: float) -> str:
    deg = clamp(deg, -90.0, 90.0)
    sign, d, m, s = deg_to_dms(deg)
    pm = "+" if sign >= 0 else "-"
    return f"{pm}{d:02d}*{m:02d}:{s:02d}#"


def parse_lx200_signed_deg(s: str) -> float:
    s = s.strip()
    sign = 1
    if s.startswith("-"):
        sign = -1
        s = s[1:]
    elif s.startswith("+"):
        s = s[1:]
    s = s.replace("°", "*")
    d_str, m_str = s.split("*", 1)
    d = int(d_str)
    m = int(m_str)
    return sign * (d + m / 60.0)


def fmt_lx200_lat(deg: float) -> str:
    deg = clamp(deg, -90.0, 90.0)
    sign, d, m, _ = deg_to_dms(deg)
    pm = "+" if sign >= 0 else "-"
    return f"{pm}{d:02d}*{m:02d}#"


def fmt_lx200_lon(deg_east: float) -> str:
    deg_east = ((deg_east + 180.0) % 360.0) - 180.0
    sign, d, m, _ = deg_to_dms(deg_east)
    pm = "+" if sign >= 0 else "-"
    return f"{pm}{d:03d}*{m:02d}#"


def julian_date(t_utc: dt.datetime) -> float:
    if t_utc.tzinfo is None:
        t_utc = t_utc.replace(tzinfo=dt.timezone.utc)
    t_utc = t_utc.astimezone(dt.timezone.utc)
    y = t_utc.year
    m = t_utc.month
    d = t_utc.day
    hour = t_utc.hour + t_utc.minute / 60.0 + t_utc.second / 3600.0 + t_utc.microsecond / 3.6e9
    if m <= 2:
        y -= 1
        m += 12
    A = y // 100
    B = 2 - A + (A // 4)
    JD = int(365.25 * (y + 4716)) + int(30.6001 * (m + 1)) + d + B - 1524.5 + hour / 24.0
    return JD


def gmst_deg(t_utc: dt.datetime) -> float:
    JD = julian_date(t_utc)
    T = (JD - 2451545.0) / 36525.0
    gmst = 280.46061837 + 360.98564736629 * (JD - 2451545.0) + 0.000387933 * (T * T) - (T * T * T) / 38710000.0
    return wrap_deg(gmst)


def lst_hours(t_utc: dt.datetime, lon_deg_east: float) -> float:
    return wrap_hours((gmst_deg(t_utc) + lon_deg_east) / 15.0)
