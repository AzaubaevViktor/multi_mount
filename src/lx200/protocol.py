from enum import StrEnum


class AlignmentMode(StrEnum):
    ALT_AZ = "A"
    LAND = "L"
    POLAR = "P"


class Protocol:
    ALIGNMENT_QUERY_BYTE = b"\x06"
    TERMINATOR = "#"
