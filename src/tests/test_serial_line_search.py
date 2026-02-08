import pytest

from serial_wrapper.wrapper import (
    SerialLine,
    SerialLineSearchDirectoryError,
    SerialLineSearchInvalidPattern,
    SerialLineSearchNotFound,
    SerialLineSearchError,
)


USB_SERIAL_PATTERN = r"^tty\.usbserial.*$"


def test_serial_line_search_match(tmp_path):
    target = tmp_path / "tty.usbserial-A1B2"
    target.write_text("", encoding="ascii")

    path = SerialLine.search(USB_SERIAL_PATTERN, directory=str(tmp_path))

    assert path == str(target)


def test_serial_line_search_not_found(tmp_path):
    with pytest.raises(SerialLineSearchNotFound):
        SerialLine.search(USB_SERIAL_PATTERN, directory=str(tmp_path))


def test_serial_line_search_invalid_pattern():
    with pytest.raises(SerialLineSearchInvalidPattern):
        SerialLine.search("[")


def test_serial_line_search_directory_error(tmp_path):
    missing = tmp_path / "missing"
    with pytest.raises(SerialLineSearchDirectoryError):
        SerialLine.search(USB_SERIAL_PATTERN, directory=str(missing))


def test_serial_line_search_requires_pattern():
    with pytest.raises(SerialLineSearchError):
        SerialLine.search("")
