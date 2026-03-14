from datetime import datetime, timezone

import pytest

from utils.polar_align_lib import (
    ObserverSite,
    PlateSolve,
    PolarAlignmentSession,
    format_solution_coordinates,
    interactive_main,
    parse_solution_coordinates,
    parse_timestamp,
)


def test_parse_timestamp_accepts_z_suffix() -> None:
    timestamp = parse_timestamp("2026-03-14T20:15:00Z")

    assert timestamp == datetime(2026, 3, 14, 20, 15, tzinfo=timezone.utc)


def test_parse_solution_coordinates_accepts_plate_solve_text() -> None:
    coord = parse_solution_coordinates('Solution coordinates: RA (14h 41m 57s) DEC ( 41° 24\' 42")')

    assert coord.ra_hours == pytest.approx(14.699166666666667)
    assert coord.dec_deg == pytest.approx(41.41166666666667)


def test_interactive_main_runs_full_alignment_workflow() -> None:
    site = ObserverSite(latitude_deg=43.238949, longitude_deg=76.889709)
    solves = [
        PlateSolve(ra_hours=5.2, dec_deg=20.1, timestamp=datetime(2026, 3, 14, 20, 0, tzinfo=timezone.utc), label="solve-1"),
        PlateSolve(ra_hours=5.9, dec_deg=20.2, timestamp=datetime(2026, 3, 14, 20, 10, tzinfo=timezone.utc), label="solve-2"),
        PlateSolve(ra_hours=6.6, dec_deg=20.1, timestamp=datetime(2026, 3, 14, 20, 20, tzinfo=timezone.utc), label="solve-3"),
    ]
    session = PolarAlignmentSession(site=site)

    for solve in solves:
        session.add_solve(solve)

    result = session.solve_alignment()
    responses = iter([
        "43.238949",
        "76.889709",
        "solve",
        format_solution_coordinates(solves[0].ra_hours, solves[0].dec_deg),
        "2026-03-14T20:00:00+00:00",
        "solve",
        format_solution_coordinates(solves[1].ra_hours, solves[1].dec_deg),
        "2026-03-14T20:10:00+00:00",
        "solve",
        format_solution_coordinates(solves[2].ra_hours, solves[2].dec_deg),
        "2026-03-14T20:20:00+00:00",
        "compute",
        "verify",
        format_solution_coordinates(result.target.target_ra_hours, result.target.target_dec_deg),
        "2026-03-14T20:25:00+00:00",
        "show",
        "quit",
    ])
    output_lines = []

    def fake_input(prompt: str) -> str:
        try:
            return next(responses)
        except StopIteration as exc:
            raise AssertionError(f"Unexpected prompt: {prompt}") from exc

    exit_code = interactive_main(input_fn=fake_input, output_fn=output_lines.append)
    joined_output = "\n".join(output_lines)

    assert exit_code == 0
    assert "Parsed Solution coordinates: RA (05h 12m 00s) DEC (+20° 06' 00\")" in joined_output
    assert "Three solves collected. Run 'compute' to estimate polar error." in joined_output
    assert "Entered live adjustment mode. Use 'verify' after another plate solve." in joined_output
    assert "Remaining error:" in joined_output
    assert "Alignment is within 2 arcmin of the target." in joined_output
    assert "State: VERIFIED" in joined_output
    assert "Solution coordinates: RA (" in joined_output


def test_parse_timestamp_rejects_naive_values() -> None:
    with pytest.raises(ValueError, match="timezone offset"):
        parse_timestamp("2026-03-14T20:15:00")
