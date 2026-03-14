# Reconstructed Test Plan

This reconstruction keeps the two-level test strategy described in the package:

- `src/tests/units/` covers fast contracts:
  - serial transport search and prefix-aware reads;
  - SkyWatcher/TMC2209 speed contract details;
  - guide-speed routing in `Combiner`;
  - LX200 limit handling and TCP framing;
  - `PolarCompensator` stability and takeover math;
  - monitor snapshot generation.
- `src/tests/hw/` keeps the acceptance layout for real devices:
  - low-level motor behavior for SkyWatcher and TMC2209;
  - axis behavior for RA and DEC;
  - combined mount behavior;
  - polar-compensation behavior;
  - LX200 end-to-end behavior.

Recommended runs:

```bash
PYTHONPYCACHEPREFIX=/tmp/reproduce_pycache .venv/bin/python -m pytest -c reproduce_from_description/pytest.ini -q reproduce_from_description/src/tests/units
```

```bash
REPRODUCE_HW=1 PYTHONPYCACHEPREFIX=/tmp/reproduce_pycache .venv/bin/python -m pytest -c reproduce_from_description/pytest.ini -q reproduce_from_description/src/tests/hw
```
