# Target Tree

The rebuilt repository should contain at least the following functional layout:

```text
src/
  __main__.py
  logging_setup.py
  serial_wrapper/
    wrapper.py
  lx200/
    base.py
    base_server.py
    protocol.py
  sky/
    __init__.py
    axis.py
    combiner.py
    constants.py
    lx200.py
    motor.py
    physics.py
    polar_compensator.py
  skywatcher/
    motor.py
    protocol.py
  tmc2209/
    __init__.py
    motor.py
  utils/
    method_call_chain.py
  web_control/
    web.py
    static/
      index.html
      app.js
      app.css

telescope_dec/
  src/
    main.cpp

src/tests/
  hw/
  units/
  base/

README.md
ARCHITECTURE.md
TESTS_PLAN.md
pytest.ini
```

Core reconstruction responsibilities:

- `serial_wrapper/wrapper.py`: generic serial transport with search, connect, reset, query, bulk read, close.
- `sky/motor.py`: abstract motor contract used by Axis.
- `skywatcher/motor.py`: RA backend compatible with SkyWatcher controller protocol.
- `tmc2209/motor.py`: DEC backend compatible with Arduino firmware protocol.
- `sky/physics.py`: typed coordinate and speed arithmetic.
- `sky/axis.py`: one-axis state machine and tracking compensation.
- `sky/combiner.py`: two-axis mount composition and guide-speed routing.
- `sky/polar_compensator.py`: polar error estimation and guide takeover.
- `lx200/base.py`: LX200 command parsing and domain dispatch.
- `lx200/base_server.py`: TCP framing and client serving.
- `sky/lx200.py`: adapter from LX200 to Combiner.
- `web_control/web.py`: standalone monitor server and static UI.
- `src/__main__.py`: runtime wiring.
- `telescope_dec/src/main.cpp`: DEC firmware implementing the host-facing serial protocol.

The rebuilt code does not need to match the original line-for-line. It should match behavior, boundaries, and acceptance expectations.
