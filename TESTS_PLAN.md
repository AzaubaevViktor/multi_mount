# Test Plan

## Назначение

Этот документ фиксирует фактическую текущую раскладку тестов в репозитории. Здесь нет старого roadmap-а: только то, что реально лежит в дереве, что собирает `pytest` по умолчанию и какие пробелы уже видны.

## Что реально запускается по умолчанию

- test runner: `pytest`
- конфигурация: `[tool.pytest.ini_options]` в `pyproject.toml` (бывший `pytest.ini`)
- `pythonpath = src`
- default `testpaths`:
  - `src/tests/hw`
  - `src/tests/units`

`src/tests/units` — 210 тестов, ~4.4 с, железо не нужно: это то, что гоняют CI и
pre-commit. `src/tests/hw` требует подключённой монтировки и в CI не идёт.

## Активные быстрые тесты

### Симулятор и хаос

Ядро быстрого слоя: поддельный порт (`src/sim/`) с виртуальными часами, поэтому
шестидесятисекундный сценарий исполняется за нулевое реальное время.

- `test_sim_fake_serial.py` — семантика поддельного порта: полуоткрытое
  состояние, `PortNotOpenError`, отвал устройства как `OSError(ENXIO)`;
- `test_sim_skywatcher_protocol.py`, `test_sim_tmc_protocol.py` — что
  симуляторы плат отвечают ровно то, что описано в референсе команд;
- `test_sim_integration.py` — драйвер против симулятора целиком;
- `test_sim_usb_unplug.py` — отвал USB посреди сессии сквозь весь транспорт;
- `test_chaos_transport.py` — потеря байтов, частичная запись, воспроизводимость
  по seed;
- `test_chaos_sessions.py` — фаззинг сессий; инварианты вида «остановленная ось
  не считается едущей» и «линия не остаётся полуоткрытой». Здесь же живут 4
  `xfail` — известные дефекты DEC-протокола (см. «Очередь»).

### Регресс-тесты на плохие паттерны из логов

`test_red_p1_voltage_storm.py`, `test_red_p2_usb_unplug.py`,
`test_red_p3_startup_silence.py`, `test_red_p6_sync_keeps_tracking.py`,
`test_red_p7_polling_log_level.py`, `test_red_p8_log_format.py` — по одному на
паттерн из `PLAN.md` §1. Писались красными до фиксов, сейчас зелёные; трогать их
без нужды не стоит, они пиннят ровно то поведение, из-за которого сессии падали.

### Протокол и время

- `test_skywatcher_protocol.py`, `test_tmc2209_response_parsing.py` — разбор
  ответов, включая обрезанные и склеенные;
- `test_physics_clock.py` — виртуальные часы как источник времени;
- `test_serial_line_state.py` — конечный автомат `SerialLine`.

### Прикладной уровень

### `src/tests/units/test_combiner_guide_speed.py`

Проверяет расчёт guide speed в `Combiner`:

- соответствие длительности pulse итоговой скорости;
- знак скоростей для north/south;
- маршрутизацию guide-команд в правильное направление оси.

### `src/tests/units/test_lx200_limits.py`

Проверяет базовые LX200 limit-команды:

- `SET_HIGHEST_ELEVATION`;
- `SET_MINIMUM_ELEVATION`.

### `src/tests/units/test_motor_speed_rounding.py`

Проверяет low-level договорённости двух motor backend'ов:

- квантизацию и округление speed;
- переключение SkyWatcher в highspeed mode;
- отказ на отрицательных скоростях.

### `src/tests/units/test_polar_compensator.py`

Проверяет математику `PolarCompensator`:

- `compute_pole_offset`;
- `compute_guide_speeds`;
- поведение компенсатора при стабильных и нестабильных guide pulse;
- сброс счётчиков и состояния после таймаутов/скачков скоростей.

### Остальное без железа

`test_axis_halt_all.py`, `test_combiner_stop_all.py`, `test_lx200_base_server.py`,
`test_lx200_handler.py`, `test_sky_lx200_presets.py`, `test_manual_control.py`,
`test_polar_align_lib.py`, `test_logging_setup.py`, `test_method_call_chain.py`,
`test_stdout_dashboard.py` — точечные проверки соответствующих модулей;
подробного описания не требуют, но входят в обязательный прогон.

## Активные hardware / integration тесты

### `src/tests/hw/test_1_skywatcher_motor_hw.py`

Низкоуровневые проверки реального `SkyWatcherMotor`:

- установка позиции;
- relative GOTO;
- run mode и направление;
- фактическая скорость;
- ограничения в GOTO mode.

### `src/tests/hw/test_1_tmc2209_motor_hw.py`

Низкоуровневые проверки реального `TMC2209Motor`:

- установка позиции;
- target move;
- run mode;
- достижение целевой скорости;
- ограничения в GOTO mode.

### `src/tests/hw/test_2_axis_ra_hw.py`

Проверяет `AxisRA` на реальном железе:

- set/get позиции;
- tracking и drift;
- ручные east/west движения;
- соответствие моторной скорости запрошенной;
- GOTO и возврат в TRACK.

### `src/tests/hw/test_2_axis_dec_hw.py`

Проверяет `AxisDEC` на реальном железе:

- set/get позиции;
- north/south tracking;
- ручные движения;
- соответствие моторной скорости;
- GOTO и возврат в TRACK.

### `src/tests/hw/test_3_combiner_hw.py`

Проверяет двухосевой `Combiner` напрямую:

- совместную работу RA и DEC;
- общую позицию;
- `set_sky_speed`, `move`, `goto_to`, `halt_*`;
- возврат обеих осей в tracking.

### `src/tests/hw/test_4_polar_compensator.py`

Проверяет поведение `PolarCompensator` на реальной двухосевой системе:

- захват стабильной guide-последовательности;
- takeover после прекращения внешнего guiding;
- обновление компенсации после смены позиции;
- запрет компенсации во время `GOTO` и `SLEW`;
- сброс обратно к sidereal/zero без стабильного guiding.

### `src/tests/hw/test_5_sky_lx200_hw.py`

Проверяет `SkyLX200` поверх реального `Combiner`:

- `handle_alignment`;
- `sync_telescope`;
- чтение координат;
- preset-скорости guide/center/find/max;
- `move_*`, `halt_*`, `guide_*`, `slew_to`.

### `src/tests/hw/test_6_sky_lx200_polar_compensation.py`

Проверяет ту же полярную компенсацию, но уже через LX200 surface:

- replay стабильного guiding;
- изменение компенсации после `sync_telescope`;
- отключение компенсации после нового внешнего guiding.

### `src/tests/hw/test_7_combiner_hw_v2.py`

Крупный end-to-end harness для текущего runtime:

- снимки состояния mount и motors;
- tracking mode допуски;
- `SYNC`, `SLEW`, `GOTO`, `HALT`;
- guide-команды по всем направлениям;
- текущий жизненный цикл polar compensation.

## Архивные тесты

`src/tests/old` удалён. То, что он покрывал, закрыто заново: `sky.physics` —
`test_physics_clock.py`, `serial_wrapper` — `test_serial_line_state.py` и
хаос-набор, guide/splitter — `test_combiner_guide_speed.py`. Непокрытым из
архива не осталось ничего: web-control удалён целиком.

## Рекомендуемый запуск

### Быстрый прогон без железа

```bash
.venv/bin/python -m pytest -q src/tests/units
```

### Полный hardware-прогон

```bash
.venv/bin/python -m pytest -q src/tests/hw
```

### Проверка только коллекции

```bash
.venv/bin/python -m pytest --collect-only -q
```

## Видимые пробелы

- нет marker-based разделения hardware-наборов по типам оборудования;
- нет happy-path и нагрузочного слоя поверх симулятора: сейчас он проверен
  протоколом и хаосом, но не длинной штатной сессией (`PLAN.md` §3 этап 2 и 4);
- нет replay записанной LX200-сессии из `logs/` против симулятора;
- модель TX-кольца прошивки в `src/sim/tmc_sim.py` не прогнана реальным
  паттерном опроса из логов, поэтому «переполнение недостижимо после правок» —
  пока утверждение, а не измерение.

## Выполнено

По этапам `PLAN.md` §3, состояние на 2026-07-25:

| Этап | Состояние | Где лежит |
|---|---|---|
| 1 — протокольные unit-тесты | ✅ | `test_skywatcher_protocol.py`, `test_tmc2209_response_parsing.py`, `test_logging_setup.py` |
| 2 — симулятор + интеграция | 🟡 | `test_sim_*.py`: connect, трекинг, goto в обоих режимах, TMC free-ride. Нет сквозного сценария через LX200-стек и guide-пульса через плату |
| 3а — fault injection | ✅ | `test_red_p1/p2/p3`, `test_sim_usb_unplug.py`. Не покрыт таймаут «TMC молчит вместо `ready`» |
| 3б — хаос-инжиниринг | ✅ | `src/sim/chaos.py`, `test_chaos_transport.py`, `test_chaos_sessions.py` |
| 4 — нагрузка и инфраструктура | ⬜ | не начат |

Регресс-гварды на паттерны П1, П2, П3, П6, П7, П8 — все зелёные (были красными
до фиксов). Прогон: 206 passed, 4 xfailed за ~4.4 с.

Hardware-набор не пострадал: собирается (302 теста), виртуальные часы —
opt-in фикстура, по умолчанию везде реальное время.

## Что железо добавило в план (2026-07-25)

Обе платы разобраны на живом железе (`docs/protocol/`). Симулятор строился по
логам, то есть по реконструкции; теперь есть эталон, и с ним надо сверяться.
Режимы отказа, которые симулятор обязан воспроизводить, но пока не умеет:

- **обрезка TX-кольца DEC** на 255 байтах без терминатора, со склейкой со
  следующим ответом; обрыв по границе поля даёт валидный неполный кадр;
- **потеря команд в RX-буфере DEC** (64 байта), пока плата занята 73 мс в
  `full_status`, плюс огрызок, портящий следующую команду;
- **потеря второй команды**, отправленной одной записью без паузы, — на обеих
  платах;
- **кламп периода RA** на `период_1×/100`: драйвер считает, что едет на 800×,
  железо едет в 11 раз медленнее;
- **сброс флага инициализации** от `:E`, отправленной во время вращения.

Отдельно: типы + mypy сделаны (191 ошибка → 0), mypy стоит в CI и pre-commit.

## Очередь

Тестовая часть общей очереди (полная — в `PLAN.md` §7):

1. Этап 2 до конца: сквозной happy path через LX200-стек, guide-пульс через
   симулятор; этап 4 целиком — нагрузка и replay сессии из
   `logs/2026-03-26_01-50-54`.
2. Таймаут «TMC не прислал `ready`» отдельным тестом (п. 10 этапа 3а).
3. Верификация прошивки: реальный паттерн опроса против модели TX-кольца.
4. 4 `xfail` в `test_chaos_sessions.py` — дефекты DEC-протокола: нет
   контрольной суммы и длины (потеря цифры *внутри* значения проходит все
   проверки), `set_speed` не сверяет эхо, наружу течёт сырой
   `KeyError`/`ValueError`, повторная `START_MOTION` продлевает цель GOTO.
   Закрываются изменением протокола, а не заплаткой на хосте.
5. mypy: типов пока нет, каркас в `pyproject.toml` закомментирован.

## Итог

Быстрый слой стал основным: 210 тестов без железа, включая симулятор монтировки
с виртуальным временем и хаос-инжиниринг транспорта. Hardware-набор по-прежнему
единственный источник правды о реальном поведении моторов, полярной компенсации
и связки `Combiner` + `SkyLX200`, но ловить протокольные и транспортные дефекты
он больше не обязан.
