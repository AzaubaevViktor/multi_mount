# Test Plan

## Назначение

Этот документ фиксирует фактическую текущую раскладку тестов в репозитории. Здесь нет старого roadmap-а: только то, что реально лежит в дереве, что собирает `pytest` по умолчанию и какие пробелы уже видны.

## Что реально запускается по умолчанию

- test runner: `pytest`
- конфигурация: `pytest.ini`
- `pythonpath = src`
- default `testpaths`:
  - `src/tests/hw`
  - `src/tests/units`

Это важно: каталог `src/tests/old` не участвует в обычном прогоне и служит архивом прежних тестов/экспериментов.

## Активные быстрые тесты

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

`src/tests/old` содержит тесты, которые раньше покрывали:

- `sky.physics` и контейнеры координат;
- `serial_wrapper`;
- старые web-control проверки;
- раннюю guide/splitter логику.

Они полезны как reference material, но по текущему `pytest.ini` не запускаются.

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

По текущему дереву видно несколько слабых мест:

- активный fast-suite почти не покрывает `sky.physics`, `serial_wrapper` и `web_control`;
- часть исторических unit-тестов живёт только в `src/tests/old`;
- нет marker-based разделения hardware-наборов по типам оборудования;
- нет отдельного CI-safe smoke profile поверх активных тестов.

## Итог

Сейчас проект лучше всего покрыт на уровнях:

- реального поведения моторов и осей на железе;
- полярной компенсации;
- интеграции `Combiner` и `SkyLX200`.

Быстрый unit-layer есть, но он значительно уже исторического набора и уже не покрывает несколько вспомогательных модулей, которые остаются только в архиве.
