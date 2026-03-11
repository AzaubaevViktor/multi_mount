# Test Plan

## Назначение

Этот документ фиксирует текущий тестовый план проекта по уже существующим тестам в `src/tests`.
Это не roadmap новых тестов, а сводка того, что реально проверяется сейчас, как запускать наборы и где остаются видимые пробелы.

## Как устроены тесты

- Test runner: `pytest`
- Основной путь: `src/tests`
- Конфигурация: `pytest.ini`
- `pythonpath = src`, поэтому тесты импортируют runtime-модули напрямую
- В наборе есть два типа проверок:
  - быстрые unit/math/protocol тесты без железа
  - hardware/integration тесты, завязанные на реальные устройства и последовательные порты

## Матрица покрытия

### 1. Координаты, типы и астроматематика

#### `src/tests/test_ra_dec_containers.py`

Проверяет контейнеры RA/DEC:

- парсинг валидных строковых представлений RA и DEC
- отбрасывание невалидных форматов
- конвертацию из секунд, часов, градусов и угловых секунд
- округление строкового представления на границах
- roundtrip-переходы между внутренним числом и текстом

#### `src/tests/test_sky_physics.py`

Проверяет базовую физику и типы из `sky.physics`:

- sidereal/stellar speed
- нормализацию и wrap для `Ha`
- signed shortest delta для движения по RA
- арифметику `Ha`, `Dec`, `HaPerSecond`, `DecPerSecond`
- negation и type preservation
- ошибки при неподдерживаемых операциях

#### `src/tests/test_sky_physics_test_model.py`

Проверяет тестовую модель физики/координат:

- согласованность преобразований в модельных сценариях
- roundtrip и инварианты для вспомогательной физической логики

#### `src/tests/test_guide_compensator.py`

Проверяет полярную компенсацию:

- модельную реакцию компенсатора на polar misalignment для набора параметров

### 2. SkyWatcher RA axis

#### `src/tests/test_skywatcher_revu24.py`

Проверяет формат `revu24`:

- кодирование/декодирование значений
- roundtrip случайных значений
- обработку невалидных входов

#### `src/tests/test_skywatcher_status_roundtrip.py`

Проверяет сериализацию статуса SkyWatcher:

- преобразование status bytes в команды
- обратное кодирование команд движения
- игнорирование runtime-флагов, не влияющих на motion-status

#### `src/tests/test_skywatcher_goto_math.py`

Проверяет математику GOTO для SkyWatcher:

- вычисления, связанные с выбором направления и параметров GOTO

#### `src/tests/test_skywatcher_set_ra_rate.py`

Проверяет управление RA rate:

- остановку мотора при нулевой скорости
- запуск мотора при смене rate из idle
- запрет лишнего старта в `GOTO`-режиме
- остановку оси перед сменой motion mode

#### `src/tests/test_skywatcher.py`

Hardware-oriented тесты для реального SkyWatcher mount:

- roundtrip установки/чтения HA
- `slew_delta` в обе стороны и на разных дистанциях
- запрет ручного движения во время GOTO
- корректную скорость и направление при `move_ra`
- корректную скорость и направление при tracking
- остановку мотора при нулевом tracking rate

### 3. TMC2209 DEC axis

#### `src/tests/test_tmc2209_adapter.py`

Проверяет Python adapter для DEC-контроллера:

- парсинг строковых ответов протокола
- обработку ошибок и невалидных префиксов
- нормализацию ключей параметров
- форматирование bool/value команд
- диапазоны speed и acceleration
- преобразование DEC в шаги мотора
- обработку некорректных microsteps и `steps_per_rev`

#### `src/tests/test_tmc2209_lx200.py`

Проверяет LX200-обвязку над TMC2209:

- знак tracking rate
- соответствие между rate и направлением north/south

#### `src/tests/test_tmc2209_adapter_hw.py`

Hardware-oriented тесты для реального TMC2209 adapter:

- установку позиции
- relative move через `slew_delta` в обе стороны
- special-case для нулевого delta
- run speed/direction
- stop во время continuous run
- stop во время target move

### 4. Комбинированная логика осей и splitter

#### `src/tests/test_axis_guide_logic.py`

Проверяет комбинированную guide-логику:

- противоположные rate/sign для east-west guide
- противоположные rate/sign для north-south guide
- обновление current/sky tracking rate
- немедленное применение команд без ожидания compensate interval
- независимость RA/DEC команд в splitter-сценариях

#### `src/tests/test_x_splitter_hw_v2.py`

Крупный hardware/integration набор для `LX200Splitter`:

- `SYNC` обновляет координаты mount
- mount стартует в tracking mode по умолчанию
- направления `SLEW` в разных системах координат
- `GOTO` переводит mount к целевой точке
- `HALT` возвращает систему из `SLEW` и `GOTO` обратно в tracking
- RA guiding rates для разных направлений
- поведение polar compensator:
  - остается выключенным после несогласованных guide-команд
  - входит в guiding mode после стабильных guide-последовательностей
  - сбрасывается после смены guide-паттерна

### 5. LX200/web/utility

#### `src/tests/test_serial_line_search.py`

Проверяет поиск serial line:

- успешный поиск по шаблону
- ситуацию not found
- невалидный pattern
- ошибку при передаче директории
- требование обязательного pattern

#### `src/tests/test_web_control.py`

Проверяет web control example/registry:

- структуру monitor/action API
- обновление registry после записи в field

## Текущий охват по уровням

### Unit / fast

Стабильно покрыты:

- парсинг и форматирование координат
- численная астроматематика и скорость слежения
- кодирование/декодирование протоколов
- преобразования статусов и motion modes
- логика rate/sign/direction
- часть splitter/guide-логики на моках и тестовых double-объектах

### Hardware / integration

Покрыты реальные сценарии для:

- SkyWatcher RA mount
- TMC2209 DEC controller
- end-to-end splitter с командами `SYNC`, `SLEW`, `GOTO`, `HALT`, `GUIDE`

Эти тесты полезны как регрессия реального поведения, но зависят от:

- подключенного железа
- корректных serial device names
- стабильного физического состояния mount/motor
- допусков по времени и скорости

## Рекомендуемый запуск

### Быстрый локальный прогон без железа

```bash
pytest -q \
  src/tests/test_ra_dec_containers.py \
  src/tests/test_sky_physics.py \
  src/tests/test_sky_physics_test_model.py \
  src/tests/test_guide_compensator.py \
  src/tests/test_serial_line_search.py \
  src/tests/test_tmc2209_adapter.py \
  src/tests/test_tmc2209_lx200.py \
  src/tests/test_axis_guide_logic.py \
  src/tests/test_skywatcher_revu24.py \
  src/tests/test_skywatcher_status_roundtrip.py \
  src/tests/test_skywatcher_goto_math.py \
  src/tests/test_skywatcher_set_ra_rate.py \
  src/tests/test_web_control.py
```

### Прогон hardware-наборов

```bash
pytest -q \
  src/tests/test_skywatcher.py \
  src/tests/test_tmc2209_adapter_hw.py \
  src/tests/test_x_splitter_hw_v2.py
```

## Видимые пробелы

По текущему репозиторию уже видно, что не хватает или покрыто слабо:

- явного разделения hardware-тестов через `pytest` markers
- единого smoke-набора, который всегда безопасно гонять в CI
- отдельных unit-тестов для `lx200.protocol` и `lx200.base/base_server`
- более полного покрытия `web_control/web.py`, а не только example/registry
- edge-case сценариев, которые уже перечислены в `README.md`:
  - goto между разными удаленными точками `A -> B`
  - slews из крайних положений
  - длинные цепочки `SYNC -> GOTO/SLEW -> HALT -> verify`
  - дополнительные комбинации guide/slew/goto состояний

## Итог

Существующий тестовый набор уже хорошо покрывает:

- математику координат и скоростей
- RA/DEC протоколы и адаптеры
- критическую guide/tracking логику
- реальные end-to-end сценарии на железе

Основной следующий шаг для зрелости набора не в добавлении еще одного общего плана, а в разделении быстрых и hardware-тестов на уровне маркеров/CI-профилей.
