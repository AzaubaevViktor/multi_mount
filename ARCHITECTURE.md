# Multi-Mount Architecture

## Назначение

Проект собирает гибридную монтировку из двух разных физических осей и публикует наружу один LX200-совместимый endpoint:

- RA управляется через SynScan/SkyWatcher backend.
- DEC управляется через самодельный контроллер на Arduino + TMC2209.
- клиент (`INDI`, `KStars`, `Ekos`) видит это как одну монтировку.

Текущая архитектура уже реализована и строится вокруг композиции `LX200SimpleServer -> LX200Splitter -> RA/DEC axis handlers -> hardware adapters`.

## Верхнеуровневая схема

```text
LX200 client
  -> LX200SimpleServer
    -> LX200Handler.handle()
      -> LX200Splitter
        -> SkyWatcherLX200 (RA)
          -> SkyWatcherMount
            -> SerialLine
        -> TMC2209LX200 (DEC)
          -> TMC2209Adapter
            -> SerialLine
```

Отдельно от LX200-стека существует HTTP monitor:

```text
MonitorMixin objects
  -> MonitorRegistry
    -> MonitorRequestHandler / ThreadingHTTPServer
      -> SSE + static UI
```

## Основные модули

### `src/__main__.py`

Точка сборки production-конфигурации:

- ищет serial-устройства;
- создаёт `SkyWatcherMount` и `TMC2209Adapter`;
- оборачивает их в `SkyWatcherLX200` и `TMC2209LX200`;
- соединяет обе оси через `LX200Splitter`;
- запускает `LX200SimpleServer`.

### `src/lx200/base_server.py`

`LX200SimpleServer` поднимает TCP-сервер и работает на уровне байтового протокола LX200:

- принимает поток команд `:<cmd>#`;
- отдельно обрабатывает alignment query (`0x06`);
- декодирует команду и передаёт её в `lx200.handle(...)`;
- сериализует ответ обратно в LX200-формат.

Сервер не содержит логики движения и не знает о конкретных типах mount/backend.

### `src/lx200/base.py`

Главный протокольный и доменный слой.

Содержит:

- `LX200Commands` с поддерживаемыми командами LX200;
- базовый контракт `LX200Base`;
- общий `LX200Handler`, который:
  - хранит target RA/DEC;
  - парсит команды;
  - маршрутизирует их в методы управления;
  - ведёт список активных manual-direction halt-команд;
- обобщённый `LX200AxisHandler`, который реализует общую механику оси.

`LX200AxisHandler` является фактическим общим ядром для RA/DEC и отвечает за:

- хранение mount-позиции и motor-позиции;
- хранение текущей sky tracking speed;
- очередь `AxisCommand`;
- фоновую компенсацию позиции по фактическому движению мотора;
- периодический telemetry logging;
- обработку `set_tracking_rate`, `halt_motion`, `halt_direction`;
- пересчёт guide-rate из ширины guide pulse.

Это не чистая state machine из старого документа. В текущем коде поведение строится вокруг:

- текущей sky-rate;
- очереди команд оси;
- background thread, который либо применяет команды, либо периодически компенсирует координату по данным мотора.

### `src/lx200/splitter.py`

`LX200Splitter` композиционно объединяет две независимые оси под одним интерфейсом `LX200Handler`.

Ответственность:

- RA-команды делегируются `SkyWatcherLX200`;
- DEC-команды делегируются `TMC2209LX200`;
- общие команды (`connect`, `halt_all`, `set_slew_to_find`, `get_distance`) агрегируются;
- координируется polar compensation через `PolarCompensator`.

`PolarCompensator`:

- принимает внешние guide-rate изменения;
- ждёт стабилизации;
- оценивает polar misalignment;
- периодически пересчитывает corrective guide rates;
- возвращает поправки обратно в обе оси через `set_tracking_rate(..., update_sky_rate=True)`.

То есть guide-пульсы в текущей архитектуре используются не только как разовое управление, но и как источник оценки ошибки полярного выравнивания.

### `src/skywatcher/skywatcher_lx200.py`

`SkyWatcherLX200` реализует RA-ось поверх `SkyWatcherMount`.

Особенности:

- наследуется от `LX200RAHandler`;
- использует backend mount для tracking/manual slew;
- GOTO реализован через отдельный поток `_check_goto`;
- поток контролирует достижение цели, останавливает goto и отслеживает overshoot;
- текущая mount-координата RA хранится отдельно от raw-положения мотора и поддерживается общим механизмом компенсации из `LX200AxisHandler`.

Фактически RA backend умеет:

- подключение и запуск tracking;
- чтение статуса и положения;
- ручное движение east/west;
- goto на delta-based SynScan API;
- остановку движения и возврат к tracking.

### `src/tmc2209/tmc2209_lx200.py`

`TMC2209LX200` реализует DEC-ось поверх `TMC2209Adapter`.

Особенности:

- наследуется от `LX200DECHandler`;
- использует профили движения (`guide`, `slew`, `goto_fast`, `goto_slow`);
- сам управляет `microsteps`, `speed`, `accel`, `direction`, `run/halt`;
- tracking DEC реализован как signed speed через guide-profile;
- goto DEC запускается сразу через target mode адаптера;
- manual movement поддерживается только по направлениям north/south.

В отличие от RA backend, DEC backend ближе к low-level моторному контролю и сам выбирает speed-profile.

### `src/skywatcher/skywatcher.py`

Низкоуровневый драйвер SynScan/SkyWatcher mount:

- serial-протокол к mount;
- чтение статуса;
- tracking;
- slew;
- работа с реальным положением RA.

### `src/tmc2209/tmc2209_adapter.py`

Низкоуровневый драйвер DEC-контроллера:

- line-based serial protocol к Arduino;
- статусы `phase/mode/position/...`;
- команды `run`, `halt`, `set_speed`, `set_acceleration`, `set_position`, `slew_delta`, `set_target_mode`, `set_free_ride_mode`;
- конвертации между steps, speed in steps/s и DEC axis units.

### `src/serial_wrapper/wrapper.py`

Общий serial transport:

- поиск устройства;
- line-based transactions;
- timeout/retry boundary для hardware adapters.

### `src/sky/physics.py` и `src/lx200/protocols.py`

Доменные типы и преобразования:

- `Ha`, `Dec`, `AxisPos`;
- axis speed types;
- арифметика координат, wrapping, parsing/formatting;
- это базовый словарь типов, на котором строятся `lx200`, `skywatcher` и `tmc2209`.

### `src/web_control/web.py`

Независимый от LX200 стек веб-мониторинга.

Состав:

- `MonitorField`, `MonitorAction`, `MonitorGroup`;
- `MonitorMixin` для декларативного описания monitor surface;
- `MonitorRegistry` с polling diff loop;
- `MonitorRequestHandler` на базе standard library HTTP server;
- SSE endpoint `/events`;
- static UI в `src/web_control/static`.

Этот слой пока не встроен в production startup из `src/__main__.py`, но покрыт отдельными тестами и предназначен для live inspection / manual control.

### `telescope_dec/src/main.cpp`

Прошивка Arduino для DEC-контроллера:

- исполняет текстовый serial protocol;
- управляет TMC2209;
- предоставляет Python-адаптеру минимальный транспортный API для статуса и движения.

## Поток команд LX200

### 1. Подключение клиента

1. `LX200SimpleServer.serve_forever()` запускает TCP listener.
2. При старте сервер вызывает `splitter.connect()`.
3. `LX200Splitter.connect()` подключает RA и DEC оси и запускает `PolarCompensator`.

### 2. Обработка команды

1. Клиент отправляет `:<cmd>#`.
2. `LX200SimpleServer` выделяет команду из byte stream.
3. `LX200Handler.handle()` превращает первые два символа в `LX200Commands`.
4. `LX200Handler._do_handle()` вызывает метод доменного уровня.
5. Конкретная реализация (`LX200Splitter`, `SkyWatcherLX200`, `TMC2209LX200`) выполняет действие.
6. Ответ сериализуется назад в LX200-формат.

### 3. Команды координат

- `Sr` / `Sd` только записывают target RA/DEC во внутреннее состояние `LX200Handler`.
- `CM` вызывает sync обеих осей через сохранённые target values.
- `MS` запускает slew/goto по обеим осям, но в текущем коде response path для `MS` ещё не доведён до корректного LX200 результата и возвращает `False`.

Это важный факт текущей реализации: архитектура уже поддерживает slew/goto, но протокольный ответ на `MS` остаётся упрощённым.

## Модель оси в текущем коде

Вместо единой формализованной state machine текущая ось хранит:

- `_mount_position_raw` — логическая mount-координата;
- `_motor_position_raw` — последнее известное положение мотора;
- `_sky_track_rate` — текущая целевая скорость оси в sky units;
- `_axis_command_queue` — очередь команд изменения режима;
- `_last_update_s` — время последней синхронизации;
- backend-specific runtime (`_goto_to`, speed profile, hardware status).

Движение оси состоит из двух параллельных процессов:

- command path:
  - `set_tracking_rate`, `halt_motion`, `halt_direction` кладутся в очередь;
  - `_compensate_tracking_rate()` вычитывает их и применяет через `_set_tracking_speed()` / `_halt_motion()`;
- compensation path:
  - при отсутствии команд поток читает фактическое motor position;
  - сравнивает ожидаемое смещение с реальным;
  - корректирует логическую mount position.

Это даёт два уровня координат:

- motor position: что реально сообщает железо;
- mount position: что видит LX200-клиент после компенсации tracking/guiding/manual motion.

## RA и DEC различаются по ответственности

### RA

- tracking включён по умолчанию;
- guide меняет tracking rate вокруг sidereal speed;
- manual movement east/west идёт через SkyWatcher mount;
- goto управляется фоновым supervisory thread.

### DEC

- базовый tracking rate обычно `0`;
- guide и tracking используют signed DEC speed;
- manual movement north/south идёт напрямую через TMC2209;
- goto использует target mode и speed profiles;
- low-level профиль выбирается в Python слое `TMC2209LX200`.

## Параллелизм

В проекте несколько независимых фоновых потоков:

- `LX200SimpleServer`: отдельный thread на клиента;
- каждый `LX200AxisHandler`:
  - telemetry thread;
  - compensate thread;
- `SkyWatcherLX200`:
  - goto supervision thread;
- `PolarCompensator`:
  - correction thread;
- `MonitorRegistry`:
  - polling diff thread;
- `ThreadingHTTPServer`:
  - отдельные HTTP request threads.

Синхронизация в основном строится на:

- `threading.RLock` для позиции оси;
- `queue.Queue` для axis commands;
- `threading.Event` внутри `PolarCompensator`.

## Тестовое покрытие

Автотесты в `src/tests` покрывают:

- координатную математику и parsing;
- logic guide/polar compensation;
- splitter behavior;
- serial wrapper;
- SkyWatcher math/status conversions;
- TMC2209 adapter и `TMC2209LX200`;
- web monitor registry и example monitor.

Есть и hardware-oriented тесты, которые проверяют интеграцию с реальными устройствами.

## Ограничения и известные архитектурные долги

- `ARCHITECTURE.md` раньше описывал целевую, но не фактическую layered state-machine архитектуру; теперь документ отражает текущий код.
- `LX200Handler._do_handle()` частично смешивает protocol parsing и command orchestration.
- Ответ на `MS` пока не оформлен как полноценный LX200 status/result.
- Общий status model для RA/DEC backend'ов ещё не унифицирован.
- В `LX200AxisHandler` уже есть TODO о выносе моторного интерфейса и унификации статусов.
- Web monitor существует отдельно от основного server startup.
- Полярная компенсация встроена в splitter и тесно связана с тем, как guide-команды меняют tracking rate.

## Направление дальнейшей эволюции

Если развивать текущую архитектуру без полного переписывания, естественные шаги такие:

- унифицировать hardware status contract для SkyWatcher и TMC2209;
- отделить LX200 protocol layer от orchestration layer;
- формализовать axis state model поверх уже существующей command queue;
- довести `SLEW` response semantics до корректного LX200 поведения;
- встроить web monitor в основной runtime как штатный диагностический интерфейс.
