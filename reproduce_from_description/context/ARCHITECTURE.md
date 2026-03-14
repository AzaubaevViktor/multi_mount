# Multi-Mount Architecture

## Назначение

Проект собирает гибридную монтировку из двух физических осей и публикует наружу один LX200-совместимый endpoint:

- RA управляется через SynScan/SkyWatcher backend;
- DEC управляется через самодельный контроллер на Arduino + TMC2209;
- клиент (`INDI`, `KStars`, `Ekos`) видит это как одну монтировку.

Текущая архитектура построена вокруг связки `LX200SimpleServer -> SkyLX200 -> Combiner -> AxisRA/AxisDEC -> Motor backends`.

## Верхнеуровневая схема

```text
LX200 client
  -> LX200SimpleServer
    -> LX200Handler.handle()
      -> SkyLX200
        -> Combiner
          -> AxisRA
            -> SkyWatcherMotor
              -> SerialLine
          -> AxisDEC
            -> TMC2209Motor
              -> SerialLine
          -> PolarCompensator
```

Отдельно существует HTTP monitor infrastructure:

```text
MonitorMixin objects
  -> MonitorRegistry
    -> MonitorRequestHandler / ThreadingHTTPServer
      -> SSE + static UI
```

В `src/__main__.py` web server сейчас запускается отдельно и с пустым registry, то есть это заготовка инфраструктуры, а не полноценный production monitor.

## Основные модули

### `src/__main__.py`

Точка сборки текущего runtime:

- ищет serial-устройства;
- создаёт `SkyWatcherMotor` и `TMC2209Motor`;
- оборачивает их в `AxisRA` и `AxisDEC`;
- соединяет оси через `Combiner`;
- создаёт `SkyLX200`;
- поднимает `LX200SimpleServer`;
- параллельно стартует `MonitorServer`, но без зарегистрированных monitor objects.

### `src/lx200/base_server.py`

`LX200SimpleServer` поднимает TCP-сервер и работает на уровне байтового протокола LX200:

- принимает поток команд `:<cmd>#`;
- отдельно обрабатывает alignment query (`0x06`);
- декодирует команду и передаёт её в `lx200.handle(...)`;
- сериализует ответ обратно в LX200-формат.

Сервер не знает ничего о конкретной механике RA/DEC, но пока ещё допускает несколько одновременных клиентов на одном handler instance.

### `src/lx200/base.py`

Протокольный и доменный слой LX200.

Содержит:

- `LX200Commands` с поддерживаемыми командами;
- базовый контракт `LX200Base`;
- `LX200Handler`, который:
  - хранит target RA/DEC;
  - парсит LX200-команды;
  - маршрутизирует команды в доменные методы;
  - ведёт список активных manual-move направлений для `Qe/Qw/Qn/Qs`.

Этот файл больше не содержит отдельного axis-specific handler слоя: общая логика оси живёт в `src/sky/axis.py`.

### `src/sky/lx200.py`

`SkyLX200` является адаптером между LX200-командами и двухосевой моделью `Combiner`.

Он:

- проксирует `connect()` / `stop()` в `Combiner`;
- читает и синхронизирует координаты;
- запускает `goto_to(...)`;
- переключает preset-скорости guide/center/find/max;
- маршрутизирует ручные движения и guide pulse-команды по направлениям.

### `src/sky/combiner.py`

`Combiner` связывает RA и DEC оси в одну логическую монтировку.

Ответственность:

- объединять `AxisRA` и `AxisDEC`;
- отдавать общую позицию `PointCoordinates`;
- запускать `goto_to`, `move`, `halt_*`, `set_sky_speed`;
- пересчитывать guide speed по длительности pulse;
- координировать `PolarCompensator`.

`PolarCompensator` в текущем коде не просто хранит смещение полюса, а использует повторяющиеся guide-команды как вход для оценки ошибки полярного выравнивания и затем возвращает corrective speeds обратно в `Combiner.set_sky_speed(...)`.

### `src/sky/axis.py`

Здесь живёт фактическое общее ядро RA/DEC.

`Axis`:

- хранит логическую позицию mount;
- хранит текущую целевую sky-speed;
- держит очередь `AxisCommand`;
- запускает фоновый `_motion_convertor` thread;
- переводит доменные команды (`change_speed`, `move`, `goto_to`, `halt_*`) в операции мотора;
- периодически компенсирует логическую координату по фактическому смещению мотора.

Есть две конкретные реализации:

- `AxisRA` поверх `Ha` / `HaPerSecond`;
- `AxisDEC` поверх `Dec` / `DecPerSecond`.

Текущая модель оси не оформлена как отдельная state machine таблица. Поведение строится вокруг:

- текущего `AxisMotionMode`;
- очереди команд;
- блокировки мотора;
- фоновой компенсации позиции при отсутствии новых команд.

### `src/skywatcher/motor.py`

Низкоуровневый драйвер SkyWatcher/SynScan RA-мотора.

Он отвечает за:

- serial-команды mount controller;
- чтение статуса и позиции;
- run-mode и goto-mode;
- конвертацию между шагами и `Ha`;
- выбор low-speed / high-speed режимов.

### `src/tmc2209/motor.py`

Низкоуровневый Python backend для DEC-контроллера на Arduino.

Он отвечает за:

- line-based serial protocol;
- парсинг статуса `phase/mode/position/...`;
- команды `position`, `speed`, `acceleration`, `direction`, `delta`, `run`, `stop`, `mode`, `set`;
- конвертацию между шагами и `Dec`.

### `src/serial_wrapper/wrapper.py`

Общий serial transport:

- поиск устройства по шаблону;
- line-based запросы с timeout;
- reconnect/reset helper-методы для железа.

### `src/sky/physics.py`

Доменные типы и арифметика:

- `Ha`, `Dec`, `AxisPos`;
- `HaPerSecond`, `DecPerSecond`, `Second`;
- parsing / formatting / wrapping;
- инварианты для координат и скоростей.

### `src/web_control/web.py`

Независимый стек веб-мониторинга:

- `MonitorField`, `MonitorAction`, `MonitorGroup`;
- `MonitorMixin` для декларативного описания monitor surface;
- `MonitorRegistry` с polling diff loop;
- `MonitorRequestHandler` и `MonitorServer`;
- SSE endpoint `/events`;
- static UI в `src/web_control/static`.

Этот слой существует отдельно от LX200 runtime и пока не подключён к живым объектам в `src/__main__.py`.

### `telescope_dec/src/main.cpp`

Прошивка Arduino для DEC-контроллера:

- управляет TMC2209;
- исполняет line-based serial protocol;
- экспортирует статус, конфигурацию и команды движения для `TMC2209Motor`.

## Поток команды LX200

### 1. Подключение клиента

1. `LX200SimpleServer.serve_forever()` запускает TCP listener.
2. Перед входом в accept loop сервер вызывает `sky_lx200.connect()`.
3. `SkyLX200.connect()` делегирует подключение в `Combiner.connect()`.
4. `Combiner.connect()` подключает обе оси и запускает поток полярной компенсации.

### 2. Обработка команды

1. Клиент отправляет `:<cmd>#`.
2. `LX200SimpleServer` выделяет команду из byte stream.
3. `LX200Handler.handle()` превращает первые символы в `LX200Commands`.
4. `LX200Handler._do_handle()` вызывает доменный метод.
5. `SkyLX200` проксирует вызов в `Combiner`, а тот уже в нужную ось или в обе оси сразу.
6. Ответ сериализуется назад в LX200-формат.

### 3. Координатные команды

- `Sr` / `Sd` только обновляют target RA/DEC внутри `LX200Handler`.
- `CM` вызывает `sync_telescope(...)` с сохранёнными target values.
- `MS` запускает `goto_to(...)`, но response path для `MS` пока остаётся упрощённым и возвращает `False`.

## Модель оси в текущем коде

Каждая ось хранит:

- логическую mount-позицию (`_ra_position` / `_dec_position`);
- последнюю известную позицию мотора;
- текущую sky-speed;
- очередь команд;
- текущий `AxisMotionMode`;
- служебное состояние для manual move и GOTO.

Движение оси состоит из двух переплетённых путей:

- command path:
  - публичные методы кладут `AxisCommand` в очередь;
  - `_motion_convertor` вычитывает команду и выполняет нужную моторную операцию;
- compensation path:
  - когда очередь пуста, `_motion_convertor` читает положение мотора;
  - сравнивает ожидаемое и фактическое смещение;
  - корректирует логическую mount-позицию.

Именно поэтому клиент видит не raw encoder state, а логическую координату после компенсации tracking / guide / halt.

## RA и DEC различаются по hardware semantics

### RA

- tracking по умолчанию построен вокруг sidereal speed;
- ручное движение east/west меняет скорость относительно tracking;
- SkyWatcher backend сам хранит часть motion state mount'а;
- goto выполняется через delta-based команды mount controller.

### DEC

- базовый tracking rate обычно `0`;
- speed является signed относительно north/south;
- backend напрямую управляет `direction`, `speed`, `acceleration`, `mode`, `run`;
- goto использует target mode контроллера.

## Параллелизм

В проекте несколько фоновых потоков:

- `LX200SimpleServer`: отдельный thread на клиента;
- `AxisRA` и `AxisDEC`: по одному `_motion_convertor` thread на ось;
- `Combiner`: thread полярной компенсации;
- `MonitorRegistry`: polling diff thread;
- `ThreadingHTTPServer`: отдельные HTTP request threads.

Синхронизация строится в основном на:

- `threading.RLock` вокруг операций мотора;
- `queue.Queue` для команд оси;
- `threading.Event` внутри `Combiner` и `PolarCompensator`.

## Тестовое покрытие

По умолчанию `pytest` собирает только:

- `src/tests/hw` для hardware/integration сценариев;
- `src/tests/units` для быстрых проверок без железа.

`src/tests/old` содержит архивные тесты и больше не участвует в обычном прогоне.

## Ограничения и текущие долги

- `LX200Handler._do_handle()` всё ещё смешивает parsing и command orchestration.
- Ответ на `MS` пока не оформлен как полноценный LX200 status/result.
- Общий status contract для SkyWatcher и TMC2209 ещё не унифицирован.
- При переходе DEC через полюс RA пока не зеркалится на `+12h`.
- Web monitor пока не связан с живыми runtime object instances в `src/__main__.py`.
