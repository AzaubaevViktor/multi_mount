# Multi-Mount Architecture

## Цель

Описать целевую архитектуру по слоям для гибридной монтировки (RA + DEC) и формализовать поведение осевого контроллера `L3` через state machine и вызовы `L4`.

---

## Слои

### L1. LX200 Server + Protocol Endpoint

Зона ответственности:
- TCP-сервер, парсинг входящих LX200-команд, формирование ответов.
- Никакой бизнес-логики движения.

Должен уметь:
- Принять `:<cmd>#`.
- Передать вызов в `L2`.
- Вернуть ответ в формате LX200.

Не должен уметь:
- Управлять мотором напрямую.
- Решать, как считать delta или выбирать профиль скоростей.

---

### L2. Splitter / Composite Mount

Зона ответственности:
- Маршрутизация команд между RA/DEC осевыми контроллерами (`L3`).
- Композиция статусов и результатов.

Должен уметь:
- Делегировать осевые команды в нужный `L3`.
- Для составных команд (`SYNC`, `SLEW`, `HALT_ALL`) агрегировать результаты осей.
- Возвращать в L1 корректный общий результат (`ok = ra_ok and dec_ok`).

Не должен уметь:
- Знать low-level протокол железа.
- Выполнять осевую state machine.

---

### L3. Axis Controller (RA/DEC)

Зона ответственности:
- Осевой домен в `sec/arcsec` (или эквивалентных axis units).
- State machine оси.
- Политика guide/tracking/manual/goto/halt.
- Формирование `motion intent` для L4 (что сделать), но не как именно в raw units.

Должен уметь:
- `sync`, `tracking`, `manual move`, `goto`, `guide`, `halt_X`, `halt_all`.
- Поддерживать persistent guide-offset до явного сброса `halt_all`.
- Возвращать понятный результат операций для L2.

---

### L4. Hardware Driver

Зона ответственности:
- Управление мотором/контроллером в raw units (`steps/ticks/period`).
- Конверсия осевых величин (`sec/arcsec`) <-> raw units.
- Выбор и применение motion profiles (tracking/manual/goto fast/slow).
- Исполнение low-level команд без LX200-протокола.

Минимальный контракт:

```python
class AxisHwDriver(Protocol):
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def snapshot(self) -> HwSnapshot: ...
    def set_position_axis(self, pos_axis: float) -> None: ...
    def stop(self, graceful: bool = True) -> None: ...
    def run_tracking_axis(self, tracking_rate_axis: float) -> None: ...
    def run_manual_axis(self, manual_rate_axis: float) -> None: ...
    def run_goto_axis(self, target_axis: float) -> None: ...
```

`HwSnapshot` минимум:
- `running: bool`
- `direction_raw` (`+1` / `0` / `-1`)
- `phase` (`idle` / `halt` / `acceleration` / `running` / `deceleration`)
- `profile` (`stop` / `tracking` / `slow` / `fast`) — speed and accel
- `motor_position: int` — raw motor position
- `position: int` — position in sec/arces
- `target_raw: int | None`

---

## L3: модель данных

Переменные состояния оси:
- `state: AxisState`
- `base_tracking_rate: float`
- `guide_offset_rate: float`
- `effective_tracking_rate = base_tracking_rate + guide_offset_rate`
- `tracking_rate_before_motion: float | None`
- `manual_rate: float | None`
- `goto_target_axis: AxisPos | None`
- `mount_position: AxisPos`
- `motor_position: int` — sec/arcsec

Важно:
- `guide_offset_rate` сохраняется после guide-команд.
- Вход в `MANUAL`/`GOTO` не должен менять tracking-rate сам по себе.
- При выходе из `MANUAL`/`GOTO` восстанавливается `tracking_rate_before_motion`.
- Сброс `guide_offset_rate` выполняется только через `halt_all`.

---

## L3 State Machine

Состояния:
- `DISCONNECTED`
- `TRACKING`
- `MANUAL`
- `GOTO`
- `FAULT`

События:
- `connect`
- `disconnect`
- `sync_to(pos)`
- `set_tracking_base(rate)`
- `guide(direction, ms)`
- `move_manual(direction)`
- `halt_X(direction)`
- `manual_stop`
- `slew_to(target)`
- `goto_tick` (периодический polling L4)
- `halt_all`
- `hw_error`

---

## Переходы L3 через вызовы L4

| Событие | From | Действия L3 | Вызовы L4 | To |
|---|---|---|---|---|
| `connect` | `DISCONNECTED` | инициализация rates/offset | `connect()`, `run_tracking_axis(base+offset)` | `TRACKING` |
| `disconnect` | `TRACKING/MANUAL/GOTO/FAULT` | очистка runtime-контекста | `stop(graceful=True)`, `disconnect()` | `DISCONNECTED` |
| `sync_to(pos)` | `TRACKING/MANUAL` | обновление local pose | `set_position_axis(pos)` | без смены |
| `set_tracking_base(rate)` | `TRACKING` | `base_tracking_rate = rate` | `run_tracking_axis(base+offset)` | `TRACKING` |
| `set_tracking_base(rate)` | `MANUAL/GOTO` | только сохранить новый base, применить позже | нет немедленного вызова | без смены |
| `guide(dir, ms)` | `TRACKING` | пересчитать `guide_offset_rate` | `run_tracking_axis(base+offset)` | `TRACKING` |
| `guide(dir, ms)` | `MANUAL/GOTO` | пересчитать `guide_offset_rate`, отложить применение | нет немедленного вызова | без смены |
| `move_manual(dir)` | `TRACKING` | `tracking_rate_before_motion = base+offset`, выбрать знак и manual-rate | `run_manual_axis(manual_signed_rate)` | `MANUAL` |
| `move_manual(dir)` | `GOTO` | отменить goto, сохранить previous tracking-rate, перейти в ручное | `stop(graceful=True)`, `run_manual_axis(manual_signed_rate)` | `MANUAL` |
| `halt_X(dir)` | `TRACKING` | no-op (останавливать нечего) | нет вызова | `TRACKING` |
| `halt_X(dir)` | `MANUAL` | остановить manual для активной оси/направления и восстановить previous tracking-rate; `effective_tracking_rate` не менять | `run_tracking_axis(tracking_rate_before_motion or (base+offset))` | `TRACKING` |
| `halt_X(dir)` | `GOTO` | прервать goto и восстановить previous tracking-rate; `effective_tracking_rate` не менять | `stop(graceful=True)`, `run_tracking_axis(tracking_rate_before_motion or (base+offset))` | `TRACKING` |
| `manual_stop` | `MANUAL` | завершить manual, восстановить previous tracking-rate | `run_tracking_axis(tracking_rate_before_motion or (base+offset))` | `TRACKING` |
| `slew_to(target)` | `TRACKING/MANUAL` | сохранить target; при входе из `TRACKING` сохранить previous tracking-rate | если `MANUAL`: `stop(graceful=True)`; затем `run_goto_axis(target)` | `GOTO` |
| `goto_tick` (target reached) | `GOTO` | завершить goto, восстановить previous tracking-rate | `run_tracking_axis(tracking_rate_before_motion or (base+offset))` | `TRACKING` |
| `goto_tick` (timeout/overshoot/error) | `GOTO` | аварийное завершение | `stop(graceful=True)` | `FAULT` или `TRACKING` по политике |
| `halt_all` | `TRACKING/MANUAL/GOTO/FAULT` | policy-команда: отмена goto/manual; optional `guide_offset_rate = 0` | `stop(graceful=True)`, `run_tracking_axis(base_tracking_rate)` | `TRACKING` |
| `hw_error` | любой connected | лог + блокировка оси | `stop(graceful=False)` | `FAULT` |

---

## Базовые сценарии

### 1) Tracking (номинальный цикл)

1. `L2 -> L3.set_tracking_base(rate)`
2. `L3` вычисляет `effective = base + guide_offset`
3. `L3 -> L4.run_tracking_axis(effective)` (профиль и raw-конверсия внутри L4)
4. Ось в `TRACKING`

### 2) Manual движение

1. `L2 -> L3.move_manual(direction)`
2. `L3` сохраняет previous tracking-rate (`tracking_rate_before_motion`), вычисляет signed rate.
3. `L3 -> L4.run_manual_axis(manual_signed_rate)`
4. Ось в `MANUAL`
5. `L2 -> L3.halt_X(direction)` и восстановление сохранённого tracking-rate.
6. `halt_X` не меняет `effective_tracking_rate`; он только завершает текущее движение.

### 3) GOTO

1. `L2 -> L3.slew_to(target)`
2. `L3` передаёт target оси.
3. `L3 -> L4.run_goto_axis(target)` (конверсия и fast/slow профиль внутри L4)
4. `L3` переходит в `GOTO`, запускает `goto_tick` polling.
5. По достижению цели: `L3 -> L4.run_tracking_axis(previous_tracking_rate)`, переход в `TRACKING`.

### 4) Guide с persistent offset

1. `L2 -> L3.guide(direction, ms)`
2. `L3` обновляет `guide_offset_rate` (накапливаемо).
3. Если state=`TRACKING`: сразу `L4.run_tracking_axis(base+offset)`.
4. Если state=`MANUAL/GOTO`: offset сохраняется и применится при возврате в `TRACKING`.
5. Сброс только через `halt_all`.

---

## Общий Код Статуса

Единая структура статуса нужна для:
- `L4 -> L3`: нормализованный hardware snapshot.
- `L3 -> L2`: нормализованный осевой статус.
- `L2 -> L1`: агрегированный статус монтировки.

Рекомендуемые модели:

```python
@dataclass(frozen=True)
class AxisStatusCommon:
    axis: Literal["ra", "dec"]
    connected: bool
    state: AxisState                   # DISCONNECTED/TRACKING/MANUAL/GOTO/FAULT
    running: bool
    motion: Literal["idle", "tracking", "manual", "goto", "fault"]
    effective_tracking_rate: float
    tracking_rate_before_motion: float | None
    guide_offset_rate: float
    mount_position: float              # sec for RA, arcsec for DEC
    motor_position_raw: int
    direction_sign: int                # -1 / 0 / +1
    phase: str                         # idle/hold/acceleration/running/deceleration
    profile: str                       # tracking/manual/goto_fast/goto_slow/stop
    target_position: float | None
    error: str | None
    raw: dict[str, Any]                # оригинальный статус железки

@dataclass(frozen=True)
class MountStatusCommon:
    ra: AxisStatusCommon
    dec: AxisStatusCommon
    connected: bool
    slewing: bool
    guiding_active: bool
    faulted: bool
```

### Маппинг из текущих железок

SkyWatcher (`SkyWatcherStatus`):
- `running <- status.running`
- `direction_sign <- +1 if FORWARD else -1`
- `phase <- "running" if running else "hold"` (детального accel/decel нет в протоколе)
- `profile <- "goto"` если `slew_mode=GOTO`, иначе `"tracking/manual"` по состоянию L3
- `raw <- {"raw": ..., "slew_mode": ..., "speed_mode": ..., ...}`

TMC2209 (`TMC2209Status`):
- `running <- phase not in {"idle", "hold"}`
- `phase <- status.phase`
- `profile <- выводится из L3-команды + L4 профиля`
- `target_position <- из target (raw -> axis units)`
- `raw <- {"mode": ..., "target_set": ..., "speed": ..., "actual_speed": ..., ...}`

### Инварианты статуса

- `halt_X` не меняет `effective_tracking_rate`.
- `halt_X` должен менять только `motion/state` и флаг `running`.
- `halt_all` может иметь policy-сброс (`guide_offset_rate = 0`) и это должно быть явно отражено в статусе.

---

## Политика результатов для LX200

Минимальное правило для `L2`:
- `SYNC`: `ok = ra_ok and dec_ok`
- `SLEW`: `ok = ra_ok and dec_ok`
- `HALT_ALL`: `ok = ra_ok and dec_ok` (если политика не допускает partial-ok)

Рекомендация:
- В логах хранить раздельно `ra_result`, `dec_result`, `combined_result`.
- В L1 всегда возвращать ответ строго по ожидаемому формату LX200 для конкретной команды.
