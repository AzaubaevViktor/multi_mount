# Sky-Watcher Motor Controller Command Set

Источник: `skywatcher_motor_controller_command_set.pdf`  
Ниже текст из PDF приведён к нормальному Markdown, с выправленной структурой и таблицами.  
Смысл не менял. Исправлял только явные артефакты переноса, OCR и форматирования.

## 1. How the motor controller controls motor speed

| Пункт | Содержание |
|---|---|
| Timer T1 | In the motor controller, there is a hardware timer T1 that is used to generate stepping pulse for stepper motor or reference position for servomotor. The input clock frequency of the timer, plus the preset value of this timer, determine the slewing speed of the motors. |
| Low-speed interrupt behavior | When T1 generates an interrupt, it might drive the motor to move 1 step (1 micro-step or 1 encoder tick) for low-speed slewing. |
| High-speed interrupt behavior | It may drive the motor to move up to 32 steps for high-speed slewing. This method applies to motor controller firmware version 2.xx. |
| Firmware 3.xx+ note | For motor controller firmware 3.xx or above, the motor controller always drives 1 step per interrupt. |

## 2. Two motion modes

| Режим | Содержание |
|---|---|
| GOTO mode | The master device tells the motor controller the desired destination, and then sends a `Start` command. The motor controller moves the motor to that destination. The master device can check motor status, real-time position, and cancel slewing during GOTO. |
| Speed (Tracking) mode | The master device calculates a proper preset value for T1 and sends it to the motor controller, and then sends a `Start` command. The motor controller slews at the desired speed. The master device can check motor status, real-time position, and cancel the slewing. |

### Notes

| Пункт | Содержание |
|---|---|
| Mode selection | There is a command used to select between the two motion modes for the next `Start` command. Generally, the motor should be at full stop before setting the motion mode. |
| Auto-return mode | Generally, the motor controller returns to `Speed Mode` when the motor stops automatically. |

### Typical slewing session

| Шаг | Действие |
|---|---|
| 1 | Check whether the motor is in full stop status. If not, stop it. |
| 2 | Set the motion mode. |
| 3 | Set parameters, for example destination or preset value of T1. |
| 4 | Send the `Start` command. |
| 5 | For GOTO: check motor status to confirm that the motor stops, which generally means arriving at the destination. For Speed mode: send `Stop` command to end the session. |

## 3. Calculation on master device

A Sky-Watcher motor controller does not do complex calculation. The master device does it instead.

### 3.1 Calculate angle

| Пункт | Содержание |
|---|---|
| Internal representation | A Sky-Watcher motor controller only counts the steps or the ticks of an incremental encoder on the motor shaft. |
| CPR | The master device can inquire the motor controller for the resolution of the telescope axis, meaning how many steps the telescope axis has for one revolution. This is called CPR (Counts Per Revolution). |
| Conversion | With CPR, the master device can convert an angle to steps or vice versa. |
| Axis note | CPR might be different for the two axes of a mount. |

### 3.2 Calculate T1 preset value

| Формула | Значение |
|---|---|
| `Speed_CountsPerSec = Speed_DegPerSec * CPR / 360` | Convert desired axis speed in deg/s to counts/s. |
| `T1_Preset = TMR_Freq / Speed_CountsPerSec` | Base formula. |
| `T1_Preset = TMR_Freq * 360 / Speed_DegPerSec / CPR` | Expanded form. |

### 3.3 T1 preset for high-speed slewing

When T1 preset value becomes too small for high-speed slewing, the controller uses another method: for each T1 interrupt, the motor controller moves `N` micro-steps for a stepper motor, or changes the reference position by `N` steps for a DC servo motor.

Currently, `N` is fixed and can be inquired from the motor controller. It might be `16`, `32`, or `64`.

| Формула | Значение |
|---|---|
| `T1_Preset = N * TMR_Freq * 360 / Speed_DegPerSec / CPR` | T1 preset for high-speed slewing. |

**Note:** when a master wants an axis to slew at high speed, it should let the motor controller know when configuring the motor to Speed (Tracking) mode. For GOTO mode, the motor controller handles this automatically.

## 4. Command format

### 4.1 General rules

| Пункт | Содержание |
|---|---|
| Command start | Every command starts with `:` |
| Command end | Every command ends with carriage return `0x0D` |
| Reset-on-second-colon | If a second `:` is received before carriage return, the motor controller abandons the previous characters and starts receiving a new command |
| Response timing | The motor controller processes the command and sends a response after it receives the carriage return |
| Normal response | Normal response starts with `=` and ends with carriage return |
| Error response | Error response starts with `!`, then error code, then carriage return |
| Encoding | All command and response characters are ASCII |

### 4.2 Command packet layout

| Поле | Размер | Содержание |
|---|---:|---|
| Leading character | 1 byte | `:` |
| Command word | 1 byte | command code |
| Channel word | 1 byte | `1` for RA/Az axis, `2` for Dec/Alt axis |
| Data | 1 to 6 bytes | ASCII `0`-`9`, `A`-`F` |
| Ending character | 1 byte | carriage return |

### 4.3 Normal response layout

| Поле | Размер | Содержание |
|---|---:|---|
| Leading character | 1 byte | `=` |
| Data | 1 to 6 bytes | ASCII `0`-`9`, `A`-`F` |
| Ending character | 1 byte | carriage return |

### 4.4 Error response layout

| Поле | Размер | Содержание |
|---|---:|---|
| Leading character | 1 byte | `!` |
| Error code | 2 bytes | ASCII `0`-`9`, `A`-`F` |
| Ending character | 1 byte | carriage return |

### 4.5 Data format

| Тип | Пример числа | Передача в data segment |
|---|---|---|
| 24-bit | `0x123456` | `"5" "6" "3" "4" "1" "2"` |
| 16-bit | `0x1234` | `"3" "4" "1" "2"` |
| 8-bit | `0x12` | `"1" "2"` |

## 5. Command set

### 5.1 Main commands

| Command | Name | Channel | Data | Response | Notes |
|---|---|---|---|---|---|
| `E` | Set Position | `*1` | 6 hex chars | `A, X` | Motor must be full stopped |
| `F` | Initialization Done | `*1 ('3')` | none | `A, X` | |
| `G` | Set Motion Mode | `*1` | bit field | `A, X` | Motor must be full stopped |
| `H` | Set Goto Target Increment | `*2` | 6 hex chars | `A, X` | |
| `M` | Set Brake Point Increment | `*1` | 6 hex chars | `A, X` | |
| `S` | Set Goto Target | `*1` | 6 hex chars | `A, X` | Motor must be full stopped |
| `I` | Set Step Period (T1 preset value) | `*1` | 6 hex chars | `A, X` | Not supported while motor is slewing in high-speed mode |
| `T` | Set Long Goto Step Period | `*1` | 6 hex chars | `A, X` | |
| `U` | Set Brake Steps | `*1` | 6 hex chars | `A, X` | |
| `J` | Start Motion | `*1` | none | `A, X` | |
| `K` | Stop Motion | `*1` | none | `A, X` | Channel will always be set to Tracking Mode after stopped |
| `L` | Instant Stop | `*1` | none | `A, X` | Channel will always be set to Tracking Mode after stopped |
| `B` | Set Sleep | `*1` | `'0'` = WakeUp, `'1'` = Sleep | `A, X` | |
| `O` | Set Aux Switch On/Off | `*1` | `'0'` = Off, `'1'` = On | `A, X` | |
| `P` | Set AutoGuide Speed | `*1` | `'0'=1x`, `'1'=0.75x`, `'2'=0.5x`, `'3'=0.25x`, `'4'=0.125x` | `A, X` | |
| `Q` | Run Bootloader Mode | `*1` | `55AA` | No response | |
| `V` | Set Polar Scope LED brightness | `*1` | 2 hex chars | `A, X` | |
| `a` | Inquire Counts Per Revolution | `*2` | none | `B, X` | |
| `b` | Inquire Timer Interrupt Freq | `'1'` | none | `B, X` | |
| `c` | Inquire Brake Steps | `*2` | none | `B, X` | |
| `h` | Inquire Goto Target Position | `*2` | none | `B, X` | |
| `i` | Inquire Step Period | `*2` | none | `B, X` | |
| `j` | Inquire Position | `*2` | none | `B, X` | |
| `k` | Inquire Increment | `*2` | `'0'` or `'1'` | `B, X` | Reset increment: `'0'` = No, `'1'` = Yes |
| `m` | Inquire Brake Point | `*2` | none | `B, X` | |
| `f` | Inquire Status | `*2` | none | `E, X` | |
| `g` | Inquire High Speed Ratio | `*2` | none | `D, X` | |
| `D` | Inquire 1X Tracking Period | `'1'` | none | `B, X` | |
| `d` | Inquire Tele. Axis Position | `*1` | none | `B, X` | |
| `e` | Inquire Motor Board Version | `*1` | none | `B, X` | `*6` format note |
| `s` | Inquire PEC period | `*1` | none | `B, X` | |
| `z` | Set Debug Flag | `*1` | none | — | |
| `W` | Extended Setting | `*1` | 6 hex chars | `X` | |
| `q` | Extended Inquire | `*1` | 6 hex chars | `X` | |
| `C` | Set EEPROM Address | `'1'` | 4 hex chars | — | |
| `N` | Set EEPROM Value | `'1'` | 2 hex chars | — | |
| `n` | Inquire EEPROM Value | `'1'` | none | — | |
| `A` | Set Register Address | `*1` | 2 hex chars | — | |
| `R` | Set Register Value | `*1` | 2 hex chars | — | |
| `r` | Inquire Register Value | `*1` | none | — | |

### 5.2 Set Motion Mode (`G`) bit fields

Из таблицы на странице 4 видно, что команда `G` использует packed flags.

| Бит / поле | Значение |
|---|---|
| B0 | `0 = Goto`, `1 = Tracking` |
| B1 | For Tracking: `0 = Slow`, `1 = Fast` |
| B1 | For Goto: `0 = Fast`, `1 = Slow` |
| B2 | `0 = S/F`, `1 = Medium` |
| B3 | `1x Slow Goto` |
| Direction bit B0 | `0 = CW`, `1 = CCW` |
| Hemisphere/axis bit B1 | `0 = North`, `1 = South` |
| Goto type bit B2 | `0 = Normal Goto`, `1 = Coarse Goto` |

### 5.3 Response formats

| Response type | Format |
|---|---|
| `A` | `0x0D` |
| `B` | 6 hex chars + `0x0D` |
| `C` | 4 hex chars + `0x0D` |
| `D` | 2 hex chars + `0x0D` |
| `E` | Bit field status + `0x0D` |
| `X` | `!` + error code + `0x0D` |

### 5.4 Status response `E`

Таблица на странице 4 даёт несколько групп битов для статуса.

| Группа | Бит | Значение |
|---|---|---|
| Motion mode | B0 | `1 = Tracking`, `0 = Goto` |
| Direction | B1 | `1 = CCW`, `0 = CW` |
| Speed | B2 | `1 = Fast`, `0 = Slow` |
| Run state | B0 | `1 = Running`, `0 = Stopped` |
| Fault state | B1 | `1 = Blocked`, `0 = Normal` |
| Init state | B0 | `0 = Not Init`, `1 = Init done` |
| Level switch | B1 | `1 = Level switch on` |

### 5.5 Notes

| Note ID | Meaning |
|---|---|
| `*1` | `'1' = CH1`, `'2' = CH2`, `'3' = Both` |
| `*2` | `'1' = CH1`, `'2' = CH2` |
| `*3` | Reset Increment: `'0' = No`, `'1' = Yes` |
| `*4` | Channel will always be set to Tracking Mode after stopped |
| `*5` | Error codes listed below |
| `*6` | Motor Board Version: `EQ x x x x 0 0`, `AZ x x x x 0 1` |

### 5.6 Error codes

| Code | Meaning |
|---|---|
| `0` | Unknown Command |
| `1` | Command Length Error |
| `2` | Motor not Stopped |
| `3` | Invalid Character |
| `4` | Not Initialized |
| `5` | Driver Sleeping |
| `7` | PEC Training is running |
| `8` | No Valid PEC data |

### 5.7 Extended Inquire IDs

| ID | Return bytes meaning |
|---|---|
| `000000` | Inquire Axis (Original) Indexer Position |
| `000001` | Inquire Status EX |

### 5.8 Extended Inquire: Status EX fields

| Byte | Meaning |
|---|---|
| Byte0 | `B0: PEC Training On/Off`, `B1: PEC Tracking On/Off` |
| Byte1 | `B0: Support dual encoder`, `B1: Support PPEC`, `B2: Support original position indexer`, `B3: Support EQ/AZ mode` |
| Byte2 | `B0: Has polar scope LED`, `B1: Two axes must start separately`, `B2: Support tracking-torque selection` |
| Byte3 | reserved / unspecified in text |
| Byte4 | reserved / unspecified in text |
| Byte5 | reserved / unspecified in text |

### 5.9 Extended Setting IDs

| ID | Meaning |
|---|---|
| `000000` | Start PEC Training |
| `000001` | Cancel PEC Training |
| `000002` | Start PEC Tracking |
| `000003` | Cancel PEC Tracking |
| `000004` | Enable Dual Encoder |
| `000005` | Disable Dual Encoder |
| `000006` | Disable full current (torque) driving at low speed |
| `000106` | Enable full current (torque) driving at low speed |
| `xxxx07` | Set Stride for Slewing |
| `000008` | Reset Axis Indexer Position |
| `000009` | Write flash buffer in RAM to flash ROM |

### 5.10 Special note for position-related data

| Пункт | Содержание |
|---|---|
| Offset rule | All position data is offset by `0x800000`. |
| Example: outgoing command | Axis position `0x000012` should be converted to `0x800012` when preparing the command. |
| Example: incoming response | Reported value `0x801234` means true position `0x001234`. |

## 6. Hardware

| Пункт | Содержание |
|---|---|
| UART | `9600 bps`, `1` start bit, `1` stop bit, no parity |
| Signal level | `5V` or `3.3V` |
| EQ mounts | On most EQ mounts, TX and RX lines are separated. The motor controller sends its response immediately after receiving and processing the command. |
| Alt/Az mounts | On most Alt/Az mounts, TX and RX lines are connected together, and another line (`Drop`) indicates that the TX/RX bus is busy. |
| Drop line behavior | The master device should pull `Drop` low when it starts to send a command and keep it low until it receives the full response or a timeout occurs. |
| Bus release | The master device should release the TX/RX bus as soon as possible after the last bit of the command is shifted out of the hardware register. |
| TX pull-up | The motor controller pulls TX high through a `5.1K` to `10K` resistor, but does not strongly drive high; other devices can pull TX low without problems. |

## 7. Wi-Fi connection

| Пункт | Содержание |
|---|---|
| Transport | The same protocol runs on SynScan Wi-Fi dongle or mounts with built-in Wi-Fi module |
| Server type | UDP server |
| Port | `11880` |
| Packet rule | Command must be sent in a single UDP packet; response is also returned in a single packet |
| AP mode IP | `192.168.4.1` |
| Station mode IP | Assigned by the router |

## 8. Useful resources

| Resource | Value |
|---|---|
| Sample code | `https://code.google.com/archive/p/skywatcher/` |
| Documents | `http://www.skywatcher.com/download/manual/application-development/` |

## 9. OCR / extraction caveats

| Область | Что важно |
|---|---|
| Page 4 command table | В исходном PDF это сложная таблица. Текст был частично нормализован по распознанному содержимому страницы и визуальной структуре таблицы. |
| Possible ambiguities | Самые уязвимые места: packed bit fields в `Set Motion Mode`, byte grouping в `Response E`, и редкие extended-команды. Для реализации протокола их лучше сверить ещё раз по исходному PDF или по реальному устройству. |