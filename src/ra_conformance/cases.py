"""The RA conformance case list — one table, read straight off the live board.

Source of every case: ``docs/protocol/RA_PROTOCOL.md``. The section column of a
case is the claim it pins; if the document and this file disagree, the document
wins and this file is the bug.

Safety split (see :class:`ra_conformance.model.Safety`):

``READ``    pure inquiries and deliberately malformed input the board rejects
            before it does anything. Safe on the live board at any moment.
``WRITE``   changes board state with the axis standing still (step period,
            motion mode, window address, position, initialization flag). Every
            such case restores what it changed in its teardown.
``MOTION``  the axis actually turns. Only ever a short run at a documented
            period, always followed by ``:K1`` and a wait for the Running bit.
``NEVER``   must not reach the live board. Two different reasons, spelled out
            in each case's note:
            * destructive or irreversible — ``:W1`` (including the UART speed
              change ``:W10D8004``), ``:R1xx``, ``:N1xx``, ``:Q155AA``;
            * documented as harmless in §3 but outside the hardware whitelist
              of ``RA_PROTOCOL_STEP_2.md`` §1.1 — the setters ``A B O P T U V
              S L`` and ``:z1``. §3 saw them answer ``=``; step 2 dropped them
              from the whitelist, and this file follows step 2.

Ordering is meaningful only in the sense that hardware runs the list as one
session. Each case is written to start from and return to the power-on state of
§12.1 (position ``0x800000``, period 110 359, tracking mode, direction CW), so
the simulator suite can also run every case against a freshly booted board.
"""

from ra_conformance.model import (
    Case,
    Exchange,
    Pause,
    Safety,
    Step,
    exact,
    matching,
    one_of,
    silence,
)

# --------------------------------------------------------------------------- #
# wire constants, §2 and §10.5
# --------------------------------------------------------------------------- #

# Revu24 (byte-reversed hex) payloads of the step periods the document names.
PERIOD_1X = b"17AF01"  # 110 359, the board's own `:D1`
PERIOD_16X = b"F11A00"  # 6 897, Fast bit still down (§10.2)
PERIOD_64X = b"BC0600"  # 1 724, Fast bit up (§10.2)
PERIOD_100X = b"4F0400"  # 1 103, the board's own floor (§10.5)
LOGICAL_ZERO = b"000080"  # 0x800000, the logical zero of §12.1

BRAKE_STEPS = 16_980  # `:c1` -> `=544200`
POSITION_OFFSET = 0x800000

# Restores the board to §12.1 after anything that touched the period, the mode
# or the position. Sent with replies ignored, so it is safe to repeat.
_RESTORE_IDLE: tuple[bytes, ...] = (
    b":K1\r",
    b":I1" + PERIOD_1X + b"\r",
    b":G110\r",
    b":E1" + LOGICAL_ZERO + b"\r",
)

_HEX_BYTE = "=[0-9A-F]{2}\r"
_HEX_WORD = "=[0-9A-F]{6}\r"


def _decode_revu24(reply: bytes) -> int:
    """``=17AF01\\r`` -> 110359. Deliberately not importing the driver's codec."""
    body = reply[1:-1].decode("ascii")
    return int(body[4:6] + body[2:4] + body[0:2], 16)


def _revu24(value: int) -> bytes:
    return b"%02X%02X%02X" % (value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF)


# Шаг 2 §2.1: на стоящей оси `:j1` дребезжит на один отсчёт — двадцать чтений
# за десять секунд дали и `N`, и `N−1`. Любое сравнение позиций, которое должно
# пережить этот дребезг, берёт этот допуск, а не ноль.
POSITION_JITTER = 2


# --------------------------------------------------------------------------- #
# §2, §3 — constants and inquiries with verbatim answers
# --------------------------------------------------------------------------- #

# command, verbatim answer, section, what the number means
_VERBATIM_INQUIRIES: tuple[tuple[bytes, bytes, str, str], ...] = (
    (b":e1\r", b"=03110A\r", "§2, §3", "прошивка 0x0311, mount code 0x0A"),
    (b":a1\r", b"=729DBE\r", "§2, §3", "CPR = 12 492 146"),
    (b":b1\r", b"=0024F4\r", "§2, §3", "TMR_Freq = 16 000 000 Гц"),
    (b":g1\r", b"=01\r", "§2.1, §13 #3", "highspeed ratio = 1, ответ ровно 2 символа"),
    (b":D1\r", b"=17AF01\r", "§2, §3", "период 1× трекинга = 110 359"),
    (b":c1\r", b"=544200\r", "§2, §3", "brake steps = 16 980"),
    (b":s1\r", b"=000000\r", "§2, §3", "PEC period = 0, PEC у платы нет"),
    (
        b":d1\r",
        b"=000080\r",
        "§3, шаг 2 §2.3",
        "Tele. Axis Position — константа `0x800000`: не следует ни за движением, ни за `:E1`",
    ),
    (b":i1\r", b"=17AF01\r", "§3, §12.1", "период шага после включения"),
    (b":n1\r", b"=00\r", "§3", "окно `:C`/`:n` после включения стоит на адресе 0x00"),
    (b":r1\r", b"=00\r", "§3, §13 #23", "регистровый файл `:A`/`:r` — заглушка"),
    (b":q1010000\r", b"=008000\r", "§3, §5.1", "Status EX: поднят только Byte2 B3"),
    (b":q1020000\r", b"=100000\r", "§5", "недокументированный ID 2 = 16"),
    (b":q1040000\r", b"=A3C9CA\r", "§5", "недокументированный ID 4, константа"),
    (b":q1050000\r", b"=49DB48\r", "§5", "недокументированный ID 5, константа"),
)

# §3.2 — every hex character is accepted as a channel; only channel 1 is an axis
_PHANTOM_CHANNELS: tuple[tuple[bytes, bytes, str], ...] = (
    (b":e2\r", b"=03110A\r", "константа платы, а не оси"),
    (b":a2\r", b"=729DBE\r", "константа платы, а не оси"),
    (b":j2\r", b"=000080\r", "позиция несуществующей оси"),
    (b":f2\r", b"=000\r", "статус несуществующей оси"),
    (b":f3\r", b"=000\r", "канал 3 «Both» особым образом не обрабатывается"),
    (b":f0\r", b"=000\r", "канал 0"),
    (b":f4\r", b"=000\r", "канал 4"),
    (b":f9\r", b"=000\r", "канал 9"),
)

# §7 — which malformed input produces which error code
_ERROR_CODES: tuple[tuple[str, bytes, bytes, str], ...] = (
    ("err_unknown_letter", b":X1\r", b"!0\r", "неизвестная буква команды"),
    ("err_k_not_implemented", b":k10\r", b"!0\r", "§4: документированная, но не реализованная"),
    ("err_extended_id_0", b":q1000000\r", b"!0\r", "§4: индексатора home-позиции нет"),
    ("err_extended_id_6", b":q1060000\r", b"!0\r", "§5: ID выше 5 не поддержаны"),
    ("err_extended_id_ff", b":q1FF0000\r", b"!0\r", "§5: ID выше 5 не поддержаны"),
    ("err_extended_id_middle_byte", b":q1000100\r", b"!0\r", "§5: перебор среднего байта ID — все `!0`"),
    ("err_extended_id_high_byte", b":q1000001\r", b"!0\r", "§5: перебор старшего байта ID — все `!0`"),
    ("err_no_channel", b":f\r", b"!0\r", "§8.2: слишком короткий пакет — не команда"),
    ("err_empty_command", b":\r", b"!0\r", "пустая команда"),
    ("err_setter_no_argument", b":I1\r", b"!1\r", "сеттер без аргумента"),
    ("err_argument_too_short", b":I112\r", b"!1\r", "2 hex вместо 6"),
    ("err_argument_too_long", b":I100000000\r", b"!1\r", "8 hex вместо 6"),
    ("err_g_no_argument", b":G1\r", b"!1\r", "неверная длина аргумента G"),
    ("err_g_argument_too_long", b":G1123456\r", b"!1\r", "неверная длина аргумента G"),
    ("err_argument_on_inquiry", b":j1000000\r", b"!1\r", "аргумент у запроса, который его не принимает"),
    ("err_one_extra_char", b":f11\r", b"!1\r", "один лишний символ"),
    ("err_huge_argument", b":I1" + b"0" * 40 + b"\r", b"!1\r", "сверхдлинный аргумент"),
    ("err_greedy_length", b":" + b"A" * 60 + b"\r", b"!1\r", "§8.2: буква распознаётся раньше длины"),
    ("err_non_hex_channel_question", b":f?\r", b"!3\r", "не-hex символ канала"),
    ("err_non_hex_channel_letter", b":fL\r", b"!3\r", "§6.1: `L` на месте канала"),
    ("err_non_hex_argument", b":I1ZZZZZZ\r", b"!3\r", "не-hex аргумент"),
    ("err_non_hex_channel_and_arg", b":IZZZZZZ\r", b"!3\r", "не-hex и канал, и аргумент"),
    ("err_lowercase_hex", b":I1abcdef\r", b"!3\r", "§13 #17: нижний регистр запрещён"),
    ("err_minus_in_argument", b":I1-00000\r", b"!3\r", "минус в аргументе"),
    ("err_space_in_argument", b":I1 00000\r", b"!3\r", "пробел в аргументе"),
)

# §4 — the full sweep of unused command letters, every one of them `!0`
_UNUSED_LETTERS = b"XYZloptuvwxy"

# §8 — framing: what is silence and what is not
_SILENT_INPUT: tuple[tuple[str, bytes, str, str], ...] = (
    ("frame_bare_cr", b"\r", "§8", "без ведущего `:` реакции нет"),
    ("frame_garbage", b"garbage\r", "§8", "мусор без двоеточия"),
    ("frame_no_colon", b"f1\r", "§8", "команда без двоеточия"),
    ("frame_hash_terminator_fl", b":fL#", "§6.1, §8", "`#` не терминатор: та самая выдуманная `:fL#`"),
    ("frame_hash_terminator_f1", b":f1#", "§6.1", "`#` не терминатор даже для валидной команды"),
    ("frame_hash_terminator_e1", b":e1#", "§6.1", "`#` не терминатор даже для валидной команды"),
)


def _verbatim_cases() -> list[Case]:
    cases: list[Case] = []
    for send, reply, section, note in _VERBATIM_INQUIRIES:
        cases.append(
            Case(
                name=send.rstrip(b"\r").decode("ascii"),
                section=section,
                safety=Safety.READ,
                steps=(Exchange(send, exact(reply)),),
                note=note,
            )
        )
    for send, reply, note in _PHANTOM_CHANNELS:
        cases.append(
            Case(
                name=f"phantom {send.rstrip(b'\r').decode('ascii')}",
                section="§3.2, §13 #16",
                safety=Safety.READ,
                steps=(Exchange(send, exact(reply)),),
                note=note,
            )
        )
    for name, send, reply, note in _ERROR_CODES:
        cases.append(
            Case(
                name=name,
                section="§7, §8.2",
                safety=Safety.READ,
                steps=(Exchange(send, exact(reply)),),
                note=note,
            )
        )
    for name, send, section, note in _SILENT_INPUT:
        cases.append(
            Case(
                name=name,
                section=section,
                safety=Safety.READ,
                steps=(Exchange(send, silence()),),
                note=note,
            )
        )
    return cases


def _unused_letters_case() -> Case:
    steps: tuple[Step, ...] = tuple(
        Exchange(b":" + bytes([letter]) + b"1\r", exact(b"!0\r")) for letter in _UNUSED_LETTERS
    )
    return Case(
        name="unused_command_letters",
        section="§4",
        safety=Safety.READ,
        steps=steps,
        note="перебор `X Y Z l o p t u v w x y` — недокументированных однобуквенных команд нет",
    )


def _status_case() -> Case:
    return Case(
        name=":f1",
        section="§3, §10.1",
        safety=Safety.READ,
        steps=(Exchange(b":f1\r", one_of(b"=100\r", b"=101\r")),),
        note="стоит, CW, режим трекинга; флаг инициализации зависит от истории сессии",
    )


# §3.1 (переписан по шагу 2), §12.1: позиция, цель, точка торможения и
# «Tele. Axis Position» — состояние сессии, а не константы платы. Прочитать их
# дословно можно было только потому, что в первой сессии ось стояла ровно в
# логическом нуле; на живой плате шага 2 `:j1` был `0x80012F`, а `:d1` при этом
# оставался `0x800000`. Поэтому здесь проверяется форма ответа, а сами
# отношения между регистрами — отдельными кейсами ниже.
_POSITION_FAMILY: tuple[tuple[bytes, str, str], ...] = (
    (b":j1\r", "§3, §12.1", "позиция: 24 бита; после включения 0x800000, но в сессии — что угодно"),
    (b":h1\r", "§3", "goto target: 24 бита, задаётся `:H1` от текущей позиции"),
    (b":m1\r", "§3.1", "brake point: производная от `:h1`, а не от позиции (шаг 2 §2.2)"),
)


def _position_family_cases() -> list[Case]:
    return [
        Case(
            name=send.rstrip(b"\r").decode("ascii"),
            section=section,
            safety=Safety.READ,
            steps=(Exchange(send, matching(_HEX_WORD, "24-битное значение")),),
            note=note,
        )
        for send, section, note in _POSITION_FAMILY
    ]


def _brake_point_case() -> Case:
    """§3.1 переписан: `:m1` считается от **цели**, а не от позиции.

    Первая сессия читала обе величины при позиции и цели, равных `0x800000`,
    и не могла их различить. Шаг 2 развёл их: `:H1` на +50 000 сдвинул `:h1` на
    +50 000 и `:m1` вместе с ним, при неподвижной позиции.
    """

    def check(replies: tuple[bytes, ...]) -> str | None:
        target = _decode_revu24(replies[1])
        brake_point = _decode_revu24(replies[2])
        expected = (target - BRAKE_STEPS) % 0x1000000
        if brake_point != expected:
            return f"`:m1` = 0x{brake_point:06X}, а `:h1` − `:c1` даёт 0x{expected:06X}"
        return None

    return Case(
        name="m1_follows_target_not_position",
        section="§3.1, шаг 2 §2.2",
        safety=Safety.WRITE,
        steps=(
            Exchange(b":H1" + _revu24(50_000) + b"\r", exact(b"=\r")),
            Exchange(b":h1\r", matching(_HEX_WORD, "цель = позиция + 50 000")),
            Exchange(b":m1\r", matching(_HEX_WORD, "точка торможения = цель − brake steps")),
        ),
        note="направление CW: `:m1` = `:h1` − 16 980; при CCW знак меняется",
        teardown=(b":H1000000\r",),
        check=check,
    )


def _tele_position_is_a_constant_case() -> Case:
    """Шаг 2 §2.3: `:d1` — не вторая копия позиции, а константа `0x800000`.

    §3 читал `:d1` равным `:j1` в каждом замере — но только потому, что ось всю
    ту сессию стояла в логическом нуле. Кейс разводит их принудительно: пишет
    позицию `0x812345` и убеждается, что `:d1` не сдвинулся.
    """
    return Case(
        name="d1_does_not_follow_the_position",
        section="§3, шаг 2 §2.3",
        safety=Safety.WRITE,
        steps=(
            Exchange(b":E1452381\r", exact(b"=\r")),
            Exchange(b":j1\r", exact(b"=452381\r")),
            Exchange(b":d1\r", exact(b"=000080\r")),
            Exchange(b":E1" + LOGICAL_ZERO + b"\r", exact(b"=\r")),
            Exchange(b":j1\r", exact(b"=" + LOGICAL_ZERO + b"\r")),
        ),
        note="`:E1` пишет позицию и только её; `:d1` остаётся `=000080` и после записи, и после сотен тысяч пройденных отсчётов",
        teardown=(b":E1" + LOGICAL_ZERO + b"\r",),
    )


def _brake_increment_is_ignored_case() -> Case:
    """Шаг 2 §2.2: `:M1` принимается с `=` и не меняет ровно ничего.

    Вендорское приложение шлёт `:M1AC0D00` (3 500 шагов) перед каждым `:J1`
    (`RA_SA_CONSOLE_PROTOCOL.md` §6). На этой плате величина торможения жёстко
    равна `:c1` = 16 980, а инкремент из `:M1` не доходит ни до `:m1`, ни до
    поведения при подъезде к цели.
    """

    def check(replies: tuple[bytes, ...]) -> str | None:
        if replies[1] != replies[3]:
            return f"`:m1` изменился после `:M1`: {replies[1]!r} -> {replies[3]!r}"
        return None

    return Case(
        name="M_break_increment_is_ignored",
        section="§3, шаг 2 §2.2",
        safety=Safety.WRITE,
        steps=(
            Exchange(b":H1" + _revu24(50_000) + b"\r", exact(b"=\r")),
            Exchange(b":m1\r", matching(_HEX_WORD, "точка торможения до `:M1`")),
            Exchange(b":M1AC0D00\r", exact(b"=\r")),
            Exchange(b":m1\r", matching(_HEX_WORD, "она же после `:M1`")),
        ),
        note="вендорские 3 500 шагов торможения плата принимает и выбрасывает",
        teardown=(b":H1000000\r", b":M1000000\r"),
        check=check,
    )


def _drifting_extended_id_case() -> Case:
    return Case(
        name=":q1030000",
        section="§5, §6.8",
        safety=Safety.READ,
        steps=(Exchange(b":q1030000\r", matching("=[0-9A-F]{2}0102\r", "=XX0102, дрейфует младший байт")),),
        note="старшие два байта всегда 0x0201; полное число вольтами не является (§6.7)",
    )


# --------------------------------------------------------------------------- #
# §8 — framing sequences that need more than one exchange
# --------------------------------------------------------------------------- #


def _framing_cases() -> list[Case]:
    return [
        Case(
            name="frame_cr_completes_command",
            section="§8",
            safety=Safety.READ,
            steps=(
                Exchange(b":f1", silence()),
                Exchange(b"\r", one_of(b"=100\r", b"=101\r")),
            ),
            note="ответа нет, пока не придёт CR; одиночный CR довершает команду",
        ),
        Case(
            name="frame_second_colon_resets",
            section="§8, §4.1 спецификации",
            safety=Safety.READ,
            steps=(
                Exchange(b":f", silence()),
                # Дочитывается `:e1`, а не `:j1`: ответ на неё — константа платы,
                # а позиция в сессии меняется и дребезжит (шаг 2 §2.1), из-за чего
                # кейс про обрамление краснел бы по причине, к обрамлению не
                # относящейся.
                Exchange(b":e1\r", exact(b"=03110A\r")),
            ),
            note="второе `:` отбрасывает недобранную команду",
        ),
        Case(
            name="frame_second_command_is_lost",
            section="§8.1, §13 #18",
            safety=Safety.READ,
            steps=(
                Exchange(b":f1\r:e1\r", one_of(b"=100\r", b"=101\r")),
                Exchange(None, silence()),
            ),
            note="две команды одной записью дают один ответ: плата не буферизует",
        ),
        Case(
            name="frame_pause_delivers_both",
            section="§8.1",
            safety=Safety.READ,
            steps=(
                Exchange(b":f1\r", one_of(b"=100\r", b"=101\r")),
                Pause(0.02),
                Exchange(b":e1\r", exact(b"=03110A\r")),
            ),
            note="те же две команды с паузой 20 мс дают оба ответа",
        ),
    ]


# --------------------------------------------------------------------------- #
# §6.8 — the `:C`/`:n` window: both supply voltages and their quirks
# --------------------------------------------------------------------------- #


def _window_int16(replies: tuple[bytes, ...], low_index: int, high_index: int) -> int:
    return (int(replies[high_index][1:3], 16) << 8) | int(replies[low_index][1:3], 16)


def _voltage_case(name: str, low_address: bytes, high_address: bytes, lo_v: float, hi_v: float, note: str) -> Case:
    def check(replies: tuple[bytes, ...]) -> str | None:
        hundredths = _window_int16(replies, 1, 3)
        if not lo_v * 100 <= hundredths <= hi_v * 100:
            return f"{hundredths / 100:.2f} В вне диапазона {lo_v:.2f}..{hi_v:.2f} В"
        return None

    return Case(
        name=name,
        section="§6.8",
        safety=Safety.WRITE,
        steps=(
            Exchange(b":C1" + low_address + b"\r", exact(b"=\r")),
            Exchange(b":n1\r", matching(_HEX_BYTE, "один байт, ровно 2 hex-символа")),
            Exchange(b":C1" + high_address + b"\r", exact(b"=\r")),
            Exchange(b":n1\r", matching(_HEX_BYTE, "один байт, ровно 2 hex-символа")),
        ),
        note=note,
        check=check,
        teardown=(b":C10000\r",),
    )


def _window_cases() -> list[Case]:
    def no_autoincrement(replies: tuple[bytes, ...]) -> str | None:
        if replies[1] != replies[2]:
            return f"адрес уехал сам: {replies[1]!r} затем {replies[2]!r}"
        return None

    def q3_mirrors_window(replies: tuple[bytes, ...]) -> str | None:
        if replies[2][1:3] != replies[1][1:3]:
            return f"`:q1030000` даёт {replies[2]!r}, а окно 0x1C — {replies[1]!r}"
        return None

    return [
        _voltage_case(
            name="volt_battery",
            low_address=b"0400",
            high_address=b"0500",
            lo_v=3.0,
            hi_v=9.0,
            note="напряжение батарей: 0x04 младший, 0x05 старший, int16 LE, /100 → вольты",
        ),
        _voltage_case(
            name="volt_usb",
            low_address=b"1C00",
            high_address=b"1D00",
            lo_v=1.0,
            hi_v=9.0,
            note="напряжение USB: 0x1C младший, 0x1D старший, тот же формат",
        ),
        Case(
            name="window_has_no_autoincrement",
            section="§6.8",
            safety=Safety.WRITE,
            steps=(
                Exchange(b":C10500\r", exact(b"=\r")),
                Exchange(b":n1\r", matching(_HEX_BYTE, "один байт")),
                Exchange(b":n1\r", matching(_HEX_BYTE, "тот же байт")),
            ),
            note="адрес приходится переустанавливать перед каждым чтением; 0x05 — старший байт батареи, стабилен",
            check=no_autoincrement,
            teardown=(b":C10000\r",),
        ),
        Case(
            name="q1030000_mirrors_window_1C",
            section="§6.8, §6.3",
            safety=Safety.WRITE,
            steps=(
                Exchange(b":C11C00\r", exact(b"=\r")),
                Exchange(b":n1\r", matching(_HEX_BYTE, "младший байт USB-канала")),
                Exchange(b":q1030000\r", matching("=[0-9A-F]{2}0102\r", "=XX0102 с тем же XX")),
            ),
            note="младшие 16 бит `:q1030000` побайтово равны 0x1C/0x1D",
            check=q3_mirrors_window,
            teardown=(b":C10000\r",),
        ),
    ]


def _window_sweep_case() -> Case:
    """§6.8: the whole window 0x00–0x3F holds exactly four non-zero bytes."""
    steps: list[Step] = []
    for address in range(0x40):
        steps.append(Exchange(b":C1%02X00\r" % address, exact(b"=\r")))
        steps.append(Exchange(b":n1\r", matching(_HEX_BYTE, "один байт")))

    def check(replies: tuple[bytes, ...]) -> str | None:
        nonzero = {index // 2 for index, reply in enumerate(replies) if index % 2 == 1 and reply[1:3] != b"00"}
        expected = {0x04, 0x05, 0x1C, 0x1D}
        if nonzero != expected:
            return (
                "ненулевые адреса окна "
                + ", ".join(f"0x{address:02X}" for address in sorted(nonzero))
                + ", а §6.8 нашёл ровно 0x04, 0x05, 0x1C, 0x1D"
            )
        return None

    return Case(
        name="window_sweep_0x00_0x3F",
        section="§6.8, §6.2",
        safety=Safety.WRITE,
        steps=tuple(steps),
        note="сплошной обход окна: живые только два напряжения, остальное нули",
        check=check,
        teardown=(b":C10000\r",),
    )


def _extended_inquire_sweep_case() -> Case:
    """§5: of the low-byte IDs only 1..5 answer, everything else is `!0`."""
    steps: tuple[Step, ...] = tuple(
        Exchange(b":q1%02X0000\r" % identifier, exact(b"!0\r")) for identifier in range(0x06, 0x20)
    )
    return Case(
        name="extended_inquire_sweep_06_1F",
        section="§5",
        safety=Safety.READ,
        steps=steps,
        note="перебор младшего байта ID выше 5: ни один не отвечает",
    )


def _timing_case() -> Case:
    """§9: the board's own processing time, the discriminator of §12.3.

    The lower bound is the load-bearing one. A reply that arrives in no time at
    all is not a serial board answering — it is a model of one, and §12.3 says
    the *only* way to tell a rebooted board from a dead link is how long the
    answer took. The simulator does not model this at all: see the note in
    ``src/tests/units/test_ra_conformance.py``.
    """
    steps: list[Step] = []
    for _ in range(5):
        steps.append(Exchange(b":f1\r", one_of(b"=100\r", b"=101\r")))
        steps.append(Exchange(b":e1\r", exact(b"=03110A\r")))
    return Case(
        name="reply_time_is_milliseconds_not_zero",
        section="§9, §12.3",
        safety=Safety.READ,
        steps=tuple(steps),
        note="собственное время обработки платы ≈1.0–1.2 мс; замеры §9 дали 1.8–4.7 мс на команду",
        timing_s=(0.001, 0.020),
    )


# --------------------------------------------------------------------------- #
# §10.5 — the board clamps the step period at 100x sidereal, whatever is written
# --------------------------------------------------------------------------- #


def _clamp_cases() -> list[Case]:
    return [
        Case(
            name="period_stored_verbatim_above_floor",
            section="§10.5",
            safety=Safety.WRITE,
            steps=(
                Exchange(b":I1" + PERIOD_16X + b"\r", exact(b"=\r")),
                Exchange(b":i1\r", exact(b"=" + PERIOD_16X + b"\r")),
                Exchange(b":I1" + PERIOD_100X + b"\r", exact(b"=\r")),
                Exchange(b":i1\r", exact(b"=" + PERIOD_100X + b"\r")),
            ),
            note="выше предела плата хранит ровно записанное (6 897 и 1 103)",
            teardown=(b":I1" + PERIOD_1X + b"\r",),
        ),
        Case(
            name="period_clamped_at_1103",
            section="§10.5, §13 #5",
            safety=Safety.WRITE,
            steps=(
                Exchange(b":I1010000\r", exact(b"=\r")),
                Exchange(b":i1\r", exact(b"=" + PERIOD_100X + b"\r")),
                Exchange(b":I1000000\r", exact(b"=\r")),
                Exchange(b":i1\r", exact(b"=" + PERIOD_100X + b"\r")),
            ),
            note="плата принимает `:I1` с любым значением, но хранит max(значение, 1103); ноль тоже 1103",
            teardown=(b":I1" + PERIOD_1X + b"\r",),
        ),
    ]


# --------------------------------------------------------------------------- #
# §10.1, §10.2 — the status byte at rest, and the Fast bit the board owns
# --------------------------------------------------------------------------- #


def _status_bits_cases() -> list[Case]:
    return [
        Case(
            name="K_does_not_return_a_standing_axis_to_tracking",
            section="§10.3, §13 #19, шаг 2 §2.4",
            safety=Safety.WRITE,
            steps=(
                Exchange(b":F1\r", exact(b"=\r")),
                Exchange(b":G120\r", exact(b"=\r")),
                Exchange(b":f1\r", exact(b"=001\r")),
                Exchange(b":K1\r", exact(b"=\r")),
                Exchange(b":f1\r", exact(b"=001\r")),
            ),
            note=(
                "примечание *4 спецификации («после `K` канал всегда в режиме трекинга») "
                "на стоящей оси НЕ выполняется: режим остаётся goto. §10.3 считал его "
                "подтверждённым, но все его замеры шли из режима трекинга, где бит и так стоял"
            ),
            teardown=(b":G110\r",),
        ),
        Case(
            name="E_at_rest_keeps_init_flag",
            section="§11",
            safety=Safety.WRITE,
            steps=(
                Exchange(b":F1\r", exact(b"=\r")),
                Exchange(b":E1" + LOGICAL_ZERO + b"\r", exact(b"=\r")),
                Pause(1.0),
                Exchange(b":f1\r", exact(b"=101\r")),
            ),
            note="контроль к квирку §11: на стоящей оси ни один сеттер флаг не сбрасывает",
            teardown=(b":E1" + LOGICAL_ZERO + b"\r",),
        ),
        Case(
            name="H_zero_increment_targets_the_current_position",
            section="§3, шаг 2 §2.2",
            safety=Safety.WRITE,
            steps=(
                Exchange(b":H1000000\r", exact(b"=\r")),
                Exchange(b":M1000000\r", exact(b"=\r")),
                Exchange(b":h1\r", matching(_HEX_WORD, "цель = позиция")),
                Exchange(b":j1\r", matching(_HEX_WORD, "она же позиция")),
            ),
            note="`:H1` — инкремент от текущей позиции, а не абсолютная цель; нулевой оставляет цель на оси",
            check=_target_equals_position,
            teardown=(b":H1000000\r",),
        ),
        Case(
            name="status_direction_bit",
            section="§10.1",
            safety=Safety.WRITE,
            steps=(
                Exchange(b":F1\r", exact(b"=\r")),
                Exchange(b":G111\r", exact(b"=\r")),
                Exchange(b":f1\r", exact(b"=301\r")),
                Exchange(b":G110\r", exact(b"=\r")),
                Exchange(b":f1\r", exact(b"=101\r")),
            ),
            note="B1 первого символа: 1 = CCW, 0 = CW",
            teardown=(b":G110\r",),
        ),
        Case(
            name="fast_bit_is_not_raised_by_G3",
            section="§10.2, §13 #15",
            safety=Safety.WRITE,
            steps=(
                Exchange(b":F1\r", exact(b"=\r")),
                Exchange(b":I1" + PERIOD_64X + b"\r", exact(b"=\r")),
                Exchange(b":G130\r", exact(b"=\r")),
                Exchange(b":f1\r", exact(b"=101\r")),
            ),
            note="запрошен быстрый режим и загружен «быстрый» период 1 724 — в покое бит Fast всё равно сброшен",
            teardown=(b":I1" + PERIOD_1X + b"\r", b":G110\r"),
        ),
    ]


# --------------------------------------------------------------------------- #
# motion: everything that needs the axis to actually turn
# --------------------------------------------------------------------------- #


def _motion_cases() -> list[Case]:
    return [
        Case(
            name="track_16x_fast_bit_down",
            section="§10.1, §10.2",
            safety=Safety.MOTION,
            steps=(
                Exchange(b":F1\r", exact(b"=\r")),
                Exchange(b":I1" + PERIOD_16X + b"\r", exact(b"=\r")),
                Exchange(b":G110\r", exact(b"=\r")),
                Exchange(b":J1\r", exact(b"=\r")),
                Exchange(b":f1\r", exact(b"=111\r")),
                Exchange(b":K1\r", exact(b"=\r")),
                Pause(2.0),
                Exchange(b":f1\r", exact(b"=101\r")),
            ),
            note="16× сидерических: идёт, CW, бит Fast сброшен",
            teardown=_RESTORE_IDLE,
        ),
        Case(
            name="track_64x_fast_bit_up",
            section="§10.2",
            safety=Safety.MOTION,
            steps=(
                Exchange(b":F1\r", exact(b"=\r")),
                Exchange(b":I1" + PERIOD_64X + b"\r", exact(b"=\r")),
                Exchange(b":G110\r", exact(b"=\r")),
                Exchange(b":J1\r", exact(b"=\r")),
                Exchange(b":f1\r", exact(b"=511\r")),
                Exchange(b":K1\r", exact(b"=\r")),
                Pause(2.0),
                Exchange(b":f1\r", exact(b"=101\r")),
            ),
            note="тот же `:G110`, но период 1 724 — плата сама поднимает бит Fast и гасит его при остановке",
            teardown=_RESTORE_IDLE,
        ),
        Case(
            name="G_while_running_is_rejected",
            section="§7 код 2",
            safety=Safety.MOTION,
            steps=(
                Exchange(b":F1\r", exact(b"=\r")),
                Exchange(b":I1" + PERIOD_16X + b"\r", exact(b"=\r")),
                Exchange(b":G110\r", exact(b"=\r")),
                Exchange(b":J1\r", exact(b"=\r")),
                Exchange(b":G111\r", exact(b"!2\r")),
                Exchange(b":K1\r", exact(b"=\r")),
            ),
            note="единственный воспроизведённый `!2 Motor not Stopped`",
            teardown=_RESTORE_IDLE,
        ),
        Case(
            name="K_is_a_ramp_not_a_switch",
            section="§10.3, §13 #19, шаг 2 §2.6",
            safety=Safety.MOTION,
            steps=(
                Exchange(b":F1\r", exact(b"=\r")),
                Exchange(b":I1" + PERIOD_100X + b"\r", exact(b"=\r")),
                Exchange(b":G110\r", exact(b"=\r")),
                Exchange(b":J1\r", exact(b"=\r")),
                # Разгон длится ~1.2 с (шаг 2 §2.6). Полсекунды хода, как было
                # здесь раньше, ось до своей скорости не доводили — и тормозила
                # она тогда быстрее, чем кейс успевал спросить.
                Pause(2.5),
                Exchange(b":K1\r", exact(b"=\r")),
                Pause(0.5),
                Exchange(b":f1\r", exact(b"=511\r")),
                Pause(2.0),
                Exchange(b":f1\r", exact(b"=101\r")),
            ),
            note="разогнанная ось тормозит ~1 с: через полсекунды после `:K1` всё ещё Running",
            teardown=_RESTORE_IDLE,
        ),
        Case(
            name="E_while_running_is_accepted",
            section="§11, §13 #7",
            safety=Safety.MOTION,
            steps=(
                Exchange(b":F1\r", exact(b"=\r")),
                Exchange(b":I1" + PERIOD_16X + b"\r", exact(b"=\r")),
                Exchange(b":G110\r", exact(b"=\r")),
                Exchange(b":J1\r", exact(b"=\r")),
                Exchange(b":E1" + LOGICAL_ZERO + b"\r", exact(b"=\r")),
                Exchange(b":K1\r", exact(b"=\r")),
            ),
            note="спецификация требует остановленный мотор, плата принимает с `=` — защита обязана быть на хосте",
            teardown=_RESTORE_IDLE,
        ),
        Case(
            name="E_while_running_may_clear_init_flag",
            section="§11, шаг 2 §2.5",
            safety=Safety.MOTION,
            steps=(
                Exchange(b":F1\r", exact(b"=\r")),
                Exchange(b":I1" + PERIOD_16X + b"\r", exact(b"=\r")),
                Exchange(b":G110\r", exact(b"=\r")),
                Exchange(b":J1\r", exact(b"=\r")),
                Exchange(b":f1\r", exact(b"=111\r")),
                Exchange(b":E1" + LOGICAL_ZERO + b"\r", exact(b"=\r")),
                Pause(1.0),
                # `=110` — квирк сработал, `=111` — не сработал. §11 сам пишет,
                # что одна и та же последовательность в разных заходах даёт
                # разный результат («Повтор той же комбинации ... флаг
                # сохранился»), и шаг 2 это увидел на железе. Кейс поэтому
                # закрепляет то, что детерминировано: ось продолжает идти, `:E`
                # принята с `=`, а флаг — как повезёт. Требование к драйверу
                # («не слать `:E` на ходу») от этого только строже.
                Exchange(b":f1\r", one_of(b"=110\r", b"=111\r")),
                Exchange(b":K1\r", exact(b"=\r")),
            ),
            note="флаг сбрасывается асинхронно и НЕ каждый раз; экспериментально 1 сброс из 2 заходов",
            teardown=_RESTORE_IDLE,
        ),
        Case(
            name="S_while_running_is_accepted",
            section="§13 #8",
            safety=Safety.NEVER,
            steps=(
                Exchange(b":F1\r", exact(b"=\r")),
                Exchange(b":I1" + PERIOD_16X + b"\r", exact(b"=\r")),
                Exchange(b":G110\r", exact(b"=\r")),
                Exchange(b":J1\r", exact(b"=\r")),
                Exchange(b":S1" + LOGICAL_ZERO + b"\r", exact(b"=\r")),
                Exchange(b":K1\r", exact(b"=\r")),
            ),
            note="`:S1` вне белого списка RA_PROTOCOL_STEP_2 §1.1 — на железо не шлём, проверяем только на симуляторе",
            teardown=_RESTORE_IDLE,
        ),
        Case(
            name="J_without_F_has_no_error_4",
            section="§7 код 4, §13 #9",
            safety=Safety.MOTION,
            steps=(
                # Снять флаг инициализации нечем, кроме самого квирка §11: `:E`
                # на ходу. Поэтому кейс сначала воспроизводит его, а уже потом
                # проверяет утверждение «`:J1` без `:F1` принимается».
                Exchange(b":F1\r", exact(b"=\r")),
                Exchange(b":I1" + PERIOD_16X + b"\r", exact(b"=\r")),
                Exchange(b":G110\r", exact(b"=\r")),
                Exchange(b":J1\r", exact(b"=\r")),
                Exchange(b":E1" + LOGICAL_ZERO + b"\r", exact(b"=\r")),
                Pause(1.0),
                Exchange(b":K1\r", exact(b"=\r")),
                Pause(2.0),
                # Сработал квирк или нет — `:J1` всё равно обязана ответить `=`,
                # а не `!4`. Ждать именно `=100` кейс больше не может: §11
                # недетерминирован (шаг 2 §2.5).
                Exchange(b":f1\r", one_of(b"=100\r", b"=101\r")),
                Exchange(b":J1\r", exact(b"=\r")),
                Exchange(b":K1\r", exact(b"=\r")),
            ),
            note="флаг инициализации у этой платы — индикатор, а не защита; `!4` не воспроизводится в принципе",
            teardown=_RESTORE_IDLE,
        ),
        Case(
            name="brake_point_follows_direction",
            section="§3.1",
            safety=Safety.MOTION,
            steps=(
                Exchange(b":F1\r", exact(b"=\r")),
                Exchange(b":I1" + PERIOD_16X + b"\r", exact(b"=\r")),
                Exchange(b":G111\r", exact(b"=\r")),
                Exchange(b":J1\r", exact(b"=\r")),
                Pause(0.5),
                Exchange(b":K1\r", exact(b"=\r")),
                Pause(2.0),
                Exchange(b":f1\r", exact(b"=301\r")),
                Exchange(b":m1\r", matching(_HEX_WORD, "точка торможения")),
                Exchange(b":h1\r", matching(_HEX_WORD, "цель")),
            ),
            note="после хода CCW статус `=301`, а brake point уходит на `+ brake steps` от цели",
            check=_brake_point_after_ccw,
            teardown=_RESTORE_IDLE,
        ),
        _speed_case("speed_at_64x_slow_mode", b":G110\r", "режим `G '1'` (медленный)"),
        _speed_case("speed_at_64x_highspeed_mode", b":G130\r", "режим `G '3'` (быстрый)"),
        _speed_ceiling_case(),
        _no_ramp_below_fast_bit_case(),
        Case(
            name="restore_logical_zero",
            section="§10.6, §12.1",
            safety=Safety.MOTION,
            steps=(
                Exchange(b":K1\r", exact(b"=\r")),
                Pause(2.0),
                Exchange(b":f1\r", one_of(b"=100\r", b"=101\r", b"=300\r", b"=301\r")),
                Exchange(b":E1" + LOGICAL_ZERO + b"\r", exact(b"=\r")),
                Exchange(b":j1\r", exact(b"=000080\r")),
                Exchange(b":I1" + PERIOD_1X + b"\r", exact(b"=\r")),
                Exchange(b":i1\r", exact(b"=17AF01\r")),
                Exchange(b":F1\r", exact(b"=\r")),
            ),
            note="финал сессии: ось стоит, позиция и период возвращены к §12.1",
            teardown=_RESTORE_IDLE,
        ),
    ]


# §10.4 в редакции шага 2. Формула `шагов/с = TMR_Freq / период` — про
# **установившуюся** скорость, и только до 64× сидерических. Две поправки,
# снятые с живой платы (шаг 2 §2.6):
#
# 1. выше порога бита Fast плата разгоняется рампой ~1 с, поэтому мерить надо
#    между двумя точками на ходу, а не от `:J1`. Старая редакция кейса мерила
#    от старта и обвиняла в разнице формулу;
# 2. на периоде 1103 (номинальные 100×) ось идёт не 14 506 шаг/с, а те же
#    ~9 000, что и на 1724. Собственный потолок платы — около 9 000 шаг/с,
#    то есть ~62× сидерических; зажим периода на 1103 до него просто не
#    дотягивается.
_SPEED_RAMP_S = 1.5
_SPEED_MEASURE_S = 2.5
_SPEED_TOLERANCE = 0.06
_TIMER_FREQ = 16_000_000
_SPEED_PERIOD = 1_724
# Измерено дважды независимо (фазы `load` и `ramp` шага 2): 8 847 и ~8 600 шаг/с
# на 1724, 9 015 на 1103. Границы взяты шире разброса и уже, чем расстояние до
# формульных 14 506.
_CEILING_SPS = (8_200, 9_800)


def _speed_case(name: str, mode_command: bytes, mode_note: str) -> Case:
    expected = _TIMER_FREQ / _SPEED_PERIOD * _SPEED_MEASURE_S

    def check(replies: tuple[bytes, ...]) -> str | None:
        # Индексы — по обменам кейса: 4 — `:j1` после разгона, 5 — после мерного окна.
        travelled = (_decode_revu24(replies[5]) - _decode_revu24(replies[4])) % 0x1000000
        if abs(travelled - expected) > expected * _SPEED_TOLERANCE:
            return (
                f"за {_SPEED_MEASURE_S} с установившегося хода ось прошла {travelled} отсчётов, "
                f"а TMR_Freq/период даёт {expected:.0f} ±{_SPEED_TOLERANCE:.0%}"
            )
        return None

    return Case(
        name=name,
        section="§10.4",
        safety=Safety.MOTION,
        steps=(
            Exchange(b":F1\r", exact(b"=\r")),
            Exchange(b":I1" + PERIOD_64X + b"\r", exact(b"=\r")),
            Exchange(mode_command, exact(b"=\r")),
            Exchange(b":J1\r", exact(b"=\r")),
            Pause(_SPEED_RAMP_S),
            Exchange(b":j1\r", matching(_HEX_WORD, "позиция после разгона")),
            Pause(_SPEED_MEASURE_S),
            Exchange(b":j1\r", matching(_HEX_WORD, "позиция в конце мерного окна")),
            Exchange(b":K1\r", exact(b"=\r")),
        ),
        note=f"{mode_note}: установившаяся скорость = TMR_Freq / период, множителя highspeed на этой плате нет",
        check=check,
        teardown=_RESTORE_IDLE,
    )


def _speed_ceiling_case() -> Case:
    """Шаг 2 §2.6: период 1103 не даёт 100× — плата упирается в ~9 000 шаг/с."""

    def check(replies: tuple[bytes, ...]) -> str | None:
        travelled = (_decode_revu24(replies[5]) - _decode_revu24(replies[4])) % 0x1000000
        rate = travelled / _SPEED_MEASURE_S
        if not _CEILING_SPS[0] <= rate <= _CEILING_SPS[1]:
            return (
                f"на периоде 1103 ось идёт {rate:.0f} шаг/с, а шаг 2 намерил "
                f"{_CEILING_SPS[0]}..{_CEILING_SPS[1]} (формула обещала бы 14 506)"
            )
        return None

    return Case(
        name="board_speed_ceiling_below_100x",
        section="§10.5, шаг 2 §2.6",
        safety=Safety.MOTION,
        steps=(
            Exchange(b":F1\r", exact(b"=\r")),
            Exchange(b":I1" + PERIOD_100X + b"\r", exact(b"=\r")),
            Exchange(b":G110\r", exact(b"=\r")),
            Exchange(b":J1\r", exact(b"=\r")),
            Pause(_SPEED_RAMP_S),
            Exchange(b":j1\r", matching(_HEX_WORD, "позиция после разгона")),
            Pause(_SPEED_MEASURE_S),
            Exchange(b":j1\r", matching(_HEX_WORD, "позиция в конце мерного окна")),
            Exchange(b":K1\r", exact(b"=\r")),
        ),
        note="зажим периода на 1103 обещает 100×, но ось идёт ~62× — потолок у платы свой",
        check=check,
        teardown=_RESTORE_IDLE,
    )


def _no_ramp_below_fast_bit_case() -> Case:
    """Шаг 2 §2.6: ниже порога бита Fast разгона нет вовсе.

    На 6 897 первые же 66 мс хода идут на полной скорости 2 320 шаг/с, и `:K1`
    останавливает ось за один опрос. Рампа появляется вместе с битом Fast.
    """

    def check(replies: tuple[bytes, ...]) -> str | None:
        # Обмены кейса: 3 — `:j1` до старта, 5 — `:j1` через полсекунды хода.
        travelled = (_decode_revu24(replies[5]) - _decode_revu24(replies[3])) % 0x1000000
        expected = _TIMER_FREQ / 6_897 * 0.5
        if abs(travelled - expected) > expected * 0.15:
            return f"за первые 0.5 с хода на 16× ось прошла {travelled}, а без разгона было бы {expected:.0f}"
        return None

    return Case(
        name="no_acceleration_ramp_at_16x",
        section="§10.2, шаг 2 §2.6",
        safety=Safety.MOTION,
        steps=(
            Exchange(b":F1\r", exact(b"=\r")),
            Exchange(b":I1" + PERIOD_16X + b"\r", exact(b"=\r")),
            Exchange(b":G110\r", exact(b"=\r")),
            Exchange(b":j1\r", matching(_HEX_WORD, "позиция до старта")),
            Exchange(b":J1\r", exact(b"=\r")),
            Pause(0.5),
            Exchange(b":j1\r", matching(_HEX_WORD, "позиция через полсекунды")),
            Exchange(b":K1\r", exact(b"=\r")),
        ),
        note="без бита Fast плата стартует сразу на полной скорости и останавливается без выбега",
        check=check,
        teardown=_RESTORE_IDLE,
    )


def _brake_point_after_ccw(replies: tuple[bytes, ...]) -> str | None:
    """§3.1 в редакции шага 2: точка торможения отсчитывается от цели.

    После хода CCW она уходит на `+ brake steps` от **цели** — сама цель к тому
    моменту равна позиции, с которой ось стартовала (`:H1` нулевым инкрементом
    в teardown предыдущего кейса), а не той, где ось встала.
    """
    brake_point = _decode_revu24(replies[-2])
    target = _decode_revu24(replies[-1])
    if (brake_point - target) % 0x1000000 != BRAKE_STEPS:
        return f"`:m1 - :h1` = {(brake_point - target) % 0x1000000}, а после хода CCW ожидается +{BRAKE_STEPS}"
    return None


def _target_equals_position(replies: tuple[bytes, ...]) -> str | None:
    target = _decode_revu24(replies[2])
    position = _decode_revu24(replies[3])
    delta = (target - position) % 0x1000000
    if min(delta, 0x1000000 - delta) > POSITION_JITTER:
        return f"`:h1` = 0x{target:06X}, а позиция 0x{position:06X} — разошлись больше чем на дребезг"
    return None


# --------------------------------------------------------------------------- #
# NEVER — what must not reach the live board
# --------------------------------------------------------------------------- #

# name, payload, what the simulator answers, why it never goes to hardware
_NEVER_CASES: tuple[tuple[str, bytes, bytes, str, str], ...] = (
    (
        "never_W_extended_setting",
        b":W1000009\r",
        b"!0\r",
        "§14, план §5",
        "Extended Setting, включая «write flash buffer to flash ROM» — необратимо",
    ),
    (
        "never_W_uart_speed",
        b":W10D8004\r",
        b"!0\r",
        "§13 #1, план §5",
        "смена скорости UART: ошибка оставляет плату на скорости, к которой не подключиться",
    ),
    (
        "never_R_set_register",
        b":R100\r",
        b"!0\r",
        "§14",
        "запись в регистры, назначение которых неизвестно",
    ),
    (
        "never_N_set_eeprom",
        b":N100\r",
        b"!0\r",
        "§14",
        "запись в окно ОЗУ, где лежат оба живых напряжения",
    ),
    (
        "never_Q_bootloader",
        b":Q155AA\r",
        b"!0\r",
        "§14",
        "перевод в загрузчик, откуда плата не отвечает по обычному протоколу",
    ),
    (
        "never_A_set_register_address",
        b":A100\r",
        b"=\r",
        "§3",
        "§3 видел `=`, но `:A1` вне белого списка RA_PROTOCOL_STEP_2 §1.1",
    ),
    (
        "never_B_sleep",
        b":B11\r",
        b"=\r",
        "§3, §7 код 5",
        "принимается с `=`, статус не меняется; вне белого списка шага 2",
    ),
    (
        "never_O_set_polar_led",
        b":O10\r",
        b"=\r",
        "§3",
        "вне белого списка шага 2",
    ),
    (
        "never_P_set_autoguide",
        b":P10\r",
        b"=\r",
        "§3",
        "вне белого списка шага 2",
    ),
    (
        "never_T_set_break_point",
        b":T1000000\r",
        b"=\r",
        "§3",
        "вне белого списка шага 2",
    ),
    (
        "never_U_set_break_step",
        b":U1000000\r",
        b"=\r",
        "§3",
        "вне белого списка шага 2",
    ),
    (
        "never_V_set_led_brightness",
        b":V100\r",
        b"=\r",
        "§3",
        "вне белого списка шага 2",
    ),
    (
        "never_L_instant_stop",
        b":L1\r",
        b"=\r",
        "§3",
        "Instant Stop: рампа не измерена, вне белого списка шага 2",
    ),
    (
        "never_z_debug_flag",
        b":z1\r",
        b"=\r",
        "§3",
        "Set Debug Flag: отвечает `=`, наблюдаемого эффекта нет; вне белого списка шага 2",
    ),
)


def _never_cases() -> list[Case]:
    return [
        Case(
            name=name,
            section=section,
            safety=Safety.NEVER,
            steps=(Exchange(send, exact(reply)),),
            note=note,
        )
        for name, send, reply, section, note in _NEVER_CASES
    ]


def build_cases() -> tuple[Case, ...]:
    """The whole list, in the order a hardware session executes it."""
    cases: list[Case] = []
    cases.extend(_verbatim_cases())
    cases.extend(_position_family_cases())
    cases.append(_status_case())
    cases.append(_brake_point_case())
    cases.append(_brake_increment_is_ignored_case())
    cases.append(_tele_position_is_a_constant_case())
    cases.append(_drifting_extended_id_case())
    cases.append(_unused_letters_case())
    cases.append(_extended_inquire_sweep_case())
    cases.append(_timing_case())
    cases.extend(_framing_cases())
    cases.extend(_window_cases())
    cases.append(_window_sweep_case())
    cases.extend(_clamp_cases())
    cases.extend(_status_bits_cases())
    cases.extend(_motion_cases())
    cases.extend(_never_cases())
    return tuple(cases)


CASES: tuple[Case, ...] = build_cases()
