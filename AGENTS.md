# Development rules
- No magic strings or numbers in logic code;
    - For numeric constants use class with classvalues
        ```python
        class Constants:
            MIN_IN_HOUR = 60
            TICK_PER_ROTATION = 1000
        ```
    - For commands/ids use Enums
        ```python
        class Directions(IntEnum):
            FORWARD = 0
            BACKWARD = 1
        class Commands(StrEnum):
            RESER = "rst"
            GET_DATA = "gtdt"
        ```
    - but you can use strings inplace in:
        - logs (use default notation `("str %s %d", var, var2)`)
        - text for exceptions (use f-strings)
- For each error create Exception classes structures
- For entities which can converts into some formats, use dataclasses
- Each class must verify consistency, or be designed so consistency is ensured by default
    - check logic, dont check types
