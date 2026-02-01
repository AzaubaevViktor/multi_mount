# Development rules
- DON'T add any imports in `__init__.py` in ANY module, just import `from lib.module import methods` in other modules
- First, try to use in-file like code style
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
    - but you can use strings or numbers inplace in:
        - logs (use default notation `("str %s %d", var, var2)`)
        - text for exceptions (use f-strings)
        - Don't make constant for simple numbers like -1, 0, 1, etc
- For each error create Exception classes structures
- For entities which can converts into some formats, use dataclasses
- Don't add dataclasses (or new config containers) just to group constants/options unless explicitly requested.
- Each class must verify consistency, or be designed so consistency is ensured by default
    - check logic, dont check types
- If you see multiple functions / methods / piece of code add `# TODO: <what should be better here>` for future fixing

- Remember about architecture, you can create and edit ARCHITECHTURE.md in every place if you want to save important piece of big picture
- Use writed methods instead writing new
- Organize it

- When you remove some valriable, or constant, make sure it's removed from all places

## Generalized rules from recent edits
- Try to generalize common rules from recent edits
- Prefer concise, context-local names over repeating prefixes in enums/types when the module already scopes them.
- Make zero usage explicit with named constants and store allowed option sets as set-like constants.
