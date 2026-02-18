# Architecture rules
- Before implement new functionality, check:
    - Check tests for examples
    - If it's already implemented in other files
    - If you can inherit from or add into existing class
- Don't change anything in references folder
- IGNORE all files in slop_src folder. You can read it ONLY if user say so
- Before starting any changes tell me about plan and what need to be clarified

# Development rules
- Don't create constants for simple values like (0, -1, 0.0), except it has some semantic
- DON'T add any imports in `__init__.py` in ANY module, just import `from lib.module import methods` in other modules
- First, try to use in-file like code style
- For each error create Exception classes structures
- For entities which can converts into some formats, create custom class
- Don't create constants containers (classes/dataclasses like `*CliConstants` or `*Config`) just to group constants/options. Use module-level constants instead.
- CLI flags for argparse must be inline string literals (no `ARG_*` constants or containers).
- Each class must verify consistency, or be designed so consistency is ensured by default
    - validate values for sanity, dont check types
- If you see multiple functions / methods / piece of code add `# TODO: <what should be better here>` for future fixing

- Remember about architecture, you can create and edit ARCHITECHTURE.md in every place if you want to save important piece of big picture
- Use writed methods instead writing new
- Organize it

- When you remove some valriable, or constant, make sure it's removed from all places
- Don't read environment variables in tests

## Generalized rules from recent edits
- Try to generalize common rules from recent edits
- Prefer concise, context-local names over repeating prefixes in enums/types when the module already scopes them.
- Make zero usage explicit with named constants and store allowed option sets as set-like constants.
- If helper logic uses one class API/state, implement it as a method of that class from the start, not as a module-level function.
