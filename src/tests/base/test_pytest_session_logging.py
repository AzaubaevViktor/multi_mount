import importlib.util
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace


MODULE_PATH = Path(__file__).with_name("conftest.py")
MODULE_SPEC = importlib.util.spec_from_file_location(
    "multi_mount_test_base_conftest",
    MODULE_PATH,
)
assert MODULE_SPEC is not None
assert MODULE_SPEC.loader is not None
SESSION_LOGGING_CONFTEST = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(SESSION_LOGGING_CONFTEST)


def test_build_unique_session_dir_uses_date_and_time_levels(tmp_path, monkeypatch):
    class FixedDateTime:
        @classmethod
        def now(cls) -> datetime:
            return datetime(2026, 3, 14, 17, 5, 46)

    monkeypatch.setattr(SESSION_LOGGING_CONFTEST, "datetime", FixedDateTime)

    session_dir = SESSION_LOGGING_CONFTEST._build_unique_session_dir(tmp_path)

    assert session_dir == tmp_path / "2026-03-14" / "17-05-46"


def test_build_unique_session_dir_adds_suffix_within_date_directory(tmp_path, monkeypatch):
    class FixedDateTime:
        @classmethod
        def now(cls) -> datetime:
            return datetime(2026, 3, 14, 17, 5, 46)

    monkeypatch.setattr(SESSION_LOGGING_CONFTEST, "datetime", FixedDateTime)
    existing_session_dir = tmp_path / "2026-03-14" / "17-05-46"
    existing_session_dir.mkdir(parents=True)

    session_dir = SESSION_LOGGING_CONFTEST._build_unique_session_dir(tmp_path)

    assert session_dir == tmp_path / "2026-03-14" / "17-05-46_01"


def test_pytest_configure_skips_plugin_registration_for_collect_only(monkeypatch):
    register_calls: list[tuple[object, str]] = []

    class PluginManager:
        def has_plugin(self, name: str) -> bool:
            return False

        def register(self, plugin: object, name: str) -> None:
            register_calls.append((plugin, name))

    plugin_inits: list[object] = []

    class FakeSessionLogsPlugin:
        def __init__(self, config: object) -> None:
            plugin_inits.append(config)

    monkeypatch.setattr(
        SESSION_LOGGING_CONFTEST,
        "SessionLogsPlugin",
        FakeSessionLogsPlugin,
    )
    config = SimpleNamespace(
        option=SimpleNamespace(collectonly=True),
        pluginmanager=PluginManager(),
    )

    SESSION_LOGGING_CONFTEST.pytest_configure(config)

    assert not plugin_inits
    assert not register_calls
