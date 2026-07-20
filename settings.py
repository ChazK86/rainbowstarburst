"""Small persistent preference store with privacy-preserving defaults."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path


@dataclass
class AppSettings:
    audio_source: str = "OFF"
    audio_program: str = "CINEMA"
    system_device_id: str | None = None
    microphone_device_id: str | None = None
    sensitivity: float = 1.0
    reactivity: float = 1.0
    reduced_motion: bool = False


def settings_path() -> Path:
    root = Path(os.environ.get("LOCALAPPDATA", Path.home()), "RainbowStarburst")
    return root / "settings.json"


def load_settings() -> AppSettings:
    path = settings_path()
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        allowed = AppSettings.__dataclass_fields__
        return AppSettings(**{key: value for key, value in data.items() if key in allowed})
    except (OSError, ValueError, TypeError):
        return AppSettings()


def save_settings(settings: AppSettings) -> None:
    path = settings_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(asdict(settings), indent=2), encoding="utf-8")
    temporary.replace(path)
