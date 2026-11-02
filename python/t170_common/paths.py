"""Mirror the C++ root/config/log overrides; importing never touches hardware."""

from dataclasses import dataclass
import os
from pathlib import Path


@dataclass(frozen=True)
class RuntimePaths:
    root: Path
    config_dir: Path
    log_dir: Path


def load_paths(root: Path | None = None) -> RuntimePaths:
    selected = root if root is not None else (
        os.environ.get("T170_ROOT") or os.environ.get("MARKET_SIMPLE_ROOT")
        or Path(__file__).resolve().parents[2]
    )
    resource_root = Path(selected).resolve()

    def directory(name: str, default: str) -> Path:
        return (resource_root / (os.environ.get(name) or default)).resolve()

    return RuntimePaths(resource_root, directory("T170_CONFIG_DIR", "config"),
                        directory("T170_LOG_DIR", "logs"))
