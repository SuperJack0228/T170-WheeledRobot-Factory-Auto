"""Mirror the C++ root/config/log overrides; importing never touches hardware."""

from dataclasses import dataclass, field
import os
from pathlib import Path


@dataclass(frozen=True)
class RuntimePaths:
    root: Path
    config_dir: Path
    log_dir: Path
    source_files: dict[str, Path] = field(default_factory=dict)

    def config_file(self, name: str) -> Path:
        return self.source_files.get("config/" + name, self.config_dir / name)


def load_paths(root: Path | None = None) -> RuntimePaths:
    selected = root if root is not None else (
        os.environ.get("T170_ROOT") or os.environ.get("MARKET_SIMPLE_ROOT")
        or Path(__file__).resolve().parents[2]
    )
    resource_root = Path(selected).resolve()

    def directory(name: str, default: str) -> Path:
        return (resource_root / (os.environ.get(name) or default)).resolve()

    files = {}
    selector = resource_root / "config/deployment.yaml"
    if not os.environ.get("T170_CONFIG_DIR") and selector.is_file():
        import yaml
        selected = yaml.safe_load(selector.read_text())
        manifest = selector.parent / selected["manifest"]
        doc = yaml.safe_load(manifest.read_text())
        files = {key: resource_root / value for key, value in doc["files"].items()}
    return RuntimePaths(resource_root, directory("T170_CONFIG_DIR", "config"),
                        directory("T170_LOG_DIR", "logs"), files)
