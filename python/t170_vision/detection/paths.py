"""工程根路径与默认模型位置。"""

from pathlib import Path
from t170_common.paths import load_paths

PROJECT_ROOT = load_paths().root / "feeding_cylindrical_parts_alg"
MODELS_DIR = PROJECT_ROOT / "models"
DEFAULT_SEG_MODEL = MODELS_DIR / "best1015.pt"


def resolve_model_path(model_path: str | Path | None = None) -> Path:
    """未指定时使用工程内 models/best1015.pt。"""
    if model_path is None or str(model_path).strip() == "":
        return DEFAULT_SEG_MODEL
    p = Path(model_path)
    if not p.is_absolute():
        p = PROJECT_ROOT / p
    return p.resolve()
