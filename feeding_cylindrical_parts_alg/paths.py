"""工程根路径与默认模型位置。"""

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
MODELS_DIR = PROJECT_ROOT / "models"
DEFAULT_SEG_MODEL = MODELS_DIR / "best.pt"


def resolve_model_path(model_path: str | Path | None = None) -> Path:
    """未指定时使用工程内 models/best.pt。"""
    if model_path is None or str(model_path).strip() == "":
        return DEFAULT_SEG_MODEL
    p = Path(model_path)
    if not p.is_absolute():
        p = PROJECT_ROOT / p
    return p.resolve()
