"""Step-1 legacy entry point; implementation lives in t170_vision.detection.segmentation_yolo."""
import sys as _sys
from pathlib import Path as _Path
from importlib import import_module as _import_module
_package_root = str(_Path(__file__).resolve().parents[2] / "python")
if _package_root not in _sys.path:
    _sys.path.insert(0, _package_root)
_impl = _import_module("t170_vision.detection.segmentation_yolo")
_sys.modules[__name__] = _impl
