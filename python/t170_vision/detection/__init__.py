"""Lazy public exports: metadata/config imports do not load Torch or YOLO."""
from importlib import import_module
_EXPORTS = {'DEFAULT_ALGORITHM_ID': 'config_loader', 'DEFAULT_CONFIG_PATH': 'config_loader', 'CentroidParams': 'config_loader', 'ClassSpec': 'config_loader', 'PnpParams': 'config_loader', 'PoseParams': 'config_loader', 'SegmentationParams': 'config_loader', 'load_pose_params': 'config_loader', 'DEFAULT_RADIUS_M': 'engine', 'CirclePoseEngine': 'engine', 'open_engine': 'engine', 'ALGO_BOX': 'types', 'ALGO_CENTROID': 'types', 'ALGO_PNP': 'types', 'AlgorithmInput': 'types', 'AlgorithmOutput': 'types', 'TargetPose': 'types', 'DEFAULT_SEG_MODEL': 'paths', 'MODELS_DIR': 'paths', 'PROJECT_ROOT': 'paths', 'resolve_model_path': 'paths'}
__all__ = list(_EXPORTS)
def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(name)
    value = getattr(import_module("." + _EXPORTS[name], __name__), name)
    globals()[name] = value
    return value
