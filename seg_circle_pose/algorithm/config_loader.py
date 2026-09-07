"""从 YAML 加载统一参数（分割 / 按类别几何 / 两种算法）。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from paths import PROJECT_ROOT, resolve_model_path

from algorithm.types import ALGO_IDS, ALGO_PNP

DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config" / "pose_params.yaml"
DEFAULT_ALGORITHM_ID = ALGO_PNP


@dataclass
class PnpParams:
    n_samples: int = 12
    reproj_thresh: float = 6.0
    max_reproj_mean: float = 10.0
    min_depth_m: float = 0.02
    min_normal_view_dot: float = 0.05


@dataclass
class CentroidParams:
    mask_erode_kernel: tuple[int, int] = (9, 9)
    min_depth_m: float = 0.05
    min_points: int = 10


@dataclass
class ClassSpec:
    name: str
    class_id: int
    radius_m: float
    enabled: bool = True
    algorithm_id: int = DEFAULT_ALGORITHM_ID
    pnp: PnpParams | None = None
    centroid: CentroidParams | None = None


@dataclass
class SegmentationParams:
    model_path: Path
    conf: float = 0.25
    iou: float = 0.7
    class_ids: list[int] | None = None


@dataclass
class PoseParams:
    """pose_params.yaml 解析结果。"""

    config_path: Path
    default_algorithm_id: int
    segmentation: SegmentationParams
    defaults_radius_m: float
    defaults_enabled: bool
    defaults_algorithm_id: int
    classes_by_id: dict[int, ClassSpec] = field(default_factory=dict)
    classes_by_name: dict[str, ClassSpec] = field(default_factory=dict)
    pnp: PnpParams = field(default_factory=PnpParams)
    centroid: CentroidParams = field(default_factory=CentroidParams)

    def get_class(self, class_id: int) -> ClassSpec | None:
        return self.classes_by_id.get(class_id)

    def radius_m_for(self, class_id: int) -> float:
        spec = self.classes_by_id.get(class_id)
        if spec is not None:
            return spec.radius_m
        return self.defaults_radius_m

    def pnp_for(self, class_id: int) -> PnpParams:
        base = self.pnp
        spec = self.classes_by_id.get(class_id)
        if spec is None or spec.pnp is None:
            return base
        return _merge_dataclass(base, spec.pnp, PnpParams)

    def centroid_for(self, class_id: int) -> CentroidParams:
        base = self.centroid
        spec = self.classes_by_id.get(class_id)
        if spec is None or spec.centroid is None:
            return base
        return _merge_dataclass(base, spec.centroid, CentroidParams)

    def algorithm_id_for(self, class_id: int) -> int:
        spec = self.classes_by_id.get(class_id)
        if spec is not None:
            return spec.algorithm_id
        return self.defaults_algorithm_id

    def enabled_class_ids(self) -> list[int] | None:
        """供 YOLO 过滤；若配置未限定则返回 segmentation.class_ids。"""
        enabled = [s.class_id for s in self.classes_by_id.values() if s.enabled]
        if not enabled:
            return self.segmentation.class_ids
        if self.segmentation.class_ids is not None:
            return [i for i in self.segmentation.class_ids if i in enabled]
        return enabled


def _merge_dataclass(base, override, cls):
    data = {f.name: getattr(base, f.name) for f in cls.__dataclass_fields__.values()}
    for f in cls.__dataclass_fields__:
        v = getattr(override, f.name, None)
        if v is not None:
            data[f.name] = v
    return cls(**data)


def _as_dict(node: Any) -> dict:
    if node is None:
        return {}
    if not isinstance(node, dict):
        raise TypeError(f"expected mapping, got {type(node)}")
    return node


def _load_yaml(path: Path) -> dict:
    try:
        import yaml
    except ImportError as e:
        raise ImportError("加载配置需要 PyYAML: pip install pyyaml") from e
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"invalid config root in {path}")
    return data


def _parse_pnp(node: dict | None) -> PnpParams:
    n = _as_dict(node)
    return PnpParams(
        n_samples=int(n.get("n_samples", 12)),
        reproj_thresh=float(n.get("reproj_thresh", 6.0)),
        max_reproj_mean=float(n.get("max_reproj_mean", 10.0)),
        min_depth_m=float(n.get("min_depth_m", 0.02)),
        min_normal_view_dot=float(n.get("min_normal_view_dot", 0.05)),
    )


def _parse_algorithm_id(value: Any, label: str) -> int:
    aid = int(value)
    if aid not in ALGO_IDS:
        raise ValueError(f"{label}: algorithm_id must be in {ALGO_IDS}, got {aid}")
    return aid


def _parse_centroid(node: dict | None) -> CentroidParams:
    n = _as_dict(node)
    kernel = n.get("mask_erode_kernel", [9, 9])
    if isinstance(kernel, (list, tuple)) and len(kernel) == 2:
        k = (int(kernel[0]), int(kernel[1]))
    else:
        k = (9, 9)
    return CentroidParams(
        mask_erode_kernel=k,
        min_depth_m=float(n.get("min_depth_m", 0.05)),
        min_points=int(n.get("min_points", 10)),
    )


def load_pose_params(config_path: str | Path | None = None) -> PoseParams:
    path = Path(config_path) if config_path else DEFAULT_CONFIG_PATH
    if not path.is_absolute():
        path = (PROJECT_ROOT / path).resolve()
    if not path.is_file():
        raise FileNotFoundError(f"参数文件不存在: {path}")

    root = _load_yaml(path)
    algo_node = _as_dict(root.get("algorithm"))
    default_algorithm_id = _parse_algorithm_id(
        algo_node.get("default_id", DEFAULT_ALGORITHM_ID),
        "algorithm.default_id",
    )
    seg_node = _as_dict(root.get("segmentation"))
    defaults_node = _as_dict(root.get("defaults"))
    classes_node = _as_dict(root.get("classes"))
    pnp_global = _parse_pnp(root.get("algorithm_pnp"))
    centroid_global = _parse_centroid(root.get("algorithm_centroid"))

    model_raw = seg_node.get("model", "models/best.pt")
    model_path = resolve_model_path(model_raw)

    class_ids = seg_node.get("class_ids")
    if class_ids is not None:
        class_ids = [int(x) for x in class_ids]

    segmentation = SegmentationParams(
        model_path=model_path,
        conf=float(seg_node.get("conf", 0.25)),
        iou=float(seg_node.get("iou", 0.7)),
        class_ids=class_ids,
    )

    defaults_radius_m = float(defaults_node.get("radius_m", 0.025))
    defaults_enabled = bool(defaults_node.get("enabled", True))
    defaults_algorithm_id = _parse_algorithm_id(
        defaults_node.get("algorithm_id", default_algorithm_id),
        "defaults.algorithm_id",
    )

    by_id: dict[int, ClassSpec] = {}
    by_name: dict[str, ClassSpec] = {}
    seen_ids: set[int] = set()

    for name, spec_node in classes_node.items():
        sn = _as_dict(spec_node)
        cid = int(sn["class_id"])
        if cid in seen_ids:
            raise ValueError(f"duplicate class_id={cid} in classes.{name}")
        seen_ids.add(cid)
        pnp_override = _parse_pnp(sn["pnp"]) if "pnp" in sn else None
        centroid_override = _parse_centroid(sn["centroid"]) if "centroid" in sn else None
        spec = ClassSpec(
            name=str(name),
            class_id=cid,
            radius_m=float(sn.get("radius_m", defaults_radius_m)),
            enabled=bool(sn.get("enabled", defaults_enabled)),
            algorithm_id=_parse_algorithm_id(
                sn.get("algorithm_id", defaults_algorithm_id),
                f"classes.{name}.algorithm_id",
            ),
            pnp=pnp_override,
            centroid=centroid_override,
        )
        by_id[cid] = spec
        by_name[name] = spec

    return PoseParams(
        config_path=path,
        default_algorithm_id=default_algorithm_id,
        segmentation=segmentation,
        defaults_radius_m=defaults_radius_m,
        defaults_enabled=defaults_enabled,
        defaults_algorithm_id=defaults_algorithm_id,
        classes_by_id=by_id,
        classes_by_name=by_name,
        pnp=pnp_global,
        centroid=centroid_global,
    )
