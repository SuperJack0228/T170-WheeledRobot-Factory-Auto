"""传送带接近路径：相对二维码的示教点，存在 config/conveyor_approach.json。"""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
PATHS_FILE = PROJECT_ROOT / "config" / "conveyor_approach.json"

_NAME_RE = re.compile(r"^[A-Za-z0-9_]{1,40}$")
_POSE_KEYS = ("x", "y", "z", "rx", "ry", "rz")


def _empty_path(name: str) -> dict[str, Any]:
    return {
        "note": "",
        "arm": "left" if "left" in name else "right",
        "side": "left" if "left" in name else "right",
        "include_target": True,
        "marker_id": 0,
        "marker_id_b": 0,
        "dual_pos_thresh_m": 0.015,
        "dual_rot_thresh_deg": 3.0,
        "dual_rel": None,
        "waypoints": [],
    }


def _norm_pose(raw: Any) -> dict[str, float] | None:
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ValueError("位姿必须是 {x,y,z,rx,ry,rz}")
    out = {}
    for key in _POSE_KEYS:
        out[key] = float(raw.get(key) or 0.0)
    return out


def _norm_waypoint(raw: Any) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise ValueError("路径点必须是对象")
    pose = _norm_pose(raw)
    assert pose is not None
    return {"name": str(raw.get("name") or "").strip(), **pose}


def _norm_path(raw: Any) -> dict[str, Any]:
    src = raw if isinstance(raw, dict) else {}
    arm = str(src.get("arm") or "left")
    if arm not in ("left", "right"):
        raise ValueError("路径 arm 只能是 left/right")
    side = str(src.get("side") or "left")
    if side not in ("left", "right", "above"):
        raise ValueError("路径 side 只能是 left/right/above")
    dual = src.get("dual_rel")
    out = {
        "note": str(src.get("note") or ""),
        "arm": arm,
        "side": side,
        "include_target": bool(src.get("include_target", True)),
        "marker_id": int(src.get("marker_id") or 0),
        "marker_id_b": int(src.get("marker_id_b") or 0),
        "dual_pos_thresh_m": float(src.get("dual_pos_thresh_m") if src.get("dual_pos_thresh_m") not in (None, "") else 0.015),
        "dual_rot_thresh_deg": float(src.get("dual_rot_thresh_deg") if src.get("dual_rot_thresh_deg") not in (None, "") else 3.0),
        "dual_rel": _norm_pose(dual) if dual else None,
        "waypoints": [_norm_waypoint(w) for w in (src.get("waypoints") or [])],
    }
    return out


def _default_doc() -> dict[str, Any]:
    return {"paths": {"left_pick": _empty_path("left_pick")}}


def load_doc() -> dict[str, Any]:
    if not PATHS_FILE.is_file():
        return _default_doc()
    try:
        data = json.loads(PATHS_FILE.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return _default_doc()
    paths = data.get("paths") if isinstance(data, dict) else None
    if not isinstance(paths, dict) or not paths:
        return _default_doc()
    clean: dict[str, Any] = {}
    for name, raw in paths.items():
        if not _NAME_RE.match(str(name)):
            continue
        try:
            clean[str(name)] = _norm_path(raw)
        except ValueError:
            continue
    if not clean:
        return _default_doc()
    return {"paths": clean}


def save_doc(doc: dict[str, Any]) -> None:
    paths = doc.get("paths") if isinstance(doc, dict) else None
    if not isinstance(paths, dict):
        raise ValueError("缺少 paths")
    clean = {}
    for name, raw in paths.items():
        if not _NAME_RE.match(str(name)):
            raise ValueError(f"路径名只能是字母数字下划线: {name}")
        clean[str(name)] = _norm_path(raw)
    PATHS_FILE.parent.mkdir(parents=True, exist_ok=True)
    PATHS_FILE.write_text(
        json.dumps({"paths": clean}, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def list_names() -> list[str]:
    return sorted(load_doc()["paths"].keys())


def get_path(name: str) -> dict[str, Any] | None:
    name = (name or "").strip()
    if not name:
        return None
    return load_doc()["paths"].get(name)


def upsert_path(name: str, raw: dict[str, Any] | None = None) -> dict[str, Any]:
    name = (name or "").strip()
    if not _NAME_RE.match(name):
        raise ValueError("路径名只能是字母数字下划线，最长 40")
    doc = load_doc()
    cur = doc["paths"].get(name) or _empty_path(name)
    if raw:
        merged = dict(cur)
        merged.update(raw)
        cur = _norm_path(merged)
    doc["paths"][name] = cur
    save_doc(doc)
    return cur


def delete_path(name: str) -> None:
    name = (name or "").strip()
    doc = load_doc()
    if name not in doc["paths"]:
        raise ValueError(f"没有路径 {name}")
    del doc["paths"][name]
    if not doc["paths"]:
        doc["paths"]["left_pick"] = _empty_path("left_pick")
    save_doc(doc)


def rpc_fields(name: str, *, index: int | None = None, include_target: bool | None = None) -> dict[str, Any]:
    """编排/试走时塞进 conveyor_goto RPC 的字段。index 只走那一个点。"""
    path = get_path(name)
    if path is None:
        raise ValueError(f"没有接近路径 {name}，请先在「路径示教」里创建")
    wps = list(path["waypoints"])
    go_target = path["include_target"] if include_target is None else include_target
    if index is not None:
        if index < 0 or index >= len(wps):
            raise ValueError(f"路径点序号 {index} 超出范围")
        wps = [wps[index]]
        go_target = False
    out: dict[str, Any] = {
        "waypoints": wps,
        "include_target": go_target,
        "marker_id": int(path["marker_id"] or 0),
        "marker_id_b": int(path["marker_id_b"] or 0),
        "dual_pos_thresh_m": float(path["dual_pos_thresh_m"]),
        "dual_rot_thresh_deg": float(path["dual_rot_thresh_deg"]),
    }
    if path.get("dual_rel"):
        out["dual_rel"] = dict(path["dual_rel"])
    return out
