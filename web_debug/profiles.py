"""Source deployment profiles. No device, camera or motion imports."""
from __future__ import annotations

import json
import os
from pathlib import Path
import re
import shutil
import tempfile
import threading

import yaml

MODELS = ("t170c_v2_3kg", "t170c_v2_5kg")
_lock = threading.RLock()


def inside(root: Path, relative: str) -> Path:
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError("档案路径超出源码目录")
    return path


def read(path: Path) -> dict:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("档案必须为 YAML 对象: " + str(path))
    return value


def atomic_yaml(path: Path, value: dict) -> None:
    fd, name = tempfile.mkstemp(prefix="." + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(value, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


class ProfileStore:
    def __init__(self, root: Path):
        self.root = root.resolve()
        self.selector = self.root / "config/deployment.yaml"

    def manifest(self, name: str) -> Path:
        path = inside(self.root, name)
        directory = (self.root / "config/deployments").resolve()
        if path.parent != directory or path.suffix != ".yaml" or not path.is_file():
            raise ValueError("请选择现有的机器人档案")
        return path

    def selected(self) -> str:
        value=read(self.selector).get("manifest")
        if not value:return ""
        selected = inside(self.selector.parent, str(value))
        return str(selected.relative_to(self.root))

    def list(self) -> dict:
        with _lock:
            profiles = []
            for path in sorted((self.root / "config/deployments").glob("*.yaml")):
                doc = read(self.manifest(str(path.relative_to(self.root))))
                missing = [name for name in doc["files"].values() if not inside(self.root, name).is_file()]
                identity = read(inside(self.root, "config/robots/" + doc["robot_id"] + "/identity.yaml"))
                profiles.append({"manifest": str(path.relative_to(self.root)), "robot_id": doc["robot_id"],
                                 "robot_model": doc["robot_model"], "host": identity["chassis"]["host"],
                                 "calibration_directory": doc["calibration_directory"], "missing_files": missing})
            return {"ok": True, "selected_manifest": self.selected(), "profiles": profiles,
                    "models": list(MODELS), "source_root": str(self.root)}

    def update(self, payload: dict) -> dict:
        from t170_common.config_workspace import ConfigWorkspace
        with _lock, ConfigWorkspace(self.root).locked():
            if payload.get("expected_manifest") != self.selected():
                raise ValueError("档案选择已被其他页面修改，请刷新后重试")
            model = payload.get("robot_model")
            if model not in MODELS:
                raise ValueError("机型只能选择 T170C V2 3 kg 或 5 kg")
            if payload.get("action") == "create":
                return self.create(payload)
            if payload.get("action") != "select":
                raise ValueError("未知档案操作")
            path = self.manifest(str(payload.get("manifest", "")))
            original = read(path)
            doc = dict(original, robot_model=model)
            # Validate the complete profile with the production materializer;
            # leave the live runtime and its calibration files untouched.
            from t170_common.deployment import resolve
            with tempfile.TemporaryDirectory(prefix="t170-profile-check-") as folder:
                candidate = Path(folder) / "candidate.yaml"
                atomic_yaml(candidate, doc)
                resolve(self.root, candidate, Path(folder) / "runtime")
            atomic_yaml(path, doc)
            try:
                atomic_yaml(self.selector, {"manifest": str(path.relative_to(self.selector.parent))})
            except BaseException:
                atomic_yaml(path, original)
                raise
            return {"ok": True, "restart_required": True, "robot_id": doc["robot_id"], "robot_model": model,
                    "message": "已保存下次启动档案；请停止主服务，再从维护或生产入口启动。"}

    def create(self, payload: dict) -> dict:
        robot_id = str(payload.get("robot_id", ""))
        if not re.fullmatch(r"TH-T170C-\d{4,8}", robot_id):
            raise ValueError("编号格式示例：TH-T170C-0125")
        template = read(self.manifest(str(payload.get("template", ""))))
        old_id = template["robot_id"]
        robots = self.root / "config/robots"
        calibration = self.root / "config/calibration" / robot_id
        manifest = self.root / "config/deployments" / (robot_id + ".yaml")
        destination = robots / robot_id
        if any(path.exists() for path in (destination, calibration, manifest)):
            raise ValueError("该机器人编号已存在，不会覆盖")
        host = str(payload.get("host", "")).strip()
        if not host or len(host) > 253 or not re.fullmatch(r"[a-zA-Z0-9.:-]+", host):
            raise ValueError("请填写底盘 IP 或主机名")
        # Copy reusable IO/tool/pose settings; camera geometry defaults to the
        # common nominal model. Task image references remain robot-specific.
        try:
            shutil.copytree(inside(self.root, "config/robots/" + old_id), destination)
            identity = read(destination / "identity.yaml")
            identity["chassis"].update(sequence_number=robot_id, host=host)
            atomic_yaml(destination / "identity.yaml", identity)
            calibration.mkdir(parents=True)
            (calibration / "tasks").mkdir()
            (calibration / "tasks/.gitkeep").touch()
            doc = json.loads(json.dumps(template).replace("config/robots/" + old_id + "/", "config/robots/" + robot_id + "/")
                             .replace("config/calibration/" + old_id + "/", "config/calibration/" + robot_id + "/"))
            doc.update(robot_id=robot_id, robot_model=payload["robot_model"])
            from t170_common.camera_nominal import nominal_files
            doc['files'].update(nominal_files())
            atomic_yaml(manifest, doc)
        except BaseException:
            shutil.rmtree(destination, ignore_errors=True)
            shutil.rmtree(calibration, ignore_errors=True)
            manifest.unlink(missing_ok=True)
            raise
        return {"ok": True, "manifest": str(manifest.relative_to(self.root)), "restart_required": False,
                "message": "档案已创建，默认使用理论手眼。核对本机相机 SN，选择为下次启动档案后，通过向导建立本机抓放参考。"}
