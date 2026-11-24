"""Persistent configuration and a single validated, idle-only apply operation."""
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import uuid

import yaml

from .deployment import resolve


def atomic_write(path, text):
    path = Path(path)
    fd, name = tempfile.mkstemp(prefix="." + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(text); stream.flush(); os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name): os.unlink(name)


def workspace_root(runtime=None):
    if os.environ.get("T170_WORKSPACE"):
        return Path(os.environ["T170_WORKSPACE"]).resolve()
    record = Path(runtime or os.environ.get("T170_RUNTIME_DIR", ".")) / "deployment_resolved.json"
    if record.is_file():
        return Path(json.loads(record.read_text())["resource_root"]).resolve()
    package = Path(__file__).resolve().parents[2]
    if (package / 'release.json').is_file():
        return Path(os.environ.get('XDG_CONFIG_HOME', str(Path.home()/'.config'))) / 't170-factory/workspace'
    return package


class ConfigWorkspace:
    def __init__(self, root, validator=None):
        self.root = Path(root).resolve()
        self.validator = validator

    @contextmanager
    def locked(self):
        with (self.root / ".t170-config.lock").open("a+") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            yield

    def manifest(self):
        selector = yaml.safe_load((self.root / "config/deployment.yaml").read_text())
        value = selector.get("manifest")
        if not value:
            raise ValueError("请先建立并选择机器人档案")
        path = (self.root / "config" / value).resolve()
        if not path.is_relative_to(self.root): raise ValueError("档案路径超出配置工作区")
        return path, yaml.safe_load(path.read_text())

    def source_file(self, file_id):
        _, doc = self.manifest()
        if file_id == "motion":
            name = "config/robots/" + doc["robot_id"] + "/quick_params.yaml"
            if name not in doc["layers"]: raise ValueError("机器人档案缺少 quick_params.yaml")
        else:
            target = {"tray_code": "config/vision/tray.yaml", "belt_code": "config/vision/belt.yaml",
                      "head_geom": "config/head/geometry.yaml", "hand_eye_fallback": "config/camera_to_base_result.yaml"}
            if file_id == "hand_eye":
                geometry = self.source_file("head_geom")
                name = str((geometry.parent / yaml.safe_load(geometry.read_text()).get(
                    "camera_to_head_yaml", "extrinsics.yaml")).relative_to(self.root))
            elif file_id in target: name = doc["files"][target[file_id]]
            else: raise ValueError("未知配置文件")
        path = (self.root / name).resolve()
        if not path.is_relative_to(self.root): raise ValueError("配置路径超出工作区")
        return path

    def checker(self):
        if self.validator: return str(self.validator)
        for directory in ("bin", "build_robot", "build_release"):
            path = self.root / directory / "t170_config_dump"
            if path.is_file(): return str(path)
        raise ValueError("找不到 t170_config_dump；请先编译或安装运行包")

    def apply(self, request=None):
        with self.locked():
            try: return self._apply(request)
            except yaml.YAMLError as error: raise ValueError("YAML 格式错误: " + str(error)) from error

    def save(self, file_id, content, request=None):
        with self.locked():
            return self._save(file_id, content, request)

    def edit(self, file_id, render, request=None):
        # Read-modify-write belongs to one transaction, including field edits.
        with self.locked():
            return self._save(file_id, render(self.source_file(file_id).read_text()), request)

    def _save(self, file_id, content, request):
        path = self.source_file(file_id)
        try: parsed = yaml.safe_load(content)
        except yaml.YAMLError as error: raise ValueError("YAML 格式错误: " + str(error)) from error
        if not isinstance(parsed, dict): raise ValueError("配置必须是 YAML 对象")
        previous = path.read_text()
        atomic_write(path, content)
        try:
            result = self._apply(request)
        except BaseException as error:
            atomic_write(path, previous)
            if isinstance(error, yaml.YAMLError): raise ValueError("YAML 格式错误: " + str(error)) from error
            raise
        result["path"] = str(path)
        return result

    def _apply(self, request):
        manifest, _ = self.manifest()
        manifest_digest = hashlib.sha256(manifest.read_bytes()).hexdigest()
        live = None
        if request:
            try: live = request({"cmd": "ping"}, timeout=3)
            except (OSError, ConnectionError): pass
        runtime = Path(live.get("runtime_root", "")) if live and live.get("ok") else None
        if runtime is not None and (not runtime.is_absolute() or not runtime.is_dir()):
            raise ValueError("主程序未返回有效运行目录，请更新并重启主程序")
        temporary = None
        preserve_candidate = False
        if runtime is None:
            temporary = tempfile.TemporaryDirectory(prefix="t170-validate-")
            candidate = Path(temporary.name) / "runtime"
        else:
            candidate = runtime / (".config-apply-" + uuid.uuid4().hex)
        try:
            record = resolve(self.root, manifest, candidate)
            # Check syntax of all copied YAML/JSON resources, not only motion values.
            for path in (candidate / "config").rglob("*"):
                if path.is_file() and path.suffix in (".yaml", ".yml", ".json"):
                    yaml.safe_load(path.read_text())
            environment = dict(os.environ, T170_ROOT=str(candidate), T170_CONFIG_DIR=str(candidate / "config"))
            check = subprocess.run([self.checker(), str(candidate / "config/effective_params.yaml"),
                                    str(candidate / "effective_config.yaml")], env=environment,
                                   capture_output=True, text=True, timeout=30)
            if check.returncode:
                raise ValueError((check.stderr or check.stdout).strip() or "配置校验失败")
            for name, digest in record["input_sha256"].items():
                if hashlib.sha256((self.root / name).read_bytes()).hexdigest() != digest:
                    raise ValueError("应用期间源配置被修改，请完成编辑后重新应用: " + str(name))
            if hashlib.sha256(manifest.read_bytes()).hexdigest() != manifest_digest:
                raise ValueError("应用期间机器人档案被修改，请重新应用")
            if runtime is None:
                return {"ok": True, "saved": True, "applied": False, "reload": False,
                        "restart_required": True, "code": "validated_offline",
                        "message": "持久配置校验通过，维护或生产服务下次启动时加载"}
            try:
                reply = request({"cmd": "reload_config", "candidate_config": str(candidate / "config/effective_params.yaml")}, timeout=30)
            except (OSError, ConnectionError):
                preserve_candidate = True
                return {"ok": True, "saved": True, "applied": None, "reload": False,
                        "restart_required": True, "code": "apply_unconfirmed",
                        "message": "配置已永久保存，主程序应用结果未确认；请查询状态后重新应用或重启"}
            if reply.get("ok"):
                record["runtime_root"] = str(runtime)
                result = {"ok": True, "saved": True, "applied": True, "reload": True,
                          "restart_required": False, "code": "reloaded", "message": "已永久保存并热更新"}
                try:
                    atomic_write(runtime / "deployment_resolved.json", json.dumps(record, ensure_ascii=False, indent=2) + "\n")
                    atomic_write(runtime / "effective_config.yaml", (candidate / "effective_config.yaml").read_text())
                except OSError as error:
                    # The C++ commit already succeeded. Never roll back its
                    # persistent source because a secondary record write failed.
                    preserve_candidate = True
                    result.update(code="reloaded_metadata_pending", metadata_warning=str(error),
                                  message="已永久保存并热更新；运行记录写入失败，候选快照已保留: " + str(candidate))
                return result
            code = reply.get("code", "reload_failed")
            if code in ("restart_required", "busy", "calibration_active", "stopping"):
                return {"ok": True, "saved": True, "applied": False, "reload": False,
                        "restart_required": True, "code": code,
                        "message": "已永久保存；" + (reply.get("error") or code) + "。可空闲后再次应用，或下次启动加载"}
            raise ValueError(reply.get("error") or "主程序拒绝应用配置")
        finally:
            if temporary: temporary.cleanup()
            elif candidate.exists() and not preserve_candidate: shutil.rmtree(candidate, ignore_errors=True)
