#!/usr/bin/env python3
"""Upgrade an installed workspace with a config backup and explicit migrations."""
import argparse
from contextlib import ExitStack
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import uuid
import yaml

PACKAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE/'python'))
from t170_common.config_workspace import ConfigWorkspace, atomic_write, workspace_root
from t170_common.installation import initialize, RESOURCE_NAMES, rotate_reference_pixels, template_config
from t170_common.task_offset_migration import migrate_task_offsets
from t170_common.maintenance_workcells import migrate_catalog


def migrate(root, templates, old_version, stamp):
    retired=[];changed_robots=[];offset_robots=[]
    for manifest in sorted((root/'config/deployments').glob('*.yaml')):
        doc=yaml.safe_load(manifest.read_text());robot=doc['robot_id']
        migrate_catalog(root,templates,doc)
        if migrate_task_offsets(root,doc):offset_robots.append(robot)
        # Site data is shared by every robot and independent of its ID.
        relative='config/sites/'+doc['site_id']+'/conveyor_heights.yaml'
        target=(root/relative).resolve()
        if not target.is_relative_to(root):raise ValueError('站点高度表路径超出工作区')
        if not target.exists():
            target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(templates/Path(relative).relative_to('config'),target)
        doc['files'].setdefault('config/conveyor_heights.yaml',relative)
        atomic_write(manifest,yaml.safe_dump(doc,sort_keys=False,allow_unicode=True))
        belt_file=(root/doc['files']['config/vision/belt.yaml']).resolve()
        if not belt_file.is_relative_to(root):raise ValueError('传送带配置路径超出工作区')
        belt=yaml.safe_load(belt_file.read_text())
        belt['belt']['height_catalog_file']='../conveyor_heights.yaml'
        atomic_write(belt_file,yaml.safe_dump(belt,sort_keys=False,allow_unicode=True))
        camera_file=(root/doc['files']['config/realsense_cameras.yaml']).resolve()
        if not camera_file.is_relative_to(root):raise ValueError('相机配置超出工作区')
        cameras=yaml.safe_load(camera_file.read_text());changed=[]
        # rc.1 used the historical left=180/right=0 convention for all
        # robots. Missing keys in later releases mean standard defaults.
        for side,standard,legacy in [('left',0,180),('right',180,0)]:
            node=cameras['realsense'][side+'_hand']
            before=legacy if old_version=='1.0.0-rc.1' else node.get('image_rotation_deg',standard)
            after=node.get('image_rotation_deg',legacy if robot=='TH-T170C-0124' else standard)
            if before!=after:changed.append(side)
            node['image_rotation_deg']=after
        atomic_write(camera_file,yaml.safe_dump(cameras,sort_keys=False,allow_unicode=True))
        if not changed:continue
        changed_robots.append(robot)
        dimensions={'width':1280,'height':720}
        for name in doc['layers']:
            layer=yaml.safe_load((root/name).read_text()) or {}
            dimensions.update({k:v for k,v in layer.get('cameras',{}).items() if k in dimensions})
        pixels=root/'config/robots'/robot/'reference_pixels.yaml'
        if pixels.is_file():
            data=rotate_reference_pixels(yaml.safe_load(pixels.read_text()),changed,dimensions['width'],dimensions['height'])
            atomic_write(pixels,yaml.safe_dump(data,sort_keys=False,allow_unicode=True))
        # New axes invalidate old task K/pixels/eye fingerprints. Keep
        # originals available, but never load them as current references.
        tasks=(root/doc['calibration_directory']).resolve()
        if not tasks.is_relative_to(root):raise ValueError('标定目录超出工作区')
        for task in ('grasp','placement'):
            for side in changed:
                path=tasks/task/(side+'.yaml')
                if path.is_file():
                    target=tasks/('orientation_before_'+stamp)/task/path.name
                    target.parent.mkdir(parents=True,exist_ok=True);path.rename(target)
                    retired.append(str(path.relative_to(root)))
    return changed_robots, retired, offset_robots


def upgrade(package, root, backup_parent, source=None):
    package, root = Path(package).resolve(), Path(root).resolve()
    source = Path(source).resolve() if source is not None else root
    new = json.loads((package / 'release.json').read_text())
    if root != source and (root.is_relative_to(source) or source.is_relative_to(root)):
        raise ValueError('新版目录与旧配置目录不能互相包含')
    if not (source / 'release.json').is_file() or not (source / 'config/deployment.yaml').is_file():
        raise ValueError('找不到旧配置；请使用 --from-workspace /旧运行包或旧工作区目录')
    root.mkdir(parents=True, exist_ok=True)
    with ExitStack() as locks:
        # Lock source and destination in a deterministic order; imports must not
        # race a web save in the old workspace.
        for directory in sorted({root, source}, key=str):
            locks.enter_context(ConfigWorkspace(directory).locked())
        try:
            ConfigWorkspace(source).manifest()  # A fresh package is not a source.
        except ValueError as error:
            raise ValueError('旧配置未选择机器人档案；首次升级请使用 '
                             '--from-workspace /旧运行包或旧工作区目录') from error
        old = json.loads((source / 'release.json').read_text())
        old_version = old.get('version', 'unknown')
        if old_version == 'unknown':
            raise ValueError('无法识别原安装版本；请确认旧目录的 release.json')
        stamp = datetime.now().strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:6]
        backup = Path(backup_parent) / stamp
        backup.mkdir(parents=True)
        shutil.copytree(source / 'config', backup / 'config', symlinks=True)
        (backup / 'release.json').write_text(json.dumps(old, indent=2) + '\n')
        links = {name: os.readlink(root / name) if (root / name).is_symlink() else None
                 for name in RESOURCE_NAMES}
        with tempfile.TemporaryDirectory(prefix='t170-upgrade-') as temporary:
            original = Path(temporary) / 'config'
            had_config = (root / 'config').exists()
            if had_config:
                shutil.copytree(root / 'config', original, symlinks=True)
            templates = original if root == package and had_config else template_config(package)
            try:
                if source != root:
                    if had_config:
                        shutil.rmtree(root / 'config')
                    shutil.copytree(source / 'config', root / 'config', symlinks=True)
                changed_robots, retired, offset_robots = migrate(root, templates, old_version, stamp)
                initialize(package, root)
                result = ConfigWorkspace(root)._apply(None)
            except BaseException:
                if (root / 'config').exists():
                    shutil.rmtree(root / 'config')
                if had_config:
                    shutil.copytree(original, root / 'config', symlinks=True)
                for name, previous in links.items():
                    target = root / name
                    if target.is_symlink():
                        target.unlink()
                    if previous is not None:
                        target.symlink_to(previous)
                raise
        result.update(version=new['version'], previous_version=old_version, backup=str(backup),
                      workspace=str(root), config=str(root / 'config'), source_workspace=str(source),
                      camera_orientation_changed=changed_robots, retired_task_references=retired,
                      task_offsets_migrated=offset_robots,
                      message='升级完成，未启动硬件。图像方向改变的机器人需通过向导重新建立抓放参考。')
        try:
            atomic_write(backup / 'upgrade_result.json', json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        except OSError as error:
            result['record_warning'] = str(error)
        return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,help='目标配置目录，默认是当前运行包根目录')
    parser.add_argument('--from-workspace',type=Path,help='导入旧运行包或旧持久工作区，原目录保持不变')
    parser.add_argument('--robot-port',type=int,default=8099)
    parser.add_argument('--web-port',type=int,default=8080)
    args=parser.parse_args()
    try:
        if not (PACKAGE/'release.json').is_file():raise ValueError('请从解压后的新版运行包执行升级入口')
        for name,port in [('主程序',args.robot_port),('网页',args.web_port)]:
            try:
                with socket.create_connection(('127.0.0.1',port),timeout=.5):
                    raise ValueError(name+'端口仍在监听，请先停止主程序和网页，再升级')
            except OSError:pass
        state=Path(os.environ.get('T170_STATE_DIR',str(Path(os.environ.get('XDG_STATE_HOME',str(Path.home()/'.local/state')))/'t170')))
        result=upgrade(PACKAGE,args.workspace or workspace_root(),state/'config_backups',args.from_workspace)
        print(json.dumps(result,ensure_ascii=False,indent=2));return 0
    except (OSError,ValueError,yaml.YAMLError,subprocess.SubprocessError) as error:
        print(str(error),file=sys.stderr);return 2


if __name__=='__main__':raise SystemExit(main())
