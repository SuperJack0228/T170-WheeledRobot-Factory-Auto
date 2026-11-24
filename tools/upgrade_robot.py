#!/usr/bin/env python3
"""Upgrade an installed workspace with a config backup and explicit migrations."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import uuid
import yaml

PACKAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE/'python'))
from t170_common.config_workspace import ConfigWorkspace, atomic_write, workspace_root
from t170_common.installation import initialize, RESOURCE_NAMES, rotate_reference_pixels


def upgrade(package, root, backup_parent):
    package,root=Path(package).resolve(),Path(root).resolve()
    new=json.loads((package/'release.json').read_text())
    workspace=ConfigWorkspace(root)
    with workspace.locked():
        old=json.loads((root/'release.json').read_text()) if (root/'release.json').exists() else {}
        old_version=old.get('version','unknown')
        if old_version=='unknown': raise ValueError('无法识别原安装版本；请先确认工作区 release.json')
        stamp=datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]
        backup=Path(backup_parent)/stamp;backup.mkdir(parents=True)
        shutil.copytree(root/'config',backup/'config',symlinks=True)
        (backup/'release.json').write_text(json.dumps(old,indent=2)+'\n')
        links={name:os.readlink(root/name) if (root/name).is_symlink() else None for name in RESOURCE_NAMES}
        retired=[];changed_robots=[]
        try:
            for manifest in sorted((root/'config/deployments').glob('*.yaml')):
                doc=yaml.safe_load(manifest.read_text());robot=doc['robot_id']
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
            initialize(package,root)
            result=workspace._apply(None) # Already holds the shared workspace lock.
        except BaseException:
            shutil.rmtree(root/'config');shutil.copytree(backup/'config',root/'config',symlinks=True)
            for name,previous in links.items():
                target=root/name
                if target.is_symlink():target.unlink()
                if previous is not None:target.symlink_to(previous)
            raise
        result.update(version=new['version'],previous_version=old_version,backup=str(backup),
                      camera_orientation_changed=changed_robots,retired_task_references=retired,
                      message='升级完成，未启动硬件。图像方向改变的机器人需通过向导重新建立抓放参考。')
        try:atomic_write(backup/'upgrade_result.json',json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        except OSError as error:result['record_warning']=str(error)
        return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path)
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
        result=upgrade(PACKAGE,args.workspace or workspace_root(),state/'config_backups')
        print(json.dumps(result,ensure_ascii=False,indent=2));return 0
    except (OSError,ValueError,yaml.YAMLError,subprocess.SubprocessError) as error:
        print(str(error),file=sys.stderr);return 2


if __name__=='__main__':raise SystemExit(main())
