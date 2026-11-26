#!/usr/bin/env python3
"""Initialize a persistent robot workspace. Never initializes hardware."""
import argparse
from pathlib import Path
import json
import sys
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(PACKAGE), str(PACKAGE / 'python')]
from t170_common.config_workspace import atomic_write, workspace_root
from web_debug.profiles import ProfileStore, read
from t170_common.installation import initialize
from t170_common.fleet import connection_settings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('robot_id')
    parser.add_argument('--model', choices=('3', '5'), required=True)
    parser.add_argument('--chassis-host', default='192.168.192.5')
    parser.add_argument('--head-serial', default='')
    parser.add_argument('--left-serial', default='')
    parser.add_argument('--right-serial', default='')
    parser.add_argument('--workspace', type=Path)
    parser.add_argument('--web-host', default='', help='上位机 IP，默认 192.168.113.编号')
    parser.add_argument('--web-port', type=int, default=None)
    args = parser.parse_args()
    try:
        root = (args.workspace or workspace_root()).resolve(); initialize(PACKAGE, root)
        store = ProfileStore(root); manifest = root / 'config/deployments' / (args.robot_id + '.yaml')
        model = 't170c_v2_' + args.model + 'kg'
        if not manifest.is_file():
            store.update({'action': 'create', 'expected_manifest': store.selected(),
                          'template': 'config/deployments/TH-T170C-0124.yaml',
                          'robot_id': args.robot_id, 'robot_model': model, 'host': args.chassis_host,
                          'web_host':args.web_host,'web_port':args.web_port if args.web_port is not None else 8080})
            camera_file = root / 'config/robots' / args.robot_id / 'cameras/realsense_cameras.yaml'
            cameras = read(camera_file)
            for name, serial in [('head', args.head_serial), ('left_hand', args.left_serial), ('right_hand', args.right_serial)]:
                cameras['realsense'][name]['serial'] = serial
            atomic_write(camera_file, yaml.safe_dump(cameras, sort_keys=False, allow_unicode=True))
        connection_file=root/'config/robots'/args.robot_id/'connection.yaml'
        if not connection_file.is_file() or args.web_host or args.web_port is not None:
            previous=read(connection_file) if connection_file.is_file() else {}
            connection=connection_settings(args.robot_id,args.web_host or previous.get('host',''),
                args.web_port if args.web_port is not None else previous.get('port',8080))
            atomic_write(connection_file,yaml.safe_dump(connection,sort_keys=False))
        (root / 'config/calibration' / args.robot_id / 'tasks').mkdir(parents=True, exist_ok=True)
        result = store.update({'action': 'select', 'expected_manifest': store.selected(),
                               'manifest': str(manifest.relative_to(root)), 'robot_model': model})
        result.update(workspace=str(root), quick_params=str(root / 'config/robots' / args.robot_id / 'quick_params.yaml'))
        connection=read(connection_file)
        result.update(web_host=connection['host'],web_port=connection['port'],
            web_url=f"http://{connection['host']}:{connection['port']}/")
        print(json.dumps(result, ensure_ascii=False, indent=2)); return 0
    except (OSError, ValueError) as error:
        print(str(error), file=sys.stderr); return 2


if __name__ == '__main__': raise SystemExit(main())
