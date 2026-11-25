"""Site station groups used as defaults for manual maintenance actions."""
from pathlib import Path
import re
import shutil
import yaml

STATION_FIELDS = ('tray_station', 'belt_station', 'out_station', 'tray2_station')
RUNTIME_FILE = 'config/maintenance_workcells.yaml'


def validate_catalog(data):
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('维护机床组配置 schema_version 必须为 1')
    groups = data.get('groups')
    if not isinstance(groups, list) or not groups:
        raise ValueError('维护机床组 groups 必须为非空列表')
    seen = set()
    for row in groups:
        if not isinstance(row, dict) or set(row) != {'group', *STATION_FIELDS}:
            raise ValueError('每组必须配置 group 及料盘一、传送带一、传送带二、料盘二站点')
        group = row['group']
        if type(group) is not int or group < 1 or group in seen:
            raise ValueError('维护机床组编号必须为不重复的正整数')
        if any(not isinstance(row[key], str) or not re.fullmatch(r'AP[0-9]+', row[key])
               for key in STATION_FIELDS):
            raise ValueError('维护站点名称必须为 AP 加数字')
        seen.add(group)
    selected = data.get('selected_group')
    if type(selected) is not int or selected not in seen:
        raise ValueError('selected_group 不在维护机床组列表中')
    return data


def load_catalog(path):
    return validate_catalog(yaml.safe_load(Path(path).read_text()))


def selected_stations(catalog):
    return next(row for row in catalog['groups'] if row['group'] == catalog['selected_group'])


def migrate_catalog(root, templates, manifest):
    """Called under the upgrade backup/rollback transaction."""
    from .config_workspace import atomic_write
    relative = 'config/sites/' + manifest['site_id'] + '/maintenance_workcells.yaml'
    path = (root / relative).resolve()
    if not path.is_relative_to(root):
        raise ValueError('维护站点配置路径超出工作区')
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(templates / Path(relative).relative_to('config'), path)
    load_catalog(path)
    manifest['files'].setdefault(RUNTIME_FILE, relative)
    # The four task stations now have a single owner: the selected site group.
    quick = root / 'config/robots' / manifest['robot_id'] / 'quick_params.yaml'
    data = yaml.safe_load(quick.read_text())
    chassis = data.get('chassis', {})
    if any(key in chassis for key in STATION_FIELDS):
        for key in STATION_FIELDS:
            chassis.pop(key, None)
        atomic_write(quick, yaml.safe_dump(data, sort_keys=False, allow_unicode=True))
