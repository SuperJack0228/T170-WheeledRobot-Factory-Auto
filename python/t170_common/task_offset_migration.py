"""One-time migration of the explicitly approved 2026-11-25 task offsets.

These values are historical migration data, not runtime defaults. After this
revision is recorded, operator changes to quick_params.yaml are preserved.
"""
from pathlib import Path
import yaml
from .config_workspace import atomic_write

REVISION = '20261125_task_offsets'
RIGHT_PLACE_Y_REVISION = '20261125_right_place_y_negative'
LEGACY_HAND_FIELDS = ('use_hand_camera', 'hand_match_xy_max_m',
                      'hand_xy_refine_max_m', 'hand_detect_invalid_redo_max', 'hand_grasp')


def standardize_task_offsets(quick):
    quick.setdefault('tray', {})['z_ref_m'] = -0.623
    grasp = quick.setdefault('head_grasp', {})
    for key in ('goal_x_offset', 'goal_y_offset', 'grasp_z_offset_m',
                'right_goal_x_offset', 'right_goal_y_offset', 'right_grasp_z_offset_m'):
        grasp[key] = 0.0
    place = quick.setdefault('tray2_place', {}).setdefault('offset', {})
    for side in ('left', 'right'):
        place[side] = dict(x=0.0, y=0.0, z=0.0)
    for name in ('conveyor', 'conveyor2'):
        belt = quick.setdefault(name, {})
        belt['grasp_offset'] = {
            'left': dict(x=0.050, y=0.120, z=0.0),
            'right': dict(x=-0.030, y=0.120, z=0.0)}
        belt['offset'] = {
            'left': dict(x=-0.070, y=-0.015, z=0.0),
            'right': dict(x=0.0, y=-0.015, z=0.0)}


def migrate_task_offsets(root, manifest):
    """Called only inside the upgrade transaction, after the full backup."""
    root = Path(root).resolve()

    def inside(name):
        path = (root / name).resolve()
        if not path.is_relative_to(root):
            raise ValueError('参数迁移路径超出工作区: ' + str(name))
        return path

    robot = manifest['robot_id']
    relative = 'config/robots/' + robot + '/quick_params.yaml'
    receipt = inside('config/robots/' + robot + '/parameter_migrations.yaml')
    history = yaml.safe_load(receipt.read_text()) if receipt.is_file() else {'applied': []}
    applied = history['applied']
    if not isinstance(applied, list):
        raise ValueError('parameter_migrations.applied 必须为列表')
    pending = REVISION not in applied
    correct_right_y = RIGHT_PLACE_Y_REVISION not in applied
    if relative not in manifest['layers']:
        raise ValueError('档案缺少本机 quick_params，无法迁移: ' + robot)
    for name in manifest['layers']:
        path = inside(name)
        data = yaml.safe_load(path.read_text())
        before = yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
        if pending and name == relative:
            standardize_task_offsets(data)
        if correct_right_y and name == relative:
            for belt in ('conveyor','conveyor2'):
                data[belt]['offset']['right']['y'] = -0.015
        grasp = data.get('head_grasp', {})
        for key in LEGACY_HAND_FIELDS:
            grasp.pop(key, None)
        # belt0 now means semi-finished; never send it for conveyor2. Preserve
        # explicit machine/business IDs instead of rebinding them to one group.
        second = data.get('conveyor2', {})
        if second.get('prefer_belt') == 'belt0':
            second['prefer_belt'] = 'belt1'
        after = yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
        if before != after:
            atomic_write(path, after)
    if pending:
        applied.append(REVISION)
    if correct_right_y:
        applied.append(RIGHT_PLACE_Y_REVISION)
    if pending or correct_right_y:
        atomic_write(receipt, yaml.safe_dump(history, sort_keys=False, allow_unicode=True))
    return pending or correct_right_y
