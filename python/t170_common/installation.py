"""Package-local configuration, with explicit external workspace compatibility."""
from pathlib import Path
import shutil
from .config_workspace import atomic_write

RESOURCE_NAMES = ('bin','lib','assets','python','web_debug','tools','deploy','release.json','environment.lock.json')


def template_config(package):
    """Prefer the visible config directory; accept older package layouts."""
    package = Path(package)
    for directory in (package / 'config', package / 'share/config'):
        if directory.is_dir():
            return directory
    raise ValueError('运行包缺少 config 目录: ' + str(package))


def initialize(package, workspace):
    package, workspace = Path(package).resolve(), Path(workspace).resolve()
    if not (package / 'release.json').is_file(): return
    workspace.mkdir(parents=True, exist_ok=True)
    if not (workspace / 'config').exists():
        shutil.copytree(template_config(package), workspace / 'config')
        atomic_write(workspace / 'config/deployment.yaml', 'manifest: null\n')
    for name in RESOURCE_NAMES:
        source = package / name; target = workspace / name
        if not source.exists(): continue
        if source.resolve() == target.resolve(): continue
        if target.is_symlink():
            if target.resolve() == source.resolve(): continue
            target.unlink()
        elif target.exists(): raise ValueError('拒绝覆盖现有资源目录: ' + str(target))
        target.symlink_to(source, target_is_directory=source.is_dir())


def rotate_reference_pixels(data, sides, width, height):
    for side in sides:
        camera=data.get('cameras', {})
        for suffix, extent in [('u',width),('v',height)]:
            key=side+'_gripper_'+suffix
            if key in camera: camera[key]=extent-1-camera[key]
        for key, profile in data.get('visual_servo', {}).items():
            if key.startswith(side+'_') and 'reference_px' in profile:
                u,v=profile['reference_px'];profile['reference_px']=[width-1-u,height-1-v]
    return data
