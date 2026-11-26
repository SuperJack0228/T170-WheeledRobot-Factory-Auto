"""Robot Web endpoints and private credentials; no device or motion imports."""
import ipaddress
from pathlib import Path
import re
import threading

import yaml

from .config_workspace import atomic_write, ConfigWorkspace

_lock = threading.RLock()


def canonical_robot_id(value):
    value = str(value).strip()
    if value.isdecimal():
        value = 'TH-T170C-' + value.zfill(4)
    if not re.fullmatch(r'TH-T170C-\d{4,8}', value):
        raise ValueError('机器人编号示例：120 或 TH-T170C-0120')
    return value


def connection_settings(robot_id, host='', port=8080):
    robot_id = canonical_robot_id(robot_id)
    if not host:
        number = int(robot_id.rsplit('-', 1)[1])
        if not 1 <= number <= 254:
            raise ValueError('编号不在 1–254 内，请单独填写上位机 IP')
        host = f'192.168.113.{number}'
    host = str(ipaddress.IPv4Address(str(host).strip()))
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError('网页端口必须在 1–65535 内')
    return dict(robot_id=robot_id, host=host, port=port)


def read_yaml(path, default):
    if not path.is_file():
        return default
    value = yaml.safe_load(path.read_text())
    if not isinstance(value, dict):
        raise ValueError('连接配置应为 YAML 对象：' + str(path))
    return value


class FleetStore:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.file = self.root / 'config/fleet.yaml'
        self.secrets = self.root / 'config/.fleet-tokens.yaml'

    def connections(self):
        result = {}
        for path in sorted((self.root / 'config/deployments').glob('*.yaml')):
            profile = read_yaml(path, {})
            robot_id = canonical_robot_id(profile['robot_id'])
            connection = read_yaml(self.root / 'config/robots' / robot_id / 'connection.yaml', {})
            result[robot_id] = connection_settings(robot_id, connection.get('host', ''), connection.get('port', 8080))
            result[robot_id]['robot_model'] = profile.get('robot_model', '')
        for robot_id, connection in read_yaml(self.file, {}).get('robots', {}).items():
            robot_id = canonical_robot_id(robot_id)
            result[robot_id] = dict(result.get(robot_id, {}), **connection_settings(
                robot_id, connection.get('host', ''), connection.get('port', 8080)))
        return result

    def token(self, robot_id):
        return read_yaml(self.secrets, {}).get('tokens', {}).get(canonical_robot_id(robot_id), '')

    def public_list(self):
        with _lock:
            tokens = read_yaml(self.secrets, {}).get('tokens', {})
            return [dict(item, token_saved=bool(tokens.get(robot_id)))
                    for robot_id, item in sorted(self.connections().items())]

    def save(self, payload):
        connection = connection_settings(payload.get('robot_id', ''), payload.get('host', ''), payload.get('port', 8080))
        token = str(payload.get('token', '')).strip()
        if len(token) > 512 or '\r' in token or '\n' in token:
            raise ValueError('访问令牌格式不正确')
        with _lock, ConfigWorkspace(self.root).locked():
            doc = read_yaml(self.file, {})
            doc.setdefault('robots', {})[connection['robot_id']] = connection
            atomic_write(self.file, yaml.safe_dump(doc, sort_keys=False))
            if token:
                secret = read_yaml(self.secrets, {})
                secret.setdefault('tokens', {})[connection['robot_id']] = token
                atomic_write(self.secrets, yaml.safe_dump(secret, sort_keys=False))
        return connection
