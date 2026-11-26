"""Persistent per-workspace web credentials; explicit overrides remain supported."""
import os
from pathlib import Path
import secrets


def persistent_token(root: Path) -> str:
    path = Path(root) / 'config/.web-token'
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        value = path.read_text().strip()
    except FileNotFoundError:
        value = ''
    if value:
        return value
    token = secrets.token_urlsafe(24)
    try:
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        # Never silently rotate a saved credential, including a damaged file.
        value = path.read_text().strip()
        if not value:
            raise ValueError(f'网页令牌文件为空，请删除后重新启动网页：{path}')
        return value
    with os.fdopen(descriptor, 'w') as stream:
        stream.write(token + '\n')
    return token
