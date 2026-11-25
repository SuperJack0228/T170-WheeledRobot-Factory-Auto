#!/usr/bin/env python3
"""Check installed Linux dependencies without starting the application or SDKs."""
import json
import os
from pathlib import Path
import re
import subprocess
import sys


def check(stage, source):
    stage, source = stage.resolve(), source.resolve()
    environment = dict(os.environ, LD_LIBRARY_PATH=str(stage / 'lib') +
                       (':' + os.environ['LD_LIBRARY_PATH'] if os.environ.get('LD_LIBRARY_PATH') else ''))
    records = {}
    binaries = [stage / 'bin/t170_factory', stage / 'bin/t170_config_dump']
    binaries += sorted((stage / 'lib').glob('*.so*'))
    for binary in binaries:
        result = subprocess.run(['ldd', str(binary)], env=environment, capture_output=True, text=True)
        output = result.stdout + result.stderr
        if result.returncode or 'not found' in output:
            raise ValueError('运行依赖未满足: ' + str(binary) + '\n' + output)
        # RPATH must resolve bundled libraries from the installed tree rather
        # than accidentally succeeding because the source checkout still exists.
        for name in re.findall(r'(?:=>\s+|^\s*)(/[^\n]+?)\s+\(0x', output, re.M):
            path = Path(name).resolve()
            if path.is_relative_to(source) and not path.is_relative_to(stage):
                raise ValueError('运行包仍依赖源码目录: ' + str(path))
        records[str(binary.relative_to(stage))] = output.splitlines()
    return records


def main():
    stage, source, lock = (Path(value).resolve() for value in sys.argv[1:])
    try:
        records = check(stage, source)
        document = json.loads(lock.read_text())
        document['installed_runtime_dependencies'] = records
        lock.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
    except (OSError, ValueError) as error:
        print(str(error), file=sys.stderr); return 2
    print('Installed runtime dependency check passed; no source-tree library dependency.')
    return 0


if __name__ == '__main__': raise SystemExit(main())
