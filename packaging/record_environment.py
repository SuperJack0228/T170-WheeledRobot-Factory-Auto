#!/usr/bin/env python3
"""Record the build host's installed runtime dependencies without changing them."""
import importlib.metadata
import json
import platform
from pathlib import Path
import subprocess
import sys

packages = {}
if platform.system() == 'Linux':
    result = subprocess.run(['dpkg-query', '-W', '-f=${Package}\t${Version}\n'], capture_output=True, text=True)
    packages = dict(line.split('\t', 1) for line in result.stdout.splitlines() if line.startswith('ti5-') and '\t' in line)
record = {'platform': platform.platform(), 'machine': platform.machine(),
          'libc': platform.libc_ver(), 'python': platform.python_version(),
          'python_executable': sys.executable, 'ti5_packages': packages,
          'python_packages': {d.metadata['Name']: d.version for d in importlib.metadata.distributions() if d.metadata['Name']}}
Path(sys.argv[1]).write_text(json.dumps(record, indent=2, sort_keys=True) + '\n')
