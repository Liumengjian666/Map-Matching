#!/usr/bin/env python3
"""Identity-only freeze. No ROS, GT or external-disk access."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def identity(path):
    path = Path(path).resolve()
    if not str(path).startswith('/home/jian/'):
        raise ValueError('freeze accepts internal artifacts only')
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return {'bytes': path.stat().st_size, 'sha256': digest.hexdigest()}


def main():
    output = Path(sys.argv[1]).resolve()
    paths = [Path(p).resolve() for p in sys.argv[2:]]
    if output.exists() or not paths:
        raise ValueError('new output and nonempty explicit input list required')
    repo = Path(__file__).resolve().parents[2]
    git = lambda *args: subprocess.check_output(['git', '-C', str(repo), *args], text=True).strip()
    result = {'code_sha': git('rev-parse', 'HEAD'), 'branch': git('branch', '--show-current'),
              'worktree_status': git('status', '--short'),
              'files': {str(path): identity(path) for path in paths},
              'gt_read': False, 'hikvision_access': False}
    with output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print('FROZEN', len(paths), 'artifacts', output)


if __name__ == '__main__':
    main()
