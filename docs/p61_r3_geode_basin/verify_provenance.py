"""Bind the actual R2 files to immutable Git blobs, not mutable manifests alone."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from prepare_basin import identity

START = '71f12c062c38ff8489124cf8e648269422919776'
REPO = Path(__file__).resolve().parents[2]


def check_bytes(actual, expected):
    if actual != expected: raise ValueError('R2 worktree bytes differ from START_SHA Git blob')


def audit():
    git = lambda *args: subprocess.check_output(['git', '-C', str(REPO), *args])
    paths = git('ls-tree', '-r', '--name-only', START, '--', 'docs/p61_r2_geode_real_data_run').decode().splitlines()
    if not paths: raise ValueError('frozen R2 tree absent')
    files = {}
    for name in paths:
        expected = git('show', START+':'+name)
        check_bytes((REPO/name).read_bytes(), expected)
        files[name] = {'bytes': len(expected), 'sha256': hashlib.sha256(expected).hexdigest()}
    root = REPO/'docs/p61_r2_geode_real_data_run'
    count = 0
    for freeze in ('screen_input_freeze.json', 'screen_output_freeze.json'):
        for name, expected in json.loads((root/freeze).read_text())['files'].items():
            if identity(name) != expected: raise ValueError('anchored frozen asset differs: '+name)
            count += 1
    return {'start_sha': START, 'r2_git_blob_files': len(files), 'frozen_input_output_checks': count,
            'all_exact': True, 'bag_read': False, 'gt_read': False, 'files': files}


if __name__ == '__main__':
    result = audit()
    with Path(sys.argv[1]).open('x') as stream: json.dump(result, stream, indent=2)
    print('GIT_ANCHORED_R2_PASS', result['r2_git_blob_files'], result['frozen_input_output_checks'])
