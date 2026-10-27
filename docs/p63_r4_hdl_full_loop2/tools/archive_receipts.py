#!/usr/bin/env python3
"""Preserve sealed receipts and verify unchanged native assets; no replay or deletion."""
import gzip
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

A = Path(__file__).resolve().parents[1]
W = A.parents[1]
D = Path('/home/jian/livox_ws/p63_r4_hdl_full_loop2_data/full_trial_01')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert (D/'RUN_RECEIPT.json').exists(), 'The trial must be sealed first.'
    target = A/'raw'
    target.mkdir(exist_ok=True)
    manifest = []
    for name in ('NATIVE_NDT.csv', 'CALLBACKS.csv', 'ADAPTER.csv', 'CAUSAL_RESULTS.csv',
                 'ROS_RECEIPTS.jsonl', 'MEMORY.csv', 'ROSLAUNCH.log'):
        source = D/name
        archive = target/(name+'.gz')
        with archive.open('wb') as stream:
            with gzip.GzipFile(filename='', fileobj=stream, mode='wb', mtime=0) as zipped:
                zipped.write(source.read_bytes())
        with gzip.open(archive, 'rb') as stream:
            assert hashlib.sha256(stream.read()).hexdigest() == sha(source)
        manifest.append({'original': str(source), 'bytes': source.stat().st_size,
                         'sha256': sha(source), 'archive': str(archive.relative_to(A)),
                         'archive_bytes': archive.stat().st_size, 'archive_sha256': sha(archive),
                         'decompressed_identity_verified': True})
    for name in ('COMMAND.json', 'EFFECTIVE_PARAMS.json', 'PUBLISHED_MAP.json', 'RUN_RECEIPT.json',
                 'LIVE_20.png', 'STOP_3377.png'):
        source = D/name
        shutil.copyfile(source, target/name)
        manifest.append({'original': str(source), 'bytes': source.stat().st_size,
                         'sha256': sha(source), 'archive': str((target/name).relative_to(A)),
                         'archive_sha256': sha(target/name)})
    (A/'evidence/RAW_ARCHIVE_MANIFEST.json').write_text(json.dumps(manifest, indent=2)+'\n')
    freeze = json.loads((A/'SOURCE_FREEZE.json').read_text())
    binaries = {p: {'actual_sha256': sha(Path(p)), 'expected_sha256': v['sha256'],
                    'unchanged': sha(Path(p)) == v['sha256']}
                for p, v in freeze['reused_binary_identity'].items()}
    upstream = {}
    for name, frozen in freeze['upstream'].items():
        path = Path('/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/src')/name
        actual = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=path, text=True).strip()
        status = subprocess.check_output(['git', '--no-optional-locks', 'status', '--porcelain'], cwd=path, text=True)
        upstream[name] = {'HEAD': actual, 'pristine': not status, 'same_as_frozen': actual == frozen['SHA']}
    recorded_sources = {p: sha(W/p) == expected for p, expected in freeze['source'].items()}
    assert all(v['unchanged'] for v in binaries.values())
    assert all(v['pristine'] and v['same_as_frozen'] for v in upstream.values())
    assert all(recorded_sources.values())
    after = {'binaries': binaries, 'upstream': upstream, 'running_sources_unchanged': recorded_sources,
             'native_rebuild': False, 'original_core_modifications': 0}
    (A/'evidence/NATIVE_AFTER.json').write_text(json.dumps(after, indent=2)+'\n')
    print('PASS: exact decompression, all six reused libraries, four pristine upstreams and running controller source identity.')


if __name__ == '__main__':
    main()
