#!/usr/bin/env python3
"""Read-only R7 input audit; prints a receipt, never starts a replay or NDT."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import time

ROOT = Path(__file__).resolve().parents[4]
DATA = Path('/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01')
ADAPTER = Path('/home/jian/livox_ws/superloc_adapter_ws/src/superloc_corridor_adapter')


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def inspect(path, expected):
    if re.fullmatch('[0-9a-f]{64}', expected) is None:
        raise ValueError('A complete historical SHA256 is required')
    row = dict(path=str(path), expected_sha256=expected, actual_sha256=None)
    try:
        row['actual_sha256'] = digest(path)
        row['status'] = 'PASS' if row['actual_sha256'] == expected else 'HASH_MISMATCH'
    except FileNotFoundError:
        row['status'] = 'MISSING'
    except OSError as error:
        row.update(status='READ_ERROR', error=str(error))
    return row


def expected_after(text, marker):
    if text.count(marker) != 1:
        raise ValueError('Historical hash field must occur exactly once: ' + marker)
    lines = text.split(marker, 1)[1].lstrip().splitlines()
    token = lines[0].split()[0] if lines else ''
    if re.fullmatch('[0-9a-f]{64}', token) is None:
        raise ValueError('Missing or malformed historical hash field: ' + marker)
    return token


def audit():
    started = time.monotonic()
    archive = ROOT / 'docs/superloc_corridor01'
    manifest = archive / 'derived_bag_manifest.txt'
    source = archive / 'P2B_ADAPTER_SOURCE_MANIFEST.md'
    text, code = manifest.read_text(), source.read_text()
    specifications = [
        (DATA/'derived/corridor01_adapted_v1.bag', expected_after(text, 'derived bag sha256:')),
        (DATA/'map/corridor01.pcd', expected_after(code, 'official map =')),
        (DATA/'map/derived/corridor01_map_normalized.pcd', expected_after(code, 'normalized map =')),
        (DATA/'calibration/corridor01_extrinsics.yaml', expected_after(text, 'extrinsics sha256 =')),
        (ADAPTER/'config/corridor01_frozen_baseline.yaml', expected_after(text, 'adapter config sha256 =')),
        (DATA/'derived/derived_cloud_hashes.csv', expected_after(text, 'derived cloud hashes:')),
        (DATA/'derived/derived_imu_hashes.csv', expected_after(text, 'derived IMU hashes:')),
        (DATA/'derived/derived_timestamp_audit.json', expected_after(text, 'timestamp audit:')),
    ]
    rows = [inspect(path, expected) for path, expected in specifications]
    return dict(dataset='SuperLoc Corridor01', files=rows,
                expected_hash_sources={str(p): digest(p) for p in (manifest, source)},
                input_files_verified=all(r['status'] == 'PASS' for r in rows),
                scientific_gate_authorized=False,
                note='File audit alone never closes frame/source/replay/chart contracts.',
                gt_pose_loaded=False, new_ndt_calls=0,
                audit_wall_seconds=time.monotonic()-started)


def self_test():
    assert expected_after('x\n' + 'a'*64, 'x') == 'a'*64
    for malformed in ('x\n'+'a'*40+'\ny\n'+'b'*64, 'x\ny '+ 'b'*64,
                      'x', 'x '+ 'a'*64+'\nx '+ 'b'*64, 'no field'):
        try:
            expected_after(malformed, 'x')
        except ValueError:
            pass
        else:
            raise AssertionError('Malformed/duplicate field accepted')
    try:
        inspect(Path(__file__), 'a'*40)
    except ValueError:
        pass
    else:
        raise AssertionError('Truncated SHA accepted')
    path = Path(__file__)
    assert inspect(path, digest(path))['status'] == 'PASS'
    assert inspect(path, '0'*64)['status'] == 'HASH_MISMATCH'
    # A regular file cannot have a child: deterministic read-error fixture.
    assert inspect(path/'child', '0'*64)['status'] == 'READ_ERROR'
    assert inspect(path.parent/'__r7_absent_fixture__', '0'*64)['status'] == 'MISSING'
    print('R7_INPUT_AUDIT_SELF_TEST_PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('audit', 'self-test'))
    args = parser.parse_args()
    if args.action == 'self-test':
        self_test()
    else:
        print(json.dumps(audit(), indent=2))
