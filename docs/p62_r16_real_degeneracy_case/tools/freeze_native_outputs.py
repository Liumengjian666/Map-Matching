#!/usr/bin/env python3
"""Freeze the one native R16 run before any evaluation poses are read."""
import csv
import datetime
import hashlib
import json
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def rows(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def main():
    here = Path(__file__).resolve().parents[1]
    run = here / 'formal_run_01'
    output = here / 'NATIVE_OUTPUT_FREEZE.json'
    if output.exists():
        raise RuntimeError('refuse to overwrite native output freeze')
    required = ['NDT_RESULTS.csv', 'GEOMETRY_AND_DEGENERACY.csv',
                'PREDICTION_DISAGREEMENT.csv', 'EVENTS.csv',
                'RUNTIME_AND_MEMORY.csv', 'ALIGN_CALLS.csv']
    files = {name: run / name for name in required}
    files['FORMAL_RUN_01.log'] = here / 'FORMAL_RUN_01.log'
    for path in files.values():
        if not path.is_file():
            raise RuntimeError(f'missing native output: {path}')
    ndt = rows(files['NDT_RESULTS.csv'])
    geom = rows(files['GEOMETRY_AND_DEGENERACY.csv'])
    events = rows(files['EVENTS.csv'])
    calls = rows(files['ALIGN_CALLS.csv'])
    if len(ndt) != 60 or len(geom) != 30 or len(events) != 30 or len(calls) != 60:
        raise RuntimeError('native output row counts do not match the frozen 30-frame contract')
    if sum(r['method'] == 'CONTROL_FULL' for r in ndt) != 30 or sum(r['method'] == 'CONTROL_SPLIT' for r in ndt) != 30:
        raise RuntimeError('Full/Split output count mismatch')
    if sum(r['detected'] == '1' for r in events) > 30 or any(r['method'] != 'CONTROL_SPLIT' for r in events):
        raise RuntimeError('event table semantics mismatch')
    if any(r['status'] != 'RETURNED' for r in calls):
        raise RuntimeError('native call ledger contains an incomplete call')
    result = {
        'task': 'P62-R16-REAL-NDT-DEGENERACY-CASE',
        'frozen_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'native_align_calls': 60, 'extra_recovery_align_calls': 0,
        'evaluation_reference_pose_values_read': False,
        'files': {name: {'bytes': path.stat().st_size, 'sha256': sha256(path)}
                  for name, path in files.items()},
        'rows': {'NDT_RESULTS.csv': len(ndt), 'GEOMETRY_AND_DEGENERACY.csv': len(geom),
                 'EVENTS.csv': len(events), 'ALIGN_CALLS.csv': len(calls)},
    }
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(f"NATIVE_OUTPUTS_FROZEN sha256={sha256(output)} align_calls=60 extra=0")


if __name__ == '__main__':
    main()
