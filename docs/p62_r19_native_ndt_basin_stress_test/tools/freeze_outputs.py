#!/usr/bin/env python3
"""Seal native R19 outputs before reading evaluation pose values."""
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
BUILD = Path('/home/jian/livox_ws/p62_r19_ndt_basin_build/run_basin')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def rows(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def main():
    prereg_path = HERE / 'WINDOW_PREREGISTRATION.json'
    prereg = json.loads(prereg_path.read_text())
    identity = json.loads((HERE / 'NDT_INPUT_FREEZE.json').read_text())
    assert sha(prereg_path) == identity['preregistration_sha256']
    assert sha(BUILD) == identity['r19_binary_sha256']
    evidence = {}
    total = 0
    for name, window in prereg['windows'].items():
        folder = HERE / name.lower()
        map_info = identity['windows'][name]
        assert sha(map_info['path']) == map_info['sha256']
        for f in window['frames']:
            assert sha(f['path']) == f['pcd_sha256']
        run = folder / 'formal_run_01'
        ndt = rows(run / 'NDT_RESULTS.csv')
        calls = rows(run / 'ALIGN_CALLS.csv')
        times = rows(run / 'RUNTIME_AND_MEMORY.csv')
        assert len(ndt) == len(calls) == 240 and len(times) == 241
        assert Counter(r['arm'] for r in ndt) == {a: 60 for a in ('C0', 'C1', 'C2', 'C3')}
        assert all(r['kind'] == 'FULL' for r in calls)
        expected = {(a, j): window['frames'][j]['raw_ordinal'] for a in ('C0', 'C1', 'C2', 'C3') for j in range(1, 61)}
        for row in ndt:
            j = int(row['frame'])
            f = window['frames'][j]
            assert expected[(row['arm'], j)] == int(row['raw_ordinal'])
            assert int(row['stamp_ns']) == f['stamp_ns'] and row['source_fnv'] == f['source_fnv']
            assert row['role'] == name
        total += len(calls)
        names = ['NDT_RESULTS.csv', 'RUNTIME_AND_MEMORY.csv', 'ALIGN_CALLS.csv', 'INITIALIZATION_AXES.csv']
        evidence[name] = {'native_calls': len(calls), 'native_rows': len(ndt),
                          'files_sha256': {n: sha(run / n) for n in names},
                          'log_sha256': sha(folder / 'FORMAL_RUN_01.log'),
                          'quality_feedback_count': sum(r['quality_feedback'] == '1' for r in ndt),
                          'admission_shadow_rejections': sum(r['quality_feedback'] == '1' and r['admission_shadow_5deg'] == '0' for r in ndt)}
    assert total == 480
    out = {'task': 'P62-R19-NATIVE-NDT-BASIN-STRESS-TEST',
           'input_freeze_sha256': sha(HERE / 'NDT_INPUT_FREEZE.json'),
           'native_output_frozen_before_evaluation_reference_values': True,
           'total_native_full_aligns': total, 'extra_stability_aligns_at_freeze': 0,
           'split_aligns': 0, 'ekf_calls': 0, 'windows': evidence}
    target = HERE / 'NATIVE_OUTPUT_FREEZE.json'
    with target.open('x') as stream:
        json.dump(out, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print('NATIVE_OUTPUT_FROZEN', sha(target), 'aligns', total)


if __name__ == '__main__':
    main()
