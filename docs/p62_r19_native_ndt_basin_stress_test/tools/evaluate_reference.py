#!/usr/bin/env python3
"""Evaluate sealed R19 optimizer trajectories against conditional benchmark poses."""
import csv
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parents[1]
GT = Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment/ground_truth/registered_poses.csv')
ARMS = ('C0', 'C1', 'C2', 'C3')
ROLES = ('WEAK', 'STRONG')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def rows(path):
    with Path(path).open(newline='') as f:
        return list(csv.DictReader(f))


def pose(row, prefix):
    values = [float(row[f'{prefix}{i}{j}']) if row[f'{prefix}{i}{j}'] != 'NA' else math.nan
              for i in range(4) for j in range(4)]
    p = np.asarray(values, dtype=float).reshape(4, 4)
    return p if np.isfinite(p).all() else None


def differences(a, b):
    if a is None or b is None:
        return math.nan, math.nan
    translation = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    c = float(np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2, -1, 1))
    return translation, math.degrees(math.acos(c))


def metrics(values):
    x = np.asarray(values, dtype=float)
    if not len(x):
        return {'n': 0, 'rmse': None, 'median': None, 'p95': None, 'max': None}
    return {'n': len(x), 'rmse': float(np.sqrt(np.mean(x*x))),
            'median': float(np.median(x)), 'p95': float(np.percentile(x, 95)), 'max': float(np.max(x))}


def write(path, header, data):
    with path.open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=header, lineterminator='\n')
        writer.writeheader()
        writer.writerows(data)


def main():
    sealed_path = HERE / 'NATIVE_OUTPUT_FREEZE.json'
    sealed = json.loads(sealed_path.read_text())
    identity = json.loads((HERE / 'NDT_INPUT_FREEZE.json').read_text())
    assert sealed['native_output_frozen_before_evaluation_reference_values']
    assert sealed['total_native_full_aligns'] == 480 and sealed['extra_stability_aligns_at_freeze'] == 0
    assert sha(GT) == identity['reference_file_sha256']
    prereg = json.loads((HERE / 'WINDOW_PREREGISTRATION.json').read_text())
    native = []
    runtime = []
    for role in ROLES:
        folder = HERE / role.lower() / 'formal_run_01'
        for name, digest in sealed['windows'][role]['files_sha256'].items():
            assert sha(folder / name) == digest, f'native output changed: {role}/{name}'
        native += rows(folder / 'NDT_RESULTS.csv')
        runtime += rows(folder / 'RUNTIME_AND_MEMORY.csv')
    assert len(native) == 480
    # First access to evaluation pose VALUES occurs only after all native SHA checks.
    wanted = {int(f['stamp_ns']) for w in prereg['windows'].values() for f in w['frames'][1:]}
    gt_values = {}
    with GT.open(newline='') as f:
        reader = csv.reader(f)
        next(reader)
        for line in reader:
            ns = int(line[0]) * 1_000_000_000 + int(line[1])
            if ns in wanted:
                gt_values[ns] = np.asarray([float(x) for x in line[2:]], dtype=float)
    assert set(gt_values) == wanted and len(wanted) == 120
    records = {}
    error_rows = []
    buckets = defaultdict(lambda: defaultdict(list))
    for row in native:
        role, arm, frame = row['role'], row['arm'], int(row['frame'])
        assert role in ROLES and arm in ARMS and 1 <= frame <= 60
        ns = int(row['stamp_ns'])
        assert ns == int(prereg['windows'][role]['frames'][frame]['stamp_ns'])
        v = gt_values[ns]
        assert v.shape == (7,) and np.isfinite(v).all() and abs(np.linalg.norm(v[3:]) - 1) < 2e-5
        t_mb = np.eye(4);t_mb[:3, :3] = Rotation.from_quat(v[3:]).as_matrix();t_mb[:3, 3] = v[:3]
        t_bl = np.asarray(json.loads((HERE / role.lower() / 'ANCHOR_INITIALIZATION.json').read_text())['T_B_L'], dtype=float)
        t_gt = t_mb @ t_bl
        p = {kind: pose(row, kind) for kind in ('tp', 'raw', 'output')}
        errs = {kind: differences(matrix, t_gt) for kind, matrix in p.items()}
        for kind in ('tp', 'raw', 'output'):
            e_t, e_r = errs[kind]
            if math.isfinite(e_t) and math.isfinite(e_r):
                buckets[(role, arm, kind)]['translation_m'].append(e_t)
                buckets[(role, arm, kind)]['rotation_deg'].append(e_r)
            error_rows.append({'role': role, 'arm': arm, 'frame': frame,
                               'raw_ordinal': row['raw_ordinal'], 'stamp_ns': ns,
                               'estimate': {'tp': 'PREDICTION', 'raw': 'RAW_NDT', 'output': 'CAUSAL_OUTPUT'}[kind],
                               'translation_error_m': e_t, 'rotation_error_deg': e_r,
                               'reference_basis': 'MAP_ANCHORED_BENCHMARK_REFERENCE; CONDITIONAL_BASE',
                               'quality_feedback': row['quality_feedback']})
        records[(role, arm, frame)] = {'row': row, 'poses': p, 'errors': errs}
    assert len(records) == 480
    comparisons = []
    for role in ROLES:
        for arm in ARMS[1:]:
            for frame in range(1, 61):
                a = records[(role, 'C0', frame)];b = records[(role, arm, frame)]
                t, r = differences(a['poses']['raw'], b['poses']['raw'])
                base_e = a['errors']['raw'];arm_e = b['errors']['raw']
                level1 = (a['row']['raw_finite'] == b['row']['raw_finite'] == '1'
                          and a['row']['converged'] == b['row']['converged'] == '1'
                          and (t > .10 or r > 1.))
                quality = a['row']['quality_feedback'] == b['row']['quality_feedback'] == '1'
                normal_ok = base_e[0] <= .3 and base_e[1] <= 3.
                position_bad = arm_e[0] > .5 and arm_e[0]-base_e[0] >= .3
                rotation_bad = arm_e[1] > 5. and arm_e[1]-base_e[1] >= 3.
                level2 = bool(level1 and quality and normal_ok and (position_bad or rotation_bad))
                comparisons.append({'role': role, 'arm': arm, 'frame': frame,
                    'raw_ordinal': a['row']['raw_ordinal'], 'stamp_ns': a['row']['stamp_ns'],
                    'c0_arm_translation_m': t, 'c0_arm_rotation_deg': r,
                    'c0_translation_error_m': base_e[0], 'arm_translation_error_m': arm_e[0],
                    'c0_rotation_error_deg': base_e[1], 'arm_rotation_error_deg': arm_e[1],
                    'c0_native_score': a['row']['native_score'], 'arm_native_score': b['row']['native_score'],
                    'c0_overlap': a['row']['overlap_0p5'], 'arm_overlap': b['row']['overlap_0p5'],
                    'c0_quality': a['row']['quality_feedback'], 'arm_quality': b['row']['quality_feedback'],
                    'level1_initialization_sensitivity': int(level1),
                    'level2_suspect_frame': int(level2)})
    persistent = []
    for role in ROLES:
        for arm in ARMS[1:]:
            run = []
            for row in [x for x in comparisons if x['role'] == role and x['arm'] == arm]:
                if row['level2_suspect_frame']:
                    run.append(row['frame'])
                else:
                    if len(run) >= 3:persistent.append({'role': role, 'arm': arm, 'start_frame': run[0], 'end_frame': run[-1], 'length': len(run)})
                    run = []
            if len(run) >= 3:persistent.append({'role': role, 'arm': arm, 'start_frame': run[0], 'end_frame': run[-1], 'length': len(run)})
    write(HERE / 'NDT_RESULTS.csv', list(native[0]), native)
    write(HERE / 'RUNTIME_AND_MEMORY.csv', list(runtime[0]), runtime)
    write(HERE / 'REFERENCE_ERROR.csv', list(error_rows[0]), error_rows)
    write(HERE / 'BASIN_COMPARISON.csv', list(comparisons[0]), comparisons)
    out = {'reference_type': 'MAP_ANCHORED_BENCHMARK_REFERENCE; CONDITIONAL_BASE',
           'reference_samples': 120, 'native_full_aligns': 480,
           'level1_count_by_window': {role: sum(x['level1_initialization_sensitivity'] for x in comparisons if x['role'] == role) for role in ROLES},
           'level2_suspect_frame_count': sum(x['level2_suspect_frame'] for x in comparisons),
           'persistent_suspect_runs': persistent,
           'metrics': {role: {arm: {kind: {metric: metrics(buckets[(role, arm, kind)][metric]) for metric in ('translation_m', 'rotation_deg')}
                          for kind in ('tp', 'raw', 'output')} for arm in ARMS} for role in ROLES},
           'native_output_freeze_sha256': sha(sealed_path)}
    with (HERE / 'EVALUATION_SUMMARY.json').open('x') as f:
        json.dump(out, f, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps({'level1': out['level1_count_by_window'], 'level2_frames': out['level2_suspect_frame_count'],
                      'persistent': persistent, 'native_full_aligns': 480}, indent=2))


if __name__ == '__main__':
    main()
