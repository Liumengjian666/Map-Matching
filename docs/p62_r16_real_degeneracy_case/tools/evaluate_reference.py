#!/usr/bin/env python3
"""Compute reference-relative errors only after verifying the native-output freeze."""
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read_rows(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def matrix_from_row(row, prefix='pose'):
    names = [f'{prefix}{i}{j}' for i in range(4) for j in range(4)]
    values = [float(row[name]) for name in names]
    return np.asarray(values, dtype=float).reshape(4, 4)


def metrics(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return {'n': 0, 'rmse': None, 'median': None, 'p95': None, 'max': None}
    return {'n': int(len(a)), 'rmse': float(np.sqrt(np.mean(a*a))),
            'median': float(np.median(a)), 'p95': float(np.percentile(a, 95)),
            'max': float(np.max(a))}


def main():
    here = Path(__file__).resolve().parents[1]
    repo = Path(__file__).resolve().parents[3]
    freeze_path = here / 'NATIVE_OUTPUT_FREEZE.json'
    if not freeze_path.is_file():
        raise RuntimeError('native outputs have not been frozen')
    freeze = json.loads(freeze_path.read_text())
    run = here / 'formal_run_01'
    for name in ('NDT_RESULTS.csv', 'PREDICTION_DISAGREEMENT.csv', 'EVENTS.csv'):
        path = run / name
        expected = freeze['files'][name]['sha256']
        if sha256(path) != expected:
            raise RuntimeError(f'native output changed after freeze: {name}')
    input_identity = json.loads((here / 'INPUT_IDENTITY_BEFORE_NDT.json').read_text())
    anchor = json.loads((here / 'ANCHOR_INITIALIZATION.json').read_text())
    gt_path = Path(next(path for path in input_identity['verified_r14b_assets']
                        if path.endswith('/ground_truth/registered_poses.csv')))
    expected_gt_sha = input_identity['verified_r14b_assets'][str(gt_path)]['sha256']
    if sha256(gt_path) != expected_gt_sha:
        raise RuntimeError('registered reference changed after native run')

    prereg = json.loads((here / 'TEST_FRAME_PREREGISTRATION.json').read_text())
    eval_frames = [frame for frame in prereg['frames'] if frame['role'] == 'EVALUATION']
    if len(eval_frames) != 30:
        raise RuntimeError('fixed 30 evaluation identities missing')
    wanted = {int(frame['stamp_ns']): frame for frame in eval_frames}
    reference = {}
    # This is the first read of evaluation pose values; NATIVE_OUTPUT_FREEZE.json
    # was written before this point and the selected window is not changed here.
    with gt_path.open(newline='') as stream:
        reader = csv.reader(stream)
        next(reader)
        for row in reader:
            ns = int(row[0]) * 1_000_000_000 + int(row[1])
            if ns in wanted:
                xyz = np.asarray([float(x) for x in row[2:5]], dtype=float)
                q_xyzw = np.asarray([float(x) for x in row[5:9]], dtype=float)
                if not np.isfinite(xyz).all() or not np.isfinite(q_xyzw).all() or abs(np.linalg.norm(q_xyzw)-1) > 2e-5:
                    raise RuntimeError(f'invalid official reference at {ns}')
                t_mb = np.eye(4)
                t_mb[:3, :3] = Rotation.from_quat(q_xyzw).as_matrix()
                t_mb[:3, 3] = xyz
                t_ml = t_mb @ np.asarray(anchor['T_B_L'], dtype=float)
                reference[ns] = t_ml
    if set(reference) != set(wanted):
        raise RuntimeError('exact reference timestamp match missing after frozen NDT run')

    ndt_rows = read_rows(run / 'NDT_RESULTS.csv')
    tp_rows = read_rows(run / 'PREDICTION_DISAGREEMENT.csv')
    ndt = {(int(row['frame']), row['method']): row for row in ndt_rows}
    tp = {int(row['frame']): row for row in tp_rows if row['method'] == 'CONTROL_FULL'}
    event_rows = {int(row['frame']): row for row in read_rows(run / 'EVENTS.csv')}
    output = here / 'REFERENCE_ERROR.csv'
    if output.exists():
        raise RuntimeError('refuse to overwrite reference evaluation')
    header = ['frame', 'raw_ordinal', 'stamp_ns', 'method', 'reference_match_ns',
              'reference_delta_ns', 'reference_basis', 'translation_error_m',
              'rotation_error_deg', 'native_converged', 'native_admissible',
              'event_detected', 'estimate_pose_flat_row_major', 'reference_T_M_L_flat_row_major']
    per_method = {}
    with output.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, header, lineterminator='\n')
        writer.writeheader()
        for frame in range(1, 31):
            tp_row = tp[frame]
            stamp = int(tp_row['stamp_ns'])
            gt = reference[stamp]
            frame_prereg = wanted[stamp]
            for method in ('PREDICTION', 'CONTROL_FULL', 'CONTROL_SPLIT'):
                if method == 'PREDICTION':
                    estimate = matrix_from_row(tp_row, 'tp')
                    conv = 'NA'
                    admitted = 'NA'
                else:
                    row = ndt[(frame, method)]
                    estimate = matrix_from_row(row)
                    conv = row['converged']
                    admitted = row['admissible']
                if not np.isfinite(estimate).all():
                    trans_err = rot_err = None
                else:
                    trans_err = float(np.linalg.norm(estimate[:3, 3] - gt[:3, 3]))
                    relative = gt[:3, :3].T @ estimate[:3, :3]
                    cosine = max(-1.0, min(1.0, (float(np.trace(relative))-1.0)/2.0))
                    rot_err = math.degrees(math.acos(cosine))
                writer.writerow({
                    'frame': frame, 'raw_ordinal': frame_prereg['raw_ordinal_one_based'],
                    'stamp_ns': stamp, 'method': method,
                    'reference_match_ns': stamp, 'reference_delta_ns': 0,
                    'reference_basis': 'MAP_ANCHORED_BENCHMARK_REFERENCE; PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION',
                    'translation_error_m': 'NA' if trans_err is None else format(trans_err, '.17g'),
                    'rotation_error_deg': 'NA' if rot_err is None else format(rot_err, '.17g'),
                    'native_converged': conv, 'native_admissible': admitted,
                    'event_detected': event_rows[frame]['detected'],
                    'estimate_pose_flat_row_major': '|'.join(format(float(x), '.17g') for x in estimate.flat),
                    'reference_T_M_L_flat_row_major': '|'.join(format(float(x), '.17g') for x in gt.flat),
                })
                item = per_method.setdefault(method, {'translation': [], 'rotation': [], 'converged': 0, 'admissible': 0})
                if trans_err is not None and rot_err is not None:
                    item['translation'].append(trans_err)
                    item['rotation'].append(rot_err)
                if method != 'PREDICTION':
                    item['converged'] += conv == '1'
                    item['admissible'] += admitted == '1'
    summary = {
        'task': 'P62-R16-REAL-NDT-DEGENERACY-CASE',
        'reference_status': 'MAP_ANCHORED_BENCHMARK_REFERENCE_NOT_INDEPENDENT_MOCAP',
        'coordinate_assumption': 'PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION; CSV export binding remains unverified',
        'test_gt_used_for_selection_prediction_or_ndt': False,
        'per_method': {},
        'event_frames': [],
        'note': 'No per-method trajectory alignment or post-hoc frame transform was fitted.'
    }
    for method, values in per_method.items():
        summary['per_method'][method] = {
            'translation_error_m': metrics(values['translation']),
            'rotation_error_deg': metrics(values['rotation']),
            'native_converged_frames': values['converged'],
            'native_admissible_frames': values['admissible'],
        }
    for frame, ev in event_rows.items():
        if ev['detected'] == '1':
            summary['event_frames'].append({'frame': frame, 'raw_ordinal': int(ev['raw_ordinal']),
                                            'stamp_ns': int(ev['stamp_ns']), 'reason': ev['reasons']})
    summary_path = here / 'REFERENCE_SUMMARY.json'
    if summary_path.exists():
        raise RuntimeError('refuse to overwrite reference summary')
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
