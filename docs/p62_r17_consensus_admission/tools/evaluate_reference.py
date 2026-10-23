#!/usr/bin/env python3
"""Read benchmark reference only after validating the frozen R17 native outputs."""
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


HERE = Path(__file__).resolve().parents[1]


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def stats(values):
    a = np.asarray(values, dtype=float)
    return dict(n=len(a), rmse=float(np.sqrt(np.mean(a*a))), p95=float(np.percentile(a, 95)),
                max=float(np.max(a)), median=float(np.median(a)))


def from_flat(s):
    return np.asarray([float(x) for x in s.split('|')], dtype=float).reshape(4, 4)


def error(est, ref):
    dt = float(np.linalg.norm(est[:3, 3] - ref[:3, 3]))
    dr = math.degrees(Rotation.from_matrix(ref[:3, :3].T @ est[:3, :3]).magnitude())
    return dt, dr


def main():
    freeze = json.loads((HERE / 'NATIVE_OUTPUT_FREEZE.json').read_text())
    for path, expected in freeze['code_and_output_sha256'].items():
        if sha(path) != expected:
            raise RuntimeError('frozen native output/source changed: ' + path)
    native = list(csv.DictReader((HERE / 'native_run_01/ADMISSION_DECISIONS.csv').open(newline='')))
    if len(native) != 90:
        raise RuntimeError('native decisions incomplete')
    r16 = HERE.parent / 'p62_r16_real_degeneracy_case'
    anchor = json.loads((r16 / 'ANCHOR_INITIALIZATION.json').read_text())
    gt_path = Path(anchor['reference_file'])
    if sha(gt_path) != anchor['reference_file_sha256']:
        raise RuntimeError('official reference identity changed')
    target_stamps = {int(row['stamp_ns']) for row in native}
    reference = {}
    with gt_path.open(newline='') as stream:
        rows = csv.reader(stream);next(rows)
        for row in rows:
            ns = int(row[0])*1_000_000_000 + int(row[1])
            if ns not in target_stamps:
                continue
            t = np.eye(4)
            t[:3, :3] = Rotation.from_quat(np.asarray(row[5:9], dtype=float)).as_matrix()
            t[:3, 3] = np.asarray(row[2:5], dtype=float)
            reference[ns] = t @ np.asarray(anchor['T_B_L'], dtype=float)
    if target_stamps != set(reference):
        raise RuntimeError('reference exact timestamp pairs missing')
    target = HERE / 'REFERENCE_ERROR.csv'
    if target.exists():
        raise RuntimeError('refuse to overwrite reference result')
    keys = ['frame', 'raw_ordinal', 'stamp_ns', 'policy', 'decision', 'accepted',
            'output_source', 'output_translation_error_m', 'output_rotation_error_deg',
            'prediction_translation_error_m', 'prediction_rotation_error_deg',
            'raw_full_translation_error_m', 'raw_full_rotation_error_deg', 'reference_basis']
    summary = {}
    with target.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, keys, lineterminator='\n');writer.writeheader()
        for row in native:
            ns = int(row['stamp_ns']); ref = reference[ns]
            out = error(from_flat(row['output_pose']), ref)
            pred = error(from_flat(row['tp_pose']), ref)
            full = error(from_flat(row['full_pose']), ref)
            accepted = row['accepted'] == '1'
            writer.writerow(dict(frame=row['frame'], raw_ordinal=row['raw_ordinal'], stamp_ns=ns,
                policy=row['policy'], decision=row['decision'], accepted=int(accepted),
                output_source='ACCEPTED_FULL' if accepted else 'CAUSAL_PREDICTION_FALLBACK',
                output_translation_error_m=format(out[0], '.17g'), output_rotation_error_deg=format(out[1], '.17g'),
                prediction_translation_error_m=format(pred[0], '.17g'), prediction_rotation_error_deg=format(pred[1], '.17g'),
                raw_full_translation_error_m=format(full[0], '.17g'), raw_full_rotation_error_deg=format(full[1], '.17g'),
                reference_basis='MAP_ANCHORED_BENCHMARK_REFERENCE_CONDITIONAL_BASE'))
            s = summary.setdefault(row['policy'], {'translation': [], 'rotation': [], 'accepted': 0,
                                                     'rejected': 0, 'raw7570': None, 'raw7571': None})
            s['translation'].append(out[0]);s['rotation'].append(out[1]);s['accepted' if accepted else 'rejected'] += 1
            if row['raw_ordinal'] in ('7570', '7571'):
                s['raw' + row['raw_ordinal']] = {'output_translation_m': out[0], 'output_rotation_deg': out[1],
                    'prediction_translation_m': pred[0], 'prediction_rotation_deg': pred[1],
                    'raw_full_translation_m': full[0], 'raw_full_rotation_deg': full[1], 'decision': row['decision']}
    for s in summary.values():
        s['translation_m'] = stats(s.pop('translation'))
        s['rotation_deg'] = stats(s.pop('rotation'))
    (HERE / 'REFERENCE_SUMMARY.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
