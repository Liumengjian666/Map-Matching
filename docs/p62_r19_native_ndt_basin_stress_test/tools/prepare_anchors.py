#!/usr/bin/env python3
"""Read exactly two anchor pose values after input-only preregistration."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parents[1]
REPO = HERE.parents[1]
GT = Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment/ground_truth/registered_poses.csv')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    prereg_path = HERE / 'WINDOW_PREREGISTRATION.json'
    prereg = json.loads(prereg_path.read_text())
    assert prereg['ndt_align_calls'] == 0 and prereg['reference_pose_values_read'] is False
    expected = 'b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e'
    assert sha(GT) == expected, 'official reference changed'
    t_bl = np.asarray(json.loads((REPO / 'docs/p62_r14b_newer_college/conditional_base_pilot/TEST_FRAME_PREREGISTRATION.json').read_text())['T_B_L'], dtype=float)
    assert t_bl.shape == (4, 4) and np.isfinite(t_bl).all()
    anchors = {name: int(data['frames'][0]['stamp_ns']) for name, data in prereg['windows'].items()}
    wanted = {int(f['stamp_ns']) for w in prereg['windows'].values() for f in w['frames']}
    matched = set()
    values = {}
    with GT.open(newline='') as stream:
        reader = csv.reader(stream)
        next(reader)
        for row in reader:
            ns = int(row[0]) * 1_000_000_000 + int(row[1])
            if ns in wanted:
                matched.add(ns)
            if ns in anchors.values():
                values[ns] = np.array([float(v) for v in row[2:]], dtype=float)
    assert matched == wanted and len(values) == 2, 'timestamp contract not closed'
    for name, stamp in anchors.items():
        v = values[stamp]
        assert v.shape == (7,) and np.isfinite(v).all() and abs(np.linalg.norm(v[3:]) - 1) < 2e-5
        t_mb = np.eye(4)
        t_mb[:3, :3] = Rotation.from_quat(v[3:]).as_matrix()
        t_mb[:3, 3] = v[:3]
        t_ml = t_mb @ t_bl
        folder = HERE / name.lower()
        path = folder / 'ANCHOR_MATRIX.txt'
        with path.open('x') as f:
            f.write(' '.join(format(float(x), '.17g') for x in t_ml.flat) + '\n')
        record = {'role': name, 'anchor_stamp_ns': stamp,
                  'anchor_raw_ordinal': prereg['windows'][name]['frames'][0]['raw_ordinal'],
                  'reference_file': str(GT), 'reference_sha256': expected,
                  'all_122_timestamps_exactly_matched': len(matched) == 122,
                  'only_two_anchor_pose_values_read': True, 'evaluation_pose_values_read': False,
                  'coordinate_assumption': 'PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION',
                  'T_M_L_anchor': t_ml.tolist(), 'T_B_L': t_bl.tolist(),
                  'anchor_matrix_sha256': sha(path), 'preregistration_sha256': sha(prereg_path)}
        with (folder / 'ANCHOR_INITIALIZATION.json').open('x') as f:
            json.dump(record, f, indent=2, allow_nan=False)
            f.write('\n')
        print(name, 'anchor', record['anchor_raw_ordinal'], 'stamp', stamp, 'T_M_L', record['anchor_matrix_sha256'])


if __name__ == '__main__':
    main()
