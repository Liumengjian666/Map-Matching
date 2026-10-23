#!/usr/bin/env python3
"""Read only the preregistered anchor pose and freeze the fixed map origin."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def rigid(t):
    return (t.shape == (4, 4) and np.isfinite(t).all()
            and np.linalg.norm(t[3] - [0, 0, 0, 1]) < 1e-12
            and np.linalg.norm(t[:3, :3].T @ t[:3, :3] - np.eye(3)) < 1e-8
            and abs(np.linalg.det(t[:3, :3]) - 1) < 1e-8)


def main():
    repo = Path(__file__).resolve().parents[3]
    out = Path(__file__).resolve().parents[1]
    prereg_path = out / 'TEST_FRAME_PREREGISTRATION.json'
    prereg = json.loads(prereg_path.read_text())
    anchor_path = out / 'ANCHOR_INITIALIZATION.json'
    if anchor_path.exists():
        raise RuntimeError('refuse to overwrite anchor initialization')
    data = Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment')
    gt_path = data / 'ground_truth/registered_poses.csv'
    asset_manifest = json.loads((repo / 'docs/p62_r14b_newer_college/ASSET_MANIFEST.json').read_text())
    gt_expected = next(x for x in asset_manifest['assets'] if x['path'] == str(gt_path))
    if gt_path.stat().st_size != gt_expected['size'] or sha256(gt_path) != gt_expected['sha256']:
        raise RuntimeError('registered_poses.csv identity mismatch')

    wanted = {int(frame['stamp_ns']) for frame in prereg['frames']}
    reference_times = set()
    anchor_values = None
    with gt_path.open(newline='') as stream:
        reader = csv.reader(stream)
        next(reader)
        for row in reader:
            stamp = int(row[0]) * 1_000_000_000 + int(row[1])
            if stamp in wanted:
                reference_times.add(stamp)
            if stamp == int(prereg['frames'][0]['stamp_ns']):
                anchor_values = [float(v) for v in row[2:]]
    if reference_times != wanted or anchor_values is None:
        raise RuntimeError('selected timestamps do not all have an exact reference match')

    old = json.loads((repo / 'docs/p62_r14b_newer_college/conditional_base_pilot/TEST_FRAME_PREREGISTRATION.json').read_text())
    t_bc = np.asarray(old['T_B_C'], dtype=float)
    t_ci = np.asarray(old['T_C_I'], dtype=float)
    t_il = np.asarray(old['T_I_L'], dtype=float)
    t_bl = np.asarray(old['T_B_L'], dtype=float)
    if not all(rigid(t) for t in (t_bc, t_ci, t_il, t_bl)) or np.linalg.norm(t_bc @ t_ci @ t_il - t_bl) > 1e-8:
        raise RuntimeError('R14B frozen Base-to-LiDAR transform chain is inconsistent')
    xyz = np.asarray(anchor_values[:3], dtype=float)
    q_xyzw = np.asarray(anchor_values[3:7], dtype=float)
    if not np.isfinite(anchor_values).all() or abs(np.linalg.norm(q_xyzw) - 1) > 1e-5:
        raise RuntimeError('anchor reference pose is invalid')
    t_mb = np.eye(4)
    t_mb[:3, :3] = Rotation.from_quat(q_xyzw).as_matrix()
    t_mb[:3, 3] = xyz
    t_ml = t_mb @ t_bl
    if not rigid(t_mb) or not rigid(t_ml):
        raise RuntimeError('anchor transform is not a valid SE(3)')

    scans_path = out / 'SCANS.csv'
    with scans_path.open('x', newline='') as stream:
        fields = ['local_index', 'raw_ordinal', 'stamp_ns', 'path', 'points', 'header_bytes', 'point_step', 'pcd_bytes', 'pcd_sha256', 'expected_source_points', 'expected_source_fnv']
        writer = csv.DictWriter(stream, fields, lineterminator='\n')
        writer.writeheader()
        for frame in prereg['frames']:
            writer.writerow({
                'local_index': frame['local_index'],
                'raw_ordinal': frame['raw_ordinal_one_based'],
                'stamp_ns': frame['stamp_ns'],
                'path': frame['path'],
                'points': 65536,
                'header_bytes': 250,
                'point_step': 29,
                'pcd_bytes': frame['bytes'],
                'pcd_sha256': frame['sha256'],
                'expected_source_points': frame['preprocessed_source_points'],
                'expected_source_fnv': frame['source_fnv'],
            })
    matrix_path = out / 'ANCHOR_MATRIX.txt'
    with matrix_path.open('x') as stream:
        stream.write(' '.join(format(float(v), '.17g') for v in t_ml.flatten()) + '\n')
    result = {
        'task': 'P62-R16-REAL-NDT-DEGENERACY-CASE',
        'anchor_raw_ordinal_one_based': prereg['frames'][0]['raw_ordinal_one_based'],
        'anchor_stamp_ns': prereg['frames'][0]['stamp_ns'],
        'reference_file': str(gt_path),
        'reference_file_sha256': sha256(gt_path),
        'exact_time_matches_for_all_31_preregistered_scans': len(reference_times),
        'evaluation_pose_values_read': False,
        'only_anchor_pose_values_read': True,
        'coordinate_assumption': 'PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION',
        'map_frame': 'official Leica M',
        'reference_pose_frame_assumption': 'T_M_B from R14B conditional Base interpretation',
        'extrinsic_chain': 'T_M_L_anchor = T_M_B_anchor * T_B_C * T_C_I * T_I_L',
        'T_B_C': t_bc.tolist(), 'T_C_I': t_ci.tolist(), 'T_I_L': t_il.tolist(), 'T_B_L': t_bl.tolist(),
        'T_M_B_anchor': t_mb.tolist(), 'T_M_L_anchor': t_ml.tolist(),
        'T_M_L_anchor_sha256': sha256(matrix_path),
        'scan_table_sha256': sha256(scans_path),
        'fixed_roi_radius_m': 50.0,
        'test_gt_used_for_prediction_or_roi': False,
    }
    with anchor_path.open('x') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'anchor_ordinal': result['anchor_raw_ordinal_one_based'],
                      'anchor_stamp_ns': result['anchor_stamp_ns'],
                      'exact_matches': len(reference_times),
                      'T_M_L': t_ml.tolist(),
                      'anchor_receipt_sha256': sha256(anchor_path)}, indent=2))


if __name__ == '__main__':
    main()
