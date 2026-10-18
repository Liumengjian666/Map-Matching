"""Validate R2 assets, profile 100 Hessians, and preregister exactly 20x12 starts."""
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from basin_math import effective, perturbations, uniform_rows

DOC = Path(__file__).resolve().parent
R2 = DOC.parent / 'p61_r2_geode_real_data_run'
RUN = Path('/home/jian/livox_ws/p61_r2_geode_runs')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''): h.update(block)
    return h.hexdigest()


def table(path):
    with Path(path).open() as stream: return list(csv.DictReader(stream))


def write_csv(path, rows, fields=None):
    with Path(path).open('x', newline='') as stream:
        if fields is None and not rows: raise ValueError('empty table requires explicit schema')
        writer = csv.DictWriter(stream, fieldnames=fields or list(rows[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(rows)


def identity(path):
    return {'bytes': Path(path).stat().st_size, 'sha256': sha(path)}


def initial_pose(base, eta):
    """Same finite chart and float pose carrier as frozen coupledPoseAtEta."""
    theta = eta[3:]; angle = np.linalg.norm(theta)
    R = np.eye(3)
    if angle >= 1e-14:
        x, y, z = theta / angle
        K = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
        R += np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
    pose = base.astype(np.float32).copy()
    pose[:3, 3] += np.float32(.8) * eta[:3].astype(np.float32)
    pose[:3, :3] = (R @ base[:3, :3].astype(float)).astype(np.float32)
    return pose


def prepare(output):
    output = Path(output)
    if output.exists(): raise ValueError('new preregistration directory required')
    checked = {}
    # These R2 manifests include the original exported PCD/NPY and run outputs.
    for path in (R2 / 'screen_input_freeze.json', R2 / 'screen_output_freeze.json'):
        for name, value in json.loads(path.read_text())['files'].items():
            if not str(Path(name).resolve()).startswith('/home/jian/'):
                raise ValueError('only internal frozen assets allowed')
            actual = identity(name)
            if actual != value: raise ValueError('R2 frozen identity changed: ' + name)
            checked[name] = actual
        checked[str(path)] = identity(path)
    for entry in json.loads((R2 / 'artifact_manifest.json').read_text()):
        path = R2 / entry['archive']
        if identity(path) != {k: entry[k] for k in ('bytes', 'sha256')}:
            raise ValueError('R2 archive differs: ' + str(path))
    scans = table(R2 / 'screen_sample_manifest.csv')
    hessians = table(R2 / 'screen_hessian.csv')
    poses = table(R2 / 'screen_alignments.csv')
    if len(scans) != 100 or not ([r['scan_index'] for r in scans] ==
            [r['scan_index'] for r in hessians] == [r['scan_index'] for r in poses]):
        raise ValueError('R2 100-pair identity/order mismatch')
    profiles, curvature, lookup = {}, [], {}
    for scan, h, pose in zip(scans, hessians, poses):
        key = scan['scan_index']
        if any(scan['stamp_ns'] != r['stamp_ns'] for r in (h, pose)):
            raise ValueError('R2 timestamp mismatch')
        if any(h[k] != pose[k] for k in ('source_hash', 'target_hash', 'source_points', 'target_points')):
            raise ValueError('R2 processed input mismatch')
        for role in ('source', 'target'):
            path = scan[role + '_path']
            if sha(path) != scan[role + '_pcd_sha256']:
                raise ValueError('R2 raw PCD SHA mismatch')
        H = np.array([[float(h[f'H{i}{j}']) for j in range(6)] for i in range(6)])
        result = effective(H); profiles[key] = result
        row = {'scan_index': key, 'stamp_ns': scan['stamp_ns'], 'valid': 1,
               'rr_condition': result['rr_condition'], 'solve_residual': result['solve_residual'],
               'congruence_error': result['congruence_error'],
               'weak_curvature_loss': 1 - result['eigenvalues'][0] / result['tt_eigenvalues'][0],
               'trace_curvature_loss': 1 - np.trace(result['S']) / np.trace(H[:3, :3])}
        for i in range(3):
            row[f'tt_lambda{i}'] = result['tt_eigenvalues'][i]
            row[f'eff_lambda{i}'] = result['eigenvalues'][i]
            row[f'weak{i}'] = result['vectors'][i, 0]
            row[f'strong{i}'] = result['vectors'][i, 2]
            for j in range(3): row[f'S{i}{j}'] = result['S'][i, j]
        curvature.append(row); lookup[key] = (scan, h, pose)
    # Selection uses time only, regardless of all computed curvatures.
    selected = uniform_rows(scans)
    initials, selected_rows = [], []
    for scan in selected:
        key = scan['scan_index']; _, h, pose = lookup[key]
        selected_rows.append(scan)
        base = np.array([[float(pose[f'pose{i}{j}']) for j in range(4)] for i in range(4)], dtype=np.float32)
        for perturb in perturbations(profiles[key]):
            tag = f"{key}_{perturb['family']}_{perturb['amplitude_m']:.2f}_{perturb['sign']:+d}"
            row = {**scan, 'candidate_id': tag, 'family': perturb['family'],
                   'amplitude_m': perturb['amplitude_m'], 'sign': perturb['sign'],
                   'status': perturb['status'], 'rotation_deg': perturb['rotation_deg'],
                   'source_hash': h['source_hash'], 'target_hash': h['target_hash'],
                   'source_points': h['source_points'], 'target_points': h['target_points'],
                   'nominal_energy': h['mean_energy']}
            for i in range(6): row[f'eta{i}'] = perturb['eta'][i]
            initial = initial_pose(base, perturb['eta'])
            for i in range(4):
                for j in range(4):
                    row[f'initial{i}{j}'] = float(initial[i, j]); row[f'nominal{i}{j}'] = float(base[i, j])
            for i in range(6):
                for j in range(6): row[f'nominal_H{i}{j}'] = h[f'H{i}{j}']
            initials.append(row)
    output.mkdir(parents=True)
    write_csv(output / 'effective_curvature.csv', curvature)
    write_csv(output / 'sample_manifest.csv', selected_rows)
    write_csv(output / 'initial_manifest.csv', initials)
    for path in output.iterdir(): checked[str(path)] = identity(path)
    checked[str(DOC / 'PROTOCOL.md')] = identity(DOC / 'PROTOCOL.md')
    frozen = {'source_start_sha': '71f12c062c38ff8489124cf8e648269422919776',
              'selection': 'nearest_uniform_sensor_time_in_R2_100_earlier_tie',
              'indices': [int(r['scan_index']) for r in selected], 'starts': len(initials),
              'ready': sum(r['status'] == 'READY' for r in initials),
              'out_of_bounds': sum(r['status'] == 'OUT_OF_BOUNDS' for r in initials),
              'gt_read': False, 'bag_read': False, 'files': checked}
    with (output / 'preregistration.json').open('x') as stream:
        json.dump(frozen, stream, indent=2, allow_nan=False)
    print(json.dumps({k: v for k, v in frozen.items() if k != 'files'}, indent=2))


if __name__ == '__main__': prepare(sys.argv[1])
