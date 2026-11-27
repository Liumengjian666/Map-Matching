"""Prepare freeze manifests and scorer plans. Reference fields go to a separate receipt."""
import csv
import hashlib
import os
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
Q1 = HERE.parent / 'p64_r2_q1_candidate_search'
Q2 = HERE.parent / 'p64_r2_q2_bounded_search_adaptation'
ARCH = Path(os.environ.get('P64_Q1_RUN', '/home/jian/p64_r2_q1_ws/run'))
FIELDS = [f'T{i}{j}' for i in range(4) for j in range(4)]
PAIRS = {
    'H307': [('primary', 0, 1), ('both_wrong', 1, 2)],
    'H515': [('boundary', 0, 1), ('postfit_stress', 0, 2)],
    'H309': [('supplement', 0, 1)],
    'O164': [('synthetic_control', 0, 1)],
}

def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))

def write(path, rows):
    with path.open('x') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

def matrix(row):
    return np.array([float(row[key]) for key in FIELDS], np.float32).reshape(4, 4)

def digest(path):
    with Path(path).open('rb') as stream:
        value = hashlib.sha256()
        for block in iter(lambda: stream.read(1 << 20), b''):
            value.update(block)
        return value.hexdigest()

def neighborhood(center):
    poses = []
    for meters, degrees in [(0.025, 0.5), (0.05, 1.)]:
        poses.append(center)
        for axis in range(6):
            for sign in [1, -1]:
                step = np.eye(4)
                if axis < 3:
                    step[axis, 3] = sign * meters
                else:
                    rv = np.zeros(3)
                    rv[axis-3] = np.deg2rad(sign * degrees)
                    step[:3, :3] = Rotation.from_rotvec(rv).as_matrix()
                poses.append((center @ step).astype(np.float32))
    return np.stack(poses)

def main(run, archive):
    run.mkdir(exist_ok=True)
    online = {fields[0]: fields for line in (ARCH/'input/online.tsv').read_text().splitlines() if (fields := line.split('\t'))}
    historical = {r['candidate_id']: r for r in read(Q2/'fixed_candidate_manifest.csv')}
    boundary = read(Q2/'H515_boundary_fit_receipt.csv')[2]
    fits = read(Q2/'online_fit.csv')
    rank = read(Q2/'final_ranking_comparison.csv')
    q1 = {r['sample_id']: r for r in read(Q1/'qualification_sample_manifest.csv')}
    hypotheses, labels, samples = [], [], []
    packs = {}
    def add(frame, ident, transform, origin, error_t, error_r):
        hypotheses.append(dict(frame=frame,candidate_id=ident,provenance=origin,**dict(zip(FIELDS,transform.ravel()))))
        labels.append(dict(frame=frame,candidate_id=ident,translation_m=error_t,rotation_deg=error_r))
        packs.setdefault(frame, []).append(transform)
    for frame, keys in [('H307',['P2648','N1406','N1739']), ('H515',['N2200','N2425'])]:
        for key in keys:
            row = historical[f'{frame}_{key}']
            add(frame, row['candidate_id'], matrix(row), 'Q2 fixed_candidate_manifest.csv',
                row['translation_error_m'], row['rotation_error_deg'])
    add('H515','H515_C3_terminal',matrix(boundary),'Q2 H515_boundary_fit_receipt.csv row 3',
        boundary['error_translation_m'],boundary['error_rotation_deg'])
    for metric in ('park','native'):
        cond = lambda r: r['sample_id']=='H309' and r['method']=='S1' and r['metric']==metric and r['policy']=='C3'
        winner, = [r for r in rank if cond(r) and r['final_rank']=='1']
        fit, = [r for r in fits if cond(r) and r['index']==winner['index']]
        add('H309',f"H309_{metric}_{fit['index']}",matrix(fit),'Q2 S1 C3 terminal of sealed rank-1 seed',
            winner['final_translation_error_m'],winner['final_rotation_error_deg'])
    nominal = np.array(online['O164'][19:35],np.float32).reshape(4,4)
    control = nominal.copy()
    control[:3,:3] = Rotation.from_euler('z',5,degrees=True).as_matrix() @ nominal[:3,:3]
    add('O164','O164_native',nominal,'Q1 archive online.tsv native pose','','')
    add('O164','O164_synthetic_yaw5',control,'Synthetic +5deg world yaw at fixed translation','','')
    for frame, transforms in packs.items():
        _, map_path, scan, *_ = online[frame]
        samples.append(dict(frame=frame,sensor='Livox Mid-360' if frame.startswith('O') else 'Azure Kinect',
            protocol='A / exposed development',map_pcd=map_path,source_pcd=scan,
            map_sha256=digest(map_path),source_sha256=digest(scan),
            q1_filtered_source_sha256=digest(ARCH / f'gate/{frame}_source_xyz.bin'),
            q1_filtered_target_sha256=digest(ARCH / f'gate/{frame}_target_xyz.bin')))
        assert samples[-1]['map_sha256'] in q1[frame].values(), frame
        assert samples[-1]['source_sha256'] in q1[frame].values(), frame
        poses = np.stack([neighborhood(t) for t in transforms])
        np.savez(run/f'{frame}_poses.npz', nominal=np.stack(transforms),poses=poses)
        for suffix, indices in [('nominal',[0]), ('perturb',[i for i in range(26) if i not in (0,13)])]:
            with (run/f'{frame}_{suffix}.tsv').open('x') as out:
                for k, p in enumerate(poses):
                    for i in indices:
                        print(frame,map_path,scan,k,i,*map(float,p[i].ravel()),sep='\t',file=out)
    write(archive/'sample_manifest.csv',samples)
    write(archive/'fixed_hypothesis_manifest.csv',hypotheses)
    write(run/'historical_labels_DO_NOT_USE_FOR_SELECTION.csv',labels)

if __name__ == '__main__':
    main(Path(sys.argv[1]), Path(sys.argv[2]))
