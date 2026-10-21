"""Pre-register one Base-assumption pilot; read only anchor pose values.

Reference timestamps are read as integers. Evaluation pose strings are neither
converted nor emitted here. No registration or prior blocked script is run.
"""
import csv
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

HERE = Path(__file__).resolve().parent
WS = HERE.parents[2]
ROOT = Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment')
DATA = Path('/home/jian/rosbag/public_datasets/newer_college_r14b/conditional_base_pilot_20261021')
START = 'c614f5fd1d8837b4c96ec10cc1199e41d5b2418b'
sys.path.insert(0, str(HERE.parent))
from inspect_inputs import sha, stable_snapshot


def save(name, value):
    with (HERE / name).open('x') as f:
        json.dump(value, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.write('\n')


def rigid(t):
    return (t.shape == (4, 4) and np.isfinite(t).all()
            and np.linalg.norm(t[3] - [0, 0, 0, 1]) < 1e-12
            and np.linalg.norm(t[:3, :3].T @ t[:3, :3] - np.eye(3)) < 1e-8
            and abs(np.linalg.det(t[:3, :3]) - 1) < 1e-8)


def pcd_format(path):
    if path.stat().st_size > 8 * 1024**2 or path.is_symlink():
        raise ValueError('PCD_FORMAT_BLOCKED: invalid source path/size')
    with path.open('rb') as f:
        h = {}
        for _ in range(64):
            raw = f.readline(4097)
            if not raw or len(raw) > 4096:
                raise ValueError('PCD_FORMAT_BLOCKED: bad header')
            line = raw.decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            key, *values = line.split(); h[key] = values
            if key == 'DATA':
                break
        offset = f.tell(); payload = f.read()
    expected = {'FIELDS': ['x','y','z','intensity','t','reflectivity','ring','noise','range'],
                'SIZE': ['4','4','4','4','4','2','1','2','4'],
                'TYPE': ['F','F','F','F','U','U','U','U','U'],
                'COUNT': ['1']*9, 'DATA': ['binary']}
    if any(h.get(k) != v for k, v in expected.items()):
        raise ValueError('PCD_FORMAT_BLOCKED: schema changed')
    n = int(h['POINTS'][0]); w = int(h['WIDTH'][0]); height = int(h['HEIGHT'][0])
    if n != w*height or n <= 0 or len(payload) < n*29 or any(payload[n*29:]):
        raise ValueError('PCD_FORMAT_BLOCKED: records or nonzero suffix')
    points = np.ndarray((n,), dtype=np.dtype({'names':['x','y','z','t'],
        'formats':['<f4','<f4','<f4','<u4'], 'offsets':[0,4,8,16], 'itemsize':29}),
        buffer=payload[:n*29])
    finite = np.isfinite(points['x']) & np.isfinite(points['y']) & np.isfinite(points['z'])
    if not finite.any():
        raise ValueError('PCD_FORMAT_BLOCKED: no finite XYZ')
    return {'header': h, 'header_bytes': offset, 'point_step':29, 'points':n,
            'finite_xyz':int(finite.sum()), 'declared_point_bytes':n*29,
            'tail_bytes':len(payload)-n*29, 'tail_all_zero':True,
            'declared_records_sha256':hashlib.sha256(payload[:n*29]).hexdigest(),
            't_min_raw':int(points['t'].min()), 't_max_raw':int(points['t'].max())}


def main():
    head = subprocess.check_output(['git','-C',str(WS),'rev-parse','HEAD'], text=True).strip()
    if head != START:
        raise RuntimeError('Unexpected continuation baseline')
    if (HERE / 'TEST_FRAME_PREREGISTRATION.json').exists() or DATA.exists():
        raise RuntimeError('Refuse repeated preparation/overwrite')
    free = os.statvfs(HERE).f_bavail * os.statvfs(HERE).f_frsize
    if free < (20+9)*1024**3:
        raise RuntimeError('STORAGE_BLOCKED')
    previous = json.loads((HERE.parent / 'ASSET_MANIFEST.json').read_text())
    needed = [ROOT/'ground_truth/registered_poses.csv',
              ROOT/'prior_map/new-college-29-01-2020-5cm-resolution.ply',
              ROOT/'calibration/kalibr_output/cam-ouster-imu/camchain-ouster_imu-cam-rooster_2020-03-11-10-05-35_0.yaml',
              ROOT/'calibration/kalibr_output/ouster_imu_lidar_transforms.yaml']
    inputs = []
    for p in needed:
        old = next(a for a in previous['assets'] if a['path'] == str(p))
        if p.stat().st_size != old['size'] or sha(p) != old['sha256']:
            raise RuntimeError('Frozen side input identity changed')
        inputs.append({'path':str(p),'bytes':p.stat().st_size,'sha256':sha(p)})
    with (HERE.parent/'LOCAL_SCAN_INVENTORY.csv').open() as f:
        inventory = list(csv.DictReader(f))
    first = int(inventory[0]['filename_stamp_ns']); last = int(inventory[-1]['filename_stamp_ns'])
    chosen = next(i for i, r in enumerate(inventory)
                  if 20*(int(r['filename_stamp_ns'])-first) > last-first)
    group = inventory[chosen:chosen+21]
    if len(group) != 21:
        raise RuntimeError('Insufficient fixed consecutive scans')
    stamps = [int(r['filename_stamp_ns']) for r in group]
    if any(b <= a for a,b in zip(stamps,stamps[1:])):
        raise RuntimeError('Nonincreasing selected timestamps')
    # Only anchor pose fields are parsed. Test rows contribute timestamps only.
    reference_times = set(); anchor_row = None
    with needed[0].open() as f:
        reader = csv.reader(f); next(reader)
        for row in reader:
            ns = int(row[0])*10**9 + int(row[1]); reference_times.add(ns)
            if ns == stamps[0]:
                anchor_row = list(map(float,row[2:]))
    if anchor_row is None or any(t not in reference_times for t in stamps):
        raise RuntimeError('REFERENCE_TIME_CLOSURE_BLOCKED')
    cam = yaml.safe_load(needed[2].read_text()); sensor = yaml.safe_load(needed[3].read_text())
    t_ci = np.array(cam['cam0']['T_cam_imu'], dtype=float)
    def from_sensor(key):
        a = sensor[key]; t = np.eye(4)
        q = np.array(a['rotation'], dtype=float)
        if abs(np.linalg.norm(q)-1)>1e-10:
            raise ValueError('Nonunit official extrinsic quaternion')
        t[:3,:3] = Rotation.from_quat(q).as_matrix(); t[:3,3] = a['translation']
        return t
    t_il = from_sensor('os1_lidar_to_os1_imu'); t_li = from_sensor('os1_imu_to_os1_lidar')
    t_bc = np.eye(4); t_bc[:3,:3] = [[0,0,1],[-1,0,0],[0,-1,0]]
    if not all(map(rigid,[t_ci,t_il,t_li,t_bc])) or np.linalg.norm(t_il@t_li-np.eye(4))>1e-10:
        raise ValueError('Base-chain mathematical contradiction')
    t_bl = t_bc@t_ci@t_il
    t_mb = np.eye(4); q = np.array(anchor_row[3:])
    if not np.isfinite(anchor_row).all() or abs(np.linalg.norm(q)-1)>1e-5:
        raise ValueError('Invalid anchor reference')
    t_mb[:3,:3] = Rotation.from_quat(q).as_matrix(); t_mb[:3,3] = anchor_row[:3]
    anchor = t_mb@t_bl
    if not rigid(anchor):
        raise ValueError('Invalid anchor SE3')
    save('PRODUCTION_BEFORE.json', stable_snapshot())
    DATA.mkdir(parents=True, exist_ok=False)
    for name in ('build','map','run'):
        (DATA/name).mkdir()
    frames = []
    for i,r in enumerate(group):
        path = Path(r['path']); fmt = pcd_format(path)
        frames.append({'local_index':i,'raw_ordinal_one_based':int(r['inventory_index_zero_based'])+1,
                       'stamp_ns':stamps[i],'path':str(path),'bytes':path.stat().st_size,
                       'sha256':sha(path),'role':'ANCHOR_ONLY' if i==0 else 'EVALUATION',
                       'reference_match_ns':stamps[i],'reference_delta_ns':0,
                       'canonical_path':None,'format':fmt})
    save('TEST_FRAME_PREREGISTRATION.json', {
        'task':'P62-R14B-BASE-CONVENTION-CONDITIONAL-PILOT','assumption':'PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION',
        'baseline':START,'rule':'first21 input-valid consecutive scans strictly after5% full time span; no alternate window',
        'sequence_start_ns':first,'sequence_end_ns':last,'selected_start_zero_based':chosen,
        'frames':frames,'T_B_C':t_bc.tolist(),'T_C_I':t_ci.tolist(),'T_I_L':t_il.tolist(),
        'T_L_I':t_li.tolist(),'T_B_L':t_bl.tolist(),'T_M_B_anchor':t_mb.tolist(),
        'T_M_L_anchor':anchor.tolist(),'anchor_raw_quaternion_norm':float(np.linalg.norm(q)),
        'source_inputs':inputs,'data_root':str(DATA),'initial_available_bytes':free,
        'file_level_reference_axis_certified':False,'deskew':False,'test_pose_values_read':False})
    with (HERE/'SCANS.csv').open('x', newline='') as f:
        fields=['local_index','raw_ordinal','stamp_ns','path','points','header_bytes','point_step']
        writer=csv.DictWriter(f,fields,lineterminator='\n');writer.writeheader()
        for r in frames:
            writer.writerow(dict(local_index=r['local_index'],raw_ordinal=r['raw_ordinal_one_based'],
                stamp_ns=r['stamp_ns'],path=r['path'],points=r['format']['points'],
                header_bytes=r['format']['header_bytes'],point_step=r['format']['point_step']))
    with (HERE/'ANCHOR_MATRIX.txt').open('x') as f:
        f.write(' '.join(format(x,'.17g') for x in anchor.flatten())+'\n')
    print(json.dumps({'selected_raw_ordinals':[r['raw_ordinal_one_based'] for r in frames],
                      'preregistration_sha256':sha(HERE/'TEST_FRAME_PREREGISTRATION.json'),
                      'anchor_T_M_L':anchor.tolist()}, indent=2))


if __name__=='__main__':
    main()
