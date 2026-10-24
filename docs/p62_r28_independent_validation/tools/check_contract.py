#!/usr/bin/env python3
"""Read-only asset/frame check. Does not parse reference pose values or run ROS."""
import hashlib
import importlib.util
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import yaml

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parents[1]
REPO = HERE.parents[1]
DATA = Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment')


def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda: f.read(1048576), b''):
            h.update(b)
    return h.hexdigest()


def save(name, obj):
    if (HERE / name).exists():
        assert json.loads((HERE / name).read_text()) == obj, 'refuse to replace different evidence: ' + name
        return
    with (HERE / name).open('x') as f:
        json.dump(obj, f, indent=2, allow_nan=False)
        f.write('\n')


spec = importlib.util.spec_from_file_location('r23_prepare', REPO / 'docs/p62_r23_real_imu_closure/tools/prepare_inputs.py')
r23 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(r23)
save('STABLE_BEFORE.json', r23.stable_snapshot())
calib = DATA / 'calibration/kalibr_output'
cam = calib / 'cam-ouster-imu/camchain-ouster_imu-cam-rooster_2020-03-11-10-05-35_0.yaml'
lidar = calib / 'ouster_imu_lidar_transforms.yaml'
reference = DATA / 'ground_truth/registered_poses.csv'
assert sha(reference) == 'b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e'
ci = np.asarray(yaml.safe_load(cam.read_text())['cam0']['T_cam_imu'])
il = np.eye(4)
il[:3, :3] = np.diag([-1., -1., 1.])
raw = yaml.safe_load(lidar.read_text())
assert raw['os1_lidar_to_os1_imu']['rotation'] == [0., 0., -1., 0.]
il[:3, 3] = raw['os1_lidar_to_os1_imu']['translation']
li = np.eye(4)
li[:3, :3] = np.diag([-1., -1., 1.])
li[:3, 3] = raw['os1_imu_to_os1_lidar']['translation']
bc = np.array([[0., 0., 1., 0.], [-1., 0., 0., 0.], [0., -1., 0., 0.], [0., 0., 0., 1.]])
matrices = {'T_B_C': bc, 'T_C_I': ci, 'T_I_L': il, 'T_L_I': li,
            'T_C_L': ci @ il, 'T_B_L': bc @ ci @ il}
checks = {}
for k, t in matrices.items():
    err = float(np.linalg.norm(t[:3, :3].T @ t[:3, :3] - np.eye(3)))
    det = float(np.linalg.det(t[:3, :3]))
    assert np.isfinite(t).all() and err < 1e-10 and abs(det - 1) < 1e-10
    assert np.array_equal(t[3], [0., 0., 0., 1.])
    checks[k] = {'orthogonality_error': err, 'determinant': det}
assert np.linalg.norm(il @ li - np.eye(4)) < 1e-12
save('FRAME_MATRICES.json', {'units': 'metres; quaternion xyzw',
     'convention': 'T_A_B maps B into A; column vectors; rightmost transform first',
     'matrices': {k: v.tolist() for k, v in matrices.items()}, 'checks': checks,
     'T_I_L_inverse_error': float(np.linalg.norm(il @ li - np.eye(4))),
     'base_optical_axis_rotation_deg': math.degrees(math.acos((np.trace(bc[:3, :3]) - 1) / 2)),
     'file_semantics_confirmed': False})
inputs = [reference, cam, lidar]
inputs += [REPO / x for x in ['docs/p40_newer_college_download/download_manifest.json',
           'docs/p62_r19_native_ndt_basin_stress_test/WINDOW_PREREGISTRATION.json',
           'docs/p62_r22_frozen_relative_holdout/WINDOW_IDENTITY.json',
           'src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp',
           'src/dog_prior_map_localization/src/dog_prior_map_ekf_node_core.cpp']]
save('INPUT_IDENTITIES.json', [{'path': str(p), 'bytes': p.stat().st_size, 'sha256': sha(p)} for p in inputs])
# No reference rows are decoded. Raw ordinal is one-based, as in R19/R22.
def stamp(p):
    sec, ns = re.fullmatch(r'cloud_(\d+)_(\d{9})\.pcd', p.name).groups()
    return int(sec) * 10**9 + int(ns)
files = sorted((p for p in (DATA / 'raw_format/ouster_scan').glob('cloud_*.pcd')
                if re.fullmatch(r'cloud_\d+_\d{9}\.pcd', p.name)), key=stamp)
assert len(files) == 15301
prior = [(767, 787), (7552, 7582), (10972, 11032), (14542, 14602),
         (2600, 2660), (4800, 4860), (6500, 6560), (9400, 9460)]
windows = {}
for label, anchor in [('A', 12000), ('B', 13500)]:
    assert not any(anchor <= hi and lo <= anchor + 60 for lo, hi in prior)
    frames = [{'raw_ordinal': n, 'stamp_ns': stamp(files[n-1]), 'path': str(files[n-1]),
               'pcd_sha256': sha(files[n-1]), 'role': 'ANCHOR' if n == anchor else 'EVALUATION'}
              for n in range(anchor, anchor + 61)]
    windows[label] = {'anchor': anchor, 'evaluation': [anchor+1, anchor+60], 'frames': frames,
                     'source_fnv': None, 'map_identity': None, 'initialization_matrix': None}
save('WINDOW_CANDIDATES.json', {'status': 'INDEX_AND_RAW_IDENTITY_FROZEN_NOT_EXECUTION_READY',
     'reason_A_replaced': '11000-11060 intersects R19 10972-11032; next multiple of 1000 after exposed window is 12000',
     'reference_pose_values_read': False, 'known_prior_formal_intervals': prior,
     'input_geometry_prescreen_exposure': 'R16/R19 used sparse geometry-only prescreens; not a claim of never-seen raw geometry',
     'windows': windows})
print('SE3/calibration/inverse checks PASS; raw identity 122 scans frozen; G0 remains BLOCKED.')
