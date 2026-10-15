#!/usr/bin/env python3
"""Freeze two conditional frame hypotheses; no alignment or parameter fitting."""
import argparse
import hashlib
import json
from pathlib import Path
import re

import cv2
import numpy as np
import open3d as o3d
import yaml

WORKSPACE = Path(__file__).resolve().parents[2]
ROOT = Path('/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def certificate(T):
    R = T[:3, :3]
    error = float(np.linalg.norm(R.T @ R - np.eye(3)))
    det = float(np.linalg.det(R))
    return dict(finite=bool(np.isfinite(T).all()), rotation_orthogonality_error=error,
        determinant=det, rigid=bool(np.isfinite(T).all() and
        np.array_equal(T[3], [0,0,0,1]) and error <= 1e-5 and abs(det-1) <= 1e-5))


def viewpoint(path):
    with path.open('rb') as stream:
        for _ in range(64):
            fields = stream.readline().split()
            if fields and fields[0] == b'VIEWPOINT':
                return [float(value) for value in fields[1:]]
            if fields and fields[0] == b'DATA':
                break
    raise ValueError('Missing PCD VIEWPOINT')


def main(input):
    selection = json.loads((input/'selection.json').read_text())
    lineage = json.loads((input/'gt_lineage.json').read_text())
    if lineage['archive_SHA256'] != 'b8e355f153e5d9124ff6b0b976c951598aadbf21dab5c152839acfa86c87e254' or not lineage['all_translation_and_quaternion_numeric_values_exact']:
        raise ValueError('Unverified official GT numeric lineage')
    runtime = WORKSPACE/'src/dog_prior_map_localization/config/p18_corridor01_ssd_map.yaml'
    config = yaml.safe_load(runtime.read_text())
    E = np.eye(4)
    E[:3,:3] = np.array(config['deskew']['T_imu_lidar_rotation_row_major']).reshape(3,3)
    E[:3,3] = config['deskew']['T_imu_lidar_translation_m']
    calibration = ROOT/'calibration/corridor01_extrinsics.yaml'
    original_E = np.array(yaml.safe_load(calibration.read_text())['laser_to_imu']['data']).reshape(4,4)
    u, _, vh = np.linalg.svd(original_E[:3,:3])
    polar = u @ vh
    if not np.allclose(E[:3,:3],polar,atol=1e-12,rtol=0) or not np.array_equal(E[:3,3],original_E[:3,3]):
        raise ValueError('Frozen runtime extrinsic differs from existing proper projection')
    initializer = Path('/home/jian/livox_ws/superloc_adapter_ws/config/corridor01_init.yaml')
    text = initializer.read_text()
    M_L0 = np.eye(4)
    M_L0[:3,3] = np.fromstring(re.search(r'translation: \[([^]]+)\]',text)[1],sep=',')
    M_L0[:3,:3] = np.fromstring(re.search(r'rotation_matrix: \[([^]]+)\]',text)[1].replace(';',','),sep=',').reshape(3,3)
    N_M = np.linalg.inv(M_L0)
    raw_map = ROOT/'map/corridor01.pcd'
    norm_map = WORKSPACE/'.p18_map_recovery/assets/recovery.oEmedI/corridor01_map_normalized.pcd'
    if sha(raw_map) != '4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a' or sha(norm_map) != '103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f':
        raise ValueError('Map SHA mismatch')
    a, b = [np.asarray(o3d.io.read_point_cloud(str(path)).points) for path in (raw_map,norm_map)]
    if a.shape != b.shape or a.shape != (338210,3) or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError('Map point shape or validity')
    residual = np.linalg.norm(a @ N_M[:3,:3].T + N_M[:3,3] - b,axis=1)
    if residual.max() > 5e-5:
        raise ValueError('Map normalization parity failed')
    prior = ROOT/'initial_pose/corridor01.yaml'
    storage = cv2.FileStorage(str(prior),cv2.FILE_STORAGE_READ)
    try:
        official_Y = np.eye(4)
        official_Y[:3,:3] = storage.getNode('extrinsicRotation_world_darpa').mat()
        official_Y[:3,3] = storage.getNode('extrinsicTranslation_world_darpa').mat().ravel()
    finally:
        storage.release()
    GT = np.array(selection['T_W_GT'])
    H1, H2 = N_M @ GT, N_M @ GT @ E
    matrices = dict(T_I_L_original=original_E,T_I_L_frozen_proper=E,T_M_L0=M_L0,T_N_M=N_M,
        T_world_darpa_unknown_semantics=official_Y,T_W_GT=GT,H1_conditional_T_N_L=H1,H2_conditional_T_N_L=H2)
    for name in ['T_I_L_frozen_proper','T_N_M','T_W_GT','H1_conditional_T_N_L','H2_conditional_T_N_L']:
        if not certificate(matrices[name])['rigid']:
            raise ValueError('Illegal matrix: '+name)
    with (input/'hypotheses.f64').open('xb') as stream:
        stream.write(np.array([H1,H2],dtype='<f8').tobytes())
    record = dict(target_map=str(norm_map), target_map_SHA256=sha(norm_map),
        hypothesis_world_assumption='W_GT equals original raw PCD XYZ frame M; TESTED CONDITION, NOT CERTIFIED',
        T_M_W_certified=None,
        formulas={'H1':'T_N_L = T_N_M * T_W_L_GT (conditional W=M)',
                  'H2':'T_N_L = T_N_M * T_W_I_GT * T_I_L (conditional W=M)'},
        H3_status='NOT_CONSTRUCTED_MISSING_BODY_TO_LIDAR_TRANSFORM',
        H4_status='NOT_CONSTRUCTED_MISSING_CERTIFIED_MAP_TO_GT_WORLD_TRANSFORM',
        matrices={k:dict(values=v.tolist(),**certificate(v)) for k,v in matrices.items()},
        H1_H2_translation_difference_m=float(np.linalg.norm(H1[:3,3]-H2[:3,3])),
        H1_H2_rotation_difference_deg=float(np.arccos(np.clip((np.trace(E[:3,:3])-1)/2,-1,1))*180/np.pi),
        map_parity=dict(points=338210,max_error_m=float(residual.max()),median_error_m=float(np.median(residual)),
            threshold_m=5e-5,raw_viewpoint=viewpoint(raw_map),normalized_viewpoint=viewpoint(norm_map),
            XYZ_transformed=True,viewpoint_was_not_a_second_applied_transform=True),
        source_SHA256={str(p):sha(p) for p in [Path(__file__),runtime,calibration,initializer,prior]},
        ndt_choice='H2 chosen by official IMU evaluation contract plus exact official GT numeric lineage, NOT by NN score',
        full_NDT_budget=1,geometry_selection_GT_bracket=selection['GT_bracket'])
    with (input/'hypothesis_freeze.json').open('x') as stream:
        json.dump(record,stream,indent=2);stream.write('\n')
    print(json.dumps({k:record[k] for k in ['H1_H2_translation_difference_m','H1_H2_rotation_difference_deg','map_parity']},indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    main(**vars(parser.parse_args()))
