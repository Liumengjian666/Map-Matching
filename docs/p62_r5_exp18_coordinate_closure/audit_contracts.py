#!/usr/bin/env python3
"""Pure algebra and frozen-metadata checks; never reads large sensor files."""
import argparse
import csv
import hashlib
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
R4 = HERE.parent/'p62_r4_exp18_g0_metadata_audit'
NS = '{http://www.ros.org/wiki/xacro}'
SMALL_FILE_LIMIT = 8 * 1024 * 1024


def small_bytes(path):
    """Reject unexpected inputs before reading; bound even a concurrent growth."""
    if path.is_symlink() or path.stat().st_size > SMALL_FILE_LIMIT:
        raise RuntimeError('Refuse symlink/oversize metadata: '+str(path))
    with path.open('rb') as handle:
        content = handle.read(SMALL_FILE_LIMIT + 1)
    if len(content) > SMALL_FILE_LIMIT:
        raise RuntimeError('Metadata grew past read limit: '+str(path))
    return content


def origin(element):
    substitutions = {'${pi}': np.pi, '${-pi}': -np.pi, '${pi/2}': np.pi/2}
    xyz = [float(x) for x in element.attrib['xyz'].split()]
    angles = [substitutions[x] if x in substitutions else float(x)
              for x in element.attrib['rpy'].split()]
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', angles).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def from_quaternion(values):
    q = np.asarray(values['quaternion'], dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-5:
        raise ValueError('Invalid declared xyzw quaternion')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(q).as_matrix()
    matrix[:3, 3] = values['translation']
    if not np.isfinite(matrix).all():
        raise ValueError('Nonfinite transform')
    return matrix


def matrices():
    phasma = ET.parse(R4/'sources/phasma.urdf.xacro').getroot()
    alpha = ET.parse(HERE/'sources/alphasense.xacro').getroot()
    hesai = ET.parse(HERE/'sources/hesai_parametric.xacro').getroot()
    t_p_mount = origin(phasma.find(NS+'alphasense').find('origin'))
    t_mount_i = origin(alpha.find('.//joint[@name="imu_joint"]/origin'))
    t_p_h = origin(phasma.find(NS+'hesai_device').find('origin'))
    t_h_l = origin(hesai.find('.//joint[@name="hesai_base_to_pandar"]/origin'))
    t_p_i = t_p_mount @ t_mount_i
    t_p_l = t_p_h @ t_h_l
    config = yaml.safe_load(Path('/home/jian/livox_ws/p62_r2_hilti_exp18_g0_data/assets/calibration/calibration_files/lidar_calibration.yaml').read_text())
    sensors = config['sensors']
    return {'T_I_L_URDF': np.linalg.solve(t_p_i, t_p_l),
            'T_I_P_URDF': np.linalg.inv(t_p_i),
            'T_I_L_YAML_xyzw': from_quaternion(sensors['PandarXT-32']['extrinsics']),
            'T_I_P_YAML_xyzw': from_quaternion(sensors['measurement_tip']['extrinsics']),
            'T_I_base_URDF': origin(phasma.find('.//joint[@name="alphasense_imu_to_base"]/origin'))}


def profile_transform():
    text = (HERE/'sources/mola_hilti2022.sh').read_text()
    imu_branch = text.split('    imu)')[1].split('    lidar)')[0]
    vals = {key: float(re.search(r'\$\{LIDAR_POSE_'+key+r':=([^}]+)\}', imu_branch)[1])
            for key in ['X', 'Y', 'Z', 'YAW', 'PITCH', 'ROLL']}
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('ZYX', [vals['YAW'], vals['PITCH'], vals['ROLL']], degrees=True).as_matrix()
    matrix[:3, 3] = [vals[k] for k in ['X', 'Y', 'Z']]
    return matrix


def boundary_policy():
    with (R4/'imu_coverage.csv').open() as handle:
        rows = list(csv.DictReader(handle))
    blocked = [int(r['scan_index']) for r in rows if r['hypothetical_support_if_absolute_seconds'] != 'COVERED']
    if blocked != [1] or len(rows) != 1094:
        raise RuntimeError('Unexpected R4 boundary population')
    return {'raw_scans_preserved': 1094, 'prospective_complete_fusion_scans': 1093,
            'first_eligible_scan': 2, 'last_eligible_scan': 1094,
            'excluded_from_fusion_only': [1], 'current_time_unit_certification': 'STILL_CONDITIONAL',
            'rule': 'Proposed for next approval only: no point cropping, no extrapolation; retain scan 1 as unavailable',
            'missing_start_ns': 47303251}


def snapshot():
    records = []
    for row in json.loads((R4/'asset_identity_after.json').read_text())['assets']:
        path = Path(row['path'])
        stat = path.stat()
        large = stat.st_size > 100000000
        current = None if large else hashlib.sha256(small_bytes(path)).hexdigest()
        if stat.st_size != row['bytes'] or (not large and current != row['sha256']):
            raise RuntimeError('Asset mismatch: '+str(path))
        records.append({'path': str(path), 'bytes': stat.st_size, 'device': stat.st_dev,
                        'inode': stat.st_ino, 'mtime_ns': stat.st_mtime_ns, 'ctime_ns': stat.st_ctime_ns,
                        'R4_verified_sha256': row['sha256'], 'current_sha256': current,
                        'method': 'STAT_ONLY_NO_NEW_CONTENT_HASH' if large else 'SHA256_RECHECK'})
    frozen = {}
    for path in sorted(R4.rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            frozen[str(path.relative_to(R4))] = hashlib.sha256(small_bytes(path)).hexdigest()
    return {'assets': records, 'R4_files_sha256': frozen,
            'large_input_content_rereads': 0, 'GT_files_hashed_only': 2,
            'GT_numeric_semantic_reads_current_round': 0}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--snapshot', choices=['before', 'after'])
    args = parser.parse_args()
    if args.snapshot:
        value = snapshot()
        if args.snapshot == 'after':
            before = json.loads((HERE/'asset_identity_before.json').read_text())
            if value != before:
                raise RuntimeError('Frozen assets or metadata changed')
        destination = HERE/('asset_identity_'+args.snapshot+'.json')
    else:
        mats = matrices()
        mats['T_I_L_MOLA_profile'] = profile_transform()
        comparisons = {}
        for a, b in [('T_I_L_YAML_xyzw', 'T_I_L_MOLA_profile'), ('T_I_L_YAML_xyzw', 'T_I_L_URDF'), ('T_I_P_YAML_xyzw', 'T_I_P_URDF')]:
            relative = np.linalg.solve(mats[a], mats[b])
            comparisons[a+'__'+b] = {'translation_difference_m': float(np.linalg.norm(relative[:3, 3])),
                                    'rotation_difference_deg': float(np.degrees(np.linalg.norm(Rotation.from_matrix(relative[:3, :3]).as_rotvec())))}
        value = {'transforms': {k: v.tolist() for k,v in mats.items()}, 'comparisons': comparisons,
                 'boundary_policy': boundary_policy(), 'T_M_W': 'NOT_ESTABLISHED_NO_MATRIX_ASSUMED',
                 'warning': 'Algebra and consumer agreement do not certify physical accuracy or GT export semantics'}
        destination = HERE/'numerical_contract_checks.json'
    with destination.open('x') as handle:
        json.dump(value, handle, indent=2)
        handle.write('\n')
    print(destination)


if __name__ == '__main__':
    main()
