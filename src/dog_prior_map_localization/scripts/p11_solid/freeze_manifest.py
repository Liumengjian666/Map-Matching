#!/usr/bin/env python3
"""Archive already verified input identity and exact source/binary hashes.

The two large bag hashes below were computed from whole files in this task;
this script never reprocesses raw clouds or reads GT.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(4*1024*1024), b''):
            h.update(b)
    return {'path': str(path), 'bytes': path.stat().st_size, 'sha256': h.hexdigest()}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--workspace', type=Path, required=True)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    package = a.workspace / 'src/dog_prior_map_localization'
    source_paths = list((package / 'third_party/solid').rglob('*')) + [
        package / 'CMakeLists.txt', package / 'launch/dog_prior_map_localization_split.launch',
        package / 'config/dog_prior_map_localization_ndt.yaml',
        package / 'src/solid_place_recognition.cpp', package / 'src/solid_shadow_node.cpp',
        package / 'src/dog_prior_map_ndt_node.cpp', package / 'src/fusion/ndt_observation.cpp',
        package / 'src/imu_processor.cpp', package / 'src/core/oosm_fixed_lag_scheduler.cpp',
        package / 'scripts/build_livox_keyframe_scan_db.py',
        package / 'scripts/solid_reference_database_builder.cpp',
        package / 'include/dog_prior_map_localization/solid_place_recognition.hpp',
        a.workspace / 'devel/lib/dog_prior_map_localization/dog_prior_map_ndt_node_cpp',
        a.workspace / 'devel/lib/dog_prior_map_localization/dog_prior_map_ekf_node_cpp',
        a.workspace / 'devel/lib/dog_prior_map_localization/solid_shadow_node_cpp',
        a.workspace / 'devel/lib/libp11_solid_place_recognition.so',
    ]
    map_path = Path('/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd')
    poses = Path('/home/jian/rosbag/loop2/loop2mapping/pcd/lidar_poses.txt')
    result = {
        'protocol': 'P11_SOLID_LOOP2_REFERENCE_LOOP3_QUERY_V1',
        'GT_LOADED': False, 'reference_sequence': 'Loop2 independent mapping',
        'query_sequence': 'Loop3 independent recording',
        'raw_mapping_bag': {'path': '/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag',
                            'sha256': 'd17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae',
                            'verification': 'WHOLE_FILE_RECOMPUTED_THIS_TASK'},
        'raw_query_bag': {'path': '/media/jian/HIKVISION/Jian_FASTLIVO_Data/dogrosbag/loop2_map_match_experiment_2026_08_08_17_36_19/dateset/loop3/bag/loop3_2026-08-11-16-09-07.bag',
                          'sha256': 'e7663609a90fd9e59552fefd7f2c4f9c339e8464610f86bfa1e8f2cff2e79df2',
                          'verification': 'WHOLE_FILE_RECOMPUTED_THIS_TASK',
                          'scans': 5817, 'imu_samples': 116330},
        'map': digest(map_path), 'mapping_state_poses': digest(poses),
        'reference_frame': 'camera_init map; FAST-LIVO2 state/IMU origin',
        'reference_time': 'raw scan header paired to original mapping pose, max .12s; no interpolation',
        'reference_pose_limitation': 'Historical map-generation physical lidar/imu extrinsic not independently closed; never use as exact NDT pose',
        'runtime_source': 'exact NDT filtered_points; livox_frame; deskew disabled unchanged',
        'source_and_binaries': [digest(f) for f in source_paths if f.is_file()],
        'database': digest(a.root / 'solid_database.txt'),
        'reference_pcds': [digest(f) for f in sorted((a.root / 'reference_scans/scans').glob('*.pcd'))],
        'generated_pose_index': digest(a.root / 'reference_scans/keyframe_poses.txt'),
        'code_commit': subprocess.check_output(['git', '-C', str(a.workspace), 'rev-parse', 'HEAD'], text=True).strip(),
    }
    if result['map']['sha256'] != '016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca':
        raise RuntimeError('Map changed')
    if result['mapping_state_poses']['sha256'] != '42d0c3060f77f83ea438916c0b305a9f7320c3ff7161148a7a475f4791022ed9':
        raise RuntimeError('Mapping poses changed')
    a.output.write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
