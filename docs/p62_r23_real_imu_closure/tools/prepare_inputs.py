#!/usr/bin/env python3
"""R23 anchor-only initialization and real IMU coverage. No reference CSV read."""
import bisect
import csv
import hashlib
import json
import math
import subprocess
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parents[1]
REPO = HERE.parents[1]
R22 = REPO / 'docs/p62_r22_frozen_relative_holdout'
DATA = Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment')
IMU = DATA / 'raw_format/ouster_imu/data.csv'
TILES = Path('/media/jian/HIKVISION/paper rosbag/NewerCollege/.p45_local_map/tiles/tile_manifest.json')
STABLE = Path('/home/jian/livox_ws/dog_visual_loc_ws')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1048576), b''):
            h.update(b)
    return h.hexdigest()


def save_json(path, value):
    with path.open('x') as f:
        json.dump(value, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.write('\n')


def stable_snapshot():
    def git(*args):
        return subprocess.check_output(['git', '--no-optional-locks', '-C', str(STABLE), *args])
    index = Path(git('rev-parse', '--git-path', 'index').decode().strip())
    if not index.is_absolute():
        index = STABLE / index
    protected = ['src/dog_prior_map_localization/src/' + s for s in (
        'dog_prior_map_ndt_node.cpp', 'dog_prior_map_ekf_node.cpp',
        'dog_prior_map_ekf_node_core.cpp', 'local_map_tiles.cpp', 'imu_processor.cpp')]
    protected += ['src/dog_prior_map_localization/launch/' + s for s in (
        'dog_prior_map_localization_split.launch', 'newer_college_local_map.launch')]
    protected += ['src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml']
    return {'HEAD': git('rev-parse', 'HEAD').decode().strip(),
            'branch': git('branch', '--show-current').decode().strip(),
            'status_sha256': hashlib.sha256(git('status', '--porcelain=v1', '-uall')).hexdigest(),
            'index_sha256': sha(index),
            'protected_files_sha256': {p: sha(STABLE / p) if (STABLE / p).is_file() else None for p in protected}}


def load_imu():
    records = []
    with IMU.open(newline='') as f:
        for row in csv.reader(f):
            if not row or row[0].startswith('#'):
                continue
            assert len(row) == 9
            stamp = int(row[1]) * 10**9 + int(row[2])
            assert 0 <= int(row[2]) < 10**9
            values = [float(x) for x in row[3:]]
            assert all(math.isfinite(x) for x in values)
            if records:
                assert stamp > records[-1]['stamp_ns']
            records.append({'counter': int(row[0]), 'stamp_ns': stamp,
                            'gyro_radps': values[:3], 'accel_mps2': values[3:]})
    assert records
    return records


def inspect_pcd(frame):
    path = Path(frame['path'])
    raw = path.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == frame['pcd_sha256'], str(path)
    assert len(raw) == frame['pcd_bytes']
    header_end = raw.index(b'DATA binary\n') + len(b'DATA binary\n')
    header = raw[:header_end].decode('ascii')
    required = ('FIELDS x y z intensity t reflectivity ring noise range\n',
                'SIZE 4 4 4 4 4 2 1 2 4\n', 'TYPE F F F F U U U U U\n',
                'COUNT 1 1 1 1 1 1 1 1 1\n', 'WIDTH 1024\n', 'HEIGHT 64\n',
                'POINTS 65536\n', 'DATA binary\n')
    assert all(t in header for t in required)
    declared_end = header_end + 65536 * 29
    assert declared_end <= len(raw) and not any(raw[declared_end:])
    times = np.ndarray((65536,), dtype='<u4', buffer=raw, offset=header_end + 16, strides=(29,))
    lo, hi = int(times.min()), int(times.max())
    assert 0 <= lo <= hi and 0 < hi <= 150000000
    return {'header_bytes': header_end, 'point_step': 29, 'declared_points': 65536,
            'zero_tail_bytes': len(raw) - declared_end,
            'point_t_min_ns': lo, 'point_t_max_ns': hi,
            'scan_end_ns': frame['stamp_ns'] + hi,
            'declared_records_and_zero_padding_checked': True}


def main():
    out = HERE / 'input'
    configs = HERE / 'config'
    out.mkdir(exist_ok=False)
    configs.mkdir(exist_ok=True)
    before = stable_snapshot()
    old = json.loads((R22 / 'WINDOW_IDENTITY.json').read_text())
    runtime_path = REPO / 'src/dog_prior_map_localization/config/p41_newer_college_runtime.yaml'
    original_init_path = REPO / 'src/dog_prior_map_localization/config/p44_newer_college_base_oracle_start.yaml'
    runtime = yaml.safe_load(runtime_path.read_text())
    init_prior = yaml.safe_load(original_init_path.read_text())['initialization']
    t_il = np.eye(4)
    t_il[:3, :3] = np.asarray(runtime['deskew']['T_imu_lidar_rotation_row_major']).reshape(3, 3)
    t_il[:3, 3] = runtime['deskew']['T_imu_lidar_translation_m']
    assert np.isfinite(t_il).all() and np.linalg.norm(t_il[:3, :3].T @ t_il[:3, :3] - np.eye(3)) < 1e-10
    assert abs(np.linalg.det(t_il[:3, :3]) - 1) < 1e-10
    assert runtime['deskew']['reference_time'] == runtime['lidar_update']['scan_reference_time'] == 'start'
    assert runtime['deskew']['state_frame'] == 'imu'
    records = load_imu()
    ns = [r['stamp_ns'] for r in records]
    tiles = json.loads(TILES.read_text())
    assert tiles['tile_size_m'] == 20 and tiles['global_target_point_count'] > 0
    manifest = {'task': 'P62-R23-REAL-IMU-LOCALIZATION-CLOSURE',
                'start_sha': '5b3a18405e6168befbe55a8158f282f09607d684',
                'source_window_identity': str(R22 / 'WINDOW_IDENTITY.json'),
                'source_window_identity_sha256': sha(R22 / 'WINDOW_IDENTITY.json'),
                'reference_values_read': False,
                'initialization_pose_source': 'frozen R22 anchor T_map_lidar only',
                'coordinate_assumption': 'PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION; conditional CSV axis identity',
                'T_A_B_definition': 'maps coordinates in B into A',
                'T_imu_lidar': t_il.tolist(),
                'T_lidar_imu': np.linalg.inv(t_il).tolist(),
                'init_conversion': 'T_map_imu = T_map_lidar @ inverse(T_imu_lidar)',
                'c3_rule': 'left multiply T_map_lidar rotation by map-Z +8deg; keep lidar translation; recompute T_map_imu',
                'lidar_time': 'integer filename/header timestamp is scan start; t:uint32 nanoseconds from scan start; derived time:float seconds',
                'deskew_reference': 'scan start; existing ekf_imu_fastlio state_frame=imu; no new Deskew model',
                'imu_time': 'original Ouster CSV sec+nsec, no fitted time shift',
                'imu_source': {'path': str(IMU), 'sha256': sha(IMU), 'records': len(records),
                               'first_stamp_ns': ns[0], 'last_stamp_ns': ns[-1]},
                'map': {'tile_manifest_path': str(TILES), 'tile_manifest_sha256': sha(TILES),
                        'tiles': len(tiles['tiles']), 'global_target_points': tiles['global_target_point_count'],
                        'tile_size_m': tiles['tile_size_m'], 'source_path': tiles['input_path'],
                        'source_sha256_from_manifest': tiles['input_sha256'],
                        'tile_payloads_not_rehashed_in_preparation': True},
                'warmup': {'performed': False,
                           'condition': 'explicit anchor state with existing zero-velocity/zero-bias priors; these are not measured zero motion or estimated bias',
                           'gravity_map_mps2': init_prior['gravity_map_mps2'],
                           'first_interval': 'real anchor boundary sample seeds midpoint; replay every subsequent real IMU sample'},
                'runtime_config_sha256': sha(runtime_path),
                'original_initialization_config_sha256': sha(original_init_path),
                'stable_before': before, 'windows': {}}
    for role in ('W1', 'W3'):
        frozen = old['windows'][role]
        frames = frozen['frames']
        assert len(frames) == 61
        anchor = int(frames[0]['stamp_ns'])
        first_eval = int(frames[1]['stamp_ns'])
        boundary_idx = bisect.bisect_right(ns, anchor) - 1
        assert boundary_idx >= 0
        boundary = records[boundary_idx]
        assert 0 <= anchor - boundary['stamp_ns'] <= 20000000
        end_idx = bisect.bisect_right(ns, frames[-1]['stamp_ns'] + 150000000)
        assert end_idx < len(ns)
        max_gap = max(b - a for a, b in zip(ns[boundary_idx:end_idx], ns[boundary_idx+1:end_idx+1]))
        assert max_gap <= 20000000
        checked = []
        for index, f in enumerate(frames):
            assert f['local_index'] == index
            current = dict(f)
            current.update(inspect_pcd(f))
            bracket_index = bisect.bisect_right(ns, current['scan_end_ns'])
            assert 0 < bracket_index < len(ns)
            assert ns[bracket_index] - ns[bracket_index - 1] <= 20000000
            current['post_end_imu_bracket_ns'] = ns[bracket_index]
            current['role'] = 'ANCHOR_ONLY_NO_NDT' if index == 0 else 'EVALUATION'
            current['historical_r22_source_fnv_semantics'] = 'raw preprocessing identity; not post-Deskew Source identity'
            checked.append(current)
        source_scan_table = R22 / role.lower() / 'SCANS.csv'
        with (out / (role + '_R22_RAW_SCANS.csv')).open('x') as f:
            f.write(source_scan_table.read_text())
        window = {'anchor_raw_ordinal': frames[0]['raw_ordinal'],
                  'anchor_stamp_ns': anchor,
                  'first_evaluation_raw_ordinal': frames[1]['raw_ordinal'],
                  'first_evaluation_stamp_ns': first_eval,
                  'last_evaluation_stamp_ns': frames[-1]['stamp_ns'],
                  'evaluation_scan_count': 60,
                  'publisher_start_stamp_ns': first_eval,
                  'imu_replay_start_stamp_ns': anchor,
                  'expected_boundary_imu_stamp_ns': boundary['stamp_ns'],
                  'boundary_age_ns': anchor - boundary['stamp_ns'],
                  'boundary_imu': boundary,
                  'first_published_imu_ns': records[boundary_idx + 1]['stamp_ns'],
                  'max_imu_gap_ns': max_gap,
                  'covered_imu_records_including_boundary_and_tail': end_idx - boundary_idx + 1,
                  'no_anchor_ndt': True,
                  'smoke': {'supported': role == 'W1', 'arm': 'C0', 'first_eval_ordinal': frames[1]['raw_ordinal'], 'scan_count': 20},
                  'frames': checked, 'initializations': {}}
        base = np.asarray(frozen['anchor_T_M_L'], dtype=float)
        for arm in ('C0', 'C3'):
            t_ml = base.copy()
            if arm == 'C3':
                th = math.radians(8.0)
                rz = np.array([[math.cos(th), -math.sin(th), 0],
                               [math.sin(th), math.cos(th), 0], [0, 0, 1]])
                t_ml[:3, :3] = rz @ t_ml[:3, :3]
            t_mi = t_ml @ np.linalg.inv(t_il)
            closure = float(np.linalg.norm(t_mi @ t_il - t_ml))
            assert closure < 1e-12 and np.isfinite(t_mi).all()
            init = {'explicit_state_enable': True,
                    'state_stamp_sec': anchor // 10**9, 'state_stamp_nsec': anchor % 10**9,
                    'map_T_imu_translation_m': t_mi[:3, 3].tolist(),
                    'map_T_imu_rotation_row_major': t_mi[:3, :3].reshape(-1).tolist(),
                    'velocity_map_mps': init_prior['velocity_map_mps'],
                    'gyro_bias_imu_radps': init_prior['gyro_bias_imu_radps'],
                    'accel_bias_imu_mps2': init_prior['accel_bias_imu_mps2'],
                    'gravity_map_mps2': init_prior['gravity_map_mps2'],
                    'boundary_imu_stamp_sec': boundary['stamp_ns'] // 10**9,
                    'boundary_imu_stamp_nsec': boundary['stamp_ns'] % 10**9,
                    'boundary_imu_accel_mps2': boundary['accel_mps2'],
                    'boundary_imu_gyro_radps': boundary['gyro_radps'],
                    'ndt_seed_enable': True,
                    'ndt_seed_map_T_lidar_translation_m': t_ml[:3, 3].tolist(),
                    'ndt_seed_map_T_lidar_rotation_row_major': t_ml[:3, :3].reshape(-1).tolist()}
            path = configs / (role.lower() + '_' + arm.lower() + '_initialization.yaml')
            with path.open('x') as f:
                yaml.safe_dump({'experiment': {'initialization_contract': 'CONDITIONAL_BASE_FRAME',
                                              'initialization_mode': 'ANCHOR_REFERENCE_ONLY_ZERO_VELOCITY_BIAS_PRIORS',
                                              'window': role, 'initialization_arm': arm},
                                'initialization': init}, f, sort_keys=False)
            window['initializations'][arm] = {'path': str(path), 'sha256': sha(path),
                                               'T_map_lidar': t_ml.tolist(), 'T_map_imu': t_mi.tolist(),
                                               'closure_norm': closure}
        manifest['windows'][role] = window
    after = stable_snapshot()
    assert after == before, 'stable state changed during read-only input preparation'
    manifest['stable_after'] = after
    manifest['stable_unchanged'] = True
    manifest['G0_input_status'] = 'PASS_REAL_IMU_COVERAGE_ANCHOR_EXTRINSIC_AND_PCD_READABILITY'
    manifest['ndt_align_calls'] = 0
    manifest['ros_processes_started'] = 0
    save_json(out / 'INITIALIZATION_AND_TIME_CONTRACT.json', manifest)
    print('R23_INPUT_CONTRACT_PASS; anchor reference only; no new GT read; no ROS/NDT')
    for name, window in manifest['windows'].items():
        print(name, 'lidar_start', window['publisher_start_stamp_ns'],
              'imu_replay_start', window['imu_replay_start_stamp_ns'],
              'boundary', window['expected_boundary_imu_stamp_ns'],
              'max_gap_ns', window['max_imu_gap_ns'])


if __name__ == '__main__':
    main()
