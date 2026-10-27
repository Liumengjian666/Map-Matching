#!/usr/bin/env python3
"""Summarize sealed R4 receipts only; never starts ROS or reads a GT trajectory."""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

A = Path(__file__).resolve().parents[1]
D = Path('/home/jian/livox_ws/p63_r4_hdl_full_loop2_data/full_trial_01')


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(values):
    values = np.asarray(list(values), dtype=float)
    finite = values[np.isfinite(values)]
    result = {'n_total': len(values), 'n_finite': len(finite),
              'n_nonfinite': int(len(values) - len(finite))}
    for name, value in zip(('Mean', 'P50', 'P95', 'P99', 'Max', 'Min'),
                           ([np.mean(finite), *np.percentile(finite, [50, 95, 99]),
                             np.max(finite), np.min(finite)] if len(finite) else [None]*6)):
        result[name] = None if value is None else float(value)
    return result


def field(data, name):
    return [float(row[name]) for row in data]


def quality(data):
    names = ['native_fitness_m2', 'native_inlier_error_m2', 'inlier_fraction',
             'iterations', 'source_points', 'step_translation_m', 'step_rotation_deg',
             'UKF_pred_to_NDT_translation_m', 'UKF_pred_to_NDT_rotation_deg']
    return {name: stats(field(data, name)) for name in names}


def pose_difference(first, second):
    first, second = np.asarray(first, float), np.asarray(second, float)
    if not np.isfinite([first, second]).all():
        return [float('nan'), float('nan')]
    translation = float(np.linalg.norm(first[:3]-second[:3]))
    denominator = np.linalg.norm(first[3:])*np.linalg.norm(second[3:])
    rotation = float(np.degrees(2*np.arccos(np.clip(abs(first[3:]@second[3:])/denominator, 0, 1))))
    return [translation, rotation]


def main():
    # Receipt exists only after shutdown and all streams have been closed.
    receipt = json.loads((D/'RUN_RECEIPT.json').read_text())
    identity = json.loads((A/'INPUT_IDENTITY.json').read_text())
    causal, native, callback, adapter, memory = [rows(D/name) for name in
        ('CAUSAL_RESULTS.csv', 'NATIVE_NDT.csv', 'CALLBACKS.csv', 'ADAPTER.csv', 'MEMORY.csv')]
    inputs = rows(A/'INPUT_FRAMES.csv')
    count = len(causal)
    ros_receipts = [json.loads(line) for line in (D/'ROS_RECEIPTS.jsonl').read_text().splitlines()]
    odom_stamps = [r['stamp_ns'] for r in ros_receipts if r['kind'] == 'odom']
    status_stamps = [r['stamp_ns'] for r in ros_receipts if r['kind'] == 'status']
    stamps = [int(r['stamp_ns']) for r in causal]
    # Partial native calls remain in the raw archive even for an execution-contract stop.
    checks = {
        'receipt_callback_count': count == receipt['complete_callbacks'],
        'native_callback_adapter_equal': len(native) == len(callback) == len(adapter) == count,
        'published_processed_equal': receipt['published_scans'] == count,
        'causal_ordinals_prefix': [int(r['ordinal_1based']) for r in causal] == list(range(1, count+1)),
        'causal_stamps_input_prefix': stamps == [int(r['stamp_ns']) for r in inputs[:count]],
        'native_stamps_equal': [int(r['stamp_ns']) for r in native] == stamps,
        'callback_stamps_equal': [int(r['stamp_ns']) for r in callback] == stamps,
        'adapter_stamps_equal': [int(r['stamp_ns']) for r in adapter] == stamps,
        'odom_stamps_equal': odom_stamps == stamps,
        'status_stamps_equal': status_stamps == stamps,
        'callback_call_numbers': [int(r['ndt_calls_so_far']) for r in callback] == list(range(1, count+1)),
        'adapter_xyz_equal': all(r['xyz_sha256'] == f['xyz_sha256'] for r, f in zip(adapter, inputs)),
        'adapter_payload_equal': all(r['raw_message_sha256'] == f['transport_seq_sha256'] for r, f in zip(adapter, inputs)),
        'target_constant': bool(native) and all(r['map_frozen'] == '1' and
            int(r['target_points']) == 424323 and r['target_xyz_sha256'] ==
            'd4ae17e75e22b2c660df1ca08140ad3675479c79f214da6253706c239ca4325e' for r in native),
        'map_file_unchanged': digest(Path(identity['map'])) == identity['map_sha256'],
        'bag_size_mtime_unchanged': (Path(identity['bag']).stat().st_size == identity['bag_bytes'] and
            Path(identity['bag']).stat().st_mtime_ns == identity['bag_mtime_ns']),
    }
    times = {key: stats(field(causal, key)) for key in
             ('ndt_compute_ms', 'callback_ms', 'diagnostic_ms', 'bridge_ms')}
    times['input_to_all_receipts_handshake_ms'] = stats(receipt['per_frame_handshake_wall_ms'])
    peaks = {key: max(field(memory, key), default=0) for key in
             ('manager_KiB', 'rviz_KiB', 'owned_descendants_KiB', 'whole_owned_tree_KiB', 'input_recorder_KiB')}
    peaks['manager_native_getrusage_KiB'] = max(field(native, 'peak_RSS_KiB'), default=0)
    peaks['manager_authoritative_KiB'] = max(peaks['manager_KiB'], peaks['manager_native_getrusage_KiB'])
    interval = [r for r in causal if 600 <= float(r['relative_s']) <= 650]
    interval_state = ('NOT_REACHED' if not interval else
                      'COMPLETE' if float(causal[-1]['relative_s']) >= 650 else 'PARTIAL_STOPPED')
    interval_info = {'state': interval_state, 'fixed_relative_seconds': [600, 650],
                     'frames': len(interval), 'first_ordinal': int(interval[0]['ordinal_1based']) if interval else None,
                     'last_ordinal': int(interval[-1]['ordinal_1based']) if interval else None,
                     'converged': sum(r['native_converged'] == '1' for r in interval),
                     'external_valid': sum(r['external_valid'] == 'True' for r in interval),
                     'quality_and_motion': quality(interval),
                     'repeated_structure_wrong_match': 'NOT_EVALUABLE_WITHOUT_INDEPENDENT_REFERENCE'}
    planned_interval = [r for r in inputs if 600 <=
                        (int(r['stamp_ns'])-identity['first_lidar_header_ns'])/1e9 <= 650]
    interval_info['planned_input_ordinals'] = [int(r['ordinal_1based']) for r in planned_interval]
    invalid = [r for r in causal if r['external_valid'] != 'True']
    groups = []
    for row in invalid:
        ordinal = int(row['ordinal_1based'])
        if not groups or ordinal != int(groups[-1][-1]['ordinal_1based'])+1:
            groups.append([])
        groups[-1].append(row)
    invalid_intervals = [{'first': int(group[0]['ordinal_1based']),
                         'last': int(group[-1]['ordinal_1based']), 'frames': len(group),
                         'start_relative_s': float(group[0]['relative_s']),
                         'end_relative_s': float(group[-1]['relative_s']),
                         'quality_and_motion': quality(group)} for group in groups]
    invalid_only_inlier = all(r['finite'] == 'True' and r['native_converged'] == '1' and
                             np.isfinite(float(r['native_fitness_m2'])) and
                             np.isfinite(float(r['native_inlier_error_m2'])) and
                             float(r['inlier_fraction']) < .50 for r in invalid)
    runtime_names = ['ordinal_1based', 'stamp_ns', 'relative_s', 'ndt_compute_ms', 'callback_ms',
                     'diagnostic_ms', 'bridge_ms', 'manager_getrusage_peak_KiB']
    with (A/'RUNTIME_MEMORY.csv').open('w', newline='') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(runtime_names + ['input_to_all_receipts_handshake_ms'])
        for row, elapsed in zip(causal, receipt['per_frame_handshake_wall_ms']):
            writer.writerow([row[k] for k in runtime_names] + [elapsed])
    trajectory_names = ['ordinal_1based', 'stamp_ns', 'relative_s', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw',
                        'raw_tx', 'raw_ty', 'raw_tz', 'raw_qx', 'raw_qy', 'raw_qz', 'raw_qw', 'external_valid']
    with (A/'TRAJECTORY.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=trajectory_names, extrasaction='ignore', lineterminator='\n')
        writer.writeheader()
        writer.writerows(causal)
    previous_raw = [0, 0, 0, 0, 0, 0, 1]
    raw_motion = []
    added = ['raw_step_translation_m', 'raw_step_rotation_deg', 'raw_to_UKF_translation_m',
             'raw_to_UKF_rotation_deg', 'native_probability', 'target_points', 'target_xyz_sha256', 'map_frozen']
    with (A/'NDT_RESULTS.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(causal[0])+added, lineterminator='\n')
        writer.writeheader()
        for row, trace in zip(causal, native):
            raw = [float(row[k]) for k in ('raw_tx', 'raw_ty', 'raw_tz', 'raw_qx', 'raw_qy', 'raw_qz', 'raw_qw')]
            filtered = [float(row[k]) for k in ('px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw')]
            motion = pose_difference(previous_raw, raw)+pose_difference(raw, filtered)
            raw_motion.append(motion)
            writer.writerow(dict(row, **dict(zip(added, motion + [trace['probability'], trace['target_points'],
                               trace['target_xyz_sha256'], trace['map_frozen']]))))
            previous_raw = raw
    counters = {'planned_input': 10905, 'published': receipt['published_scans'], 'completed': count,
                'native_align_calls': len(native), 'native_converged': sum(r['converged'] == '1' for r in native),
                'UKF_completed_update_receipts': len(odom_stamps),
                'UKF_internal_counter_instrumented': False,
                'external_valid': sum(r['external_valid'] == 'True' for r in causal),
                'longest_external_invalid_run': max([int(r['invalid_run']) for r in causal], default=0),
                'nonfinite_output_count': sum(r['finite'] != 'True' for r in causal),
                'single_steps_over_R3_threshold': sum(float(r['step_translation_m']) > 2 or
                                                       float(r['step_rotation_deg']) > 20 for r in causal),
                'max_consecutive_large_steps': max([int(r['jump_run']) for r in causal], default=0),
                'extra_align_calls': 0, 'imu_calls': 0, 'formal_input_trials': 1}
    result = {'TASK': 'P63-R4-HDL-FULL-LOOP2-FROZEN-BASELINE',
              'verdict': receipt['verdict'] if all(checks.values()) else 'EXECUTION_CONTRACT_FAIL',
              'stop_reason': receipt['reason'], 'postrun_lineage_checks': checks, 'counters': counters,
              'R4_diagnostic_CODE_SHA': json.loads((D/'COMMAND.json').read_text())['R4_diagnostic_CODE_SHA'],
              'START_SHA': '78dfc7ce6649b4c8fb25626e55a8e3efd82ef70b',
              'R3_native_CODE_SHA': 'eb6ff37ab791193ff1a803952d79e148b9ecea6c',
              'input': identity, 'full_planned_sensor_span_s': identity['sensor_span_s'],
              'processed_frame_coverage_fraction': count/10905,
              'processed_last_relative_s': receipt['last_processed_relative_s'],
              'run_wall_s_including_startup_shutdown': receipt['wall_s'],
              'time_ms': times, 'RSS_KiB': peaks, 'quality_and_motion': quality(causal),
              'raw_NDT_motion_and_filter_difference': {
                  name: stats([row[i] for row in raw_motion]) for i, name in enumerate(added[:4])},
              'observation_600_650s': interval_info,
              'external_invalid_intervals': invalid_intervals,
              'external_invalid_failing_component_is_inlier_fraction': invalid_only_inlier,
              'first_filtered_pose_xyzw': receipt['first_output'], 'last_filtered_pose_xyzw': receipt['last_output'],
              'sampled_RSS_interval_s': 0.05, 'accuracy': 'NOT_EVALUABLE_NO_CERTIFIED_INDEPENDENT_GT',
              'real_time_10Hz': 'NOT_ESTABLISHED_OFFLINE_HANDSHAKE',
              'map_utm_transform': False, 'GT_or_old_trajectory_read': False,
              'pre_launch_rejections_without_input': 1,
              'memory_scope': 'RSS sums count shared mappings repeatedly; not unique physical RAM. Manager includes localization, map server and trace; whole owned tree also includes RViz, ROS tools and input/recorder.'}
    (A/'results.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    (A/'evidence/POSTRUN_LINEAGE_CHECKS.json').write_text(json.dumps(checks, indent=2)+'\n')
    print(json.dumps({'verdict': result['verdict'], 'counters': counters, 'time_ms': times,
                      'RSS_KiB': peaks, 'observation': {k:v for k,v in interval_info.items() if k != 'quality_and_motion'}}, indent=2))


if __name__ == '__main__':
    main()
