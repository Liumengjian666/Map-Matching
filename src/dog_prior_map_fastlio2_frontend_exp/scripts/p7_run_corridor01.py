#!/usr/bin/env python3
"""Run the frozen P7-B Corridor prefix; comparison is strictly posthoc."""
import argparse
import collections
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import subprocess

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
DATASET = Path('/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01')
INPUT = DATASET / 'results/p6_i6c_framework/input'
MAP = DATASET / 'map/derived/corridor01_map_normalized.pcd'
PARAMS = PACKAGE / 'docs/p6_i6d_full_algorithm/corridor01_params_official_calibration.txt'
INITIALIZATION_STAMP_NS = 1517157224188979000
EXPECTED_SHA = {
    INPUT / 'input_manifest.txt': '6d722ec6946570cc09d984c1a8ac7ebaaff799f012f9386ae169e3d47747a043',
    INPUT / 'imu.csv': '7dc881d4ebfeacea9354be569a5952e5e466ccdd366637e52a7797a6f37457aa',
    INPUT / 'filter_scans.csv': 'f992626762f1f2321f78212c0b0c8be9e65b196bd1a638b4dc6c4fe51f846450',
    INPUT / 'scans.csv': '59179734b433a8d81d5c4ead554556f7d650f2865c87f7205a0e3f1e8c7286af',
    INPUT / 'request_xyz_f32.bin': 'beb700449a60a7f31c33e107b938862782d83afd7c93e589c323371db4be1610',
    MAP: '103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f',
    PARAMS: '7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d',
}


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def percentile(values, fraction):
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def summarize(output, frame_limit):
    trajectory = rows(output / 'trajectory.csv')
    registrations = rows(output / 'registration.csv')
    runtime = rows(output / 'runtime.csv')
    if any(len(data) != frame_limit for data in (trajectory, registrations, runtime)):
        raise RuntimeError('output_frame_count_mismatch')
    for data in (trajectory, registrations, runtime):
        if [int(row['transaction_id']) for row in data] != list(range(1, frame_limit + 1)):
            raise RuntimeError('output_transaction_sequence_mismatch')
    if any(a['stamp_ns'] != b['stamp_ns'] for a, b in zip(trajectory, registrations)):
        raise RuntimeError('output_timestamp_mismatch')
    stamps = [int(row['stamp_ns']) for row in trajectory]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise RuntimeError('nonmonotonic_output_timestamps')
    if any(not math.isfinite(float(value)) for row in trajectory for value in row.values()):
        raise RuntimeError('nonfinite_output_state')
    statuses = collections.Counter(row['status'] for row in registrations)
    updates = sum(int(row['lidar_update_applied']) for row in registrations)
    mismatches = sum(row['source_hash_expected_available'] == '1' and
                     row['source_hash_match'] != '1' for row in registrations)
    if mismatches:
        raise RuntimeError('source_hash_mismatch')
    for row in registrations:
        if row['status'] not in ('INSUFFICIENT_POINTS', 'NONFINITE_TERMINAL'):
            for key in ('raw_x', 'raw_y', 'raw_z', 'raw_qx', 'raw_qy', 'raw_qz', 'raw_qw'):
                if not math.isfinite(float(row[key])):
                    raise RuntimeError('nonfinite_raw_terminal')
    summary = dict(frames=frame_limit, state_finite=True, source_hash_mismatches=mismatches,
                   frozen_source_hash_available_frames=sum(
                       int(row['source_hash_expected_available']) for row in registrations),
                   outcomes=dict(statuses), lidar_updates=updates,
                   prediction_only_frames=frame_limit - updates,
                   ndt_align_calls=frame_limit - statuses['INSUFFICIENT_POINTS'],
                   first_ineffective_tx=next((int(row['transaction_id']) for row in registrations
                                              if row['effective'] == '0'), None))
    for field in ('prediction_ms', 'cloud_io_ms', 'ndt_total_ms', 'ndt_alignment_ms',
                  'ikfom_update_ms', 'frame_total_ms'):
        values = [float(row[field]) for row in runtime]
        if not all(math.isfinite(value) and value >= 0 for value in values):
            raise RuntimeError('invalid_runtime_measurement')
        summary['mean_' + field] = statistics.mean(values)
    summary['p95_ndt_alignment_ms'] = percentile(
        [float(row['ndt_alignment_ms']) for row in runtime], 0.95)
    resource_text = (output / 'resources.txt').read_text()
    rss_lines = [line for line in resource_text.splitlines()
                 if 'Maximum resident set size (kbytes):' in line]
    if len(rss_lines) != 1:
        raise RuntimeError('missing_peak_rss')
    summary['peak_RSS_MB'] = int(rss_lines[0].split(':')[1]) / 1024.0
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


def normalized_quaternion(q):
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm == 0:
        raise RuntimeError('invalid_comparison_quaternion')
    return [value / norm for value in q]


def matrix_quaternion(m):
    # Same normalized rotation representation as legacy poseFromMatrix.
    trace = m[0] + m[5] + m[10]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = [(m[9] - m[6]) / s, (m[2] - m[8]) / s, (m[4] - m[1]) / s, s / 4]
    else:
        i = max(range(3), key=lambda axis: m[axis * 4 + axis])
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1 + m[i * 4 + i] - m[j * 4 + j] - m[k * 4 + k]) * 2
        q = [0.0] * 4
        q[i] = s / 4
        q[j] = (m[j * 4 + i] + m[i * 4 + j]) / s
        q[k] = (m[k * 4 + i] + m[i * 4 + k]) / s
        q[3] = (m[k * 4 + j] - m[j * 4 + k]) / s
    return normalized_quaternion(q)


def pose_delta(a, b):
    translation = math.sqrt(sum((x - y) ** 2 for x, y in zip(a[:3], b[:3])))
    qa, qb = normalized_quaternion(a[3:]), normalized_quaternion(b[3:])
    if sum(x * y for x, y in zip(qa, qb)) < 0:
        qb = [-value for value in qb]
    difference = math.sqrt(sum((x - y) ** 2 for x, y in zip(qa, qb)))
    rotation = math.degrees(4 * math.asin(min(1.0, difference / 2)))
    return translation, rotation


def csv_pose(row, prefix, legacy=False):
    xyz = ('tx', 'ty', 'tz') if legacy else ('x', 'y', 'z')
    return [float(row[prefix + '_' + suffix]) for suffix in xyz + ('qx', 'qy', 'qz', 'qw')]


def compare_reference(output, legacy, recomputed_hashes):
    """Only called after estimator exit; no comparison data enter the runner."""
    current = rows(output / 'registration.csv')
    states = rows(output / 'trajectory.csv')
    old = rows(legacy / 'events.csv')[:len(current)]
    old_states = rows(legacy / 'trajectory.csv')[:len(current)]
    hashes = rows(recomputed_hashes)[:len(current)]
    if not all(len(data) == len(current) for data in (old, old_states, hashes)):
        raise RuntimeError('incomplete_reference')
    for group in zip(current, states, old, old_states, hashes):
        if len({int(row['transaction_id']) for row in group}) != 1:
            raise RuntimeError('comparison_transaction_mismatch')
        if len({int(row['stamp_ns']) for row in group[:4]}) != 1:
            raise RuntimeError('comparison_stamp_mismatch')
    first_limiter = next((int(row['transaction_id']) for row in old
                          if row['step_limited'] == '1'), None)
    first_ineffective = next((int(row['transaction_id']) for row in current
                             if row['effective'] == '0'), None)
    boundaries = [tx for tx in (first_limiter, first_ineffective) if tx is not None]
    prefix_last = min(boundaries) - 1 if boundaries else len(current)
    comparisons = []
    for new, state, reference, old_state, expected in zip(current, states, old, old_states, hashes):
        matrix = [float(value) for value in reference['M0_pose_map_T_lidar_rowmajor'].split(';')]
        if len(matrix) != 16:
            raise RuntimeError('invalid_reference_matrix')
        pred_delta = pose_delta(csv_pose(state, 'predicted_imu'),
                                csv_pose(old_state, 'predictor_imu', True))
        raw_delta = None
        if new['status'] not in ('INSUFFICIENT_POINTS', 'NONFINITE_TERMINAL') and \
                all(math.isfinite(value) for value in matrix):
            raw = [matrix[3], matrix[7], matrix[11]] + matrix_quaternion(matrix)
            raw_delta = pose_delta(csv_pose(new, 'raw'), raw)
        corrected_delta = pose_delta(csv_pose(state, 'corrected_imu'),
                                     csv_pose(old_state, 'corrected_imu', True))
        comparisons.append(dict(tx=int(new['transaction_id']), predictor=pred_delta, raw=raw_delta,
                                corrected=corrected_delta,
                                hash_match=new['source_hash_actual'] == expected['source_hash'],
                                iterations_match=new['iterations'] == reference['M0_iterations'],
                                converged_match=new['converged'] == reference['M0_converged']))
    prefix = [row for row in comparisons if row['tx'] <= prefix_last]
    divergence = next((row['tx'] for row in comparisons
                       if row['corrected'][0] >= 1e-4 or row['corrected'][1] >= 1e-3), None)
    report = dict(LEGACY_FIRST_STEP_LIMITED_TX=first_limiter,
                  P7_FIRST_INEFFECTIVE_NDT_TX=first_ineffective,
                  FIRST_TRAJECTORY_DIVERGENCE_TX=divergence,
                  PARITY_PREFIX_LAST_TX=prefix_last,
                  source_hash_mismatch_count=sum(not row['hash_match'] for row in comparisons),
                  iteration_mismatch_count=sum(not row['iterations_match'] for row in prefix),
                  converged_mismatch_count=sum(not row['converged_match'] for row in prefix),
                  hash_reference='RECOMPUTED_WITH_UNCHANGED_LEGACY_PREPROCESSING',
                  trajectory_divergence_tolerance_m=1e-4,
                  trajectory_divergence_tolerance_deg=1e-3)
    for name in ('predictor', 'raw'):
        for coordinate, unit in ((0, 'translation_delta_m'), (1, 'rotation_delta_deg')):
            report['max_' + name + '_' + unit] = max(
                (row[name][coordinate] for row in prefix if row[name] is not None), default=None)
    report['tx1'] = comparisons[0]
    report['parity_prefix_pass'] = bool(prefix) and not report['source_hash_mismatch_count'] and \
        not report['iteration_mismatch_count'] and not report['converged_mismatch_count'] and \
        all(row['raw'] is not None and row['predictor'][0] < 1e-4 and row['predictor'][1] < 1e-3 and
            row['raw'][0] < 1e-4 and row['raw'][1] < 1e-3 for row in prefix)
    (output / 'p7b_parity_report.txt').write_text('\n'.join(
        '{}={}'.format(key, json.dumps(value)) for key, value in report.items()) + '\n')
    (output / 'parity.json').write_text(json.dumps(report, indent=2) + '\n')
    if divergence is not None:
        index = divergence - 1
        (output / 'first_divergence.json').write_text(json.dumps(dict(
            registration=current[index], trajectory=states[index],
            legacy_registration=old[index], legacy_trajectory=old_states[index]), indent=2) + '\n')
    if not report['parity_prefix_pass']:
        raise RuntimeError('parity_prefix_gate_failed')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frame-limit', required=True, type=int, choices=(1, 100))
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--executable', type=Path,
                        default=WORKSPACE / 'build/p7_b/p7_single_state_runner')
    args = parser.parse_args()
    if args.output_dir.exists():
        raise RuntimeError('output_directory_already_exists')
    actual_sha = {str(path): sha256(path) for path in EXPECTED_SHA}
    for path, expected in EXPECTED_SHA.items():
        if actual_sha[str(path)] != expected:
            raise RuntimeError('input_SHA_mismatch:' + str(path))
    if not args.executable.is_file():
        raise RuntimeError('runner_not_built')
    args.output_dir.mkdir(parents=True)
    (args.output_dir / 'input_sha256.json').write_text(json.dumps(actual_sha, indent=2) + '\n')
    command = ['/usr/bin/time', '-v', '-o', str(args.output_dir / 'resources.txt'),
               str(args.executable), str(INPUT / 'imu.csv'), str(INPUT / 'filter_scans.csv'),
               str(INPUT / 'scans.csv'), str(INPUT / 'request_xyz_f32.bin'), str(MAP), str(PARAMS),
               str(args.output_dir / 'trajectory.csv'), str(args.output_dir / 'registration.csv'),
               str(args.output_dir / 'runtime.csv'), str(args.frame_limit),
               str(INITIALIZATION_STAMP_NS)]
    environment = os.environ.copy()
    environment.pop('LD_LIBRARY_PATH', None)
    (args.output_dir / 'command.json').write_text(json.dumps(command, indent=2) + '\n')
    with (args.output_dir / 'run.log').open('w') as log:
        completed = subprocess.run(command, env=environment, stdout=log, stderr=subprocess.STDOUT)
    if completed.returncode != 0:
        raise RuntimeError('runner_failed:see ' + str(args.output_dir / 'run.log'))
    print(json.dumps(summarize(args.output_dir, args.frame_limit), indent=2))


if __name__ == '__main__':
    main()
