#!/usr/bin/env python3
"""Run the frozen P7 Corridor prefix; all shadow comparison is strictly posthoc."""
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


def shadow_uobs_counts(uobs):
    computed = [row for row in uobs if row['uobs_computed'] == '1']
    valid = [row for row in computed if row['uobs_valid'] == '1']
    classified = [row for row in computed if row['classification_valid'] == '1']
    return dict(UOBS_COMPUTED=len(computed), UOBS_VALID=len(valid),
                UOBS_INVALID=len(computed) - len(valid),
                MAP_SUPPORT_INSUFFICIENT=sum(row['map_support_status'] == 'MAP_SUPPORT_INSUFFICIENT'
                                             for row in computed),
                NO_VALID_GEOMETRIC_CORRESPONDENCES=sum(
                    row['map_support_status'] == 'NO_VALID_GEOMETRIC_CORRESPONDENCES' for row in computed),
                UOBS_NUMERICAL_FAILURE=sum(row['map_support_status'] == 'NUMERICAL_FAILURE'
                                           for row in computed),
                SCHUR_DIAGNOSTIC_UNAVAILABLE=sum(
                    row['schur_status'] == 'SCHUR_DIAGNOSTIC_UNAVAILABLE' for row in computed),
                JOINT_CLASSIFIER_INVALID=len(computed) - len(classified),
                weak_dimension_histogram=dict(collections.Counter(
                    int(row['weak_dimension']) for row in classified)))


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
    uobs = rows(output / 'uobs.csv')
    if len(uobs) != frame_limit or any(None in row for row in uobs):
        raise RuntimeError('invalid_uobs_csv_shape')
    if any(a['transaction_id'] != b['transaction_id'] or a['stamp_ns'] != b['stamp_ns']
           for a, b in zip(uobs, registrations)):
        raise RuntimeError('uobs_transaction_timestamp_mismatch')
    for row, registration in zip(uobs, registrations):
        if row['ndt_effective'] != registration['effective'] or \
                row['uobs_computed'] != registration['effective']:
            raise RuntimeError('uobs_not_success_only')
        if float(row['translation_length_scale_m']) != 0.8 or \
                float(row['weak_relative_ratio']) != 0.05:
            raise RuntimeError('uobs_physical_scale_or_ratio_changed')
        if row['uobs_valid'] == '1':
            matrices = [[float(row[f'{prefix}_r{i}c{j}']) for i in range(6) for j in range(6)]
                        for prefix in ('Hphys', 'Hbar')]
            if not all(math.isfinite(value) for matrix in matrices for value in matrix):
                raise RuntimeError('nonfinite_valid_uobs_matrix')
            hphys, hbar = matrices
            scale = [1.0] * 3 + [0.8] * 3
            magnitude = max(1.0, max(abs(value) for value in hbar))
            if max(abs(hbar[i * 6 + j] - scale[i] * hphys[i * 6 + j] * scale[j])
                   for i in range(6) for j in range(6)) > 1e-10 * magnitude:
                raise RuntimeError('uobs_csv_physical_normalization_mismatch')
        if row['classification_valid'] == '1':
            weak, reliable = int(row['weak_dimension']), int(row['reliable_dimension'])
            if not 0 <= weak <= 6 or reliable != 6 - weak:
                raise RuntimeError('uobs_invalid_joint_dimensions')
            basis = [[float(row[f'{prefix}_r{i}c{j}']) for i in range(6)]
                     for prefix, count in (('weak_basis', weak), ('reliable_basis', reliable))
                     for j in range(count)]
            for i in range(6):
                for j in range(6):
                    product = sum(x * y for x, y in zip(basis[i], basis[j]))
                    if not math.isfinite(product) or abs(product - int(i == j)) > 1e-8:
                        raise RuntimeError('uobs_csv_bases_not_orthogonal')
    summary.update(shadow_uobs_counts(uobs))
    for field in ('uobs_ms', 'classifier_ms', 'resident_rss_kb'):
        values = [float(row[field]) for row in runtime]
        if not all(math.isfinite(value) and value >= 0 for value in values):
            raise RuntimeError('invalid_shadow_runtime_measurement')
    for field in ('uobs_ms', 'classifier_ms'):
        values = [float(time[field]) for time, row in zip(runtime, uobs)
                  if row['uobs_computed'] == '1']
        summary['mean_' + field] = statistics.mean(values) if values else 0.0
        summary['p95_' + field] = percentile(values, 0.95) if values else 0.0
    rss = [float(row['resident_rss_kb']) / 1024.0 for row in runtime]
    summary['resident_rss_by_frame_MB'] = rss
    if len(rss) >= 40:
        summary['resident_rss_frames_11_30_mean_MB'] = statistics.mean(rss[10:30])
        summary['resident_rss_last_20_mean_MB'] = statistics.mean(rss[-20:])
        summary['resident_rss_last_50_span_MB'] = max(rss[-50:]) - min(rss[-50:])
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


def compare_shadow(output, baseline, summary):
    """Executed only after runner exit; baseline artifacts never enter the estimator."""
    new_states, old_states = rows(output / 'trajectory.csv'), rows(baseline / 'trajectory.csv')
    new_reg, old_reg = rows(output / 'registration.csv'), rows(baseline / 'registration.csv')
    if not new_states or len(new_states) != len(old_states) or len(new_reg) != len(old_reg):
        raise RuntimeError('incomplete_shadow_reference')
    # 17-digit round-trippable state output: require exact equality, not a relaxed gate.
    if new_states != old_states:
        raise RuntimeError('shadow_changed_filter_state')
    without_time = lambda row: {key: value for key, value in row.items() if key != 'alignment_ms'}
    if [without_time(row) for row in new_reg] != [without_time(row) for row in old_reg]:
        raise RuntimeError('shadow_changed_ndt_result')
    original = json.loads((baseline / 'summary.json').read_text())
    report = dict(trajectory_parity=True, ndt_parity=True,
                  all_state_fields_exact=True, all_registration_fields_except_timing_exact=True,
                  max_corrected_translation_delta_m=0.0, max_corrected_rotation_delta_deg=0.0,
                  prechange_mean_frame_ms=original['mean_frame_total_ms'],
                  shadow_mean_frame_ms=summary['mean_frame_total_ms'],
                  prechange_peak_RSS_MB=original['peak_RSS_MB'],
                  shadow_peak_RSS_MB=summary['peak_RSS_MB'],
                  trajectory_sha256=sha256(output / 'trajectory.csv'),
                  prechange_trajectory_sha256=sha256(baseline / 'trajectory.csv'),
                  frames=summary['frames'], outcomes=summary['outcomes'],
                  first_ineffective_tx=summary['first_ineffective_tx'])
    if summary['frames'] == 100 and (summary['outcomes'] != {
            'SUCCESS': 93, 'ITERATION_LIMIT_EXHAUSTED': 7} or
            summary['first_ineffective_tx'] != 47 or summary['lidar_updates'] != 93):
        raise RuntimeError('unexpected_p7c_prefix_summary')
    (output / 'p7c_shadow_report.json').write_text(json.dumps(report, indent=2) + '\n')
    (output / 'p7c_shadow_report.txt').write_text('\n'.join(
        f'{key}={json.dumps(value)}' for key, value in report.items()) + '\n')
    return report


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
    parser.add_argument('--shadow-reference', type=Path,
                        help='P7-B output directory, read only after runner exit')
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
               str(args.output_dir / 'runtime.csv'), str(args.output_dir / 'uobs.csv'), str(args.frame_limit),
               str(INITIALIZATION_STAMP_NS)]
    environment = os.environ.copy()
    environment.pop('LD_LIBRARY_PATH', None)
    (args.output_dir / 'command.json').write_text(json.dumps(command, indent=2) + '\n')
    with (args.output_dir / 'run.log').open('w') as log:
        completed = subprocess.run(command, env=environment, stdout=log, stderr=subprocess.STDOUT)
    if completed.returncode != 0:
        raise RuntimeError('runner_failed:see ' + str(args.output_dir / 'run.log'))
    summary = summarize(args.output_dir, args.frame_limit)
    if args.shadow_reference:
        compare_shadow(args.output_dir, args.shadow_reference, summary)
    print(json.dumps({key: value for key, value in summary.items()
                      if key != 'resident_rss_by_frame_MB'}, indent=2))


if __name__ == '__main__':
    main()
