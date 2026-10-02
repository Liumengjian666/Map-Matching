#!/usr/bin/env python3
"""POSTHOC ONLY: adapt P7 CSV columns to the existing P6 Corridor evaluator.

This script never runs an estimator. Both replay processes must have exited
before the official GT is opened. Aborted replays are explicitly labelled and
only their committed prefix is evaluated. Alignment/association/error calculations are
delegated, unchanged, to p6_i6c_report_corridor01.py.
"""
import argparse
import csv
import importlib.util
import json
import re
from pathlib import Path

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
EVALUATOR = PACKAGE / 'scripts/p6_i6c_report_corridor01.py'
GT = Path('/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/gt/corridor01_gt.txt')
EVALUATION_START = 1517157224.188979
MODES = ('FULL_POSE', 'MATURE_SOL_REMAP')


def read_rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def adapt_trajectory(rows, destination):
    # Rename positions only: no pose, timestamp or numerical transformation.
    renamed = []
    for row in rows:
        converted = dict(row)
        for axis in 'xyz':
            converted['corrected_imu_t' + axis] = converted.pop('corrected_imu_' + axis)
        renamed.append(converted)
    with destination.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(renamed[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(renamed)


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def replay_exit(root, count):
    # GNU time writes this terminal marker after its child has exited, even
    # when the estimator fails. A running/incomplete process is not evaluable.
    resource = (root / 'resources.txt').read_text()
    statuses = re.findall(r'^\s*Exit status: (\d+)\s*$', resource, re.MULTILINE)
    if len(statuses) != 1:
        raise RuntimeError('both_estimators_must_exit_before_GT')
    exit_status = int(statuses[0])
    failures = re.findall(r'^FIRST_BAD_TX=(\d+) error=(.+)$',
                          (root / 'run.log').read_text(), re.MULTILINE)
    if exit_status == 0 and count != 2726:
        raise RuntimeError('successful_full_replay_is_incomplete')
    if exit_status != 0 and (len(failures) != 1 or int(failures[0][0]) != count + 1):
        raise RuntimeError('aborted_replay_missing_precise_failure_evidence')
    return dict(expected_frames=2726, committed_frames=count, replay_exit_status=exit_status,
                full_replay_passed=exit_status == 0,
                first_bad_tx=int(failures[0][0]) if failures else None,
                failure_reason=failures[0][1] if failures else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--full-pose-dir', type=Path, required=True)
    parser.add_argument('--remap-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise RuntimeError('posthoc_output_directory_already_exists')
    runs = dict(zip(MODES, (args.full_pose_dir, args.remap_dir)))
    summaries, states, inputs, exits = {}, {}, {}, {}
    wrapper = load_module('p7_posthoc_runtime_reporting', PACKAGE / 'scripts/p7_run_corridor01.py')
    for mode, root in runs.items():
        inputs[mode] = json.loads((root / 'input_sha256.json').read_text())
        states[mode] = read_rows(root / 'trajectory.csv')
        count = len(states[mode])
        exits[mode] = replay_exit(root, count)
        if count < 100:
            raise RuntimeError('insufficient_committed_prefix_for_posthoc')
        if [int(row['transaction_id']) for row in states[mode]] != list(range(1, count + 1)):
            raise RuntimeError('posthoc_transaction_sequence_mismatch')
    # Both independent processes have exited now. Reuse the existing wrapper's
    # output integrity/runtime reporting, with an explicit failed-full label.
    for mode, root in runs.items():
        summaries[mode] = wrapper.summarize(root, len(states[mode]))
        if summaries[mode]['mode'] != mode:
            raise RuntimeError('posthoc_baseline_mode_mismatch')
        summaries[mode].update(exits[mode])
        (root / 'summary.json').write_text(json.dumps(summaries[mode], indent=2) + '\n')
    if inputs[MODES[0]] != inputs[MODES[1]]:
        raise RuntimeError('ablation_frozen_inputs_differ')
    common_count = min(len(state) for state in states.values())
    if [row['stamp_ns'] for row in states[MODES[0]][:common_count]] != \
            [row['stamp_ns'] for row in states[MODES[1]][:common_count]]:
        raise RuntimeError('ablation_timestamps_differ')

    helper = load_module('p7_existing_corridor_evaluator', EVALUATOR)
    # First GT access in this workflow: after both estimator exits, never online.
    gt_hash = helper.sha256(GT)
    if gt_hash != helper.EXPECTED_GT_SHA256:
        raise RuntimeError('official_Corridor01_GT_SHA256_mismatch')
    gt = np.loadtxt(GT, comments='#', ndmin=2)
    if np.any(np.diff(gt[:, 0]) <= 0) or not np.isfinite(gt).all():
        raise RuntimeError('invalid_posthoc_GT')
    args.output_dir.mkdir(parents=True)
    report = dict(evaluator_path=str(EVALUATOR), evaluator_sha256=helper.sha256(EVALUATOR),
                  evaluator_reused=True, alignment='PREFIX_10S_SE3_SCALE_1',
                  evaluation_start_s=EVALUATION_START, gt_sha256=gt_hash,
                  GT_USED_ONLINE=False, gt_opened_after_both_replay_exits=True,
                  both_full_replays_passed=all(exit['full_replay_passed'] for exit in exits.values()),
                  common_committed_prefix_frames=common_count, modes={})
    for mode, root in runs.items():
        adapted = args.output_dir / f'trajectory_{mode}.csv'
        adapt_trajectory(states[mode], adapted)
        metrics, crossings = helper.evaluate(mode, args.output_dir, gt[:, 0], gt[:, 1:4], gt[:, 4:8],
                                            EVALUATION_START)
        errors = read_rows(args.output_dir / f'trajectory_errors_{mode}.csv')
        maximum = max(errors, key=lambda row: float(row['translation_error_m']))
        metrics['max_error_timestamp_s'] = float(maximum['stamp_s'])
        metrics['replay_exit'] = exits[mode]
        metrics['persistent_crossings_s'] = {str(key): value for key, value in crossings.items()}
        row_by_time = {int(row['stamp_ns']) * 1e-9: row for row in states[mode]}
        uobs_by_tx = {row['transaction_id']: row for row in read_rows(root / 'uobs.csv')}
        grouped = {3: [], 4: []}
        for error in errors:
            state = row_by_time[float(error['stamp_s'])]
            local = uobs_by_tx[state['transaction_id']]
            if local['classification_valid'] == '1' and int(local['weak_dimension']) in grouped:
                grouped[int(local['weak_dimension'])].append(float(error['translation_error_m']))
        metrics['weak_frame_position_errors'] = {
            str(weak): dict(samples=len(values), **helper.stats(np.asarray(values)))
            if values else dict(samples=0) for weak, values in grouped.items()}
        registration = read_rows(root / 'registration.csv')
        metrics['stability'] = dict(NDT_SUCCESS=summaries[mode]['outcomes'].get('SUCCESS', 0),
                                   NDT_FAILURE=len(states[mode]) - summaries[mode]['outcomes'].get('SUCCESS', 0),
                                   prediction_only=summaries[mode]['prediction_only_frames'],
                                   state_failure_count=int(exits[mode]['replay_exit_status'] != 0),
                                   UOBS_VALID=summaries[mode]['UOBS_VALID'],
                                   weak_histogram=summaries[mode]['weak_dimension_histogram'],
                                   lidar_updates=summaries[mode]['lidar_updates'],
                                   ndt_align_calls=summaries[mode]['ndt_align_calls'])
        metrics['runtime'] = {key: summaries[mode][key] for key in
                              ('mean_frame_total_ms', 'p95_frame_total_ms', 'peak_RSS_MB',
                               'mean_solution_remap_ms', 'mean_uobs_ms')}
        metrics['maximum_measurement_correction'] = {key: value for key, value in summaries[mode].items()
                                                     if key.startswith(('max_raw_', 'max_safe_'))}
        metrics['trajectory_sha256'] = helper.sha256(root / 'trajectory.csv')
        metrics['registration_sha256'] = helper.sha256(root / 'registration.csv')
        metrics['source_hashes'] = [row['source_hash_actual'] for row in registration]
        # Unequal aborted horizons cannot be treated as a fair full-run ATE
        # comparison. Report the shared, already-generated prefix separately,
        # using precisely the same mature evaluator, not a new alignment.
        common_mode = mode + '_COMMON_PREFIX'
        adapt_trajectory(states[mode][:common_count], args.output_dir / f'trajectory_{common_mode}.csv')
        common_metrics, _ = helper.evaluate(common_mode, args.output_dir, gt[:, 0], gt[:, 1:4], gt[:, 4:8],
                                            EVALUATION_START)
        metrics['common_committed_prefix_metrics'] = common_metrics
        report['modes'][mode] = metrics
    source_hashes = [report['modes'][mode].pop('source_hashes') for mode in MODES]
    report['source_hash_mismatch_count'] = sum(a != b for a, b in zip(*source_hashes))
    if report['source_hash_mismatch_count']:
        raise RuntimeError('ablation_preprocessed_source_identity_mismatch')
    (args.output_dir / 'evaluation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
