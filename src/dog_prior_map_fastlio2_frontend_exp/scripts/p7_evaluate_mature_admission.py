#!/usr/bin/env python3
"""Posthoc P7-E evidence, using the unchanged P6 evaluator and frozen D results.

Never runs an estimator, adjusts alignment gates or feeds diagnostics back into
tracking. An unevaluable short prefix remains unevaluable.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--admission-dir', type=Path, required=True)
    parser.add_argument('--frozen-full-dir', type=Path, required=True)
    parser.add_argument('--frozen-remap-dir', type=Path, required=True)
    parser.add_argument('--frozen-innovation-csv', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise RuntimeError('posthoc_output_already_exists')
    scripts = Path(__file__).resolve().parent
    adapter = load('existing_p7_adapter', scripts / 'p7_evaluate_solution_remapping.py')
    wrapper = load('existing_p7_wrapper', scripts / 'p7_run_corridor01.py')
    states = wrapper.rows(args.admission_dir / 'trajectory.csv')
    full_states = wrapper.rows(args.frozen_full_dir / 'trajectory.csv')
    remap_states = wrapper.rows(args.frozen_remap_dir / 'trajectory.csv')
    adapter.replay_exit(args.frozen_full_dir, len(full_states))
    adapter.replay_exit(args.frozen_remap_dir, len(remap_states))
    resource = (args.admission_dir / 'resources.txt').read_text()
    if resource.count('Exit status: 0') != 1 or 'stop_reason=' not in (args.admission_dir / 'run.log').read_text():
        raise RuntimeError('admission_estimator_must_exit_normally_before_GT')
    summary = json.loads((args.admission_dir / 'summary.json').read_text())
    admission = wrapper.rows(args.admission_dir / 'admission.csv')
    registration = wrapper.rows(args.admission_dir / 'registration.csv')
    validated = wrapper.admission_summary(admission, registration, summary['requested_frames'])
    if summary['mode'] != 'MATURE_ADMISSION' or len(states) != len(admission):
        raise RuntimeError('admission_mode_or_count_mismatch')
    frozen_innovation = wrapper.rows(args.frozen_innovation_csv)
    if len(frozen_innovation) != len(full_states) or any(
            row['predicted_and_corrected_state_exact'] != '1' or int(row['transaction_id']) != i
            for i, row in enumerate(frozen_innovation, 1)):
        raise RuntimeError('frozen_covariance_reconstruction_not_verified')
    common_count = min(len(states), len(full_states))
    if [row['stamp_ns'] for row in states[:common_count]] != \
            [row['stamp_ns'] for row in full_states[:common_count]]:
        raise RuntimeError('common_prefix_timestamps_differ')
    inputs = [(root / 'input_sha256.json').read_text()
              for root in (args.admission_dir, args.frozen_full_dir, args.frozen_remap_dir)]
    if not all(json.loads(value) == json.loads(inputs[0]) for value in inputs):
        raise RuntimeError('frozen_inputs_differ')

    # First GT access: all three estimator processes have exited.
    evaluator = adapter.load_module('unchanged_p6_evaluator', adapter.EVALUATOR)
    if evaluator.sha256(adapter.GT) != evaluator.EXPECTED_GT_SHA256:
        raise RuntimeError('official_GT_hash_mismatch')
    gt = np.loadtxt(adapter.GT, comments='#', ndmin=2)
    if not np.isfinite(gt).all() or np.any(np.diff(gt[:, 0]) <= 0):
        raise RuntimeError('invalid_GT')
    args.output_dir.mkdir(parents=True)
    report = dict(GT_USED_ONLINE=False, evaluator_reused=True,
                  evaluator_path=str(adapter.EVALUATOR),
                  evaluator_sha256=evaluator.sha256(adapter.EVALUATOR),
                  admission=validated, common_prefix_frames=common_count, metrics={})
    for mode, prefix in [('MATURE_ADMISSION', states),
                         ('MATURE_ADMISSION_COMMON', states[:common_count]),
                         ('FULL_POSE_COMMON', full_states[:common_count])]:
        adapter.adapt_trajectory(prefix, args.output_dir / f'trajectory_{mode}.csv')
        try:
            metrics, _ = evaluator.evaluate(mode, args.output_dir, gt[:, 0], gt[:, 1:4], gt[:, 4:8],
                                            adapter.EVALUATION_START)
            report['metrics'][mode] = dict(status='EVALUATED', **metrics)
        except RuntimeError as error:
            if 'too few GT-corresponding samples' not in str(error):
                raise
            count = sum(int(row['stamp_ns']) * 1e-9 >= adapter.EVALUATION_START and
                        evaluator.interpolate_gt(int(row['stamp_ns']) * 1e-9,
                        gt[:, 0], gt[:, 1:4], gt[:, 4:8]) is not None for row in prefix)
            report['metrics'][mode] = dict(status='NOT_EVALUABLE', samples=count,
                                          reason=str(error), minimum_samples=100)
    full119 = frozen_innovation[118]
    remap119 = wrapper.rows(args.frozen_remap_dir / 'registration.csv')[118]
    distance = math.sqrt(sum((float(remap119['raw_' + axis]) -
                             float(remap119['initial_' + axis])) ** 2 for axis in 'xyz'))
    report['tx119'] = dict(FULL_POSE=full119,
        MATURE_SOL_REMAP=dict(ndt_status=remap119['status'], distance_m=distance,
                             nis=None, decision='NDT_TERMINAL_REJECT',
                             nis_status='NOT_ELIGIBLE_NDT_INEFFECTIVE'),
        MATURE_ADMISSION=admission[118] if len(admission) >= 119 else
            dict(status='NOT_REACHED_AFTER_LOST', LOST_TX=validated['LOST_TX']))
    report['frozen_FULL_first_distance_reject'] = next((row for row in frozen_innovation
        if row['decision'] == 'AUTOWARE_INITIAL_TO_RESULT_DISTANCE_REJECT'), None)
    report['frozen_FULL_first_NIS_reject'] = next((row for row in frozen_innovation
        if row['decision'] == 'MAHALANOBIS_NIS_REJECT'), None)
    report['frozen_FULL_last20_before_1682'] = frozen_innovation[-20:]
    report['frozen_innovation_provenance'] = 'POSTHOC_RECONSTRUCTION_NOT_ORIGINAL_D_NIS_LOG'
    report['artifact_sha256'] = {str(path): evaluator.sha256(path) for path in (
        args.admission_dir / 'trajectory.csv', args.admission_dir / 'admission.csv',
        args.frozen_full_dir / 'trajectory.csv', args.frozen_full_dir / 'registration.csv',
        args.frozen_innovation_csv)}
    (args.output_dir / 'evaluation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(admission=validated, metrics=report['metrics'], tx119=report['tx119']), indent=2))


if __name__ == '__main__':
    main()
