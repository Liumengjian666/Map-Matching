#!/usr/bin/env python3
"""Frozen-input R2 execution and descriptive (GT-free) report, no policy tuning."""
import argparse
import csv
import hashlib
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]
OUT = PKG / 'docs/p6_i6e_r2_projected_consistency'
R1 = PKG / 'docs/p6_i6e_r1_source_repair'
POLICIES = ['LEGACY_BASE_NO_GATE', 'ADAPTIVE_NO_GATE',
            'BASE_SELECTED_NIS', 'ADAPTIVE_SELECTED_NIS']
VISUAL_SHA = {'Floor01': 'd2db72284b93bfa6d5fb6a9d813f20d45c3fda3e4f38710dd367fcfb68b0d66c',
              'Corridor01': 'f192b2d77ac427c62357ac6592acebabf678c60fbe762dc4cbe48422de8c5a63'}


def read(path):
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    assert all(None not in row and None not in row.values() for row in rows), path
    return rows


def artifact(dataset, policy, suffix):
    return OUT / dataset.lower() / policy / f'B4_{dataset}_{suffix}'


def write_csv(path, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def legacy_regression():
    lines = []
    for dataset in VISUAL_SHA:
        for suffix in ('trajectory.csv', 'reliability.csv', 'reliability.csv.visual_updates.csv'):
            old = read(R1 / (dataset.lower() + '_15s') / f'B4_{dataset}_{suffix}')
            new = read(artifact(dataset, 'P0', suffix))
            assert len(old) == len(new), (dataset, suffix, 'row count')
            checked = 0
            for index, (a, b) in enumerate(zip(old, new)):
                for key in a:
                    if key.endswith('_ms'):
                        continue
                    checked += 1
                    # Exact serialized equality, including compound matrix fields.
                    assert a[key] == b[key], (dataset, suffix, index + 1, key, a[key], b[key])
            lines.append(f'{dataset} {suffix}: PASS; {len(old)} rows; {checked} old non-timing fields exactly equal')
    (OUT / 'legacy_regression.txt').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines), flush=True)


def run(executable):
    for index, policy in enumerate(POLICIES):
        for dataset, sha in VISUAL_SHA.items():
            visual = R1 / (dataset.lower() + '_15s') / 'frozen_visual_quality.csv'
            assert hashlib.sha256(visual.read_bytes()).hexdigest() == sha
            directory = OUT / dataset.lower() / f'P{index}'
            if directory.exists():
                raise RuntimeError(f'refusing to overwrite {directory}')
            directory.mkdir(parents=True)
            command = [sys.executable, str(PKG / 'scripts/p6_i6d_run.py'),
                       '--dataset', dataset, '--profile', 'B4', '--frame-limit',
                       '144' if dataset == 'Floor01' else '149', '--executable', str(executable),
                       '--visual-csv', str(visual), '--output', str(directory),
                       '--r2-policy', policy, '--task-label', 'PAPER-P6-I6E-R2']
            with (directory / 'run.log').open('w') as log:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
            print(f'COMPLETE {dataset} P{index}', flush=True)
        if index == 0:
            legacy_regression()  # Must pass BEFORE any added policy starts.


def number(row, key):
    return float(row[key])


def first(rows, predicate, field='time_s'):
    return next((row[field] for row in rows if predicate(row)), 'NONE_WITHIN_WINDOW')


def norm(row, prefix):
    return math.sqrt(sum(number(row, prefix + axis) ** 2 for axis in 'xyz'))


def table(rows, keys):
    def fmt(x):
        return f'{x:.8g}' if isinstance(x, float) else str(x)
    return '\n'.join(['| ' + ' | '.join(keys) + ' |',
                      '| ' + ' | '.join(['---'] * len(keys)) + ' |'] +
                     ['| ' + ' | '.join(fmt(row[key]) for key in keys) + ' |' for row in rows])


def summarize():
    legacy_regression()
    metrics, windows, divergences, shadows_all, linear_all, comparisons_all = [], [], [], [], [], []
    for dataset in VISUAL_SHA:
        baseline = read(artifact(dataset, 'P0', 'trajectory.csv'))
        for index in range(4):
            policy = f'P{index}'
            r = read(artifact(dataset, policy, 'reliability.csv'))
            t = read(artifact(dataset, policy, 'trajectory.csv'))
            runtime = read(artifact(dataset, policy, 'runtime.csv'))
            v = read(artifact(dataset, policy, 'reliability.csv.visual_updates.csv'))
            support = read(artifact(dataset, policy, 'reliability.csv.map_support_diagnostic.csv'))
            innovation = read(artifact(dataset, policy, 'reliability.csv.r2_projected_innovation.csv'))
            shadows = read(artifact(dataset, policy, 'reliability.csv.r2_shadow_counterfactual.csv'))
            comparison = read(artifact(dataset, policy, 'reliability.csv.r2_nonlocal_basis_comparison.csv'))
            linear = read(artifact(dataset, policy, 'reliability.csv.r2_linearization_consistency.csv'))
            assert len(t) == len(r) == len(innovation) == len(comparison)
            assert all(a['transaction_id'] == b['transaction_id'] for a, b in zip(t, r))
            for row in shadows:
                assert row['official_state_unchanged'] == '1'
                assert row['prior_state_hash'] == row['official_state_hash_after']
            for tx in set(row['transaction_id'] for row in shadows):
                group = [row for row in shadows if row['transaction_id'] == tx]
                assert len(group) == 4 and len(set(row['prior_state_hash'] for row in group)) == 1
            shadows_all += [dict(dataset=dataset, **row) for row in shadows]
            linear_all += [dict(dataset=dataset, policy=policy, **row) for row in linear]
            comparisons_all += [dict(dataset=dataset, policy=policy, **row) for row in comparison]
            times = [number(row, 'total_ms') for row in runtime]
            m = dict(dataset=dataset, policy=policy, frames=len(t),
                     ndt_calls=sum(int(row['ndt_call_count']) for row in r),
                     m0_converged=sum(row['M0_converged'] == '1' for row in r),
                     uobs_valid=sum(row['uobs_valid'] == '1' for row in r),
                     map_support_insufficient=sum(row['uobs_map_support_sufficient'] == '0' for row in r),
                     zero_correspondences=sum(int(row['uobs_valid_correspondences']) == 0 for row in r),
                     lidar_attempted=sum(row['lidar_update_attempted'] == '1' for row in r),
                     lidar_committed=sum(row['lidar_update_committed'] == '1' for row in r),
                     lidar_nis_rejected=sum(row['lidar_nis_rejected'] == '1' for row in r),
                     visual_triggered=sum(row['triggered'] == '1' for row in v),
                     visual_quality_passed=sum(row['quality_passed'] == '1' for row in v),
                     visual_updated=sum(row['status'] == 'APPLIED_PROJECTED_COMPLEMENTARY_TRANSLATION' for row in v),
                     imu_only_intervals=sum(row['route_actual'] == 'IMU_ONLY_THIS_INTERVAL' for row in r),
                     visual_only_intervals=sum(row['route_actual'] == 'VISUAL_ONLY_THIS_INTERVAL' for row in r),
                     first_nis_rejection_time=first(r, lambda row: row['lidar_nis_rejected'] == '1'),
                     first_large_prediction_ndt_difference_time=first(support, lambda row:
                         number(row, 'predicted_to_nominal_translation_m') > .20 or
                         number(row, 'predicted_to_nominal_rotation_rad') > math.radians(2)),
                     first_map_support_loss_time=first(r, lambda row: row['uobs_map_support_sufficient'] == '0'),
                     first_0_support_transaction=first(r, lambda row: int(row['uobs_valid_correspondences']) == 0, 'transaction_id'),
                     first_imu_only_time=first(r, lambda row: row['route_actual'] == 'IMU_ONLY_THIS_INTERVAL'),
                     position_sigma_max=max(number(row, 'maximum_position_sigma_m') for row in r),
                     rotation_sigma_max=max(number(row, 'maximum_rotation_sigma_rad') for row in r),
                     velocity_norm_max=max(norm(row, 'velocity_post_') for row in t),
                     gyro_bias_norm_max=max(norm(row, 'gyro_bias_post_') for row in t),
                     accel_bias_norm_max=max(norm(row, 'accel_bias_post_') for row in t),
                     runtime_mean_ms=float(np.mean(times)), runtime_p95_ms=float(np.percentile(times, 95)),
                     runtime_max_ms=max(times))
            metrics.append(m)
            # First numerical difference (1e-9 m) and material difference (existing .20 m criterion).
            for label, threshold in [('numerical', 1e-9), ('material_translation', .20)]:
                found = False
                for a, b, row in zip(baseline, t, r):
                    distance = math.sqrt(sum((number(a, 'corrected_imu_t' + axis) -
                                             number(b, 'corrected_imu_t' + axis)) ** 2 for axis in 'xyz'))
                    if distance > threshold:
                        divergences.append(dict(dataset=dataset, policy=policy, criterion=label,
                            transaction_id=row['transaction_id'], time_s=row['time_s'],
                            delta_from_P0_m=distance, predictor_xyz=';'.join(b['predictor_imu_t' + axis] for axis in 'xyz'),
                            predictor_quaternion_xyzw=';'.join(b['predictor_imu_q' + axis] for axis in 'xyzw'),
                            velocity_pre=';'.join(b['velocity_pre_' + axis] for axis in 'xyz'),
                            support=row['uobs_valid_correspondences'], route=row['route_actual']))
                        found = True
                        break
                if not found:
                    divergences.append(dict(dataset=dataset, policy=policy, criterion=label,
                        transaction_id='NONE', time_s='NONE', delta_from_P0_m=0,
                        predictor_xyz='NA', predictor_quaternion_xyzw='NA', velocity_pre='NA', support='NA', route='NA'))
            if dataset == 'Corridor01':
                for lo, hi in [(2, 5), (5, 9), (9, 13)]:
                    selected = [row for row in r if lo <= number(row, 'time_s') < hi]
                    windows.append(dict(policy=policy, window=f'{lo}-{hi}s', frames=len(selected),
                        lidar_committed=sum(row['lidar_update_committed'] == '1' for row in selected),
                        nis_rejected=sum(row['lidar_nis_rejected'] == '1' for row in selected),
                        imu_only=sum(row['route_actual'] == 'IMU_ONLY_THIS_INTERVAL' for row in selected),
                        min_support=min(int(row['uobs_valid_correspondences']) for row in selected),
                        maximum_position_sigma=max(number(row, 'maximum_position_sigma_m') for row in selected)))
    write_csv(OUT / 'mode_metrics.csv', metrics)
    write_csv(OUT / 'window_metrics.csv', windows)
    write_csv(OUT / 'first_divergences.csv', divergences)
    (OUT / 'shadow_isolation_test.txt').write_text(
        f'PASS: {len(shadows_all)} branch rows; four identical hashed priors per selected transaction; '
        'official snapshot exactly unchanged after every branch. Full state/P exact assertions execute in C++.\n')
    keys = ['dataset', 'policy', 'ndt_calls', 'lidar_committed', 'lidar_nis_rejected',
            'visual_updated', 'first_0_support_transaction', 'position_sigma_max', 'velocity_norm_max']
    (OUT / 'metrics_tables.md').write_text(table(metrics, keys) + '\n\n' + table(windows, list(windows[0])) + '\n')
    focused = [row for row in shadows_all if row['dataset'] == 'Corridor01']
    focuskeys = ['transaction_id', 'policy', 'reliable_rank', 'selected_residual_norm',
                 'selected_noise_trace', 'selected_nis', 'selected_nis_threshold',
                 'update_committed', 'delta_position_norm', 'delta_velocity_norm']
    comparison_focus = [row for row in comparisons_all if row['dataset'] == 'Corridor01' and
                        row['policy'] == 'P0' and row['transaction_id'] in {'24','34','47','60','79','88','95','120','124','127'}]
    linear_focus = [row for row in linear_all if row['dataset'] == 'Corridor01' and
                    row['policy'] == 'P0' and row['transaction_id'] in {'47','60','88'}]
    text = '# FIRST DIVERGENCE — descriptive evidence, not ATE\n\n'
    text += 'Numerical criterion = >1e-9 m corrected-position difference from P0; material criterion = >0.20 m (existing terminal translation limit reused descriptively, not a new online gate). Full predicted quaternion and pre-update velocity are in first_divergences.csv.\n\n'
    text += table(divergences, ['dataset','policy','criterion','transaction_id','time_s','delta_from_P0_m','predictor_xyz','support'])
    text += '\n\n## Same-state one-step counterfactuals\n\nAll branches use the same hash/prior per transaction; no re-running IMU/NDT/vision. Full hashes, raw residual, covariance spectrum, state corrections and posterior sigma are in the P0 shadow CSV. These are not independent closed-loop priors.\n\n'
    text += table(focused, focuskeys)
    text += '\n\n## Local vs combined row-space\n\nDifferent ranks require different thresholds; smaller NIS alone is not better. Full selected covariance matrices are retained in the run CSV.\n\n'
    text += table(comparison_focus, ['transaction_id','local_selected_rank','combined_selected_rank','additional_removed_dimension','local_nis','local_threshold','combined_nis','combined_threshold'])
    text += '\n\n## Nonlinear leakage\n\n'
    text += table(linear_focus, ['transaction_id','weak_index','nominal_leakage','predicted_leakage','rotation_difference_rad','fd_selected_weak_response_norm'])
    text += '\n\n## Closed-loop windows\n\n' + table(windows, list(windows[0])) + '\n'
    (OUT / 'FIRST_DIVERGENCE_ANALYSIS.md').write_text(text)
    print(table(metrics, keys), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--executable', type=Path)
    args = parser.parse_args()
    if args.run:
        if args.executable is None:
            parser.error('--run requires --executable')
        run(args.executable.resolve())
    summarize()


if __name__ == '__main__':
    main()
