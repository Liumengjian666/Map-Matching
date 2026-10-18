#!/usr/bin/env python3
"""Analyze frozen native outputs only; no registration, GT or external disk reads."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

DOC = Path(__file__).resolve().parent
sys.path.insert(0, str(DOC.parent / 'p60_r4_ground_challenge_ndt_degeneracy'))
from statistics_core import distribution, weak_label, axis_similarity, parse_resource
from check_geometry_gate import table, check


def qualified(row): return row['comparable'] == '1' and weak_label(row)


def runs(rows):
    result, current = [], []
    for row in rows + [None]:
        eligible = row is not None and qualified(row)
        aligned = not current or (eligible and axis_similarity(current[-1], row) >= math.cos(math.radians(20)))
        if not eligible or not aligned:
            if len(current) >= 3:
                indices = [int(r['scan_index']) for r in current]
                result.append({'start_index': indices[0], 'end_index': indices[-1], 'sample_count': len(indices),
                               'indices': indices, 'start_ns': int(current[0]['stamp_ns']), 'end_ns': int(current[-1]['stamp_ns']),
                               'span_sec': (int(current[-1]['stamp_ns']) - int(current[0]['stamp_ns'])) * 1e-9,
                               'all_intermediate_raw_indices_measured': all(b == a + 1 for a, b in zip(indices, indices[1:]))})
            current = []
        if eligible: current.append(row)
    return result


def statistics(rows):
    valid = [r for r in rows if r['comparable'] == '1' and r['spd'] == '1']
    result = {'rows': len(rows), 'comparable': sum(r['comparable'] == '1' for r in rows),
              'spd': sum(r['spd'] == '1' for r in rows), 'qualified_spd': len(valid),
              'severe_weak_labels': sum(qualified(r) for r in rows),
              'severe_weak_indices': [int(r['scan_index']) for r in rows if qualified(r)],
              'sampled_runs': runs(rows)}
    for key in ('ratio6', 'condition6', 'ratio_translation', 'ratio_rotation', 'lambda0', 'support_fraction',
                'source_points', 'target_points', 'gradient_norm', 'mean_energy', 'jet_ms'):
        result[key] = distribution([float(r[key]) for r in valid])
    vectors = np.array([[float(r[f'weak{i}']) for i in range(3)] for r in valid])
    blocks = np.array([[float(r[f'translation_weak{i}']) for i in range(3)] for r in valid])
    result['weak6_translation_dominant_xyz_count'] = [int(np.sum(np.argmax(abs(vectors), axis=1) == i)) for i in range(3)] if len(vectors) else [0] * 3
    result['translation_block_weak_dominant_xyz_count'] = [int(np.sum(np.argmax(abs(blocks), axis=1) == i)) for i in range(3)] if len(blocks) else [0] * 3
    result['weak6_translation_norm'] = distribution(np.linalg.norm(vectors, axis=1)) if len(vectors) else distribution([])
    return result


def verify_freeze(path):
    freeze = json.loads(path.read_text())
    for name, identity in freeze['files'].items():
        data = Path(name).read_bytes()
        if len(data) != identity['bytes'] or hashlib.sha256(data).hexdigest() != identity['sha256']:
            raise ValueError('frozen artifact changed: ' + name)


def write_csv(path, rows):
    with path.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(rows)


def main(run, output):
    for phase in ('smoke', 'screen'): verify_freeze(run / f'{phase}_output_freeze.json')
    output.mkdir(exist_ok=False)
    result = {'task': 'P61-R2-GEODE-SHIELD-TUNNEL-REAL-DATA-RUN', 'final_result': 'GEODE_NATIVE_GEOMETRY_COMPLETED',
              'context': 'RAW_NATIVE_LIDAR_GEOMETRY_DIAGNOSTIC', 'phases': {}, 'gt_used': False,
              'deskew_calls': 0, 'ekf_updates': 0, 'oosm_updates': 0, 'coupled_calls': 0, 'fused_smoke_completed': False,
              'hessian_not_covariance': True, 'fov_crop': False}
    math_check = {'matrices': 0, 'max_relative_eigen_error': 0., 'max_relative_vector_residual': 0.}
    groups, weak_rows = [], []
    all_indices = []
    for phase, expected in (('smoke', 20), ('screen', 100)):
        gate = check(run / phase, expected)
        if phase == 'smoke' and not gate['engineering_smoke_pass']: raise ValueError('smoke did not pass')
        rows = table(run / phase / 'hessian.csv'); aligns = table(run / phase / 'alignments.csv')
        stats = statistics(rows)
        stats['execution'] = gate; stats['resource'] = parse_resource((run / f'{phase}_resource.txt').read_text())
        for key in ('fitness', 'iterations', 'align_ms', 'target_build_ms'):
            stats[key] = distribution([float(r[key]) for r in aligns])
        for row in rows:
            all_indices.append(int(row['scan_index']))
            weak_rows.append({'phase': phase, 'scan_index': row['scan_index'], 'stamp_ns': row['stamp_ns'],
                              'comparable': row['comparable'], 'spd': row['spd'], 'severe_weak': int(qualified(row)),
                              'ratio6': row['ratio6'], 'ratio_translation': row['ratio_translation'],
                              'weak_tx': row['weak0'], 'weak_ty': row['weak1'], 'weak_tz': row['weak2']})
            if row['jet_valid'] == '1':
                H = np.array([[float(row[f'H{i}{j}']) for j in range(6)] for i in range(6)])
                eigen = np.linalg.eigvalsh(H); saved = np.array([float(row[f'lambda{i}']) for i in range(6)])
                v = np.array([float(row[f'weak{i}']) for i in range(6)])
                error = float(np.max(abs(eigen - saved) / np.maximum(1., abs(saved))))
                residual = float(np.linalg.norm(H @ v - saved[0] * v) / max(1., np.linalg.norm(H)))
                assert error < 1e-8 and residual < 1e-8
                math_check['matrices'] += 1
                math_check['max_relative_eigen_error'] = max(math_check['max_relative_eigen_error'], error)
                math_check['max_relative_vector_residual'] = max(math_check['max_relative_vector_residual'], residual)
        result['phases'][phase] = stats
    baseline_rows = [r for r in table(DOC.parent / 'p60_r5_ndt_resolution_correction/corrected_hessian.csv')
                     if r['arm'] == 'FULL_360' and r['evaluation_pose'] == 'R5_FULL_360_COMMON']
    baseline = statistics(baseline_rows)
    # Jet times are not a benchmark comparison across separate historical runs.
    result['frozen_ground_challenge_full360'] = baseline
    for name, stats in [('GEODE_smoke', result['phases']['smoke']), ('GEODE_screen', result['phases']['screen']),
                        ('P60_R5_frozen_FULL360', baseline)]:
        groups.append({'group': name, 'rows': stats['rows'], 'qualified_spd': stats['qualified_spd'],
                       'severe_weak_labels': stats['severe_weak_labels'],
                       **{key + '_median': stats[key]['median'] for key in ('ratio6', 'condition6', 'ratio_translation', 'lambda0', 'support_fraction')},
                       'weak6_dominant_X': stats['weak6_translation_dominant_xyz_count'][0],
                       'weak6_dominant_Y': stats['weak6_translation_dominant_xyz_count'][1],
                       'weak6_dominant_Z': stats['weak6_translation_dominant_xyz_count'][2]})
    smoke = {r['scan_index']: r for r in table(run / 'smoke/alignments.csv')}
    screen = {r['scan_index']: r for r in table(run / 'screen/alignments.csv')}
    parity = []
    for k in sorted(set(smoke) & set(screen), key=int):
        a, b = smoke[k], screen[k]
        fields = ('source_hash', 'target_hash', 'fitness', 'iterations') + tuple(f'pose{i}{j}' for i in range(4) for j in range(4))
        parity.append({'scan_index': k, 'identical_input_pose_fitness_iterations': int(all(a[key] == b[key] for key in fields))})
    assert all(row['identical_input_pose_fitness_iterations'] for row in parity)
    result.update(total_align_calls=120, total_native_jet_calls=120, actual_grid_audits=360,
                  unique_pair_indices=len(set(all_indices)), repeated_across_phases=len(all_indices) - len(set(all_indices)),
                  independent_math_checks=math_check)
    write_csv(output / 'geometry_comparison.csv', groups)
    write_csv(output / 'weak_direction_records.csv', weak_rows)
    write_csv(output / 'smoke_screen_overlap_parity.csv', parity)
    with (output / 'results.json').open('x') as stream: json.dump(result, stream, indent=2, allow_nan=False)
    rows = table(run / 'screen/hessian.csv')
    times = (np.array([int(r['stamp_ns']) for r in rows], dtype=np.int64) - int(rows[0]['stamp_ns'])) * 1e-9
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for ax, key, threshold in zip(axes, ('ratio6', 'ratio_translation', 'support_fraction'), (.001, .01, .5)):
        ax.plot(times, [float(r[key]) for r in rows], '.-', label=key)
        ax.axhline(threshold, color='k', ls=':', label='Frozen diagnostic threshold')
        ax.grid(alpha=.25); ax.legend(); ax.set_ylabel(key)
    axes[0].set_yscale('log'); axes[1].set_yscale('log'); axes[2].set_xlabel('Sensor seconds since first sampled Source')
    fig.suptitle('100 native Avia tunnel pairs; gaps between samples are NOT measured')
    fig.tight_layout(); fig.savefig(output / 'curvature_support_time.png', dpi=160); plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for stats, label in ((result['phases']['screen'], 'GEODE native Avia'), (baseline, 'Frozen Ground FULL360')):
        axes[0].plot(range(3), stats['weak6_translation_dominant_xyz_count'], 'o-', label=label)
    axes[0].set(xticks=range(3), xticklabels=['scanner X', 'scanner Y', 'scanner Z'], ylabel='Dominant translation component count')
    axes[0].legend(); axes[0].grid(alpha=.2)
    eigen = [[float(r[f'lambda{i}']) for r in rows if r['spd'] == '1' and r['comparable'] == '1'] for i in range(6)]
    axes[1].boxplot(eigen, labels=list(map(str, range(6)))); axes[1].set(yscale='log', xlabel='Sorted eigenvalue index', ylabel='Normalized mean-energy Hessian curvature')
    fig.tight_layout(); fig.savefig(output / 'weak_axes_and_spectrum.png', dpi=160); plt.close(fig)
    print(json.dumps({'final_result': result['final_result'], 'groups': groups, 'runs': result['phases']['screen']['sampled_runs']}, indent=2))


if __name__ == '__main__': main(Path(sys.argv[1]), Path(sys.argv[2]))
