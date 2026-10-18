#!/usr/bin/env python3
"""Analyze the frozen one-pass screen. No ROS/GT/sensor input reads."""
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from statistics_core import ARMS, axis_similarity, classify, distribution, parse_resource, weak_label


def table(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    with path.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(rows)


def save_plot(fig, path):
    if path.exists():
        raise FileExistsError(path)
    fig.savefig(path, dpi=160, bbox_inches='tight'); plt.close(fig)


def analyze(directory):
    freeze = json.loads((directory / 'run_freeze.json').read_text())
    for filename, identity in freeze['files'].items():
        data = Path(filename).read_bytes()
        if len(data) != identity['bytes'] or hashlib.sha256(data).hexdigest() != identity['sha256']:
            raise ValueError('frozen run artifact changed: ' + filename)
    rows = table(directory / 'hessian_statistics.csv')
    aligns = table(directory / 'alignments.csv')
    manifest = table(directory / 'sample_manifest.csv')
    assert len(rows) == 600 and len(aligns) == 300 and len(manifest) == 100
    primary = {arm: {int(r['scan_index']): r for r in rows if r['arm'] == arm and r['evaluation_pose'] == 'FULL_360_COMMON'} for arm in ARMS}
    assert all(len(primary[a]) == 100 for a in ARMS)
    ids = sorted(primary[ARMS[0]])
    assert ids == [int(r['scan_index']) for r in manifest]
    assert all(primary[ARMS[1]][k]['source_points'] == primary[ARMS[2]][k]['source_points'] for k in ids)
    assert all(len({primary[a][k]['target_hash'] for a in ARMS}) == 1 for k in ids)
    assert all(float(r['grid_leaf_m']) == 1.0 for r in aligns)
    result = classify(primary)
    result.update({'task': 'P60-R4-GROUND-CHALLENGE-NDT-DEGENERACY-SCREEN',
        'start_sha': freeze['start_sha'], 'run_code_sha': freeze['code_sha'],
        'binary_sha256': freeze['files']['/home/jian/livox_ws/p60_r4_ground_challenge_degeneracy_runs/build/p60_r4_screen']['sha256'],
        'scan_pairs': len(ids), 'align_calls': sum(int(r['align_calls']) for r in aligns),
        'native_jet_calls': sum(int(r['jet_calls']) for r in rows), 'explicit_dynamic_score_calls': 0,
        'coupled_refinement_calls': 0, 'gt_read': False, 'deskew': False, 'ekf': False,
        'source_start_ns': int(manifest[0]['stamp_ns']), 'source_end_ns': int(manifest[-1]['stamp_ns']),
        'context': 'RAW_ADJACENT_SCAN_REGISTRATION_DIAGNOSTIC; NOT prior-map localization or accuracy evaluation',
        'fov_reference': 'scanner +X; crop after shared production Source preprocessing',
        'ndt_resolution_parameter_m': .8, 'actual_target_gaussian_leaf_m': 1.,
        'runtime': parse_resource(Path(next(p for p in freeze['files'] if p.endswith('/screen_resource.txt'))).read_text()),
        'push_executed': False, 'groups': {}})
    comparison = []
    for arm in ARMS:
        common = list(primary[arm].values())
        own = [r for r in rows if r['arm'] == arm and r['evaluation_pose'] == 'OWN_RESULT']
        a = [r for r in aligns if r['arm'] == arm]
        stats = {'pairs': len(common), 'converged': sum(r['converged'] == '1' for r in a),
                 'common_comparable': sum(r['comparable'] == '1' for r in common),
                 'common_spd': sum(r['spd'] == '1' for r in common),
                 'own_spd': sum(r['spd'] == '1' for r in own),
                 'weak_screen_labels': sum(weak_label(r) for r in common)}
        for key in ['source_points', 'target_points', 'support_fraction', 'ratio6', 'condition6',
                    'ratio_translation', 'ratio_rotation', 'lambda0', 'mean_energy', 'gradient_norm', 'jet_ms']:
            stats[key] = distribution([float(r[key]) for r in common])
        stats['total_curvature_lambda0'] = distribution([float(r['lambda0']) * int(r['source_points']) for r in common])
        for key in ['fitness', 'align_ms', 'iterations', 'translation_norm_m', 'rotation_deg']:
            stats[key] = distribution([float(r[key]) for r in a])
        stats['own_ratio6'] = distribution([float(r['ratio6']) for r in own])
        result['groups'][arm] = stats
        comparison.append({'arm': arm, 'pairs': 100, 'converged': stats['converged'], 'common_spd': stats['common_spd'],
            'source_points_median': stats['source_points']['median'], 'ratio6_median': stats['ratio6']['median'],
            'condition6_median': stats['condition6']['median'], 'translation_ratio_median': stats['ratio_translation']['median'],
            'lambda0_mean_energy_median': stats['lambda0']['median'], 'lambda0_total_energy_median': stats['total_curvature_lambda0']['median'],
            'support_fraction_median': stats['support_fraction']['median'], 'fitness_m2_median': stats['fitness']['median'],
            'align_ms_mean': stats['align_ms']['mean'], 'align_ms_p95': stats['align_ms']['p95'],
            'align_ms_max': stats['align_ms']['max'], 'weak_screen_labels': stats['weak_screen_labels']})
    paired = []
    for k in ids:
        a,b,c = [primary[arm][k] for arm in ARMS]
        paired.append({'scan_index': k, 'stamp_ns': a['stamp_ns'], 'b_points': b['source_points'], 'c_points': c['source_points'],
            'fov_over_count_ratio6': float(b['ratio6'])/float(c['ratio6']),
            'count_over_full_ratio6': float(c['ratio6'])/float(a['ratio6']),
            'fov_over_count_lambda0_per_point': float(b['lambda0'])/float(c['lambda0']),
            'count_over_full_lambda0_per_point': float(c['lambda0'])/float(a['lambda0']),
            'fov_count_weak6_abs_cos': axis_similarity(b,c,'weak',6),
            'fov_count_translation_axis_abs_cos': axis_similarity(b,c),
            'full_count_weak6_abs_cos': axis_similarity(a,c,'weak',6),
            'common_comparable': int(all(x['comparable'] == '1' for x in (a,b,c))),
            'common_spd': int(all(x['spd'] == '1' for x in (a,b,c)))})
    result['paired_statistics'] = {key: distribution([float(r[key]) for r in paired]) for key in list(paired[0])[4:-2]}
    write_csv(directory / 'group_comparison.csv', comparison)
    write_csv(directory / 'paired_comparison.csv', paired)
    with (directory / 'results.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)

    fig, axes = plt.subplots(1, 3, figsize=(14, 4), sharey=True)
    for ax, arm in zip(axes, ARMS):
        values = [[float(r['lambda'+str(i)]) for r in primary[arm].values() if r['spd'] == '1'] for i in range(6)]
        ax.boxplot(values, labels=[str(i) for i in range(6)], showfliers=True)
        ax.set_yscale('log'); ax.set_title(arm); ax.set_xlabel('Ascending eigenvalue index'); ax.grid(alpha=.2)
    axes[0].set_ylabel('Mean-energy Hessian curvature (normalized chart)')
    fig.suptitle('Same FULL_360 reference poses; all 100 pairs')
    save_plot(fig, directory / 'eigenvalue_distribution.png')

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    for j, arm in enumerate(ARMS):
        vectors = np.array([[float(r['weak'+str(i)]) for i in range(3)] for r in primary[arm].values()])
        lengths = np.linalg.norm(vectors, axis=1); vectors = vectors / lengths[:, None]
        # Eigenvector signs are arbitrary: canonicalize strongest translation component.
        for v in vectors:
            if v[np.argmax(np.abs(v))] < 0: v *= -1
        axes[0,j].quiver(np.zeros(len(vectors)), np.zeros(len(vectors)), vectors[:,0], vectors[:,1],
                         angles='xy', scale_units='xy', scale=1, alpha=.2)
        axes[0,j].set(xlim=(-1.1,1.1), ylim=(-1.1,1.1), title=arm, xlabel='Target-scanner X', ylabel='Target-scanner Y')
        axes[0,j].set_aspect('equal'); axes[0,j].grid(alpha=.2)
        axes[1,j].boxplot([abs(vectors[:,i]) for i in range(3)], labels=['|X|','|Y|','|Z|'])
        axes[1,j].set(ylim=(0,1.05), ylabel='Normalized translation part of weakest 6D axis')
    fig.suptitle('Weakest axes in each previous-scan frame; NOT world trajectory directions')
    save_plot(fig, directory / 'weak_direction_visualization.png')

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    times = (np.array([int(primary[ARMS[0]][k]['stamp_ns']) for k in ids], dtype=np.int64) - int(manifest[0]['stamp_ns'])) * 1e-9
    for arm in ARMS:
        values = [primary[arm][k] for k in ids]
        axes[0].plot(times,[float(r['ratio6']) for r in values], label=arm)
        axes[1].plot(times,[float(r['lambda0']) for r in values], label=arm)
        axes[2].plot(times,[float(r['ratio_translation']) for r in values], label=arm)
    for ax in axes: ax.grid(alpha=.25); ax.legend()
    axes[0].set_yscale('log'); axes[0].axhline(.001,color='k',ls=':',label='descriptive screen')
    axes[0].set_ylabel('6D min/max eigenvalue ratio')
    axes[1].set_ylabel('Smallest mean-energy curvature')
    axes[2].set_ylabel('Translation block min/max ratio'); axes[2].set_xlabel('Seconds since first sampled Source')
    save_plot(fig, directory / 'common_pose_curvature_time.png')
    print(json.dumps({k:v for k,v in result.items() if k not in ['groups','paired_statistics']},indent=2))


if __name__ == '__main__':
    analyze(Path(__file__).resolve().parent)
