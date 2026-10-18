#!/usr/bin/env python3
"""Read frozen R4/R5 outputs only; never runs registration or reads GT."""
import csv
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

DOC = Path(__file__).resolve().parent
R4 = DOC.parent / 'p60_r4_ground_challenge_ndt_degeneracy'
sys.path.insert(0, str(R4))
from statistics_core import ARMS, classify, distribution, parse_resource, weak_label


def table(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    with path.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(rows)


def verify(freeze):
    for path, expected in freeze['files'].items():
        data = Path(path).read_bytes()
        if len(data) != expected['bytes'] or hashlib.sha256(data).hexdigest() != expected['sha256']:
            raise ValueError('frozen artifact changed: ' + path)


def main(run, out):
    out.mkdir(exist_ok=False)
    code = json.loads((run / 'run_code_freeze.json').read_text())
    frozen = json.loads((run / 'output_freeze.json').read_text())
    verify(code); verify(frozen)
    input_freeze = json.loads((run / 'input_freeze.json').read_text())
    for path, sha in input_freeze['files'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != sha:
            raise ValueError('R4 reference changed: ' + path)
    old_rows = table(R4 / 'hessian_statistics.csv')
    rows = table(run / 'formal/corrected_hessian.csv')
    old_align = table(R4 / 'alignments.csv')
    aligns = table(run / 'formal/corrected_alignments.csv')
    grid = table(run / 'formal/grid_audit.csv')
    parity = table(run / 'formal/input_parity.csv')
    assert len(rows) == 600 and len(aligns) == 300 and len(grid) == 1000 and len(parity) == 300
    assert all(r['pass'] == '1' for r in parity + grid)
    actual = float(np.float32(.8))
    assert all(all(float(r[k]) == actual for k in ('leaf_x', 'leaf_y', 'leaf_z')) for r in grid)
    assert all(float(r['grid_leaf_m']) == 1. for r in old_align)
    assert sum(int(r['align_calls']) for r in aligns) == 300
    assert sum(int(r['jet_calls']) for r in rows) == 600
    labels = ('R4_GRID1_FIXED', 'R5_GRID08_FIXED', 'R5_GRID08_NEW')
    sets = [old_rows, rows, rows]
    evaluation = ('FULL_360_COMMON', 'R4_FULL_360_FROZEN', 'R5_FULL_360_COMMON')
    groups = {}
    for label, source, ref in zip(labels, sets, evaluation):
        groups[label] = {arm: {int(r['scan_index']): r for r in source
                              if r['arm'] == arm and r['evaluation_pose'] == ref} for arm in ARMS}
        assert all(len(group) == 100 for group in groups[label].values())
    ids = sorted(groups[labels[0]][ARMS[0]])
    assert all(sorted(group) == ids for item in groups.values() for group in item.values())
    for label in labels:
        for k in ids:
            assert groups[label][ARMS[1]][k]['source_points'] == groups[label][ARMS[2]][k]['source_points']
            for arm in ARMS:
                for key in ('stamp_ns', 'source_points', 'target_points', 'source_hash', 'target_hash'):
                    assert groups[label][arm][k][key] == groups[labels[0]][arm][k][key]
    checks = {'matrices': 0, 'max_eigenvalue_relative_error': 0., 'max_weak_axis_relative_residual': 0.,
              'max_symmetry_relative_residual': 0.}
    for row in rows:
        if row['jet_valid'] != '1':
            continue
        H = np.array([[float(row[f'H{i}{j}']) for j in range(6)] for i in range(6)])
        values, _ = np.linalg.eigh(H)
        saved = np.array([float(row[f'lambda{i}']) for i in range(6)])
        vector = np.array([float(row[f'weak{i}']) for i in range(6)])
        scale = max(1., np.linalg.norm(H))
        error = float(np.max(abs(values - saved) / np.maximum(1., abs(saved))))
        residual = float(np.linalg.norm(H @ vector - saved[0] * vector) / scale)
        symmetry = float(np.linalg.norm(H - H.T) / scale)
        assert error < 1e-8 and residual < 1e-8 and symmetry < 1e-10
        assert (values[0] > 0) == (row['spd'] == '1')
        checks['matrices'] += 1
        checks['max_eigenvalue_relative_error'] = max(checks['max_eigenvalue_relative_error'], error)
        checks['max_weak_axis_relative_residual'] = max(checks['max_weak_axis_relative_residual'], residual)
        checks['max_symmetry_relative_residual'] = max(checks['max_symmetry_relative_residual'], symmetry)
    result = {'task': 'P60-R5-NDT-TARGET-RESOLUTION-CORRECTION',
              'start_sha': '5a12a77639a7fa6c2efa69d610a0b27adf8337f8', 'code_sha': code['code_sha'],
              'pairs': 100, 'align_calls': 300, 'native_jet_calls': 600,
              'input_parity_rows': 300, 'input_parity_pass': True,
              'actual_gaussian_leaf_xyz_m': [actual] * 3, 'grid_audit_pass': True, 'grid_audit_rows': 1000,
              'converged': sum(r['converged'] == '1' for r in aligns),
              'nonrigid_align_results': sum(r['status'] == 'NONFINITE_OR_NONRIGID' for r in aligns),
              'invalid_jets': sum(r['jet_valid'] != '1' for r in rows),
              'references': {}, 'alignment_groups': {}, 'independent_math_checks': checks,
              'runtime': parse_resource((run / 'resource.txt').read_text()),
              'gt_read': False, 'hikvision_access': False,
              'coupled_refinement_calls': 0, 'extra_align_calls': 0, 'ekf': False, 'deskew': False,
              'push_executed': False,
              'context': 'RAW_ADJACENT_SCAN_DIAGNOSTIC_NOT_LOCALIZATION_ACCURACY'}
    summary, paired, resolution, changes = [], [], [], []
    for label in labels:
        primary = groups[label]
        result['references'][label] = {'frozen_r4_classification': classify(primary), 'groups': {}}
        for arm in ARMS:
            common = list(primary[arm].values())
            stats = {'comparable': sum(r['comparable'] == '1' for r in common),
                     'spd': sum(r['spd'] == '1' for r in common),
                     'weak_labels': sum(weak_label(r) for r in common),
                     'weak_label_indices': [int(r['scan_index']) for r in common if weak_label(r)],
                     'non_spd_indices': [int(r['scan_index']) for r in common if r['spd'] != '1']}
            for key in ('ratio6', 'condition6', 'ratio_translation', 'ratio_rotation', 'lambda0',
                        'gradient_norm', 'mean_energy', 'support_fraction'):
                stats[key] = distribution([float(r[key]) for r in common])
            vectors = np.array([[float(r[f'weak{i}']) for i in range(3)] for r in common])
            stats['weak6_translation_dominant_xyz_count'] = [int(np.sum(np.argmax(abs(vectors), axis=1) == i)) for i in range(3)]
            stats['weak6_translation_norm'] = distribution(np.linalg.norm(vectors, axis=1))
            result['references'][label]['groups'][arm] = stats
            summary.append({'reference': label, 'arm': arm, 'comparable': stats['comparable'], 'spd': stats['spd'],
                            'weak_labels': stats['weak_labels'], **{k + '_median': stats[k]['median'] for k in
                             ('ratio6', 'condition6', 'ratio_translation', 'lambda0', 'support_fraction')}})
        for k in ids:
            a, b, c = [primary[arm][k] for arm in ARMS]
            paired.append({'reference': label, 'scan_index': k,
                           'ratio6_fov_over_count': float(b['ratio6']) / float(c['ratio6']),
                           'lambda0_fov_over_count': float(b['lambda0']) / float(c['lambda0']),
                           'lambda0_count_over_full': float(c['lambda0']) / float(a['lambda0'])})
        selection = [r for r in paired if r['reference'] == label]
        result['references'][label]['paired'] = {key: distribution([r[key] for r in selection]) for key in
                                               ('ratio6_fov_over_count', 'lambda0_fov_over_count', 'lambda0_count_over_full')}
    for k in ids:
        for arm in ARMS:
            old = groups[labels[0]][arm][k]
            for label in labels[1:]:
                new = groups[label][arm][k]
                resolution.append({'scan_index': k, 'arm': arm, 'new_reference': label,
                    'same_pose_grid_only': int(label == labels[1]),
                    'old_comparable': old['comparable'], 'new_comparable': new['comparable'],
                    'old_spd': old['spd'], 'new_spd': new['spd'],
                    **{f'{key}_{side}': row[key] for key in ('ratio6', 'ratio_translation', 'lambda0', 'support_fraction')
                       for side, row in (('r4', old), ('r5', new))},
                    'lambda0_new_over_old': float(new['lambda0']) / float(old['lambda0']),
                    'ratio6_new_over_old': float(new['ratio6']) / float(old['ratio6'])})
    for arm in ARMS:
        current = [r for r in aligns if r['arm'] == arm]
        old_by_id = {int(r['scan_index']): r for r in old_align if r['arm'] == arm}
        stats = {'converged': sum(r['converged'] == '1' for r in current)}
        for key in ('source_points', 'target_points', 'fitness', 'iterations', 'align_ms'):
            stats[key] = distribution([float(r[key]) for r in current])
        for row in current:
            old = old_by_id[int(row['scan_index'])]
            to_pose = lambda r: np.array([[float(r[f'pose{i}{j}']) for j in range(4)] for i in range(4)])
            a, b = to_pose(old), to_pose(row)
            dt = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
            rotation_delta = a[:3, :3] @ b[:3, :3].T
            dr = float(np.degrees(np.arctan2(np.linalg.norm([rotation_delta[2, 1] - rotation_delta[1, 2],
                      rotation_delta[0, 2] - rotation_delta[2, 0], rotation_delta[1, 0] - rotation_delta[0, 1]]) / 2,
                      (np.trace(rotation_delta) - 1) / 2)))
            changes.append({'scan_index': row['scan_index'], 'arm': arm, 'pose_change_m': dt, 'pose_change_deg': dr,
                            'fitness_old': old['fitness'], 'fitness_new': row['fitness'],
                            'iterations_old': old['iterations'], 'iterations_new': row['iterations']})
        for key in ('pose_change_m', 'pose_change_deg'):
            stats[key] = distribution([r[key] for r in changes if r['arm'] == arm])
        result['alignment_groups'][arm] = stats
    corrected = [result['references'][label]['frozen_r4_classification'] for label in labels[1:]]
    valid = all(item['comparable_triplets'] >= 80 for item in corrected)
    positive = any(item['final_result'] in ('NATIVE_DEGENERACY_EVIDENCE_OBSERVED', 'FOV_INDUCED_DEGENERACY_OBSERVED') for item in corrected)
    result['validity_gate_pass'] = valid
    result['final_result'] = ('RESOLUTION_SENSITIVE_WEAKNESS_OBSERVED' if positive else 'DEGENERACY_STILL_NOT_DEMONSTRATED') if valid else 'COMPARABILITY_GATE_FAILED_STOP'
    for name, data in (('group_comparison.csv', summary), ('paired_comparison.csv', paired),
                       ('resolution_comparison.csv', resolution), ('alignment_changes.csv', changes)):
        write_csv(out / name, data)
    with (out / 'results.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    for j, arm in enumerate(ARMS):
        for label in labels:
            common = [groups[label][arm][k] for k in ids]
            axes[0, j].plot(range(6), np.median([[float(r[f'lambda{i}']) for i in range(6)] for r in common], axis=0), marker='o', label=label)
            axes[1, j].plot(ids, [float(r['ratio6']) for r in common], label=label, alpha=.8)
        axes[0, j].set(title=arm, yscale='log', xlabel='Eigenvalue index', ylabel='Median mean-energy curvature')
        axes[1, j].set(yscale='log', xlabel='Frozen scan index', ylabel='6D min/max ratio')
        axes[1, j].axhline(.001, color='k', ls=':')
        for ax in axes[:, j]: ax.grid(alpha=.2); ax.legend(fontsize=7)
    fig.suptitle('Fixed-pose grid effect and new-pose within-run effect kept separate')
    fig.tight_layout(); fig.savefig(out / 'resolution_curvature_comparison.png', dpi=160); plt.close(fig)
    fig, axes = plt.subplots(1, 3, figsize=(14, 4), sharey=True)
    for j, arm in enumerate(ARMS):
        counts = [result['references'][label]['groups'][arm]['weak6_translation_dominant_xyz_count'] for label in labels]
        for i, axis in enumerate('XYZ'):
            axes[j].bar(np.arange(3) + (i - 1) * .25, np.array(counts)[:, i], width=.25, label=axis)
        axes[j].set(xticks=np.arange(3), xticklabels=['R4 1m', 'R5 old pose', 'R5 new pose'], title=arm, ylabel='Count / 100')
        axes[j].legend()
    fig.suptitle('Dominant translation component of weakest 6D eigenvector; scanner axes')
    fig.tight_layout(); fig.savefig(out / 'weak_direction_comparison.png', dpi=160); plt.close(fig)
    print(json.dumps({'result': result['final_result'], 'converged': result['converged'], 'comparability': corrected,
                      'runtime': result['runtime']}, indent=2))


if __name__ == '__main__':
    main(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
