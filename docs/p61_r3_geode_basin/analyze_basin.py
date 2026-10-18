"""Frozen-output-only classification. Never runs NDT, accesses GT, or changes thresholds."""
from collections import Counter
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from prepare_basin import table, write_csv, identity
from clusters import complete_link, unique_initials, distance, separated, medoid, competing_pair
from run_frozen import bind_preflight


def matrix(row, prefix):
    return np.array([[float(row[f'{prefix}{i}{j}']) for j in range(4)] for i in range(4)])


def distribution(values):
    a = np.array(values, dtype=float)
    a = a[np.isfinite(a)]
    return {'count': len(a), 'mean': float(a.mean()) if len(a) else None,
            'median': float(np.median(a)) if len(a) else None,
            'p95': float(np.percentile(a, 95)) if len(a) else None,
            'min': float(a.min()) if len(a) else None, 'max': float(a.max()) if len(a) else None}


def verify_receipt(root, mode):
    record = json.loads((root / f'{mode}_receipt.json').read_text())
    if record['exit_code'] != 0: raise ValueError('unsuccessful run')
    for name, frozen in record['outputs'].items():
        if identity(name) != frozen: raise ValueError('run output modified: ' + name)
    return record


def analyze(root, preregistered, output):
    root, pre, output = Path(root), Path(preregistered), Path(output)
    verify_receipt(root, 'verify'); receipt = verify_receipt(root, 'align')
    preflight = json.loads((root / 'verify_receipt.json').read_text())
    bind_preflight(preflight, pre / 'preregistration.json', root / 'build/basin_probe')
    bind_preflight(receipt, pre / 'preregistration.json', root / 'build/basin_probe')
    frozen = json.loads((pre / 'preregistration.json').read_text())
    for name, expected in frozen['files'].items():
        if identity(name) != expected: raise ValueError('input changed after run')
    starts = table(pre / 'initial_manifest.csv')
    align = table(root / 'align/alignments.csv'); deriv = table(root / 'align/derivatives.csv')
    ids = [r['candidate_id'] for r in starts]
    if ids != [r['candidate_id'] for r in align] or ids != [r['candidate_id'] for r in deriv]:
        raise ValueError('missing or reordered candidates')
    rows, clusters, pair_rows = [], [], []
    math_errors = []; rigid_outputs = {}
    for initial, a, d in zip(starts, align, deriv):
        row = {**initial, **a, **d}
        if a['source_hash'] != initial['source_hash'] or a['target_hash'] != initial['target_hash']:
            raise ValueError('processed source/target changed')
        row['translation_to_nominal_m'] = row['rotation_to_nominal_deg'] = float('nan')
        row['nominal_separated'] = 0; row['competitive_energy'] = 0
        if a['rigid'] == '1':
            pose, nominal = matrix(a, 'pose'), matrix(initial, 'nominal')
            rigid_outputs[row['candidate_id']] = pose
            t, r = distance(pose, nominal)
            row.update(translation_to_nominal_m=t, rotation_to_nominal_deg=r,
                       nominal_separated=int(separated(pose, nominal)))
        row['energy_difference'] = float(d['mean_energy']) - float(initial['nominal_energy'])
        row['competitive_energy'] = int(d['jet_valid'] == '1' and
            abs(row['energy_difference']) <= .01 * max(1., abs(float(initial['nominal_energy']))))
        if d['jet_valid'] == '1':
            H = np.array([[float(d[f'H{i}{j}']) for j in range(6)] for i in range(6)])
            g = np.array([float(d[f'g{i}']) for i in range(6)])
            values = np.linalg.eigvalsh(H)
            saved = np.array([float(d[f'lambda{i}']) for i in range(6)])
            err = np.max(abs(values-saved)/np.maximum(1., abs(values)))
            if err > 1e-8: raise ValueError('independent final eigenvalues differ')
            math_errors.append(float(err))
            if values[0] > 0:
                delta = np.linalg.solve(H, g)
                if not np.isclose(np.linalg.norm(delta), float(d['newton_step_norm']), rtol=1e-8, atol=1e-10):
                    raise ValueError('independent stationary step differs')
                strict = float(d['support_fraction']) >= .5 and np.linalg.norm(g) <= 1e-3 and np.linalg.norm(delta) <= 1e-3
                if strict != (d['strict_stationary'] == '1'):
                    raise ValueError('strict stationary guard mismatch')
        rows.append(row)
    for index in frozen['indices']:
        pair = [r for r in rows if int(r['scan_index']) == index]
        by_id = {r['candidate_id']: r for r in pair}
        poses = {k: rigid_outputs[k] for k, r in by_id.items() if r['converged'] == '1' and k in rigid_outputs}
        initials = {k: matrix(r, 'initial') for k, r in by_id.items()}
        nominal = matrix(pair[0], 'nominal')
        groups = complete_link(poses); qualifying = []
        for k, group in enumerate(groups):
            unique = unique_initials(group, initials)
            stationary_unique = unique_initials([x for x in group if by_id[x]['strict_stationary'] == '1'], initials)
            repeat = len(unique) >= 2
            far = all(separated(poses[x], nominal) for x in group)
            competitive = all(by_id[x]['competitive_energy'] for x in group)
            if repeat and competitive and len(stationary_unique) >= 2: qualifying.append(group)
            representative = medoid(group, poses)
            t, r = distance(poses[representative], nominal)
            cluster_id = f'{index}:C{k+1}'
            clusters.append({'scan_index': index, 'cluster_id': cluster_id, 'members': ';'.join(group),
                             'size': len(group), 'unique_physical_starts': len(unique),
                             'strict_stationary_unique_starts': len(stationary_unique),
                             'repeat': int(repeat), 'far_from_nominal': int(far),
                             'competitive_energy': int(competitive), 'medoid': representative,
                             'translation_to_nominal_m': t, 'rotation_to_nominal_deg': r,
                             'max_energy_difference': max(abs(by_id[x]['energy_difference']) for x in group),
                             'max_gradient_norm': max(float(by_id[x]['gradient_norm']) for x in group),
                             'max_newton_step_norm': max(float(by_id[x]['newton_step_norm']) for x in group)})
            for x in group: by_id[x]['cluster_id'] = cluster_id
        local = [c for c in clusters if c['scan_index'] == index]
        pair_rows.append({'scan_index': index, 'starts': len(pair),
                          'align_calls': sum(int(r['align_calls']) for r in pair),
                          'converged': sum(r['converged'] == '1' for r in pair),
                          'cluster_count': len(groups), 'repeat_clusters': sum(c['repeat'] for c in local),
                          'far_repeat_clusters': sum(c['repeat'] and c['far_from_nominal'] for c in local),
                          'competitive_far_repeat_clusters': sum(c['repeat'] and c['far_from_nominal'] and c['competitive_energy'] for c in local),
                          'stationary_competing_basins': int(competing_pair(qualifying, poses, nominal)),
                          'nominal_separated_outputs': sum(r['nominal_separated'] for r in pair)})
    for row in rows: row.setdefault('cluster_id', 'UNCLUSTERED')
    calls = sum(int(r['align_calls']) for r in rows)
    if calls != frozen['ready'] or calls > 240: raise ValueError('align budget mismatch')
    supported = sum(r['stationary_competing_basins'] for r in pair_rows)
    separated_outputs = sum(r['nominal_separated'] for r in rows)
    final = 'SUPPORTED_COMPETING_BASINS_OBSERVED' if supported else (
        'INITIALIZATION_SENSITIVITY_ONLY' if separated_outputs else 'NONLOCAL_BASIN_NOT_DEMONSTRATED')
    families = []
    for family in ('WEAK', 'STRONG', 'COUPLED_INITIAL'):
        selected = [r for r in rows if r['family'] == family]
        result = {'family': family, 'declared': len(selected), 'align_calls': sum(int(r['align_calls']) for r in selected),
                  'converged': sum(r['converged'] == '1' for r in selected),
                  'strict_stationary': sum(r['strict_stationary'] == '1' for r in selected),
                  'far_from_nominal': sum(r['nominal_separated'] for r in selected),
                  'competitive_energy': sum(r['competitive_energy'] for r in selected)}
        for key in ('translation_to_nominal_m', 'rotation_to_nominal_deg', 'energy_difference', 'align_ms'):
            stats = distribution([float(r[key]) for r in selected if r['align_calls'] == '1'])
            result.update({key+'_'+k: v for k, v in stats.items()})
        families.append(result)
    curvature = table(pre / 'effective_curvature.csv')
    summary = {'task': 'P61-R3-GEODE-COUPLED-BASIN-DISCRIMINATION', 'final_result': final,
               'code_sha': receipt['code_sha'], 'pairs': len(pair_rows), 'declared_starts': len(rows),
               'align_calls': calls, 'nominal_preflight_jet_calls': 20,
               'final_jet_calls': sum(int(r['jet_calls']) for r in rows),
               'converged': sum(r['converged'] == '1' for r in rows),
               'out_of_bounds': sum(r['status'] == 'OUT_OF_BOUNDS' for r in rows),
               'status_counts': dict(Counter(r['status'] for r in rows)),
               'nominal_separated_outputs': separated_outputs, 'supported_basin_pairs': supported,
               'cluster_count': len(clusters), 'repeat_clusters': sum(c['repeat'] for c in clusters),
               'far_repeat_clusters': sum(c['repeat'] and c['far_from_nominal'] for c in clusters),
               'competitive_far_repeat_clusters': sum(c['repeat'] and c['far_from_nominal'] and c['competitive_energy'] for c in clusters),
               'strict_stationary_outputs': sum(r['strict_stationary'] == '1' for r in rows),
               'non_spd_jets': sum(r['jet_valid'] == '1' and r['spd'] != '1' for r in rows),
               'max_independent_eigen_error': max(math_errors, default=None),
               'families': families, 'gt_read': False, 'bag_read': False, 'coupled_refinement_calls': 0,
               'ekf_calls': 0, 'nominal_align_calls': 0,
               'effective_curvature': {key: distribution([float(r[key]) for r in curvature]) for key in
                   ('tt_lambda0', 'eff_lambda0', 'weak_curvature_loss', 'trace_curvature_loss', 'rr_condition')}}
    output.mkdir(exist_ok=False)
    write_csv(output / 'all_outputs.csv', rows)
    cluster_fields = ['scan_index', 'cluster_id', 'members', 'size', 'unique_physical_starts',
                      'strict_stationary_unique_starts', 'repeat', 'far_from_nominal', 'competitive_energy',
                      'medoid', 'translation_to_nominal_m', 'rotation_to_nominal_deg',
                      'max_energy_difference', 'max_gradient_norm', 'max_newton_step_norm']
    write_csv(output / 'clusters.csv', clusters, fields=cluster_fields)
    write_csv(output / 'pair_summary.csv', pair_rows); write_csv(output / 'family_comparison.csv', families)
    with (output / 'results.json').open('x') as stream: json.dump(summary, stream, indent=2, allow_nan=False)
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    for j, (key, label) in enumerate([('translation_to_nominal_m', 'Separation from frozen nominal (m)'),
                                     ('energy_difference', 'Mean native energy minus nominal'),
                                     ('gradient_norm', 'Mean-energy gradient norm')]):
        data = [[float(r[key]) for r in rows if r['family'] == family and r['align_calls'] == '1'] for family in
                ('WEAK', 'STRONG', 'COUPLED_INITIAL')]
        axes[j].boxplot(data, labels=['Weak', 'Strong', 'Coupled init']); axes[j].set_ylabel(label); axes[j].grid(alpha=.2)
    fig.suptitle('Raw neighboring-scan diagnostic; no GT; NOT online localization')
    fig.tight_layout();fig.savefig(output / 'basin_comparison.png', dpi=160);plt.close(fig)
    print(json.dumps({k: summary[k] for k in ('final_result', 'align_calls', 'converged', 'out_of_bounds',
        'cluster_count', 'repeat_clusters', 'far_repeat_clusters', 'strict_stationary_outputs', 'nominal_separated_outputs')}, indent=2))


if __name__ == '__main__': analyze(*sys.argv[1:])
