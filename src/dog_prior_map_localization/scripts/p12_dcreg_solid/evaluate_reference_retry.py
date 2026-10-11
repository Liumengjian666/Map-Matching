#!/usr/bin/env python3
"""Summarize only the frozen approximate-pose retry; no GT or candidate choice."""
import csv
import json
import sys
from pathlib import Path
import numpy as np
from prepare_reference_retry import read, write


def stats(values, finite_only=False):
    x = np.asarray(values, dtype=float)
    total = len(x)
    if finite_only:
        x = x[np.isfinite(x)]
    if not len(x):
        return {'count': 0, 'excluded_nonfinite': total, 'mean': None, 'p95': None, 'max': None, 'min': None}
    if not np.isfinite(x).all():
        raise RuntimeError('nonfinite statistic')
    return {'count': len(x), 'excluded_nonfinite': total-len(x),
            'mean': float(np.mean(x)), 'p95': float(np.percentile(x, 95)), 'max': float(np.max(x)),
            'min': float(np.min(x))}


def evaluate(experiment, archive):
    freeze = json.loads((experiment / 'freeze.json').read_text())
    old = read(experiment / 'expected_candidates.csv')
    new = read(experiment / 'run/retry.csv')
    timing = read(experiment / 'run/timing.csv')
    if len(new) != 18 or len(timing) != 6:
        raise RuntimeError('incomplete 6x3 run')
    pairs = []
    for a, b in zip(old, new):
        if any(a[k] != b[k] for k in ('frame_index', 'rank', 'reference_id', 'stamp_ns')):
            raise RuntimeError('candidate association failure')
        if a['converged'] != b['A_converged'] or b['coordinate_status'] != 'APPROXIMATE_REFERENCE_STATE_POSE':
            raise RuntimeError('wrong baseline/coordinate semantics')
        if float(b['translation_change_m']) > 1e-12:
            raise RuntimeError('translation changed in orientation-only diagnostic')
        row = {k: b[k] for k in ('frame_index', 'rank', 'reference_id', 'similarity', 'coordinate_status',
                                'rotation_change_deg', 'translation_change_m', 'initial_distance_to_ndt_m',
                                'reference_header_stamp', 'reference_state_stamp', 'header_minus_state_sec')}
        row.update({f'A_{k}': a[k] for k in ('converged', 'iterations', 'call_wall_ms',
                                           'rmse_before_last_update_m', 'fitness_correspondence_fraction')})
        row.update({f'B_{k}': b[k] for k in ('B_converged', 'iterations', 'call_wall_ms',
                                           'rmse_before_last_update_m', 'fitness_correspondence_fraction',
                                           'api_setup_proxy_ms', 'valid_se3', 'failure_reason')})
        row['B_converged'] = row.pop('B_B_converged')
        row['newly_converged'] = int(a['converged'] == '0' and b['B_converged'] == '1')
        row['lost_convergence'] = int(a['converged'] == '1' and b['B_converged'] == '0')
        pairs.append(row)
    archive.mkdir(parents=True, exist_ok=True)
    write(archive / 'paired_results.csv', pairs)
    group = lambda near: [p for p in pairs if (float(p['initial_distance_to_ndt_m']) <= 5) == near]
    counts = lambda rows: {'calls': len(rows), 'A_converged': sum(int(p['A_converged']) for p in rows),
                          'B_converged': sum(int(p['B_converged']) for p in rows),
                          'newly_converged': sum(p['newly_converged'] for p in rows),
                          'lost_convergence': sum(p['lost_convergence'] for p in rows)}
    resources = (experiment.parent / 'run_resources.txt').read_text()
    rss = int(next(l.split(':')[-1] for l in resources.splitlines() if 'Maximum resident set size' in l))
    seconds = next(l.split('): ', 1)[1] for l in resources.splitlines() if 'Elapsed (wall clock)' in l)
    parts = [float(v) for v in seconds.split(':')]
    wall_s = sum(v * 60 ** i for i, v in enumerate(reversed(parts)))
    result = {'task': 'P12B-REFERENCE-POSE-CANDIDATE-RETRY-R1',
              'exact_coordinate_conversion': False, 'GT_LOADED': False, 'accuracy': 'ACCURACY_GAIN_NOT_EVALUATED',
              'frames': freeze['frames'], 'new_dcreg_calls': len(new), 'new_ndt_calls': 0,
              'feedback_calls': 0, 'overall': counts(pairs), 'near_le_5m': counts(group(True)),
              'far_gt_5m': counts(group(False)), 'valid_se3': sum(int(b['valid_se3']) for b in new),
              'nonfinite_pose': sum(int(b['nonfinite_pose']) for b in new),
              'rotation_change_deg': stats([p['rotation_change_deg'] for p in pairs]),
              'reference_state_offset_abs_sec': stats([abs(float(p['header_minus_state_sec'])) for p in pairs]),
              'A_historical_call_ms': stats([p['A_call_wall_ms'] for p in pairs]),
              'B_call_ms': stats([b['call_wall_ms'] for b in new]),
              'B_author_iteration_ms': stats([b['author_iteration_ms'] for b in new]),
              'B_api_setup_proxy_ms': stats([b['api_setup_proxy_ms'] for b in new]),
              'B_descriptor_ms': stats([t['descriptor_ms'] for t in timing]),
              'B_search_ms': stats([t['search_ms'] for t in timing]),
              'B_combination_ms': stats([t['combination_ms'] for t in timing]),
              'B_complete_scan_ms': stats([t['total_scan_ms'] for t in timing]),
              'B_delta_initial_m': stats([b['delta_from_initial_m'] for b in new], finite_only=True),
              'B_delta_initial_deg': stats([b['delta_from_initial_deg'] for b in new], finite_only=True),
              'wall_s': wall_s, 'peak_rss_kib': rss, 'peak_rss_mib': rss / 1024,
              'frames_with_A_convergence': len({p['frame_index'] for p in pairs if p['A_converged'] == '1'}),
              'frames_with_B_convergence': len({p['frame_index'] for p in pairs if p['B_converged'] == '1'}),
              'limits': ['B is approximate IMU-origin reference pose, not calibrated LiDAR pose',
                         'A historical timing is not same-run paired timing',
                         'Author convergence is NOT verified location correctness',
                         'Setup proxy includes tree/copy/wrapper, not isolated KDtree measurement']}
    (archive / 'metrics.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    evaluate(Path(sys.argv[1]), Path(sys.argv[2]))
