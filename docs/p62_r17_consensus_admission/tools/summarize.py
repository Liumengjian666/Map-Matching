#!/usr/bin/env python3
"""Summarize immutable native rows and post-freeze reference rows."""
import csv
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parents[1]


def rows(name):
    with (HERE / name).open(newline='') as stream:
        return list(csv.DictReader(stream))


def stats(values):
    a = np.asarray(list(values), dtype=float)
    return {'n': int(a.size), 'mean': float(np.mean(a)), 'p95': float(np.percentile(a, 95)),
            'max': float(np.max(a)), 'sum': float(np.sum(a))}


def main():
    decisions = rows('native_run_01/ADMISSION_DECISIONS.csv')
    runtimes = rows('native_run_01/RUNTIME_AND_MEMORY.csv')
    errors = rows('REFERENCE_ERROR.csv')
    calls = rows('native_run_01/ALIGN_CALLS.csv')
    assert len(decisions) == len(runtimes) == len(errors) == 90
    assert len(calls) == 91
    by = {(int(r['frame']), r['policy']): r for r in decisions}
    er = {(int(r['frame']), r['policy']): r for r in errors}
    policies = ['BASELINE_5DEG', 'RELAXED_6DEG', 'EVENT_CONSENSUS']
    with (HERE / 'THREE_POLICY_COMPARISON.csv').open('x', newline='') as stream:
        keys = ['frame', 'raw_ordinal']
        for p in policies:
            keys += [p + '_decision', p + '_accepted', p + '_rotation_TP_deg',
                     p + '_output_translation_error_m', p + '_output_rotation_error_deg']
        keys += ['relaxed_consensus_output_exactly_equal']
        out = csv.DictWriter(stream, keys, lineterminator='\n');out.writeheader()
        for f in range(1, 31):
            r = {'frame': f, 'raw_ordinal': 7552 + f}
            for p in policies:
                d = by[f, p];e = er[f, p]
                r.update({p + '_decision': d['decision'], p + '_accepted': d['accepted'],
                          p + '_rotation_TP_deg': d['full_rotation_TP_deg'],
                          p + '_output_translation_error_m': e['output_translation_error_m'],
                          p + '_output_rotation_error_deg': e['output_rotation_error_deg']})
            r['relaxed_consensus_output_exactly_equal'] = int(by[f, policies[1]]['output_pose'] == by[f, policies[2]]['output_pose'])
            out.writerow(r)
    runtime = {}
    for p in policies:
        rr = [r for r in runtimes if r['policy'] == p]
        runtime[p] = {name: stats(float(r[name]) for r in rr) for name in
            ('predict_ms', 'full_align_ms', 'full_aux_ms', 'full_validation_ms',
             'split_align_ms', 'split_aux_ms', 'split_validation_ms', 'decision_ms', 'arm_wall_ms')}
        runtime[p]['full_calls'] = sum(c['policy'] == p and c['kind'] == 'FULL' for c in calls)
        runtime[p]['extra_split_calls'] = sum(c['policy'] == p and c['kind'] == 'EVENT_SPLIT' for c in calls)
    runtime['shared_pcd_load_ms'] = stats(float(r['pcd_load_ms']) for r in runtimes if r['policy'] == policies[0])
    runtime['shared_source_preprocess_ms'] = stats(float(r['preprocess_ms']) for r in runtimes if r['policy'] == policies[0])
    runtime['peak_rss_KiB'] = max(int(r['peak_rss_KiB']) for r in runtimes)
    (HERE / 'RUNTIME_SUMMARY.json').write_text(json.dumps(runtime, indent=2) + '\n')
    print(json.dumps({p: {'accepted': sum(d['accepted'] == '1' for d in decisions if d['policy'] == p),
                           'full_calls': runtime[p]['full_calls'], 'extra_split': runtime[p]['extra_split_calls'],
                           'arm_wall_ms': runtime[p]['arm_wall_ms']} for p in policies}, indent=2))


if __name__ == '__main__':
    main()
