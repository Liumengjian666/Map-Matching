#!/usr/bin/env python3
"""R20 historical detection audit: reads R19 archived CSV/JSON only; no GT file."""
import csv
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
R19 = HERE.parent / 'p62_r19_native_ndt_basin_stress_test'
THRESHOLD = 0.30


def read(name):
    with (R19 / name).open(newline='') as stream:
        return list(csv.DictReader(stream))


def key(row):
    return row['role'], row['arm'], int(row['frame'])


def percentile(values, p):
    v = sorted(values)
    if not v:
        return None
    position = (len(v) - 1) * p / 100
    lo, hi = math.floor(position), math.ceil(position)
    return v[lo] + (v[hi] - v[lo]) * (position - lo)


def distribution(values):
    v = [float(x) for x in values if x != 'NA' and math.isfinite(float(x))]
    if not v:
        return {'n': 0}
    return {'n': len(v), 'min': min(v), 'mean': sum(v) / len(v),
            'median': percentile(v, 50), 'p95': percentile(v, 95), 'max': max(v)}


def errors(values):
    d = distribution(values)
    if d['n']:
        v = [float(x) for x in values]
        d['rmse'] = math.sqrt(sum(x * x for x in v) / len(v))
    return d


def longest_run(flags):
    result = run = 0
    for flag in flags:
        run = run + 1 if flag else 0
        result = max(result, run)
    return result


def records(rows):
    return [{'frame': int(r['frame']), 'raw_ordinal': int(r['raw_ordinal']),
             'stamp_ns': int(r['stamp_ns'])} for r in rows]


def main():
    native = read('NDT_RESULTS.csv')
    comparisons = read('BASIN_COMPARISON.csv')
    reference = read('REFERENCE_ERROR.csv')
    runtime = read('RUNTIME_AND_MEMORY.csv')
    historic = json.loads((R19 / 'results.json').read_text())
    assert len(native) == 480 and len(comparisons) == 360 and len(reference) == 1440
    harmful = {key(r) for r in comparisons if r['level2_suspect_frame'] == '1'}
    alarms = {key(r) for r in native if float(r['heldout_mean']) > THRESHOLD}
    assert len({key(r) for r in native}) == 480
    assert len(harmful) == historic['level2_suspect_comparisons'] == 21
    assert (len(alarms & harmful), len(alarms - harmful), len(harmful - alarms)) == (21, 14, 0)
    trajectories = {}
    for role in ('WEAK', 'STRONG'):
        for arm in ('C0', 'C1', 'C2', 'C3'):
            rows = sorted((r for r in native if r['role'] == role and r['arm'] == arm),
                          key=lambda r: int(r['frame']))
            assert [int(r['frame']) for r in rows] == list(range(1, 61))
            heldout = [r for r in rows if key(r) in alarms]
            quality_failed = [r for r in rows if r['quality_feedback'] != '1']
            # R20 event is the UNION: failed old basic quality OR high heldout loss.
            events = [r for r in rows if r['quality_feedback'] != '1' or key(r) in alarms]
            e = [r for r in reference if r['role'] == role and r['arm'] == arm]
            error_summary = {}
            for estimate in ('PREDICTION', 'RAW_NDT', 'CAUSAL_OUTPUT'):
                er = [r for r in e if r['estimate'] == estimate]
                assert len(er) == 60
                error_summary[estimate] = {
                    'translation_m': errors([r['translation_error_m'] for r in er]),
                    'rotation_deg': errors([r['rotation_error_deg'] for r in er])}
            rt = [r for r in runtime if r['role'] == role and r['arm'] == arm]
            trajectories[f'{role}/{arm}'] = {
                'samples': 60, 'r20_uses_this_initialization': arm in ('C0', 'C3'),
                'historical_native_aligns': 60, 'r20_recomputed_baseline_aligns': 0,
                'heldout_gt_0p30_alarm_count': len(heldout),
                'heldout_gt_0p30_alarm_frames': records(heldout),
                'basic_quality_fail_count': len(quality_failed),
                'basic_quality_fail_frames': records(quality_failed),
                'r20_rule_event_count_on_historical_outputs': len(events),
                'r20_rule_event_frames_on_historical_outputs': records(events),
                'map_observation_accept_count': 60 - len(quality_failed),
                'prediction_fallback_count': len(quality_failed),
                'longest_consecutive_prediction_fallback': longest_run(
                    r['quality_feedback'] != '1' for r in rows),
                'level2_suspect_count': sum(key(r) in harmful for r in rows),
                'level2_suspect_frames': records([r for r in rows if key(r) in harmful]),
                'quality_distributions': {field: distribution([r[field] for r in rows])
                                          for field in ('native_score', 'overlap_0p5', 'heldout_mean')},
                'reference_relative_error': error_summary,
                'historical_runtime': {
                    'full_align_ms': distribution([r['full_align_ms'] for r in rt]),
                    'frame_wall_ms': distribution([r['frame_wall_ms'] for r in rt]),
                    'peak_rss_KiB': max(int(r['peak_rss_KiB']) for r in rt)}}
    names = ['NDT_RESULTS.csv', 'BASIN_COMPARISON.csv', 'REFERENCE_ERROR.csv',
             'RUNTIME_AND_MEMORY.csv', 'results.json']
    result = {
        'task': 'P62-R20-QUALITY-GATED-NDT-RECOVERY-BASELINE',
        'scope': 'HISTORICAL_R19_DEVELOPMENT_SET_DETECTION_ONLY',
        'threshold': {'quantity': 'heldout_mean', 'comparison': '>', 'value': THRESHOLD,
                      'source': 'Chosen after observing R19; not independent confirmation'},
        'harmful_label': {
            'source': 'R19 BASIN_COMPARISON.csv level2_suspect_frame; raw native outputs',
            'predicate': 'Both finite/converged and basic quality valid; raw separation >0.10m OR >1deg; '
                         'C0 error <=0.3m AND <=3deg; disturbed translation >0.5m AND worsening >=0.3m '
                         'OR disturbed rotation >5deg AND worsening >=3deg',
            'frame_label_not_persistent_case': True,
            'limitation': 'MAP_ANCHORED_BENCHMARK_REFERENCE; conditional Base; not physical truth'},
        'all_480_rows': {
            'samples': 480, 'suspect': len(harmful), 'other': 480-len(harmful),
            'alarms': len(alarms), 'suspect_detected': len(alarms & harmful),
            'suspect_missed': len(harmful-alarms),
            'additional_alarm_on_other_rows': len(alarms-harmful),
            'no_alarm_on_other_rows': 480-len(alarms | harmful),
            'suspect_detection_fraction': len(alarms & harmful)/len(harmful),
            'additional_alarm_fraction_on_other_rows': len(alarms-harmful)/(480-len(harmful))},
        'interpretation': 'Other/non-suspect is not a certified correct class; 14 extra alarms '
                          'must not be called independently proven false positives. '
                          'Historical replay does not predict new causal trajectories.',
        'trajectories': trajectories,
        'source_sha256': {str(R19 / n): hashlib.sha256((R19/n).read_bytes()).hexdigest() for n in names},
        'new_ndt_aligns': 0, 'raw_reference_pose_files_read': 0}
    (HERE/'HISTORICAL_DETECTION.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result['all_480_rows'],indent=2))


if __name__ == '__main__':
    main()
