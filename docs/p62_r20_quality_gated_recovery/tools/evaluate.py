#!/usr/bin/env python3
"""Post-seal R20 reference evaluation. Never called by the online C++ runner."""
import csv
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from baseline_stats import distribution, errors, longest_run

HERE = Path(__file__).resolve().parents[1]
R19 = HERE.parent / 'p62_r19_native_ndt_basin_stress_test'
GT = Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment/ground_truth/registered_poses.csv')
POLICIES = ('QUALITY_GATE_ONLY', 'QUALITY_TRIGGERED_MULTISTART')
BASIS = 'MAP_ANCHORED_BENCHMARK_REFERENCE; CONDITIONAL_BASE'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def rows(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def key(r):
    return r['role'], r['arm'], r['policy'], int(r['frame'])


def pose(r, prefix):
    value = np.asarray([float(r[f'{prefix}{i}{j}']) if r[f'{prefix}{i}{j}'] != 'NA' else math.nan
                        for i in range(4) for j in range(4)]).reshape(4, 4)
    return value if np.isfinite(value).all() else None


def diff(a, b):
    if a is None or b is None:
        return None, None
    t = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    c = np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2, -1, 1)
    return t, math.degrees(math.acos(float(c)))


def write(path, data):
    assert data
    with path.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(data[0]), lineterminator='\n')
        writer.writeheader()
        for row in data:
            writer.writerow({k: 'NA' if v is None else v for k, v in row.items()})


def location(r):
    return {'frame': int(r['frame']), 'raw_ordinal': int(r['raw_ordinal']), 'stamp_ns': int(r['stamp_ns'])}


def first_run(rs, flags, length=3, after_frame=0):
    run = []
    for r, flag in zip(rs, flags):
        run = run + [r] if flag and int(r['frame']) > after_frame else []
        if len(run) >= length:
            return location(run[0])
    return None


def temporal(rs, error_records):
    e = [error_records[int(r['frame'])]['CAUSAL_OUTPUT'] for r in rs]
    bad = [a > .5 or b > 5 for a, b in e]
    low = [a <= .3 and b <= 3 for a, b in e]
    first_bad = next((location(r) for r, flag in zip(rs, bad) if flag), None)
    first_alarm = next((location(r) for r in rs if r['quality_event'] == '1'), None)
    first_bad_nominal = next((location(r) for r in rs if error_records[int(r['frame'])]['RAW_NDT'][0] is not None and (error_records[int(r['frame'])]['RAW_NDT'][0] > .5 or error_records[int(r['frame'])]['RAW_NDT'][1] > 5)), None)
    recovery = first_run(rs, low, after_frame=first_bad['frame']) if first_bad else None
    following = [(r, flag) for r, flag in zip(rs, bad)
                 if recovery and int(r['frame']) >= recovery['frame']]
    return {
        'bad_definition': 'causal output translation>0.5m OR rotation>5deg',
        'low_definition': 'causal output translation<=0.3m AND rotation<=3deg',
        'first_bad_output': first_bad, 'first_bad_nominal': first_bad_nominal, 'first_quality_alarm': first_alarm,
        'alarm_latency_from_first_bad_frames': first_alarm['frame'] - first_bad['frame'] if first_alarm and first_bad else None,
        'first_three_consecutive_low_error_run': first_run(rs, low),
        'first_three_low_after_bad': recovery,
        'recovery_latency_from_first_bad_frames': recovery['frame'] - first_bad['frame'] if recovery else None,
        'bad_output_count': sum(bad), 'longest_consecutive_bad_output': longest_run(bad),
        'post_recovery_bad_count': sum(flag for _, flag in following) if recovery else None,
        'post_recovery_longest_bad_run': longest_run(flag for _, flag in following) if recovery else None,
        'post_recovery_evaluated_frames': len(following) if recovery else None,
        'no_bad_onset_recovery_is_not_applicable': first_bad is None}


def main():
    # These output checks and counts precede ANY access to reference pose values.
    seal_path = HERE / 'NATIVE_OUTPUT_FREEZE.json'
    seal = json.loads(seal_path.read_text())
    assert seal['native_output_frozen_before_reference_evaluation'] is True
    assert seal['nominal_full_aligns'] == 480
    assert 0 <= seal['extra_full_aligns'] <= 48
    assert seal['total_full_aligns'] == 480 + seal['extra_full_aligns'] <= 528
    sealed = seal['files_sha256']
    for name, digest in sealed.items():
        assert sha(HERE / name) == digest, f'Sealed native output changed: {name}'
    data = defaultdict(list)
    required = ('CAUSAL_RESULTS.csv', 'EVENT_AND_CANDIDATE_LOG.csv', 'ALIGN_CALLS.csv',
                'RUNTIME_AND_MEMORY.csv', 'TRAJECTORY_RECEIPT.csv')
    for role in ('WEAK', 'STRONG'):
        for name in required:
            relative = f'{role.lower()}/formal_run_01/{name}'
            assert relative in sealed, f'Unsealed required output: {relative}'
            batch = rows(HERE / relative)
            assert all(r['role'] == role for r in batch)
            data[name].extend(batch)
    native = data['CAUSAL_RESULTS.csv']
    candidates = data['EVENT_AND_CANDIDATE_LOG.csv']
    calls = data['ALIGN_CALLS.csv']
    timing = data['RUNTIME_AND_MEMORY.csv']
    receipts = data['TRAJECTORY_RECEIPT.csv']
    assert len(native) == 480 and len({key(r) for r in native}) == 480
    assert len(candidates) == len(calls) == seal['total_full_aligns']
    assert sum(int(r['extra_aligns']) for r in native) == seal['extra_full_aligns']
    assert sum(r['kind'] == 'NOMINAL_FULL' for r in calls) == 480
    assert sum(r['kind'] == 'EXTRA_FULL' for r in calls) == seal['extra_full_aligns']
    assert len(receipts) == 8
    candidate_keys = {(key(r), int(r['candidate_id'])) for r in candidates}
    assert len(candidate_keys) == len(candidates)
    assert candidate_keys == {(key(r), int(r['candidate_id'])) for r in calls}
    prereg = json.loads((R19 / 'WINDOW_PREREGISTRATION.json').read_text())
    for r in native:
        assert r['arm'] in ('C0', 'C3') and r['policy'] in POLICIES
        assert 1 <= int(r['frame']) <= 60 and 0 <= int(r['budget_after']) <= 12
        assert int(r['extra_aligns']) in (0, 2)
        assert r['policy'] != 'QUALITY_GATE_ONLY' or int(r['extra_aligns']) == 0
        frozen = prereg['windows'][r['role']]['frames'][int(r['frame'])]
        assert int(r['stamp_ns']) == int(frozen['stamp_ns'])
        assert pose(r, 'output') is not None
    historical = json.loads((HERE / 'HISTORICAL_DETECTION.json').read_text())
    # Reuse baseline reference errors verbatim; do not rerun baseline evaluation/NDT.
    baseline_native = [r for r in rows(R19 / 'NDT_RESULTS.csv') if r['arm'] in ('C0', 'C3')]
    baseline_errors = [r for r in rows(R19 / 'REFERENCE_ERROR.csv') if r['arm'] in ('C0', 'C3')]
    assert len(baseline_native) == 240 and len(baseline_errors) == 720
    assert sha(GT) == seal['reference_file_sha256']
    wanted = {int(r['stamp_ns']) for r in native}
    assert len(wanted) == 120
    gt_values = {}
    with GT.open(newline='') as stream:
        reader = csv.reader(stream)
        next(reader)
        for line in reader:
            ns = int(line[0]) * 1_000_000_000 + int(line[1])
            if ns in wanted:
                gt_values[ns] = np.asarray([float(x) for x in line[2:]])
    assert set(gt_values) == wanted
    transforms = {}
    for role in ('WEAK', 'STRONG'):
        transforms[role] = np.asarray(json.loads((R19 / role.lower() / 'ANCHOR_INITIALIZATION.json').read_text())['T_B_L'])
    gt = {}
    for r in native:
        ns = int(r['stamp_ns'])
        v = gt_values[ns]
        assert v.shape == (7,) and np.isfinite(v).all() and abs(np.linalg.norm(v[3:]) - 1) < 2e-5
        mb = np.eye(4)
        mb[:3, :3] = Rotation.from_quat(v[3:]).as_matrix()
        mb[:3, 3] = v[:3]
        gt[r['role'], ns] = mb @ transforms[r['role']]

    error_rows, candidate_error_rows = [], []
    error_by_key = defaultdict(dict)
    for r in native:
        for prefix, estimate in (('tp', 'PREDICTION'), ('raw', 'RAW_NDT'),
                                 ('selected', 'SELECTED_OBSERVATION'), ('output', 'CAUSAL_OUTPUT')):
            a, b = diff(pose(r, prefix), gt[r['role'], int(r['stamp_ns'])])
            if estimate == 'SELECTED_OBSERVATION':
                assert (a is not None) == (r['accepted_observation'] == '1')
            error_by_key[key(r)][estimate] = (a, b)
            error_rows.append({**{k: r[k] for k in ('role', 'arm', 'policy', 'frame', 'raw_ordinal', 'stamp_ns')},
                               'estimate': estimate, 'translation_error_m': a, 'rotation_error_deg': b,
                               'accepted_observation': r['accepted_observation'], 'reference_basis': BASIS,
                               'source': 'R20_POST_SEAL_REFERENCE_EVALUATION'})
    for r in candidates:
        a, b = diff(pose(r, 'pose'), gt[r['role'], int(r['stamp_ns'])])
        candidate_error_rows.append({**{k: r[k] for k in ('role', 'arm', 'policy', 'frame', 'raw_ordinal', 'stamp_ns', 'candidate_id', 'seed_kind')},
                                    'translation_error_m': a, 'rotation_error_deg': b,
                                    'quality_gate_accepted': r['quality_gate_accepted'], 'chosen': r['chosen'],
                                    'reference_basis': BASIS})
    baseline_error_map = defaultdict(dict)
    for r in baseline_errors:
        baseline_error_map[r['role'], r['arm'], int(r['frame'])][r['estimate']] = (
            float(r['translation_error_m']), float(r['rotation_error_deg']))
    baseline_records = []
    for r in baseline_native:
        rr = dict(r, policy='BASELINE')
        rr['quality_event'] = str(int(r['quality_feedback'] != '1' or float(r['heldout_mean']) > .30))
        rr['accepted_observation'] = r['quality_feedback']
        baseline_records.append(rr)
        e = baseline_error_map[r['role'], r['arm'], int(r['frame'])]
        e['SELECTED_OBSERVATION'] = e['RAW_NDT'] if r['quality_feedback'] == '1' else (None, None)
        for estimate, (a, b) in e.items():
            error_by_key[key(rr)][estimate] = (a, b)
            error_rows.append({**{k: rr[k] for k in ('role', 'arm', 'policy', 'frame', 'raw_ordinal', 'stamp_ns')},
                               'estimate': estimate, 'translation_error_m': a, 'rotation_error_deg': b,
                               'accepted_observation': r['quality_feedback'], 'reference_basis': BASIS,
                               'source': 'R19_FROZEN_REFERENCE_ERROR_REUSE'})
    assert len(error_rows) == 2880
    summary = {}
    for role in ('WEAK', 'STRONG'):
        for arm in ('C0', 'C3'):
            for policy in ('BASELINE',) + POLICIES:
                rs = sorted((r for r in native + baseline_records
                             if (r['role'], r['arm'], r['policy']) == (role, arm, policy)),
                            key=lambda r: int(r['frame']))
                assert len(rs) == 60
                er = {int(r['frame']): error_by_key[key(r)] for r in rs}
                error_summary = {}
                for estimate in ('PREDICTION', 'RAW_NDT', 'SELECTED_OBSERVATION', 'CAUSAL_OUTPUT'):
                    vals = [er[i][estimate] for i in range(1, 61)]
                    error_summary[estimate] = {
                        'translation_m': errors([a for a, b in vals if a is not None]),
                        'rotation_deg': errors([b for a, b in vals if b is not None])}
                flags = [r['accepted_observation'] != '1' for r in rs]
                result = {'samples': 60, 'reference_relative_error': error_summary,
                          'quality_events': sum(r['quality_event'] == '1' for r in rs),
                          'accepted_map_observations': 60 - sum(flags),
                          'final_rejected_map_observations': sum(flags),
                          'prediction_fallbacks': sum(flags),
                          'longest_consecutive_prediction_fallback': longest_run(flags),
                          'temporal_error_and_detection': temporal(rs, er)}
                if policy == 'BASELINE':
                    old = historical['trajectories'][f'{role}/{arm}']
                    result.update({'new_ndt_calls': 0, 'historical_full_ndt_calls': 60,
                                   'quality_event_semantics': 'Retrospective fixed R20 rule on R19 output; no R19 online gate',
                                   'nominal_quality_distributions': old['quality_distributions'],
                                   'runtime': old['historical_runtime']})
                else:
                    cs = [c for c in candidates if (c['role'], c['arm'], c['policy']) == (role, arm, policy)]
                    rt = [r for r in timing if (r['role'], r['arm'], r['policy']) == (role, arm, policy)]
                    receipt = next(r for r in receipts if (r['role'], r['arm'], r['policy']) == (role, arm, policy))
                    extra = sum(int(r['extra_aligns']) for r in rs)
                    assert extra == int(receipt['extra_calls']) <= 12
                    assert sum(r['selected_extra'] == '1' for r in rs) == int(receipt['accepted_extra'])
                    episodes = []
                    for r in rs:
                        if r['selected_extra'] != '1':
                            continue
                        i = int(r['frame']); e = er[i]
                        later = [j for j in range(i+1, 61) if er[j]['CAUSAL_OUTPUT'][0] > .5 or er[j]['CAUSAL_OUTPUT'][1] > 5]
                        episodes.append({**location(r), 'selected_id': int(r['selected_id']),
                                         'selected_translation_error_m': e['SELECTED_OBSERVATION'][0],
                                         'selected_rotation_error_deg': e['SELECTED_OBSERVATION'][1],
                                         'translation_improvement_vs_nominal_m': e['RAW_NDT'][0] - e['SELECTED_OBSERVATION'][0] if e['RAW_NDT'][0] is not None else None,
                                         'rotation_improvement_vs_nominal_deg': e['RAW_NDT'][1] - e['SELECTED_OBSERVATION'][1] if e['RAW_NDT'][1] is not None else None,
                                         'first_subsequent_bad_output_frame': later[0] if later else None,
                                         'subsequent_bad_output_count': len(later),
                                         'remaining_evaluation_frames': 60-i})
                    result.update({
                        'nominal_full_calls': 60, 'extra_full_calls': extra, 'new_ndt_calls': 60+extra,
                        'nominal_rejections': sum(r['quality_event'] == '1' for r in rs),
                        'search_events': sum(int(r['extra_aligns']) > 0 for r in rs),
                        'budget_exhausted_events': sum(r['quality_event'] == '1' and int(r['budget_before']) == 12 for r in rs) if policy == 'QUALITY_TRIGGERED_MULTISTART' else 0,
                        'accepted_extra_candidates': sum(r['selected_extra'] == '1' for r in rs),
                        'physically_distinct_terminal_representatives_sum': sum(int(r['finite_distinct_terminals']) for r in rs),
                        'frames_with_multiple_distinct_terminals': sum(int(r['finite_distinct_terminals']) > 1 for r in rs),
                        'extra_candidates_distinct_from_nominal': sum(int(c['candidate_id']) > 0 and
                            c['translation_from_nominal_m'] != 'NA' and
                            (float(c['translation_from_nominal_m']) > .10 or float(c['rotation_from_nominal_deg']) > 1.) for c in cs),
                        'gate_legal_candidates_including_nominal': sum(c['quality_gate_accepted'] == '1' for c in cs),
                        'accepted_extra_reference_episodes': episodes,
                        'nominal_quality_distributions': {
                            name: distribution([r[field] for r in rs])
                            for name, field in (('native_score','nominal_score'), ('overlap_0p5','nominal_overlap'), ('heldout_mean','nominal_heldout_mean'))},
                        'all_candidate_quality_distributions': {f: distribution([r[f] for r in cs]) for f in ('native_score','overlap_0p5','heldout_mean')},
                        'selected_quality_distributions': {
                            name: distribution([r[field] for r in rs])
                            for name, field in (('native_score','selected_score'), ('overlap_0p5','selected_overlap'), ('heldout_mean','selected_heldout_mean'))},
                        'runtime': {
                            'frame_wall_ms': distribution([r['frame_wall_ms'] for r in rt]),
                            'normal_frame_wall_ms': distribution([r['frame_wall_ms'] for r in rt if r['event'] == '0']),
                            'event_frame_wall_ms': distribution([r['frame_wall_ms'] for r in rt if r['event'] == '1']),
                            'searched_event_frame_wall_ms': distribution([r['frame_wall_ms'] for r in rt if int(r['extra_aligns']) > 0]),
                            'nominal_align_ms': distribution([r['nominal_align_ms'] for r in rt]),
                            'extra_align_ms_per_frame': distribution([r['extra_align_ms'] for r in rt]),
                            'trajectory_wall_ms': float(receipt['trajectory_wall_ms']),
                            'peak_rss_KiB': int(receipt['peak_rss_KiB']),
                            'peak_rss_scope': 'Window-process cumulative high-water mark at trajectory completion'}})
                summary[f'{role}/{arm}/{policy}'] = result
    effects = {}
    for role in ('WEAK','STRONG'):
        base = summary[f'{role}/C0/BASELINE']['reference_relative_error']['CAUSAL_OUTPUT']
        for policy in POLICIES:
            cur = summary[f'{role}/C0/{policy}']['reference_relative_error']['CAUSAL_OUTPUT']
            dt = cur['translation_m']['rmse'] - base['translation_m']['rmse']
            dr = cur['rotation_deg']['rmse'] - base['rotation_deg']['rmse']
            effects[f'{role}/{policy}'] = {'translation_rmse_increase_m': dt,
                                          'rotation_rmse_increase_deg': dr,
                                          'material_worsening': dt > .03 or dr > .3}
    targets = {}
    for policy in ('BASELINE',)+POLICIES:
        e = summary[f'STRONG/C3/{policy}']['reference_relative_error']['CAUSAL_OUTPUT']
        targets[policy] = {'translation_rmse_m': e['translation_m']['rmse'],
                           'rotation_rmse_deg': e['rotation_deg']['rmse'],
                           'translation_target_le_0p35m': e['translation_m']['rmse'] <= .35,
                           'rotation_target_le_3deg': e['rotation_deg']['rmse'] <= 3}
    result = {'task': 'P62-R20-QUALITY-GATED-NDT-RECOVERY-BASELINE',
              'reference_type': BASIS, 'reference_samples': 120,
              'native_output_freeze_sha256': sha(seal_path),
              'new_ndt_calls': seal['total_full_aligns'],
              'new_nominal_full_calls': 480, 'new_extra_full_calls': seal['extra_full_aligns'],
              'baseline_recomputed_calls': 0, 'historical_baseline_calls_reused': 240,
              'development_set_only': True, 'trajectories': summary,
              'c0_side_effects': effects, 'strong_c3_engineering_targets': targets,
              'historical_detection': historical['all_480_rows'],
              'limitations': ['Threshold chosen using R19 development outputs',
                              'Full-source optimization includes the inherited odd-sector heldout points; quality signal is not independent validation',
                              'Map-anchored benchmark reference and conditional Base interpretation',
                              'Accepted extra candidate is not automatically reference improvement or independent correct recovery'],
              'peak_rss_KiB': max(int(r['peak_rss_KiB']) for r in timing)}
    write(HERE/'REFERENCE_ERROR.csv',error_rows)
    write(HERE/'REFERENCE_CANDIDATE_ERROR.csv',candidate_error_rows)
    for name in required:
        write(HERE/name,data[name])
    with (HERE/'EVALUATION_SUMMARY.json').open('x') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
        stream.write('\n')
    print(json.dumps({'ndt_calls':result['new_ndt_calls'], 'strong_c3':targets,
                      'c0_side_effects':effects},indent=2))


if __name__ == '__main__':
    main()
