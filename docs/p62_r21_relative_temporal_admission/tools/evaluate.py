#!/usr/bin/env python3
"""R21 post-seal evaluation. The native runner never imports this module."""
import copy
import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parents[1]
R20 = HERE.parent / 'p62_r20_quality_gated_recovery'
R19 = HERE.parent / 'p62_r19_native_ndt_basin_stress_test'
sys.path.insert(0, str(R20 / 'tools'))
from baseline_stats import distribution, errors, longest_run

GT = Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment/ground_truth/registered_poses.csv')
POLICIES = ('SINGLE_FRAME_RELATIVE_ADMISSION', 'TWO_FRAME_SHADOW_CONFIRMATION')
HISTORICAL = ('BASELINE', 'QUALITY_GATE_ONLY', 'QUALITY_TRIGGERED_MULTISTART')
ESTIMATES = (('tp','PREDICTION'), ('raw','RAW_NDT'), ('selected','SELECTED_OBSERVATION'), ('output','CAUSAL_OUTPUT'))
BASIS = 'MAP_ANCHORED_BENCHMARK_REFERENCE; CONDITIONAL_BASE'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def rows(path):
    with Path(path).open(newline='') as f:
        return list(csv.DictReader(f))


def key(r):
    return r['role'], r['arm'], r['policy'], int(r['frame'])


def pose(r, prefix):
    m = np.asarray([float(r[f'{prefix}{i}{j}']) if r[f'{prefix}{i}{j}'] != 'NA' else math.nan
                    for i in range(4) for j in range(4)], dtype=float).reshape(4, 4)
    return m if np.isfinite(m).all() else None


def difference(a, b):
    if a is None or b is None:
        return None, None
    t = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    x = float(np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2, -1, 1))
    return t, math.degrees(math.acos(x))


def write(path, data):
    assert data
    with path.open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(data[0]), lineterminator='\n')
        writer.writeheader()
        for r in data:
            writer.writerow({k: 'NA' if v is None else v for k, v in r.items()})


def location(r):
    return {k: int(r[k]) for k in ('frame', 'raw_ordinal', 'stamp_ns')}


def first_run(rs, flags, after=0, length=3):
    active = []
    for r, flag in zip(rs, flags):
        active = active + [r] if flag and int(r['frame']) > after else []
        if len(active) >= length:
            return location(active[0])
    return None


def temporal(rs, er):
    output = [er[int(r['frame'])]['CAUSAL_OUTPUT'] for r in rs]
    bad = [a > .5 or b > 5 for a, b in output]
    low = [a <= .3 and b <= 3 for a, b in output]
    bad_start = next((location(r) for r, b in zip(rs, bad) if b), None)
    alarm = next((location(r) for r in rs if r['quality_event'] == '1'), None)
    nominal_bad = next((location(r) for r in rs if er[int(r['frame'])]['RAW_NDT'][0] is not None and
                        (er[int(r['frame'])]['RAW_NDT'][0] > .5 or er[int(r['frame'])]['RAW_NDT'][1] > 5)), None)
    recovery = first_run(rs, low, after=bad_start['frame']) if bad_start else None
    following = [(r, b) for r, b in zip(rs, bad) if recovery and int(r['frame']) >= recovery['frame']]
    return {
        'bad_definition': 'actual causal output translation>0.5m OR rotation>5deg',
        'low_definition': 'actual causal output translation<=0.3m AND rotation<=3deg',
        'first_bad_output': bad_start, 'first_bad_nominal': nominal_bad, 'first_quality_alarm': alarm,
        'alarm_latency_from_first_bad_frames': alarm['frame'] - bad_start['frame'] if alarm and bad_start else None,
        'first_three_consecutive_low_error_run': first_run(rs, low),
        'first_three_low_after_bad': recovery,
        'recovery_latency_from_first_bad_frames': recovery['frame'] - bad_start['frame'] if recovery else None,
        'bad_output_count': sum(bad), 'longest_consecutive_bad_output': longest_run(bad),
        'post_recovery_bad_count': sum(b for _, b in following) if recovery else None,
        'post_recovery_longest_bad_run': longest_run(b for _, b in following) if recovery else None,
        'post_recovery_evaluated_frames': len(following) if recovery else None,
        'no_bad_onset_recovery_is_not_applicable': bad_start is None}


def count(rs, field):
    return sum(r[field] == '1' for r in rs)


def episode(r, er):
    frame = int(r['frame'])
    e = er[frame]
    future_bad = [i for i in range(frame+1, 61)
                  if er[i]['CAUSAL_OUTPUT'][0] > .5 or er[i]['CAUSAL_OUTPUT'][1] > 5]
    return {**location(r), 'selected_id': int(r['selected_id']), 'admission_reason': r['admission_reason'],
            'relative_only': r['relative_only_admitted'] == '1',
            'confirmation_success': r['confirmation_success'] == '1',
            'selected_translation_error_m': e['SELECTED_OBSERVATION'][0],
            'selected_rotation_error_deg': e['SELECTED_OBSERVATION'][1],
            'translation_improvement_vs_nominal_m': e['RAW_NDT'][0] - e['SELECTED_OBSERVATION'][0] if e['RAW_NDT'][0] is not None else None,
            'rotation_improvement_vs_nominal_deg': e['RAW_NDT'][1] - e['SELECTED_OBSERVATION'][1] if e['RAW_NDT'][1] is not None else None,
            'first_subsequent_bad_output_frame': future_bad[0] if future_bad else None,
            'subsequent_bad_output_count': len(future_bad), 'remaining_evaluation_frames': 60-frame}


def main():
    # No reference-value access before validating the complete native seal.
    seal_path = HERE / 'NATIVE_OUTPUT_FREEZE.json'
    seal = json.loads(seal_path.read_text())
    assert seal['native_output_frozen_before_reference_evaluation'] is True
    assert seal['nominal_full_aligns'] == 480
    assert 0 <= seal['extra_full_aligns'] <= 96
    assert seal['total_full_aligns'] == 480 + seal['extra_full_aligns'] <= 576
    for name, digest in seal['files_sha256'].items():
        assert sha(HERE / name) == digest, name
    names = ('CAUSAL_RESULTS.csv', 'RELATIVE_ADMISSION_LOG.csv', 'SHADOW_CONFIRMATION_LOG.csv',
             'ALIGN_CALLS.csv', 'RUNTIME_AND_MEMORY.csv', 'TRAJECTORY_RECEIPT.csv')
    data = defaultdict(list)
    for role in ('WEAK', 'STRONG'):
        for name in names:
            relative = f'{role.lower()}/formal_run_01/{name}'
            assert relative in seal['files_sha256'], relative
            batch = rows(HERE / relative)
            assert all(r['role'] == role for r in batch)
            data[name].extend(batch)
    native = data['CAUSAL_RESULTS.csv']; candidate = data['RELATIVE_ADMISSION_LOG.csv']
    shadow = data['SHADOW_CONFIRMATION_LOG.csv']; calls = data['ALIGN_CALLS.csv']
    runtime = data['RUNTIME_AND_MEMORY.csv']; receipts = data['TRAJECTORY_RECEIPT.csv']
    assert len(native) == len(shadow) == len({key(r) for r in native}) == 480
    assert len(calls) == len(candidate) == seal['total_full_aligns']
    assert sum(c['kind'] == 'NOMINAL_FULL' for c in calls) == 480
    assert sum(c['kind'] != 'NOMINAL_FULL' for c in calls) == seal['extra_full_aligns']
    assert sum(int(r['extra_aligns']) for r in native) == seal['extra_full_aligns']
    assert len(receipts) == 8
    ck = {(key(r), int(r['candidate_id'])) for r in candidate}
    assert len(ck) == len(candidate) and ck == {(key(r), int(r['candidate_id'])) for r in calls}
    shadow_map = {key(r): r for r in shadow}; assert len(shadow_map) == 480
    prereg = json.loads((R19 / 'WINDOW_PREREGISTRATION.json').read_text())
    for r in native:
        assert r['arm'] in ('C0','C3') and r['policy'] in POLICIES
        assert 1 <= int(r['frame']) <= 60 and 0 <= int(r['budget_after']) <= 12
        assert int(r['extra_aligns']) in (0,1,2)
        frozen = prereg['windows'][r['role']]['frames'][int(r['frame'])]
        assert int(r['stamp_ns']) == int(frozen['stamp_ns']) and int(r['raw_ordinal']) == int(frozen['raw_ordinal'])
        assert pose(r,'output') is not None
        s = shadow_map[key(r)]
        for field in ('pending_created','confirmation_attempted','confirmation_success','confirmation_failed','pending_in','pending_out'):
            assert s[field] == r[field]
        if r['policy'] == POLICIES[0]:
            assert all(r[f] == '0' for f in ('pending_created','confirmation_attempted','confirmation_success','confirmation_failed','pending_in','pending_out'))
        if r['accepted_observation'] == '0':
            assert s['formal_history_after_stamp_ns'] == s['formal_history_before_stamp_ns']
        else:
            assert s['formal_history_after_stamp_ns'] == r['stamp_ns']
        if r['confirmation_attempted'] == '1':
            assert int(s['pending_source_frame']) == int(r['frame'])-1 and int(s['pending_source_stamp_ns']) < int(r['stamp_ns'])
            assert int(r['extra_aligns']) == 1
        if r['nominal_gate_accepted'] == '1':
            assert int(r['extra_aligns']) == 0
    historical = json.loads((R20/'EVALUATION_SUMMARY.json').read_text())
    historical_error_rows = rows(R20/'REFERENCE_ERROR.csv')
    assert len(historical_error_rows) == 2880 and set(r['policy'] for r in historical_error_rows) == set(HISTORICAL)
    # R19/R20 baseline errors are copied verbatim, with provenance retained.
    assert sha(GT) == seal['reference_file_sha256']
    wanted = {int(r['stamp_ns']) for r in native}; assert len(wanted) == 120
    values = {}
    with GT.open(newline='') as stream:
        reader = csv.reader(stream); next(reader)
        for row in reader:
            ns = int(row[0])*1_000_000_000 + int(row[1])
            if ns in wanted:
                values[ns] = np.asarray([float(x) for x in row[2:]], dtype=float)
    assert set(values) == wanted
    transforms = {role: np.asarray(json.loads((R19/role.lower()/'ANCHOR_INITIALIZATION.json').read_text())['T_B_L'])
                  for role in ('WEAK','STRONG')}
    references = {}
    for r in native:
        ns = int(r['stamp_ns']); v = values[ns]
        assert v.shape == (7,) and np.isfinite(v).all() and abs(np.linalg.norm(v[3:])-1)<2e-5
        mb = np.eye(4); mb[:3,:3] = Rotation.from_quat(v[3:]).as_matrix(); mb[:3,3] = v[:3]
        references[r['role'],ns] = mb @ transforms[r['role']]
    error_rows = []
    error_by_key = defaultdict(dict)
    for r in native:
        for prefix, estimate in ESTIMATES:
            a,b = difference(pose(r,prefix), references[r['role'],int(r['stamp_ns'])])
            if estimate == 'SELECTED_OBSERVATION':
                assert (a is not None) == (r['accepted_observation']=='1')
            error_by_key[key(r)][estimate] = (a,b)
            error_rows.append({**{k:r[k] for k in ('role','arm','policy','frame','raw_ordinal','stamp_ns')},
                               'estimate':estimate,'translation_error_m':a,'rotation_error_deg':b,
                               'accepted_observation':r['accepted_observation'],'reference_basis':BASIS,
                               'source':'R21_POST_SEAL_REFERENCE_EVALUATION'})
    for old in historical_error_rows:
        r = dict(old)
        for f in ('translation_error_m','rotation_error_deg'):
            r[f] = None if r[f]=='NA' else float(r[f])
        error_rows.append(r)
    assert len(error_rows) == 4800
    candidate_errors = []
    for c in candidate:
        a,b = difference(pose(c,'pose'),references[c['role'],int(c['stamp_ns'])])
        candidate_errors.append({**{k:c[k] for k in ('role','arm','policy','frame','raw_ordinal','stamp_ns','candidate_id','seed_kind')},
                                 'translation_error_m':a,'rotation_error_deg':b,
                                 'basic_quality':c['basic_quality'],'absolute_gate_accepted':c['quality_gate_accepted'],
                                 'pareto_dominates':c['pareto_dominates'],'relative_only_candidate':c['relative_only_candidate'],
                                 'chosen':c['chosen'],'reference_basis':BASIS})
    summary = copy.deepcopy(historical['trajectories'])
    for k,v in summary.items():
        v['cost_provenance'] = 'Historical R19/R20 measurement; not an R21 run'
        v['r21_new_ndt_calls'] = 0
    for role in ('WEAK','STRONG'):
        for arm in ('C0','C3'):
            for policy in POLICIES:
                ident = (role,arm,policy)
                rs = sorted([r for r in native if key(r)[:3]==ident],key=lambda r:int(r['frame']))
                assert [int(r['frame']) for r in rs] == list(range(1,61))
                cs = [c for c in candidate if key(c)[:3]==ident]
                ss = [s for s in shadow if key(s)[:3]==ident]
                rt = [r for r in runtime if (r['role'],r['arm'],r['policy'])==ident]
                receipt = next(r for r in receipts if (r['role'],r['arm'],r['policy'])==ident)
                er = {int(r['frame']):error_by_key[key(r)] for r in rs}
                error_summary = {}
                for _,estimate in ESTIMATES:
                    vals = [er[i][estimate] for i in range(1,61)]
                    error_summary[estimate] = {'translation_m':errors([a for a,b in vals if a is not None]),
                                               'rotation_deg':errors([b for a,b in vals if b is not None])}
                fallbacks = [r['accepted_observation']=='0' for r in rs]
                extra = sum(int(r['extra_aligns']) for r in rs)
                statuses = Counter(s['status'] for s in ss)
                created=count(rs,'pending_created'); attempted=count(rs,'confirmation_attempted')
                successful=count(rs,'confirmation_success'); failed=count(rs,'confirmation_failed')
                assert extra == int(receipt['extra_calls']) <= 12
                for field,receipt_field in (('quality_event','quality_events'),('selected_extra','accepted_extra'),
                                             ('relative_only_admitted','relative_only_admissions'),('pending_created','pending_created'),
                                             ('confirmation_attempted','confirmation_attempts'),('confirmation_success','confirmation_success'),
                                             ('confirmation_failed','confirmation_failed')):
                    assert count(rs,field)==int(receipt[receipt_field])
                assert attempted == successful+failed
                cancelled_normal=statuses['CANCELLED_NOMINAL_ABSOLUTE_PASS']
                expired_budget=statuses['CONFIRMATION_BUDGET_EXHAUSTED']
                expired_window=statuses['PENDING_EXPIRED_WINDOW_END']
                assert created==attempted+cancelled_normal+expired_budget+expired_window
                attempts_rows=[s for s in ss if s['confirmation_attempted']=='1']
                result={
                    'samples':60,'reference_relative_error':error_summary,'quality_events':count(rs,'quality_event'),
                    'accepted_map_observations':60-sum(fallbacks),'final_rejected_map_observations':sum(fallbacks),
                    'prediction_fallbacks':sum(fallbacks),'longest_consecutive_prediction_fallback':longest_run(fallbacks),
                    'nominal_full_calls':60,'extra_full_calls':extra,'new_ndt_calls':60+extra,'r21_new_ndt_calls':60+extra,
                    'search_calls':sum(c['seed_kind'] in ('MAP_YAW_PLUS5','MAP_YAW_MINUS5') for c in cs),
                    'confirmation_calls':attempted,
                    'relative_dominance_candidate_count':count(cs,'pareto_dominates'),
                    'relative_only_candidate_count':count(cs,'relative_only_candidate'),
                    'relative_only_admissions':count(rs,'relative_only_admitted'),
                    'direct_relative_admissions':sum(r['admission_reason']=='RELATIVE_ONLY_ADMISSION' for r in rs),
                    'direct_map_admissions_without_confirmation':60-sum(fallbacks)-successful,
                    'accepted_extra_candidates':count(rs,'selected_extra'),
                    'pending_created':created,'confirmation_attempts':attempted,
                    'confirmation_success':successful,'confirmation_failed_after_real_align':failed,
                    'confirmation_absolute_success':count(ss,'confirmation_absolute'),
                    'confirmation_relative_success':count(ss,'confirmation_relative'),
                    'pending_cancelled_by_normal_nominal':cancelled_normal,'pending_expired_budget':expired_budget,
                    'pending_expired_window_end':expired_window,'shadow_status_counts':dict(statuses),
                    'confirmation_delay_frames':distribution([int(s['frame'])-int(s['pending_source_frame']) for s in attempts_rows]),
                    'confirmation_delay_ms':distribution([(int(s['stamp_ns'])-int(s['pending_source_stamp_ns']))/1e6 for s in attempts_rows]),
                    'admission_reason_counts':dict(Counter(r['admission_reason'] for r in rs)),
                    'relative_candidate_frames':[location(r) for r in rs if int(r['relative_candidate_count'])>0],
                    'recovery_admission_episodes':[episode(r,er) for r in rs if r['selected_extra']=='1'],
                    'temporal_error_and_detection':temporal(rs,er),
                    'nominal_quality_distributions':{name:distribution([r[field] for r in rs]) for name,field in
                        (('native_score','nominal_score'),('overlap_0p5','nominal_overlap'),('heldout_mean','nominal_heldout_mean'))},
                    'all_candidate_quality_distributions':{f:distribution([c[f] for c in cs]) for f in ('native_score','overlap_0p5','heldout_mean')},
                    'runtime':{
                        'frame_wall_ms':distribution([r['frame_wall_ms'] for r in rt]),
                        'normal_frame_wall_ms':distribution([r['frame_wall_ms'] for r in rt if r['event']=='0']),
                        'event_frame_wall_ms':distribution([r['frame_wall_ms'] for r in rt if r['event']=='1']),
                        'searched_or_confirmed_frame_wall_ms':distribution([r['frame_wall_ms'] for r in rt if int(r['extra_aligns'])>0]),
                        'confirmation_frame_wall_ms':distribution([r['frame_wall_ms'] for r in rt if float(r['confirmation_align_ms'])>0]),
                        'nominal_align_ms':distribution([r['nominal_align_ms'] for r in rt]),
                        'extra_align_ms_per_frame':distribution([r['extra_align_ms'] for r in rt]),
                        'confirmation_align_ms_per_frame':distribution([r['confirmation_align_ms'] for r in rt]),
                        'trajectory_wall_ms':float(receipt['trajectory_wall_ms']),'peak_rss_KiB':int(receipt['peak_rss_KiB']),
                        'peak_rss_scope':'Window-process cumulative high-water mark at trajectory completion'},
                    'cost_provenance':'New single R21 native experiment'}
                summary['/'.join(ident)]=result
    targets={};side_effects={};ed={}
    for policy in HISTORICAL+POLICIES:
        e=summary[f'STRONG/C3/{policy}']['reference_relative_error']['CAUSAL_OUTPUT']
        targets[policy]={'translation_rmse_m':e['translation_m']['rmse'],'rotation_rmse_deg':e['rotation_deg']['rmse'],
                         'translation_target_le_0p35m':e['translation_m']['rmse']<=.35,
                         'rotation_target_le_3deg':e['rotation_deg']['rmse']<=3,
                         'both_targets_met':e['translation_m']['rmse']<=.35 and e['rotation_deg']['rmse']<=3}
    for role in ('WEAK','STRONG'):
        for policy in POLICIES:
            e=summary[f'{role}/C0/{policy}']['reference_relative_error']['CAUSAL_OUTPUT']
            b=summary[f'{role}/C0/BASELINE']['reference_relative_error']['CAUSAL_OUTPUT']
            dt=e['translation_m']['rmse']-b['translation_m']['rmse'];dr=e['rotation_deg']['rmse']-b['rotation_deg']['rmse']
            side_effects[f'{role}/{policy}']={'translation_rmse_increase_m':dt,'rotation_rmse_increase_deg':dr,
                                            'material_worsening':dt>.03 or dr>.3}
        for arm in ('C0','C3'):
            dr=sorted([r for r in native if key(r)[:3]==(role,arm,POLICIES[0])],key=lambda r:int(r['frame']))
            er=sorted([r for r in native if key(r)[:3]==(role,arm,POLICIES[1])],key=lambda r:int(r['frame']))
            output_fields=[f'output{i}{j}' for i in range(4) for j in range(4)]
            exactly_equal=all(all(a[f]==b[f] for f in output_fields) for a,b in zip(dr,er))
            de=summary[f'{role}/{arm}/{POLICIES[0]}']['reference_relative_error']['CAUSAL_OUTPUT']
            ee=summary[f'{role}/{arm}/{POLICIES[1]}']['reference_relative_error']['CAUSAL_OUTPUT']
            ed[f'{role}/{arm}']={'all_output_matrices_exactly_equal':exactly_equal,
                                 'translation_rmse_E_minus_D_m':ee['translation_m']['rmse']-de['translation_m']['rmse'],
                                 'rotation_rmse_E_minus_D_deg':ee['rotation_deg']['rmse']-de['rotation_deg']['rmse'],
                                 'extra_calls_E_minus_D':summary[f'{role}/{arm}/{POLICIES[1]}']['extra_full_calls']-summary[f'{role}/{arm}/{POLICIES[0]}']['extra_full_calls']}
    result={'task':'P62-R21-RELATIVE-ADMISSION-AND-TEMPORAL-CONFIRMATION','reference_type':BASIS,
            'reference_samples':120,'development_set_only':True,'native_output_freeze_sha256':sha(seal_path),
            'new_nominal_full_calls':480,'new_extra_full_calls':seal['extra_full_aligns'],'new_ndt_calls':seal['total_full_aligns'],
            'baseline_recomputed_calls':0,'historical_reference_rows_reused':2880,'trajectories':summary,
            'strong_c3_engineering_targets':targets,'c0_side_effects':side_effects,'E_vs_D':ed,
            'peak_rss_KiB':max(int(r['peak_rss_KiB']) for r in runtime),
            'limitations':['Developed on already exposed R19/R20 windows and reference results',
                           'Full-source optimization includes Coverage odd-sector points; no independent held-out validation',
                           'Map-anchored benchmark reference and conditional Base axes',
                           'Relative admission and confirmation success are online rule outcomes, not independent correctness proof',
                           'Confirmation has a fixed one-frame lifespan and a one-pose causal shadow seed; no independent IMU motion input']}
    write(HERE/'REFERENCE_ERROR.csv',error_rows);write(HERE/'REFERENCE_CANDIDATE_ERROR.csv',candidate_errors)
    for name in names:
        write(HERE/name,data[name])
    with (HERE/'EVALUATION_SUMMARY.json').open('x') as f:
        json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'new_ndt_calls':result['new_ndt_calls'],'strong_c3':targets,'E_vs_D':ed,'c0_side_effects':side_effects},indent=2))


if __name__=='__main__':
    main()
