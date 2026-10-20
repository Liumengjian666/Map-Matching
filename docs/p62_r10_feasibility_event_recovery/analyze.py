#!/usr/bin/env python3
"""Offline audit of this R10 run only. Never invokes an optimizer or old audit."""
import argparse
import csv
import hashlib
import itertools
import json
import math
import shutil
import subprocess
import unittest
from collections import Counter
from pathlib import Path

import numpy as np

DOC = Path(__file__).resolve().parent
REPO = DOC.parents[1]
RUN = Path('/home/jian/livox_ws/p62_r10_feasibility_runs')


def identity(path):
    path = Path(path)
    h = hashlib.sha256()
    with path.open('rb') as f:
        for data in iter(lambda: f.read(8 * 1024**2), b''):
            h.update(data)
    return dict(path=str(path), bytes=path.stat().st_size, sha256=h.hexdigest())


def save(name, value):
    (DOC / name).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def rows(name):
    with (DOC / 'raw' / name).open() as f:
        return list(csv.DictReader(f))


def f(row, key):
    return float(row[key]) if row[key] != 'NA' else math.nan


def pose(row, prefix):
    return np.array([f(row, prefix + str(a) + str(b)) for a in range(4) for b in range(4)]).reshape(4, 4)


def rigid(t):
    r = t[:3, :3]
    return (np.isfinite(t).all() and np.linalg.norm(t[3] - [0, 0, 0, 1]) < 1e-6
            and np.linalg.norm(r.T @ r - np.eye(3)) < 1e-4 and abs(np.linalg.det(r) - 1) < 1e-4)


def distance(a, b):
    # AngleAxis uses a matrix-to-quaternion construction. trace/acos falsely
    # interprets tiny float-matrix nonorthogonality as angle, especially near 0.
    r = a[:3, :3].T @ b[:3, :3]
    q = np.empty(4)
    if np.trace(r) > 0:
        s = math.sqrt(float(np.trace(r)) + 1)
        q[3] = .5 * s
        q[:3] = np.array([r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1]]) * (.5/s)
    else:
        i = int(np.argmax(np.diag(r))); j=(i+1)%3; k=(j+1)%3
        s = math.sqrt(float(r[i,i]-r[j,j]-r[k,k])+1)
        q[i]=.5*s; q[3]=(r[k,j]-r[j,k])*(.5/s)
        q[j]=(r[j,i]+r[i,j])*(.5/s); q[k]=(r[k,i]+r[i,k])*(.5/s)
    angle=2*math.atan2(float(np.linalg.norm(q[:3])), abs(float(q[3])))
    return float(np.linalg.norm(a[:3, 3] - b[:3, 3])), math.degrees(angle)


def lp_vertex_min(d):
    """Independent LP vertex enumeration, not the C++ sort-and-fill algorithm."""
    n = len(d)
    assert 4 <= n <= 8 and all(math.isfinite(x) for x in d)
    cap = 2 / n
    full = n // 2
    remainder = 1 - full * cap
    values = []
    for saturated in itertools.combinations(range(n), full):
        base = cap * sum(d[i] for i in saturated)
        if n % 2:
            values.extend(base + remainder * d[i] for i in range(n) if i not in saturated)
        else:
            values.append(base)
    return min(values)


def stats(values):
    a = np.array(values, dtype=float)
    assert len(a) and np.isfinite(a).all()
    return dict(n=len(a), mean=float(a.mean()), p95=float(np.quantile(a, .95, method='linear')), max=float(a.max()))


def freeze():
    source = sorted((RUN / 'formal').iterdir()) + [RUN / name for name in
             ['RUN_STARTED.json', 'run_freeze.json', 'run_completion.json', 'run.log']]
    build = Path('/home/jian/livox_ws/p62_r10_feasibility_build')
    source += [build / name for name in ['r10_bound_build.log', 'R10_BOUND_TEST_EXECUTION.json', 'R10_BOUND_BUILD_STARTED.json']]
    records = [identity(p) for p in source]
    dest = DOC / 'raw'
    manifest = DOC / 'OUTPUT_FREEZE.json'
    if manifest.exists():
        assert json.loads(manifest.read_text())['originals'] == records, 'R10 output changed'
    else:
        dest.mkdir(exist_ok=False)
        for p in source:
            shutil.copy2(p, dest / p.name)
        save('OUTPUT_FREEZE.json', dict(originals=records, archived=[identity(dest / p.name) for p in source]))
    for record in json.loads(manifest.read_text())['archived']:
        assert identity(record['path']) == record


def analyze():
    freeze()
    frames, candidates = rows('FRAME_RESULTS.csv'), rows('CANDIDATE_POOL.csv')
    cover, observations = rows('COVERAGE_COMPARISON.csv'), rows('FINAL_OBSERVATIONS.csv')
    assert len(frames) == len(cover) == len(observations) == 100
    assert [int(r['raw_index']) for r in frames] == list(range(10641, 10741))
    assert [int(r['scan_index']) for r in frames] == list(range(1, 101))
    input_manifest = json.loads((DOC.parent / 'p62_r7_motion_selected_shadow/REAL_INPUT_MANIFEST.json').read_text())
    with open(input_manifest['scans_csv']['path']) as stream:
        scans = list(csv.DictReader(stream))
    for actual, original in zip(cover, scans):
        assert actual['source_fnv'] == original['expected_processed_fnv']
        assert actual['source_points'] == original['expected_processed_points']
        for field in ['raw_index', 'seq', 'header_ns']:
            assert actual[field] == original[field]
    losses = {}
    for block in rows('HELDOUT_BLOCK_LOSSES.csv'):
        key = (int(block['scan_index']), int(block['candidate_id']))
        losses.setdefault(key, []).append(block)
    cand = {(int(r['scan_index']), int(r['candidate_id'])): r for r in candidates}
    assert len(cand) == len(candidates) == 202
    assert Counter(r['scale'] for r in candidates) == {'CONTROL_FULL': 100, 'NOMINAL': 100, 'SMALL': 2}
    selectors = {(int(r['scan_index']), r['selector']): r for r in rows('SELECTOR_COMPARISON.csv')}
    assert len(selectors) == 300
    computed_losses, admission = {}, {}
    # Recompute all actual candidate admission from preserved matrices and block evidence.
    for key, c in cand.items():
        scan, _ = key
        evidence = sorted(losses[key], key=lambda r: int(r['block_id']))
        assert len(evidence) == 8
        active = [r for r in evidence if int(r['points']) >= 10]
        assert all((r['active'] == '1') == (int(r['points']) >= 10) for r in evidence)
        assert all(r['loss'] == 'NA' for r in evidence if r['active'] == '0')
        values = [f(r, 'loss') for r in active]
        assert len(values) == int(cover[scan-1]['B'])
        computed_losses[key] = values
        pred = pose(observations[scan-1], 'common_prediction_')
        final = pose(c, 'final_')
        assert rigid(pred) and rigid(final)
        t, r = distance(final, pred)
        assert abs(t - f(c, 'translation_from_prediction_m')) < 1e-9
        assert abs(r - f(c, 'rotation_from_prediction_deg')) < 1e-8
        valid_evidence = (len(values) >= 4 and int(cover[scan-1]['training_points']) >= 300
                          and all(math.isfinite(v) and 0 <= v <= 1 for v in values))
        legal = (c['finite'] == '1' and c['converged'] == '1'
                 and math.isfinite(f(c, 'native_mean_score')) and math.isfinite(f(c, 'fitness'))
                 and t <= .8 and r <= 5 and valid_evidence and f(c, 'active_overlap_0p5m') >= .5)
        admission[key] = legal
        assert int(c['formal_eligible']) == legal
        assert abs(float(np.mean(values)) - f(c, 'active_mean_loss')) < 1e-12
    pool_by_scan = {}
    for frame in frames:
        scan = int(frame['scan_index'])
        nominal = cand[scan, 0]
        values = computed_losses[scan, 0]
        old = nominal['converged'] != '1' or int(nominal['iterations']) >= 40 or np.median(values) > .25
        assert frame['old_trigger_defined'] == '1' and int(frame['old_trigger']) == old
        event = old or not admission[scan, 0]
        assert int(frame['new_trigger']) == event and int(frame['new_only_trigger']) == (event and not old)
        if not event:
            assert int(frame['extra_calls']) == 0 and frame['direction_status'] == 'NOT_REQUESTED'
        cs = sorted([c for (i, j), c in cand.items() if i == scan and j >= 0], key=lambda r: int(r['candidate_id']))
        dedup = []
        for c in cs:
            duplicate = -1
            for previous in dedup:
                t, r = distance(pose(c, 'final_'), pose(previous, 'final_'))
                if t <= .1 and r <= 1:
                    duplicate = int(previous['candidate_id'])
                    break
            assert int(c['duplicate_of']) == duplicate
            if duplicate < 0:
                dedup.append(c)
        ids = [int(c['candidate_id']) for c in dedup]
        pool_by_scan[scan] = ids
        assert int(frame['pool_size']) == len(ids)
        assert int(frame['distinct_alternatives']) == len(ids) - 1
        assert int(frame['eligible_alternatives']) == sum(admission[scan, j] for j in ids if j > 0)
        eligible = [j for j in ids if admission[scan, j]]
        expected_native = max(eligible, key=lambda j: f(cand[scan, j], 'native_mean_score')) if eligible else -1
        expected_mean = min(eligible, key=lambda j: np.mean(computed_losses[scan, j])) if eligible else -1
        winners = [j for j in eligible if j > 0 and len(ids) >= 2 and all(
            lp_vertex_min([b-a for a, b in zip(computed_losses[scan, j], computed_losses[scan, k])]) > .02
            for k in ids if j != k)]
        assert len(winners) <= 1
        expected_robust = winners[0] if winners else -1
        for method, expected in [('NATIVE_SCORE', expected_native), ('HELDOUT_MEAN', expected_mean), ('COVERAGE_ROBUST', expected_robust)]:
            selector = selectors[scan, method]
            assert selector['pool_ids'] == '|'.join(map(str, ids)) and int(selector['selected_id']) == expected
            if len(ids) == 1:
                assert selector['min_pairwise_advantage'] == 'NA'
        selected = expected_robust if expected_robust >= 0 else 0 if admission[scan, 0] else -1
        assert int(frame['final_selected']) == selected
        assert int(frame['observation_valid']) == (selected >= 0)
        assert int(frame['recovery_success']) == (selected > 0)
        assert int(frame['prediction_fallback']) == (selected < 0)
        final = observations[scan-1]
        assert final['observation_valid'] == frame['observation_valid']
        if selected >= 0:
            assert np.array_equal(pose(final, 'accepted_'), pose(cand[scan, selected], 'final_'))
        else:
            assert frame['final_status'] == 'NO_ACCEPTABLE_MEASUREMENT'
            assert frame['fallback_status'] == 'SHADOW_PREDICTION_ONLY_NOT_EXECUTED'
            assert all(final['accepted_' + str(a) + str(b)] == 'NA' for a in range(4) for b in range(4))
        assert int(frame['extra_calls']) <= 2
    for pair in rows('PAIRWISE_EVIDENCE.csv'):
        i, j, k = map(int, [pair['scan_index'], pair['candidate_j'], pair['candidate_k']])
        d = lp_vertex_min([b-a for a, b in zip(computed_losses[i, j], computed_losses[i, k])])
        assert abs(d - f(pair, 'D_j_k')) < 1e-12
    calls = rows('NATIVE_ALIGN_CALLS.csv')
    assert Counter(r['status'] for r in calls) == {'START': 202, 'RETURNED': 202}
    for name in ['RAW_OPTIMIZER_OUTPUTS.csv', 'RAW_NATIVE_OUTPUTS.csv']:
        assert len(rows(name)) == 202
    assert not rows('VALIDATION_FAILURES.csv')
    assert sum(int(r['extra_calls']) for r in frames) == 2 <= 20
    assert [int(r['budget_used']) for r in frames] == list(np.cumsum([int(r['extra_calls']) for r in frames]))
    log = (DOC / 'raw/run.log').read_text()
    assert log.count('SMOKE20_PASS') == 1 and 'COMPLETE frames=100 native_aligns=202' in log
    completion = json.loads((DOC / 'raw/run_completion.json').read_text())
    assert completion['returncode'] == 0 and completion['after_gate'] == 'ALL_R10_INPUTS_SOURCE_BINARY_PRODUCTION_UNCHANGED'
    times = rows('RUNTIME_AND_MEMORY.csv')
    timing = {key: stats([f(r, key) for r in times]) for key in [
        'preprocess_ms', 'full_align_ms', 'nominal_align_ms', 'extra_align_ms', 'G_ms',
        'validation_ms', 'selectors_and_record_ms', 'frame_compute_ms']}
    for key, label in [('CONTROL_FULL', 'full_native_per_call_ms'), ('NOMINAL', 'split_native_per_call_ms'), ('SMALL', 'extra_native_per_call_ms')]:
        timing[label] = stats([f(c, 'align_ms') for c in candidates if c['scale'] == key])
    selected_events = [r for r in frames if r['new_trigger'] == '1']
    result = dict(task='P62-R10-FEASIBILITY-COMPLETE-EVENT-RECOVERY',
        START_SHA='340a42190c9101f98cd483849537ced23a2a20a0',
        CODE_SHA=json.loads((DOC / 'raw/run_freeze.json').read_text())['CODE_SHA'],
        analysis_CODE_SHA=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip(),
        frames=100, raw_range=[10641,10740], smoke20='PASS', baseline_full_aligns=100, baseline_split_aligns=100,
        extra_aligns=2, total_aligns=202, extra_budget=20,
        old_predicate_trigger_count=sum(r['old_trigger'] == '1' for r in frames),
        old_predicate_definition='R9 predicate evaluated on this new R10 nominal output; R9 was NOT rerun or re-audited',
        detected_events=len(selected_events), new_only_events=sum(r['new_only_trigger']=='1' for r in frames),
        event_local_indices=[int(r['scan_index']) for r in selected_events],
        event_raw_indices=[int(r['raw_index']) for r in selected_events],
        permitted_events=sum(r['recovery_permitted']=='1' for r in frames),
        G_evaluations=sum(r['direction_status']!='NOT_REQUESTED' for r in frames),
        weak_direction_frames=sum(r['direction_status']=='WEAK_DIRECTION' for r in frames),
        seeds_generated=sum(int(r['seeds_generated']) for r in frames),
        distinct_alternatives=sum(int(r['distinct_alternatives']) for r in frames),
        eligible_distinct_alternatives=sum(int(r['eligible_alternatives']) for r in frames),
        accepted_alternatives=sum(r['recovery_success']=='1' for r in frames),
        control_full_eligible=sum(admission[i,-1] for i in range(1,101)),
        final_status_counts=dict(Counter(r['final_status'] for r in frames)),
        selector_outcomes={s:dict(Counter(selectors[i,s]['outcome'] for i in range(1,101)))
                           for s in ['NATIVE_SCORE','HELDOUT_MEAN','COVERAGE_ROBUST']},
        all_native_converged=all(r['converged']=='1' for r in candidates),
        invalid_native_results=sum(r['finite']!='1' for r in candidates),
        V2_evaluable=sum(r['V2_evaluable']=='1' for r in cover),
        active_block_counts=dict(Counter(r['B'] for r in cover)),
        timing_ms=timing, percentile_definition='numpy quantile linear (type 7); descriptive only',
        native_process_wall_s=completion['native_wall_seconds'],
        map_runtime=rows('MAP_RUNTIME.csv')[0], peak_process_RSS_MiB=max(f(r,'peak_ru_maxrss_KiB') for r in times)/1024,
        tests='ACTUAL_COLD_CPP_TESTS_AND_INDEPENDENT_OFFLINE_RECORD_CHECKS_PASS',
        engineering_result='EVENT_ADMISSION_STATE_MACHINE_ESTABLISHED',
        recovery_result='NO_ACCEPTABLE_RECOVERY_OBSERVED',
        selector_discrimination='NOT_DEMONSTRATED_ON_LEGAL_ALTERNATIVES',
        boundaries=dict(GT_reads=0,ROS_localization_nodes=0,rosbag_play=0,EKF_runs=0,feedback=0,
                        forced_diagnostic_aligns=0,LARGE_scale_aligns=0,external_CLI_runs=0,production_code_changes=0,
                        new_trajectory_estimates=0),
        asset_after_gate=completion['after_gate'],
        limitations=['No independent localization accuracy evaluation',
            'Physical distinctness is not evidence of different minima or correct recovery',
            'Only retained alternative differs from nominal by 1.0000271097 degrees, barely above the 1-degree dedup boundary; this is boundary-sensitive, not a robust branch claim',
            'Comparator disagreement on singleton nominal pools is an abstention-policy difference, not alternative discrimination',
            'Prediction fallback is Shadow bookkeeping only, never an executed EKF action',
            'frame_compute_ms excludes Source precache and map construction; per-stage component timers omit some outer I/O/fitness overhead'],
        unique_NEXT='Seek separate approval for bounded candidate-generation changes targeting admission-failed events; do not relax current acceptance thresholds or rerun now')
    save('results.json', result)
    selected_fields = ['scan_index','raw_index','reference_ns','event_reasons','direction_status','weak_count','extra_calls','final_status','fallback_status']
    with (DOC / 'LOCAL_60_64.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=selected_fields+['nominal_rotation_deg','nominal_overlap_0p5m'])
        writer.writeheader()
        for row in frames[59:64]:
            c = cand[int(row['scan_index']),0]
            writer.writerow({**{k:row[k] for k in selected_fields},
                             'nominal_rotation_deg':c['rotation_from_prediction_deg'],
                             'nominal_overlap_0p5m':c['active_overlap_0p5m']})
    print(json.dumps({k: result[k] for k in ['detected_events','extra_aligns','distinct_alternatives',
          'eligible_distinct_alternatives','final_status_counts','peak_process_RSS_MiB']}, indent=2))


class IndependentMathTests(unittest.TestCase):
    def test_lp(self):
        for n in range(4,9):
            d = np.linspace(-.1,.3,n).tolist()
            cap = 2/n
            expected = cap*sum(sorted(d)[:n//2]) + (1-cap*(n//2))*sorted(d)[n//2]
            self.assertAlmostEqual(lp_vertex_min(d), expected)
            self.assertAlmostEqual(lp_vertex_min([.02]*n), .02)
            self.assertFalse(lp_vertex_min([.02]*n) > .02)

    def test_pose(self):
        self.assertTrue(rigid(np.eye(4)))
        self.assertEqual(distance(np.eye(4), np.eye(4)), (0,0))
        bad=np.eye(4);bad[0,0]=math.nan
        self.assertFalse(rigid(bad))

    def test_float_matrix_angle(self):
        # Symmetric rounding error must not invent a rotation; trace/acos does.
        a=np.eye(4);a[0,0]-=1e-7
        self.assertEqual(distance(np.eye(4),a)[1],0)
        for angle in [1e-8,.03,2.9,179.999,180.]:
            c,s=math.cos(math.radians(angle)),math.sin(math.radians(angle))
            a=np.eye(4);a[:2,:2]=[[c,-s],[s,c]]
            self.assertAlmostEqual(distance(np.eye(4),a)[1],angle,places=10)


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--freeze-only',action='store_true')
    args=parser.parse_args()
    if args.self_test:
        unittest.main(argv=['analyze.py'])
    elif args.freeze_only:
        freeze()
    else:
        analyze()
