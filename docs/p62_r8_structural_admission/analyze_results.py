#!/usr/bin/env python3
"""Read only the one completed run; never instantiate or call an optimizer."""
import csv
import json
import shutil
from collections import Counter
from pathlib import Path

import numpy as np
from run_preflight import DOC, RUN, identity, json_new


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def stats(values):
    a = np.asarray(values, dtype=float)
    assert len(a) and np.isfinite(a).all()
    return dict(count=len(a), mean=float(a.mean()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95, method='linear')), max=float(a.max()), sum=float(a.sum()))


def pose(row, prefix):
    p = np.array([float(row[prefix+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4, 4)
    assert np.isfinite(p).all() and np.linalg.norm(p[3]-[0, 0, 0, 1]) < 1e-6
    assert np.linalg.norm(p[:3, :3].T@p[:3, :3]-np.eye(3)) < 1e-4
    assert abs(np.linalg.det(p[:3, :3])-1) < 1e-4
    return p


def difference(a, b):
    # Orthogonal polar factors remove only floating matrix roundoff for the
    # independent diagnostic angle. No recorded pose is replaced or optimized.
    ua, _, va = np.linalg.svd(a[:3, :3]); ub, _, vb = np.linalg.svd(b[:3, :3])
    r = (ua@va).T@(ub@vb)
    angle = np.degrees(np.arccos(np.clip((np.trace(r)-1)/2, -1, 1)))
    return float(np.linalg.norm(a[:3, 3]-b[:3, 3])), float(angle)


def main():
    formal = RUN/'formal'
    completion = json.loads((RUN/'shadow_completion.json').read_text())
    assert completion['returncode'] == 0 and completion['after_gate'] == 'ALL_INPUTS_SOURCE_BINARY_PCL_R6_OUTPUTS_PRODUCTION_UNCHANGED'
    # Freeze every native output before deriving statistics or copying tables.
    originals = [identity(p) for p in sorted(formal.iterdir()) if p.is_file()]
    originals += [identity(RUN/n) for n in ['shadow.log', 'shadow_freeze.json', 'shadow_completion.json']]
    freeze_path = DOC/'shadow_output_freeze.json'
    if freeze_path.exists():
        assert json.loads(freeze_path.read_text())['files'] == originals
    else:
        json_new(freeze_path, {'files': originals, 'stage': 'BEFORE_DERIVED_ANALYSIS'})
    for row in originals:
        target = DOC/Path(row['path']).name
        if target.exists():
            assert identity(target)['sha256'] == row['sha256']
        else:
            shutil.copyfile(row['path'], target)

    frames = rows(formal/'NATURAL_SHADOW_RESULTS.csv')
    diagnostics = rows(formal/'DIAGNOSTIC_PROBE_RESULTS.csv')
    candidates = rows(formal/'CANDIDATE_POOL.csv')
    selectors = rows(formal/'SELECTOR_COMPARISON.csv')
    calls = rows(formal/'NATIVE_ALIGN_CALLS.csv')
    blocks = rows(formal/'HELDOUT_BLOCK_LOSSES.csv')
    runtime = rows(formal/'RUNTIME_AND_MEMORY.csv')
    prereg = json.loads((DOC/'WINDOW_PRE_REGISTRATION.json').read_text())
    summary = json.loads((DOC/'STRUCTURAL_SUMMARY.json').read_text())
    assert len(frames) == 100 and [int(r['scan_index']) for r in frames] == list(range(1, 101))
    assert [int(r['raw_index']) for r in frames] == list(range(4982, 5082))
    for r, original in zip(frames, prereg['selected_scans']):
        assert r['source_fnv'] == original['expected_processed_fnv']
        assert int(r['full_points']) == original['expected_processed_points']
        assert int(r['training_points']) == original['training_points']
        assert int(r['header_ns']) == original['header_ns'] and int(r['reference_ns']) == original['reference_ns']
        assert r['validation_evaluable'] == '1'
    assert all(int(r['full_calls']) == int(r['split_calls']) == 1 for r in frames)
    events = sum(int(r['natural_trigger']) for r in frames)
    extra = sum(int(r['extra_calls']) for r in frames+diagnostics)
    assert extra <= 20 and all(int(r['extra_calls']) <= 2 for r in frames+diagnostics)
    assert all(int(r['extra_calls']) == 0 for r in frames if r['natural_trigger'] == '0')
    if events == 0:
        assert [int(r['scan_index']) for r in diagnostics] == [20, 40, 60, 80]
        assert sum(int(r['extra_calls']) for r in diagnostics) <= 8
    else:
        assert not diagnostics
    assert len(rows(formal/'FAILED_NATIVE_OUTPUT.csv')) == 0
    assert len(calls) == 2*(200+extra)
    for a, b in zip(calls[::2], calls[1::2]):
        assert a['status'] == 'START' and b['status'] == 'RETURNED'
        assert all(a[k] == b[k] for k in ['phase', 'scan_index', 'candidate_id', 'kind', 'cumulative_attempts'])
    assert [int(r['cumulative_attempts']) for r in calls[::2]] == list(range(1, 201+extra))
    log = (RUN/'shadow.log').read_text()
    assert 'SMOKE20_PASS' in log and 'COMPLETE frames=100' in log
    assert len(selectors) == 3*(100+len(diagnostics))
    for i in range(0, len(selectors), 3):
        group = selectors[i:i+3]
        assert len({(r['phase'], r['scan_index'], r['pool_ids']) for r in group}) == 1
        if '|' not in group[0]['pool_ids']:
            assert all(r['min_pairwise_advantage'] == 'NA' for r in group)
    assert len(blocks) == 8*len(candidates)
    indexed = {(r['phase'], r['scan_index'], r['candidate_id']): r for r in candidates if r['kind'] != 'CONTROL_FULL'}
    losses = {}
    for r in blocks:
        key = (r['phase'], r['scan_index'], r['candidate_id'])
        # CONTROL_FULL has id -1, so no collision with the common candidate pool.
        losses.setdefault(key, {})[int(r['block_id'])] = float(r['loss'])
        assert int(r['points']) >= 10 and 0 <= float(r['loss']) <= 1
    for key, r in indexed.items():
        assert abs(np.mean(list(losses[key].values()))-float(r['mean_block_loss'])) < 1e-12
    for r in frames:
        nominal = indexed[('NATURAL', r['scan_index'], '0')]
        expected = nominal['converged'] != '1' or int(nominal['iterations']) >= 40 or np.median(list(losses[('NATURAL', r['scan_index'], '0')].values())) > .25
        assert expected == bool(int(r['natural_trigger']))
    separations = []; seed_steps = []
    for r in candidates:
        pose(r, 'seed_'); pose(r, 'final_')
        if r['kind'] == 'RECOVERY':
            nominal = indexed[(r['phase'], r['scan_index'], '0')]
            sep = difference(pose(r, 'final_'), pose(nominal, 'final_'))
            step = difference(pose(r, 'seed_'), pose(nominal, 'final_'))
            separations.append(sep); seed_steps.append(step)
            assert step[0] <= .30+1e-10 and step[1] <= 3+1e-8
            if r['duplicate_of'] == '0':
                assert sep[0] <= .10+1e-8 and sep[1] <= 1+1e-5
    for r in diagnostics:
        nominal = indexed[('NATURAL', r['scan_index'], '0')]
        cached = indexed[('DIAGNOSTIC_FORCED', r['scan_index'], '0')]
        assert all(cached[k] == nominal[k] for k in cached if k.startswith('final_'))
        assert cached['align_ms'] == '0'
        assert int(r['weak_count']) >= 1 and float(r['lambda_min'])/float(r['lambda_max']) <= .01

    natural_runtime = [r for r in runtime if r['phase'] == 'NATURAL']
    diagnostic_runtime = [r for r in runtime if r['phase'] == 'DIAGNOSTIC_FORCED']
    cost = {k: stats([float(r[k]) for r in natural_runtime]) for k in
            ['preprocess_ms', 'full_align_ms', 'nominal_align_ms', 'validation_ms', 'selectors_and_pair_record_ms', 'frame_compute_ms']}
    cost['full_plus_split_with_preprocess_ms'] = stats([float(r['frame_compute_ms'])+float(r['preprocess_ms']) for r in natural_runtime])
    cost['diagnostic_frame_ms'] = stats([float(r['frame_compute_ms']) for r in diagnostic_runtime]) if diagnostic_runtime else None
    cost['recovery_align_ms'] = stats([float(r['align_ms']) for r in candidates if r['kind'] == 'RECOVERY']) if extra else None
    distinct = sum(int(r['distinct_alternatives']) for r in frames+diagnostics)
    status = ('REAL_SHADOW_DISTINCT_CANDIDATES_OBSERVED' if distinct else
              'REAL_SHADOW_TRIGGERED_NO_DISTINCT_CANDIDATES' if events else 'REAL_SHADOW_NO_NATURAL_TRIGGER')
    discrimination = ('HOLDOUT_DISCRIMINATION_NOT_EVALUABLE' if distinct == 0 else
                      'HOLDOUT_DISCRIMINATION_DEMONSTRATED' if any(int(r['proposed_selected']) > 0 for r in frames+diagnostics) else
                      'HOLDOUT_DISCRIMINATION_NOT_DEMONSTRATED')
    result = dict(task='P62-R8-STRUCTURAL-ADMISSION-AND-SHADOW', result=status, holdout_discrimination=discrimination,
                  START_SHA='b6c3bebb686697161cd645426d5d54ffe2927728',
                  PRECHECK_CODE_SHA=json.loads((RUN/'preflight_freeze.json').read_text())['CODE_SHA'],
                  SHADOW_CODE_SHA=json.loads((RUN/'shadow_freeze.json').read_text())['CODE_SHA'],
                  pre_registration_sha256=identity(DOC/'WINDOW_PRE_REGISTRATION.json')['sha256'], structural=summary,
                  smoke20='PASS_SAME_NATIVE_PROCESS', natural_frames=100, natural_events=events,
                  natural_extra_aligns=sum(int(r['extra_calls']) for r in frames), diagnostic_frames=len(diagnostics),
                  diagnostic_extra_aligns=sum(int(r['extra_calls']) for r in diagnostics),
                  native_aligns=200+extra, control_full_aligns=100, control_split_aligns=100, total_extra_aligns=extra,
                  natural_G_computed=sum(r['direction_status'] != 'NOT_REQUESTED' for r in frames),
                  diagnostic_weak_counts=[int(r['weak_count']) for r in diagnostics],
                  distinct_alternatives=distinct, recovery_converged=sum(r['converged']=='1' for r in candidates if r['kind']=='RECOVERY'),
                  all_actual_aligns_converged=sum(r['converged']=='1' for r in candidates if r['kind']!='CACHED_NOMINAL'),
                  recovery_duplicate_of_nominal=sum(r['duplicate_of']=='0' for r in candidates if r['kind']=='RECOVERY'),
                  max_recovery_vs_nominal_m=max((v[0] for v in separations),default=None),
                  max_recovery_vs_nominal_deg=max((v[1] for v in separations),default=None),
                  max_seed_translation_m=max((v[0] for v in seed_steps),default=None),
                  max_seed_rotation_deg=max((v[1] for v in seed_steps),default=None),
                  selector_counts={s:dict(Counter(r['outcome'] for r in selectors if r['selector']==s)) for s in ['NATIVE_SCORE','HELDOUT_MEAN','ROBUST_PAIRED']},
                  timing_ms=cost, quantile_method='numpy.percentile linear interpolation',
                  peak_native_ru_maxrss_KiB=max(int(r['process_ru_maxrss_KiB']) for r in runtime),
                  map_runtime=rows(formal/'map_runtime.csv')[0], native_wall_seconds=completion['native_wall_s'],
                  post_run_asset_gate=completion['after_gate'],
                  prohibited_operations=dict(GT_reads=0,EKF_runs=0,ROS_localization_nodes=0,rosbag_play=0,production_changes=0,feedback=0,second_windows=0,external_CLI=0),
                  novelty='NOT_ESTABLISHED', absolute_accuracy='NOT_EVALUATED', PUSH_EXECUTED=False,
                  NEXT='Submit a mathematical redesign contract for sparse-field structural holdout; keep fixed16 only as a limited baseline. No new run without approval.')
    # Derived reports may be regenerated, native files and the input preregistration may not.
    (DOC/'results.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(result,indent=2,ensure_ascii=False))


if __name__ == '__main__':
    main()
