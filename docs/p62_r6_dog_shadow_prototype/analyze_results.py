#!/usr/bin/env python3
"""Read frozen native outputs, independently check accounting, and archive (no optimizer)."""
import csv
import argparse
import hashlib
import itertools
import json
import math
import shutil
from collections import Counter
from pathlib import Path

import numpy as np

DOC=Path(__file__).resolve().parent
RUN=Path('/home/jian/livox_ws/p62_r6_dog_shadow_runs')
ACTUAL=RUN/'attempt02'
RAW=ACTUAL/'formal'


def identity(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda:stream.read(1024*1024),b''):digest.update(data)
    return {'path':str(path),'bytes':path.stat().st_size,'sha256':digest.hexdigest()}


def write_json(path,value):
    with path.open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def rows(name):
    with (RAW/name).open() as stream:return list(csv.DictReader(stream))


def stats(values):
    a=np.asarray(values,dtype=float)
    assert len(a) and np.isfinite(a).all()
    return {'count':len(a),'min':float(a.min()),'mean':float(a.mean()),
            'p95_linear_type7':float(np.percentile(a,95,interpolation='linear')),'max':float(a.max())}


def paired_oracle(j,k):
    # An independent LP vertex enumeration: choose four saturated 0.25 weights.
    return min(sum(k[b]-j[b] for b in choice)/4 for choice in itertools.combinations(range(8),4))


def pose(row,prefix):
    a=np.array([[float(row[prefix+str(i)+str(j)]) for j in range(4)] for i in range(4)])
    assert np.isfinite(a).all() and np.linalg.norm(a[3]-[0,0,0,1])<1e-6
    assert np.linalg.norm(a[:3,:3].T@a[:3,:3]-np.eye(3))<1e-4
    assert abs(np.linalg.det(a[:3,:3])-1)<1e-4
    return a


def rotation_deg(a,b):
    r=a[:3,:3].T@b[:3,:3]
    return math.degrees(math.acos(float(np.clip((np.trace(r)-1)/2,-1,1))))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--verify-only',action='store_true',help='Read and recompute checks without writing/optimizing')
    args=parser.parse_args()
    # Freeze everything before reading numerical results. Never overwrite old evidence.
    declared=sorted(RAW.glob('*.csv'))+[ACTUAL/n for n in ['formal.log','RUN_STARTED.json','run_freeze.json','run_completion.json']]
    declared += [RUN/n for n in ['formal.log','RUN_STARTED.json','run_freeze.json','run_completion.json',
                                'configure.log','build_final.log','build_rss_label.log','unit_native.log']]
    frozen={'method':'SHA256 before offline analysis; no changes to original run files',
            'files':[identity(path) for path in declared]}
    freeze_path=DOC/'output_freeze.json'
    if freeze_path.exists():
        assert json.loads(freeze_path.read_text())==frozen,'Original outputs changed'
    else:
        assert not args.verify_only,'Frozen output manifest missing'
        write_json(freeze_path,frozen)

    frames=rows('SHADOW_FRAME_RESULTS.csv');candidates=rows('CANDIDATE_POOL.csv')
    blocks=rows('HELDOUT_BLOCK_LOSSES.csv');selectors=rows('SELECTOR_COMPARISON.csv')
    runtime=rows('RUNTIME_AND_MEMORY.csv');calls=rows('NATIVE_ALIGN_CALLS.csv');preflight=rows('input_preflight.csv')
    assert [int(r['scan_index']) for r in frames]==list(range(1,101))
    assert len(runtime)==len(preflight)==100 and len(selectors)==300
    manifest=json.loads((DOC/'REAL_INPUT_MANIFEST.json').read_text())
    completion=json.loads((ACTUAL/'run_completion.json').read_text())
    assert completion['returncode']==0 and completion['frozen_inputs_after']=='ALL_SHA256_PASS_UNCHANGED'
    freeze=json.loads((ACTUAL/'run_freeze.json').read_text())
    lookup={(int(r['scan_index']),int(r['candidate_id'])):r for r in candidates}
    assert len(lookup)==len(candidates)
    losses={}
    for r in blocks:
        key=(int(r['scan_index']),int(r['candidate_id']))
        losses.setdefault(key,{})[int(r['block_id'])]=float(r['loss'])
        assert int(r['points'])>=10 and int(r['NN_queries'])==int(r['points'])
    start=[r for r in calls if r['phase']=='START'];returned=[r for r in calls if r['phase']=='RETURNED']
    extra=sum(int(r['extra_calls']) for r in frames)
    assert len(start)==len(returned)==200+extra and extra<=20
    assert [int(r['cumulative_attempts']) for r in start]==list(range(1,len(start)+1))
    details=[];delta_t=[];delta_r=[];median_losses=[]
    for frame,original,inp in zip(frames,manifest['raw_scans'],preflight):
        scan=int(frame['scan_index'])
        assert frame['source_fnv']==original['expected_processed_fnv']==inp['source_fnv']
        assert int(frame['header_ns'])==original['header_ns'] and int(frame['reference_ns'])==original['reference_ns']
        assert int(frame['full_calls'])==int(frame['split_calls'])==1 and int(frame['extra_calls'])<=2
        assert frame['validation_evaluable']==inp['evaluable']=='1'
        assert int(frame['training_points'])>=300
        same=[r for r in selectors if int(r['scan_index'])==scan]
        assert len({r['pool_ids'] for r in same})==1
        pool=[int(x) for x in same[0]['pool_ids'].split('|') if x]
        assert len(pool)==int(frame['unique_pool_size'])
        by_selector={r['selector']:r for r in same}
        eligible=[j for j in pool if lookup[(scan,j)]['eligible']=='1']
        block=lambda j:[losses[(scan,j)][b] for b in range(8)]
        native=max(eligible,key=lambda j:float(lookup[(scan,j)]['native_mean_score'])) if eligible else -1
        mean=min(eligible,key=lambda j:sum(block(j))/8) if eligible else -1
        robust=[j for j in eligible if j!=0 and all(paired_oracle(block(j),block(k))>.02 for k in pool if j!=k)]
        assert len(robust)<=1
        assert [int(by_selector[n]['selected_id']) for n in ['NATIVE_SCORE','HELDOUT_MEAN','ROBUST_PAIRED']]==[native,mean,robust[0] if robust else -1]
        nominal=lookup[(scan,0)]
        med=float(np.median(block(0)));median_losses.append(med)
        requested=nominal['converged']!='1' or int(nominal['iterations'])>=40 or med>.25
        assert requested==bool(int(frame['trigger']))
        if not requested:assert int(frame['extra_calls'])==0
        count=[r for r in start if int(r['scan_index'])==scan]
        assert len(count)==2+int(frame['extra_calls'])
        full=pose(lookup[(scan,-1)],'final_');split=pose(nominal,'final_')
        delta_t.append(float(np.linalg.norm(full[:3,3]-split[:3,3])));delta_r.append(rotation_deg(full,split))
        details.append({'scan_index':scan,'pool':pool,'nominal_median_block_loss':med})

    metrics=['native_mean_score','fitness','heldout_overlap_0p5m','mean_block_loss',
             'translation_from_prediction_m','rotation_from_prediction_deg','align_ms','validation_ms']
    arms={}
    for kind in sorted({r['kind'] for r in candidates}):
        selected=[r for r in candidates if r['kind']==kind]
        arms[kind]={'aligns':len(selected),'finite':sum(int(r['finite']) for r in selected),
                    'converged':sum(int(r['converged']) for r in selected),'eligible':sum(int(r['eligible']) for r in selected),
                    'iterations':dict(Counter(r['iterations'] for r in selected)),
                    'metrics':{key:stats([float(r[key]) for r in selected]) for key in metrics}}
    costs={k:stats([float(r[k]) for r in runtime]) for k in runtime[0] if k!='scan_index'}
    result={'task':'P62-R6-DOG-SHADOW-PROTOTYPE-AND-NCLT-PREFLIGHT',
        'A_result':'REAL_SHADOW_COMPLETED_WITHOUT_DISTINCT_CANDIDATES',
        'A_additional_limitations':['REAL_DISTINCT_CANDIDATES_NOT_OBSERVED','HOLDOUT_DISCRIMINATION_NOT_DEMONSTRATED'],
        'B_result':'NCLT_PREFLIGHT_BLOCKED','novelty':'NOT_ESTABLISHED',
        'git':{'branch':'research/p62-r6-dog-shadow-prototype','START_SHA':'c1769471f291932a749fc44705d8ddc30108cf31',
               'CODE_SHA':freeze['actual_CODE_SHA'],'prototype_cpp_commit':'4592128cf21c0c22d51c6a88fdb9a10ec0d64497',
               'END_SHA':'Resolve final archive commit with git log -1; self-referential SHA cannot be embedded in its own commit',
               'REMOTE_HEAD':'ABSENT at pre-run query; query again at handoff','PUSH_EXECUTED':False},
        'counts':{'real_scans':100,'smoke_scans':20,'native_real_aligns':len(start),'control_full':100,'control_split':100,
                  'requested_recovery_frames':sum(int(r['trigger']) for r in frames),'extra_native_aligns':extra,
                  'extra_budget':20,'generated_alternatives':len(candidates)-200,
                  'physical_deduplicated_alternative_branches':sum(sum(j!=0 for j in row['pool']) for row in details),
                  'robust_supported_recoveries':sum(int(r['selected_id'])>0 for r in selectors if r['selector']=='ROBUST_PAIRED'),
                  'ordinary_no_recovery_frames':sum(r['proposed_status']=='ORDINARY_NO_RECOVERY' for r in frames),
                  'robust_recovery_abstentions':sum(int(r['selected_id'])<0 for r in selectors if r['selector']=='ROBUST_PAIRED'),
                  'same_pool_checks':100,'source_preprocessing_parity':100,'synthetic_native_aligns_separate':2,
                  'loader_failures_before_main':1,'loader_failure_real_aligns':0},
        'arms':arms,'runtime':costs,'native_process_wall_seconds':completion['wall_seconds'],
        'native_peak_RSS_MiB':costs['process_ru_maxrss_KiB']['max']/1024,
        'nominal_median_block_loss':stats(median_losses),
        'full_vs_split_different_source_not_branch_evidence':{'translation_m':stats(delta_t),'rotation_deg':stats(delta_r)},
        'boundary':{'GT_reads':0,'ROS_localization_nodes':0,'rosbag_play':0,'EKF_runs':0,'feedback_updates':0,
                    'production_code_changes':0,'new_closed_loop_trajectory_estimates':0,'external_CLI':0},
        'tests':'18 C++ assertion categories PASS; 2 synthetic native aligns separate from 200 real aligns',
        'frozen_inputs_after':completion['frozen_inputs_after'],'stable_after':completion['stable_sources_after'],
        'independent_offline_checks':'PASS: accounting, inputs, identical singleton pools and triggers; real paired comparisons=0; 70-vertex oracle not exercised on real data',
        'NEXT':'Request approval for one predeclared real moving/risk window with the same frozen code and thresholds; no selection by candidate outputs or GT; do not execute now'}
    checks={'method':'CSV accounting and singleton selection checks; independent LP oracle provided but not exercised (real pairs=0)','frames':details}
    if args.verify_only:
        assert json.loads((DOC/'results.json').read_text())==result,'Archived statistics differ from raw recomputation'
        assert json.loads((DOC/'offline_checks.json').read_text())==checks,'Archived frame checks differ'
        for path in RAW.glob('*.csv'):
            assert identity(path)['sha256']==identity(DOC/path.name)['sha256'],'Archived CSV differs from original'
    else:
        write_json(DOC/'results.json',result)
        write_json(DOC/'offline_checks.json',checks)
        for path in RAW.glob('*.csv'):
            with path.open('rb') as src,(DOC/path.name).open('xb') as dst:shutil.copyfileobj(src,dst)
        for path in [ACTUAL/'run_freeze.json',ACTUAL/'run_completion.json',ACTUAL/'formal.log',RUN/'unit_native.log']:
            with path.open('rb') as src,(DOC/path.name).open('xb') as dst:shutil.copyfileobj(src,dst)
    # Verify all original outputs again, not only the copied reports.
    assert [identity(Path(r['path'])) for r in frozen['files']]==frozen['files']
    print(json.dumps({'A':result['A_result'],'B':result['B_result'],'real_aligns':len(start),
                      'triggers':result['counts']['requested_recovery_frames'],'peak_MiB':result['native_peak_RSS_MiB']},indent=2))


if __name__=='__main__':main()
