#!/usr/bin/env python3
"""Offline frozen-record accounting only. No native/PCL/ROS execution."""
import csv
import itertools
import json
import shutil
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from run_stage import DOC,RUN,REPO,identity,write_new

def rows(p):
    with p.open() as f:return list(csv.DictReader(f))

def stats(values):
    a=np.asarray(values,float)
    if not len(a):return None
    assert np.isfinite(a).all()
    return dict(n=len(a),mean=float(a.mean()),p95=float(np.percentile(a,95,method='linear')),max=float(a.max()),sum=float(a.sum()))

def matrix(r,prefix):
    p=np.array([float(r[prefix+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4)
    assert np.isfinite(p).all() and np.linalg.norm(p[3]-[0,0,0,1])<1e-6
    assert np.linalg.norm(p[:3,:3].T@p[:3,:3]-np.eye(3))<1e-4 and abs(np.linalg.det(p[:3,:3])-1)<1e-4
    return p

def distance(a,b):
    return np.linalg.norm(a[:3,3]-b[:3,3]),np.degrees(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude())

def main():
    completion=json.loads((RUN/'formal_completion.json').read_text());assert completion['returncode']==0 and completion['after_gate']=='ALL_UNCHANGED' and not completion.get('error')
    files=[identity(p) for p in sorted((RUN/'formal').iterdir()) if p.is_file()]
    files += [identity(RUN/name) for name in ['preflight.log','preflight_freeze.json','preflight_completion.json','formal.log','formal_freeze.json','formal_completion.json']]
    files += [identity(RUN/'preflight/BLOCK_SPATIAL_DISTRIBUTION.csv')]
    freeze=DOC/'output_freeze.json'
    if freeze.exists():assert json.loads(freeze.read_text())['files']==files
    else:write_new(freeze,dict(stage='Completed run outputs, before archived report generation',files=files))
    for r in files:
        destination=DOC/Path(r['path']).name
        if destination.exists():assert identity(destination)['sha256']==r['sha256']
        else:shutil.copyfile(r['path'],destination)
    cov=rows(DOC/'COVERAGE_COMPARISON.csv');pre=rows(RUN/'preflight/COVERAGE_COMPARISON.csv');assert cov==pre
    assert len(cov)==100 and [int(r['raw_index']) for r in cov]==list(range(10641,10741))
    frames=rows(DOC/'NATURAL_SHADOW_RESULTS.csv');candidates=rows(DOC/'CANDIDATE_SCALE_COMPARISON.csv')
    blocks=rows(DOC/'HELDOUT_BLOCK_LOSSES.csv');pairs=rows(DOC/'PAIRWISE_EVIDENCE.csv');selectors=rows(DOC/'SELECTOR_COMPARISON.csv')
    calls=rows(DOC/'NATIVE_ALIGN_CALLS.csv');schedule=rows(DOC/'DIAGNOSTIC_SCHEDULE.csv');timing=rows(DOC/'RUNTIME_AND_MEMORY.csv');gs=rows(DOC/'G_STATISTICS.csv')
    assert len(frames)==100 and len(schedule)==16
    assert [(int(r['scan_index']),r['scale'],int(r['sign'])) for r in schedule]==[(i,s,sign) for i in [20,40,60,80] for s in ['SMALL','LARGE'] for sign in [1,-1]]
    starts=calls[::2];ends=calls[1::2];assert len(starts)==len(ends)
    for a,b in zip(starts,ends):
        assert a['status']=='START' and b['status']=='RETURNED'
        assert all(a[k]==b[k] for k in ['phase','scan_index','candidate_id','scale','cumulative_attempts','budget_used'])
    assert [int(r['cumulative_attempts']) for r in starts]==list(range(1,len(starts)+1))
    extras=[r for r in starts if r['scale'] in ['SMALL','LARGE']];assert len(extras)<=20
    assert sum(r['scale']=='CONTROL_FULL' for r in starts)==100 and sum(r['scale']=='NOMINAL' for r in starts)==100
    assert len(starts)==200+len(extras)
    bycov={r['scan_index']:r for r in cov};index={(r['phase'],r['scan_index'],r['candidate_id']):r for r in candidates}
    loss={}
    for r in blocks:
        active=bycov[r['scan_index']]['active_mask'][int(r['block_id'])]=='1';assert active==bool(int(r['active']))
        if not active:assert r['loss']=='NA'
        else:assert np.isfinite(float(r['loss'])) and 0<=float(r['loss'])<=1
        loss.setdefault((r['phase'],r['scan_index'],r['candidate_id']),{})[int(r['block_id'])]=r['loss']
    for r in frames:
        nom=index['NATURAL',r['scan_index'],'0'];v=[float(x) for x in loss['NATURAL',r['scan_index'],'0'].values() if x!='NA']
        expected=nom['converged']!='1' or int(nom['iterations'])>=40 or np.median(v)>.25
        assert expected==bool(int(r['natural_trigger']))
        if not expected:assert int(r['extra_calls'])==0
        assert int(r['extra_calls'])<=2
    # Native pairs independently checked by enumerating all capped-simplex
    # vertices, not by subtracting two separately minimized losses.
    for r in pairs:
        b=int(r['B']);active=[i for i,c in enumerate(bycov[r['scan_index']]['active_mask']) if c=='1'];assert len(active)==b
        j=loss[r['phase'],r['scan_index'],r['candidate_j']];k=loss[r['phase'],r['scan_index'],r['candidate_k']]
        d=np.array([float(k[a])-float(j[a]) for a in active]);cap=2/b;full=b//2;rem=1-cap*full;vertex=[]
        for chosen in itertools.combinations(range(b),full):
            value=sum(cap*d[a] for a in chosen)
            if b%2:vertex += [value+rem*d[a] for a in range(b) if a not in chosen]
            else:vertex.append(value)
        assert abs(min(vertex)-float(r['D_j_k']))<1e-13
    for i in range(0,len(selectors),3):
        group=selectors[i:i+3];assert len({(r['phase'],r['scan_index'],r['pool_ids']) for r in group})==1
        if '|' not in group[0]['pool_ids']:assert all(r['min_pairwise_advantage']=='NA' for r in group)
    assert len(selectors)==312
    decision_groups=[selectors[i:i+3] for i in range(0,len(selectors),3)]
    nontrivial=[g for g in decision_groups if '|' in g[0]['pool_ids']]
    nontrivial_differences=sum(len({r['selected_id'] for r in g})>1 for g in nontrivial)
    independent=[]
    for r in candidates:
        current=matrix(r,'final_');seed=matrix(r,'seed_');nom=index[r['phase'],r['scan_index'],'0'];center=matrix(nom,'final_')
        delta=distance(current,center)
        assert abs(delta[0]-float(r['translation_from_nominal_m']))<1e-10
        assert abs(delta[1]-float(r['rotation_from_nominal_deg']))<1e-8
        if r['scale'] in ['SMALL','LARGE']:
            step=distance(seed,center);large=r['scale']=='LARGE';assert step[0]<=(1.2 if large else .30)+1e-10 and step[1]<=(8 if large else 3)+1e-5
            independent.append(dict(scan_index=r['scan_index'],candidate_id=r['candidate_id'],scale=r['scale'],rotation_vs_nominal_deg=float(delta[1]),translation_vs_nominal_m=float(delta[0]),duplicate_of=r['duplicate_of']))
        if r['scale']=='CACHED_NOMINAL':
            original=index['NATURAL',r['scan_index'],'0'];assert all(r[k]==original[k] for k in r if k.startswith('final_'));assert r['align_ms']=='0'
    distinct=[r for r in candidates if r['phase']=='DIAGNOSTIC_ONLY' and r['scale'] in ['SMALL','LARGE'] and r['duplicate_of']=='-1']
    natural_extras=sum(r['phase']=='NATURAL' for r in extras);diag_extras=len(extras)-natural_extras
    events=sum(int(r['natural_trigger']) for r in frames if r['natural_trigger']!='NA')
    nat=[r for r in timing if r['phase']=='NATURAL'];dia=[r for r in timing if r['phase']=='DIAGNOSTIC_ONLY']
    metric={key:stats([float(r[key]) for r in nat]) for key in ['preprocess_ms','full_align_ms','nominal_align_ms','extra_align_ms','G_ms','validation_ms','selectors_and_record_ms','frame_compute_ms']}
    metric['formal_both_arms_plus_preprocess_ms']=stats([float(r['frame_compute_ms'])+float(r['preprocess_ms']) for r in nat])
    metric['diagnostic_frame_ms']=stats([float(r['frame_compute_ms']) for r in dia])
    metric['small_align_ms']=stats([float(r['align_ms']) for r in candidates if r['scale']=='SMALL'])
    metric['large_align_ms']=stats([float(r['align_ms']) for r in candidates if r['scale']=='LARGE'])
    eligible_nominal=sum(index['NATURAL',str(i),'0']['formal_eligible']=='1' for i in range(1,101))
    result=dict(task='P62-R9-COVERAGE-AWARE-RECOVERY-V2',START_SHA='36930e56bcd154d48d0f4cb79d66a7defcd43a0d',
                PRECHECK_CODE_SHA=json.loads((RUN/'preflight_freeze.json').read_text())['CODE_SHA'],CODE_SHA=json.loads((RUN/'formal_freeze.json').read_text())['CODE_SHA'],
                V1_evaluable=sum(int(r['V1_evaluable']) for r in cov),V2_evaluable=sum(int(r['V2_evaluable']) for r in cov),frames=100,
                B_distribution=dict(Counter(r['B'] for r in cov)),coverage_target=.60,coverage_target_passed=sum(int(r['V2_evaluable']) for r in cov)>=60,
                natural_events=events,natural_extra_aligns=natural_extras,diagnostic_extra_aligns=diag_extras,total_extra_aligns=len(extras),total_native_aligns=len(starts),
                actual_converged=sum(r['converged']=='1' for r in candidates if r['scale']!='CACHED_NOMINAL'),
                diagnostic_distinct_alternatives=len(distinct),natural_distinct_alternatives=sum(int(r['distinct_alternatives']) for r in frames),
                distinct_records=[{k:r[k] for k in ['scan_index','raw_index','candidate_id','scale','formal_eligible','rotation_from_prediction_deg','rotation_from_nominal_deg']} for r in distinct],
                eligible_nominal_frames=eligible_nominal,ineligible_nominal_frames=100-eligible_nominal,
                diagnostic_schedule_counts=dict(Counter(r['status'] for r in schedule)),
                selector_outcomes={s:dict(Counter(r['outcome'] for r in selectors if r['selector']==s)) for s in ['NATIVE_SCORE','HELDOUT_MEAN','COVERAGE_ROBUST']},
                selector_nontrivial_choice_difference=bool(nontrivial_differences),selector_nontrivial_groups=len(nontrivial),
                holdout_discrimination=('HOLDOUT_DISCRIMINATION_NOT_EVALUABLE' if not distinct else 'HOLDOUT_DISCRIMINATION_DEMONSTRATED' if any(r['selector']=='COVERAGE_ROBUST' and int(r['selected_id'])>0 for r in selectors) else 'HOLDOUT_DISCRIMINATION_NOT_DEMONSTRATED'),
                pairwise_records=pairs,independent_candidate_rotation_check=independent,
                diagnostic_G=[r for r in gs if r['phase']=='DIAGNOSTIC_ONLY'],timing_ms=metric,
                quantile_method='numpy percentile linear',peak_ru_maxrss_KiB=max(int(r['peak_ru_maxrss_KiB']) for r in timing),
                native_wall_seconds=completion['native_wall_seconds'],map_runtime=rows(DOC/'MAP_RUNTIME.csv')[0],
                asset_gate=completion['after_gate'],PUSH_EXECUTED=False,novelty='NOT_ESTABLISHED',localization_accuracy='NOT_EVALUATED',
                execution_limits=dict(GT_reads=0,EKF_runs=0,ROS_localization_nodes=0,feedback=0,second_window=0,external_CLI=0),
                conclusion='V2_COVERAGE_TARGET_MET_DIAGNOSTIC_SEPARATION_ONLY_NO_SUPPORTED_RECOVERY',
                NEXT='Submit a fixed-data mathematical review of trigger/admission mismatch: formally ineligible nominal results can remain non-triggering. Do not relax bounds or run new experiments without approval.')
    (DOC/'results.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['diagnostic_G','timing_ms','independent_candidate_rotation_check']},indent=2,ensure_ascii=False))

if __name__=='__main__':main()
