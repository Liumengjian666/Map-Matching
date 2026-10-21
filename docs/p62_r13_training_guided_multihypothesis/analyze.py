#!/usr/bin/env python3
"""Offline R13 arithmetic/lineage verification, never align or nearest-neighbor search."""
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation

DOC=Path(__file__).resolve().parent
R11=Path('/home/jian/livox_ws/p62_r11_cause_aware_recovery_ws/docs/p62_r11_cause_aware_recovery')

def rows(p):
    with Path(p).open() as f:
        data=list(csv.DictReader(f))
    assert all(None not in r and None not in r.values() for r in data);return data
def identity(p):
    p=Path(p);h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(8*1024**2),b''):h.update(b)
    return dict(path=str(p),bytes=p.stat().st_size,sha256=h.hexdigest())
def matrix(r,prefix):return np.array([float(r[prefix+str(a)+str(b)]) for a in range(4) for b in range(4)]).reshape(4,4)
def distance(a,b):return float(np.linalg.norm(a[:3,3]-b[:3,3])),float(np.degrees(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()))
def lp(d):
    n=len(d);q=n//2;cap=2/n;v=[]
    for subset in itertools.combinations(range(n),q):
        base=cap*sum(d[i] for i in subset)
        if n%2:v.extend(base+(1-cap*q)*d[i] for i in range(n) if i not in subset)
        else:v.append(base)
    return min(v)
def stats(values):
    a=np.array(values,dtype=float);assert len(a) and np.isfinite(a).all()
    return dict(n=len(a),mean=float(a.mean()),p95=float(np.quantile(a,.95,method='linear')),max=float(a.max()),sum=float(a.sum()))
def save(name,data):
    with (DOC/name).open('x') as f:json.dump(data,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')

def main():
    receipt=json.loads((DOC/'RUN_RECEIPT.json').read_text());before=json.loads((DOC/'ASSET_IDENTITY_BEFORE.json').read_text())
    after=json.loads((DOC/'ASSET_IDENTITY_AFTER.json').read_text());build=json.loads((DOC/'BUILD_PROVENANCE.json').read_text())
    assert receipt['status']=='R13_REAL_EXPERIMENT_COMPLETED' and receipt['actual_native_processes']==1 and receipt['returncode']==0
    assert after['status']=='R13_ASSETS_UNCHANGED' and not after['differences'] and before['stable']==after['stable']
    for old in receipt['outputs']:
        assert identity(old['path'])==old
        source=Path(old['path']);archive=DOC/source.name if source.parent.name=='formal' else DOC/'raw'/source.name
        assert identity(archive)['sha256']==old['sha256']
    grid=rows(DOC/'COARSE_GRID_SCORES.csv');selected=rows(DOC/'SELECTED_SEEDS.csv');samples=rows(DOC/'COARSE_TRAINING_POINTS.csv')
    native=rows(DOC/'NATIVE_NDT_RESULTS.csv');candidates=rows(DOC/'CANDIDATE_POOL.csv');calls=rows(DOC/'ALIGN_CALLS.csv')
    raw=rows(DOC/'RAW_OPTIMIZER_OUTPUTS.csv');runtime=rows(DOC/'RUNTIME_AND_MEMORY.csv');blocks=rows(DOC/'HELDOUT_BLOCK_LOSSES.csv')
    pairs=rows(DOC/'PAIRWISE_EVIDENCE.csv');selectors=rows(DOC/'SELECTOR_COMPARISON.csv');observations=rows(DOC/'FINAL_OBSERVATIONS.csv')
    assert len(grid)==225 and len(samples)==1280 and len(selected)==len(native)==len(raw)==10 and len(calls)==20
    assert len(candidates)==15 and len(runtime)==len(observations)==5 and len(selectors)==15
    original=rows(R11/'CANDIDATE_POOL.csv');frozen_blocks=rows(R11/'BLOCK_LOSSES.csv')
    manifest=json.loads((DOC/'EVENT_INPUT_MANIFEST.json').read_text());summary=[]
    for event in range(60,65):
        e=str(event);gg=[r for r in grid if r['scan_index']==e];ss=[r for r in selected if r['scan_index']==e];nn=[r for r in native if r['scan_index']==e]
        cc=[r for r in candidates if r['scan_index']==e];sample=[r for r in samples if r['scan_index']==e]
        nominal=next(r for r in original if r['arm']=='PROPOSED_DISAGREEMENT' and r['scan_index']==e and r['candidate_id']=='0')
        tp=matrix(nominal,'seed_');t0=matrix(nominal,'final_')
        assert len(gg)==45 and len(sample)==256 and len(ss)<=2 and [int(r['grid_id']) for r in gg]==list(range(45))
        tcount=int(runtime[event-60]['training_points']);assert [int(r['training_index']) for r in sample]==[int(math.floor(j*(tcount-1)/255+.5)) for j in range(256)]
        assert all(int(r['training_sector'])%2==0 for r in sample)
        actual_nominal=next(r for r in cc if r['candidate_id']=='0')
        assert np.array_equal(matrix(actual_nominal,'final_'),t0) and np.array_equal(matrix(actual_nominal,'seed_'),tp)
        for b in range(8):
            frozen=next(r for r in frozen_blocks if r['arm']=='PROPOSED_DISAGREEMENT' and r['scan_index']==e and r['candidate_id']=='0' and r['block_id']==str(b))
            now=next(r for r in blocks if r['scan_index']==e and r['candidate_id']=='0' and r['block_id']==str(b))
            assert all(now[k]==frozen[k] for k in ['points','active','loss'])
        for index,r in enumerate(gg):
            dx=[-.4,0,.4][index//15];dy=[-.4,0,.4][(index%15)//5];yaw=[-10,-5,0,5,10][index%5]
            assert float(r['dx_m'])==dx and float(r['dy_m'])==dy and float(r['yaw_deg'])==yaw
            hat=np.zeros((4,4));z=np.radians(yaw);hat[0,1]=-z;hat[1,0]=z;hat[:3,3]=[dx,dy,0]
            independent=tp@expm(hat);double=matrix(r,'double_seed_');floated=matrix(r,'native_seed_')
            assert np.linalg.norm(double-independent)<1e-12 and np.array_equal(double.astype(np.float32).astype(float),floated)
            t,angle=distance(floated,tp);assert t<=.60 and angle<=12
            near_t,near_angle=distance(floated,t0)
            # Actual points are not on exclusion boundaries; don't hide ambiguity.
            assert abs(near_t-.25)>1e-6 and abs(near_angle-4)>1e-4
            assert bool(int(r['local_excluded']))==(near_t<.25 and near_angle<4)
        ranking=sorted([r for r in gg if r['local_excluded']=='0'],key=lambda r:(float(r['coarse_loss']),-float(r['coarse_overlap']),int(r['grid_id'])))
        picks=[]
        for r in ranking:
            if not picks:picks.append(r)
            elif len(picks)<2:
                t,angle=distance(matrix(r,'native_seed_'),matrix(picks[0],'native_seed_'))
                if t>=.30 or angle>=4:picks.append(r)
        assert [int(r['grid_id']) for r in sorted(ss,key=lambda r:int(r['selected_rank']))]==[int(r['grid_id']) for r in picks]
        for c in nn:
            j=c['candidate_id'];seed=next(r for r in ss if r['selected_rank']==j);source=next(r for r in raw if r['scan_index']==e and r['candidate_id']==j)
            assert c['source_call_or_grid']==seed['grid_id'] and np.array_equal(matrix(c,'seed_'),matrix(seed,'double_seed_'))
            assert np.array_equal(matrix(c,'final_'),matrix(source,'final_'))
            assert c['native_score']==source['native_score'] and c['converged']==source['converged']
            journal=[r for r in calls if r['new_call_id']==source['new_call_id']]
            assert len(journal)==2 and [r['status'] for r in journal]==['START','RETURNED']
            assert all(r['scan_index']==e and r['candidate_id']==j and r['grid_id']==seed['grid_id'] for r in journal)
        decision=next(r for r in selectors if r['arm']=='R13_TRAINING_GRID' and r['scan_index']==e)
        reps=[] if decision['pool_ids']=='NA' else list(map(int,decision['pool_ids'].split('|')))
        assert all(int(decision[x])<0 or int(decision[x]) in reps for x in ['native','heldout_mean','robust'])
        losses={int(r['candidate_id']):[] for r in cc}
        for candidate in losses:
            losses[candidate]=[float(r['loss']) for r in blocks if r['scan_index']==e and int(r['candidate_id'])==candidate and r['active']=='1']
        for p in pairs:
            if p['scan_index']==e:
                d=np.array(losses[int(p['k'])])-np.array(losses[int(p['j'])]);assert abs(lp(d)-float(p['D_j_k']))<1e-15
        eligible=sum(r['eligible']=='1' for r in nn);raw_distinct=sum(float(r['translation_from_nominal_m'])>.10 or float(r['rotation_from_nominal_deg'])>1 for r in nn)
        record=dict(local_event=event,raw_index=10640+event,grids_scored=45,training_points=tcount,coarse_points=256,
            selected_grid_ids=[int(r['grid_id']) for r in picks],new_native_calls=len(nn),converged=sum(r['converged']=='1' for r in nn),
            raw_distinct_outputs=raw_distinct,unique_distinct_groups=int(decision['distinct_groups']),raw_eligible_outputs=eligible,
            admissible_distinct_groups=int(decision['admissible_distinct_groups']),robust_distinct_preference=bool(int(decision['robust_distinct_preference'])),
            accepted_replacement=bool(int(decision['accepted_replacement'])),final_status=decision['final_status'],
            coarse_loss_min=min(float(r['coarse_loss']) for r in gg),coarse_loss_max=max(float(r['coarse_loss']) for r in gg),
            distinct_coarse_losses=len({r['coarse_loss'] for r in gg}),saturated_seeds=sum(float(r['coarse_loss'])==1 for r in gg))
        summary.append(record)
    assert [int(r['new_call_id']) for r in raw]==list(range(1,11))
    assert all(r['final_status']=='NO_ACCEPTABLE_MEASUREMENT' for r in selectors)
    assert all(r['observation_valid']=='0' and all(r['accepted_'+str(a)+str(b)]=='NA' for a in range(4) for b in range(4)) for r in observations)
    assert sum(int(r['coarse_NN_queries']) for r in runtime)==45*256*5
    assert json.loads((DOC/'LOCAL_REFERENCE_INVENTORY.json').read_text())['files_considered']==5728
    phases={k:stats([float(r[k]) for r in runtime]) for k in ['source_ms','coarse_45_ms','seed_selection_ms','coarse_phase_wall_ms','align_ms','native_capture_and_fitness_aux_ms','heldout_validation_ms','V3_decision_ms','extra_event_wall_ms']}
    distinct=sum(e['unique_distinct_groups'] for e in summary);admissible=sum(e['admissible_distinct_groups'] for e in summary);supported=sum(e['robust_distinct_preference'] for e in summary)
    result=dict(task='P62-R13-TRAINING-GUIDED-MULTI-HYPOTHESIS-NDT',START_SHA='58390fd173114e0e60a6dd9bf14d7c4be3f1106c',
        actual_run_CODE_SHA=receipt['actual_run_CODE_SHA'],actual_build_CODE_SHA=build['source_CODE_SHA'],
        numerical_analysis_CODE_SHA=subprocess.check_output(['git','rev-parse','HEAD'],cwd=DOC.parents[1],text=True).strip(),
        result='DISTINCT_NDT_HYPOTHESES_OBSERVED' if distinct else 'NO_DISTINCT_NDT_HYPOTHESES_OBSERVED',
        admissible_distinct_hypotheses_observed=admissible>0,robust_holdout_preference_observed=supported>0,
        correct_recovery='NOT_ESTABLISHED: no independent reference',events=summary,new_native_aligns=len(native),max_allowed=10,
        native_converged=sum(r['converged']=='1' for r in native),raw_distinct_outputs=sum(e['raw_distinct_outputs'] for e in summary),
        distinct_groups=distinct,admissible_distinct_groups=admissible,accepted_replacements=sum(e['accepted_replacement'] for e in summary),
        total_coarse_queries=45*256*5,independent_matrix_exponential_checks=225,LP_verified_directed_rows=len(pairs),
        independent_reference='INDEPENDENT_REFERENCE_NOT_AVAILABLE',novelty='NOT_ESTABLISHED',
        H1='SUPPORTED:225 deterministic training-only scores',H2='SUPPORTED:2 spaced seeds per event before first align',
        H3='SUPPORTED:5 physically distinct non-nominal groups across4 events; not proof of different local minima',
        H4='NOT_OBSERVED:all10 extra outputs fail unchanged5deg gate',H5='NOT_OBSERVED:no eligible distinct candidate or accepted recovery',
        H6='NOT_EVALUABLE:no qualified independent reference',runtime=phases,
        per_call_align_ms=stats([float(r['align_ms']) for r in native]),process_wall_s=receipt['wall_s'],
        map_runtime=rows(DOC/'MAP_RUNTIME.csv')[0],peak_process_RSS_MiB=max(float(r['process_high_water_RSS_KiB']) for r in runtime)/1024,
        timing_quantile='numpy linear/type7 descriptive only; offline5 events, not full localization',
        production_and_history='UNCHANGED',boundaries=dict(nominal_aligns=0,old_generator_aligns=0,new_extra_aligns=10,GT_data_reads=0,EKF=0,ROS_nodes=0,
            new_trajectory_estimates=0,production_changes=0,reference_inventory_runs=1,repeat_protection=0,formal_R13_processes=1,push_executed=False),
        limitations=['No independent correctness or absolute pose evidence',
            'Most raw physical differences are just above the frozen1deg threshold, not independent basin proof',
            'Training coarse ordering is initializer evidence only; correlated structural holdout is not truth',
            'All distinct results are inadmissible; no recovery/accuracy or10Hz claim',
            'The known five events, same-record map and historical causal guesses are not a blind generalization experiment',
            'Initial libusb link failure preserved, zero actual native run then'],
        unique_NEXT='Controller to freeze an independent correctness reference for bounded future evaluation before further recovery/accuracy claims; preserve this grid and gates, do not tune or launch another experiment now')
    save('results.json',result);print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
