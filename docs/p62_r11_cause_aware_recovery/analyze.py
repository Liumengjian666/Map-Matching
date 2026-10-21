#!/usr/bin/env python3
"""Independent offline analysis of this single R11 run, no optimization."""
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path
import shutil
import subprocess
import unittest
import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation

DOC=Path(__file__).resolve().parent
REPO=DOC.parents[1]
RUN=Path('/home/jian/livox_ws/p62_r11_cause_aware_runs')
ARMS=['BASELINE_WEAK','PROPOSED_DISAGREEMENT']


def ident(p):
    p=Path(p);h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(8*1024**2),b''):h.update(block)
    return dict(path=str(p),bytes=p.stat().st_size,sha256=h.hexdigest())


def save(name,value):
    (DOC/name).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')


def freeze():
    pairs=[(p,DOC/p.name) for p in sorted((RUN/'formal').iterdir()) if p.is_file()]
    pairs += [(p,DOC/'raw'/p.name) for p in sorted(RUN.iterdir()) if p.is_file()]
    build=Path('/home/jian/livox_ws/p62_r11_cause_aware_build/frozen_build_02')
    pairs += [(build/name,DOC/'raw'/name) for name in ['BOUND_BUILD_STARTED.json','bound_build.log','CMakeCache.txt']]
    record=[dict(original=ident(a),archive=str(b)) for a,b in pairs]
    manifest=DOC/'OUTPUT_FREEZE.json'
    if manifest.exists():assert json.loads(manifest.read_text())['records']==record,'R11 output identity changed'
    else:
        (DOC/'raw').mkdir(exist_ok=False)
        for a,b in pairs:
            assert not b.exists();shutil.copy2(a,b)
        save('OUTPUT_FREEZE.json',dict(records=record))
    for a,b in pairs:assert ident(a)['sha256']==ident(b)['sha256']


def rows(name):
    with (DOC/name).open() as f:
        values=list(csv.DictReader(f))
    assert all(None not in r and None not in r.values() for r in values),'CSV width mismatch: '+name
    return values


def val(r,k):return float(r[k]) if r[k]!='NA' else math.nan
def matrix(r,prefix):return np.array([val(r,prefix+str(i)+str(j)) for i in range(4) for j in range(4)]).reshape(4,4)
def rigid(t):
    r=t[:3,:3]
    return np.isfinite(t).all() and np.linalg.norm(t[3]-[0,0,0,1])<1e-6 and np.linalg.norm(r.T@r-np.eye(3))<1e-4 and abs(np.linalg.det(r)-1)<1e-4


def distance(a,b):
    # Rotation.from_matrix uses normalized quaternion extraction, avoiding the
    # near-zero trace/acos false angle of slightly nonorthogonal float matrices.
    return float(np.linalg.norm(a[:3,3]-b[:3,3])),float(np.degrees(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()))


def lp_min(d):
    n=len(d);assert 4<=n<=8 and np.isfinite(d).all();cap=2/n;q=n//2;remainder=1-q*cap;values=[]
    for active in itertools.combinations(range(n),q):
        base=cap*sum(d[i] for i in active)
        if n%2:values.extend(base+remainder*d[i] for i in range(n) if i not in active)
        else:values.append(base)
    return min(values)


def stats(a):
    a=np.asarray(a,dtype=float);assert a.size and np.isfinite(a).all()
    return dict(n=int(a.size),mean=float(a.mean()),p95=float(np.quantile(a,.95,method='linear')),max=float(a.max()))


def main():
    freeze();events=rows('EVENT_COMPARISON.csv');coverage=rows('COVERAGE.csv')
    assert len(events)==len(coverage)==100 and [int(r['raw_index']) for r in events]==list(range(10641,10741))
    assert [int(r['scan_index']) for r in events if r['new_event']=='1']==[60,61,62,63,64]
    assert all(r['old_trigger']=='0' for r in events)
    manifest=json.loads((DOC.parent/'p62_r7_motion_selected_shadow/REAL_INPUT_MANIFEST.json').read_text())
    with open(manifest['scans_csv']['path']) as f:original=list(csv.DictReader(f))
    for c,o in zip(coverage,original):
        assert c['source_fnv']==o['expected_processed_fnv'] and c['source_points']==o['expected_processed_points']
        for k in ['raw_index','seq','header_ns']:assert c[k]==o[k]
    common={(r['arm'],int(r['scan_index'])):r for r in rows('COMMON_RESULTS.csv')};assert len(common)==200
    candidates={(r['arm'],int(r['scan_index']),int(r['candidate_id'])):r for r in rows('CANDIDATE_POOL.csv')}
    decisions={(r['arm'],int(r['scan_index'])):r for r in rows('ADMISSION_AND_SELECTION.csv')};assert len(decisions)==200
    observations={(r['arm'],int(r['scan_index'])):r for r in rows('FINAL_OBSERVATIONS.csv')}
    selectors={(r['arm'],int(r['scan_index']),r['selector']):r for r in rows('SELECTORS.csv')}
    losses={}
    for r in rows('BLOCK_LOSSES.csv'):losses.setdefault((r['arm'],int(r['scan_index']),int(r['candidate_id'])),[]).append(r)
    metrics={int(r['native_call_id']):r for r in rows('RAW_NATIVE_METRICS.csv')}
    raw={int(r['native_call_id']):r for r in rows('RAW_OPTIMIZER_OUTPUTS.csv')}
    assert len(raw)==len(metrics)==212 and set(raw)==set(metrics)==set(range(1,213))
    assert not rows('FAILURES.csv')
    calls=rows('ALIGN_CALLS.csv')
    assert len(calls)==424 and all(sum(r['status']==status for r in calls)==212 for status in ['START','RETURNED'])
    eligible={};lossmap={};all_rows=list(common.values())+list(candidates.values())
    for c in all_rows:
        arm,i,j=c['arm'],int(c['scan_index']),int(c['candidate_id']);key=(arm,i,j);call=int(c['native_call_id'])
        assert call in raw and call in metrics
        for k in ['finite','converged','iterations','fitness','align_ms']:
            assert c[k]==metrics[call][k],('metric lineage',key,k)
        assert c['native_score']==metrics[call]['native_score']==raw[call]['native_score']
        assert np.array_equal(matrix(c,'final_'),matrix(raw[call],'final_'))
        assert np.array_equal(matrix(c,'seed_'),matrix(raw[call],'seed_'))
        assert rigid(matrix(c,'final_'))
        if j==0:
            assert call==2*i and np.array_equal(matrix(c,'final_'),matrix(common['CONTROL_SPLIT',i],'final_'))
        if arm in ARMS and j>0:assert raw[call]['arm']==arm and int(raw[call]['scan_index'])==i
        evidence=sorted(losses[key],key=lambda r:int(r['block_id']));assert len(evidence)==8
        assert all((r['active']=='1')==(int(r['points'])>=10) for r in evidence)
        assert all(r['loss']=='NA' for r in evidence if r['active']=='0')
        active=[val(r,'loss') for r in evidence if r['active']=='1'];lossmap[key]=active
        assert len(active)==int(coverage[i-1]['B']) and all(0<=x<=1 for x in active)
        assert abs(np.mean(active)-val(c,'mean_loss'))<1e-12
        pred=matrix(common['CONTROL_SPLIT',i],'seed_');t,r=distance(matrix(c,'final_'),pred)
        assert abs(t-val(c,'translation_from_prediction_m'))<1e-9 and abs(r-val(c,'rotation_from_prediction_deg'))<1e-7
        legal=(c['finite']=='1' and c['converged']=='1' and math.isfinite(val(c,'native_score'))
               and math.isfinite(val(c,'fitness')) and t<=.8 and r<=5 and val(c,'overlap')>=.5 and len(active)>=4)
        eligible[key]=legal;assert int(c['eligible'])==legal
    arm_stats={}
    for arm in ARMS:
        subset=[r for (a,i),r in decisions.items() if a==arm]
        for i in range(1,101):
            decision=decisions[arm,i];cs=sorted([r for (a,n,j),r in candidates.items() if a==arm and n==i],key=lambda r:int(r['candidate_id']))
            dedup=[]
            for c in cs:
                duplicate=-1
                for previous in dedup:
                    t,r=distance(matrix(c,'final_'),matrix(previous,'final_'))
                    if t<=.1 and r<=1:duplicate=int(previous['candidate_id']);break
                assert int(c['duplicate_of'])==duplicate
                if duplicate<0:dedup.append(c)
            ids=[int(c['candidate_id']) for c in dedup]
            assert int(decision['pool_size'])==len(ids)
            assert int(decision['distinct_alternatives'])==len(ids)-1
            assert int(decision['eligible_alternatives'])==sum(eligible[arm,i,j] for j in ids if j>0)
            legal=[j for j in ids if eligible[arm,i,j]]
            native=max(legal,key=lambda j:val(candidates[arm,i,j],'native_score')) if legal else -1
            mean=min(legal,key=lambda j:np.mean(lossmap[arm,i,j])) if legal else -1
            robust=[j for j in legal if j>0 and len(ids)>=2 and all(lp_min([b-a for a,b in zip(lossmap[arm,i,j],lossmap[arm,i,k])])>.02 for k in ids if k!=j)]
            assert len(robust)<=1;robust=robust[0] if robust else -1
            for method,expected in [('NATIVE_SCORE',native),('HELDOUT_MEAN',mean),('COVERAGE_ROBUST',robust)]:
                s=selectors[arm,i,method];assert int(s['selected'])==expected and s['pool_ids']=='|'.join(map(str,ids))
                if len(ids)==1:assert s['min_pairwise_advantage']=='NA'
            selected=robust if robust>=0 else 0 if eligible[arm,i,0] else -1
            assert int(decision['final_selected'])==selected and int(decision['observation_valid'])==(selected>=0)
            assert int(decision['recovery_success'])==(selected>0) and int(decision['prediction_fallback'])==(selected<0)
            obs=observations[arm,i]
            if selected>=0:assert np.array_equal(matrix(obs,'accepted_'),matrix(candidates[arm,i,selected],'final_'))
            else:assert decision['final_status']=='NO_ACCEPTABLE_MEASUREMENT' and all(math.isnan(v) for v in matrix(obs,'accepted_').flat)
            assert np.array_equal(matrix(obs,'prediction_'),matrix(common['CONTROL_SPLIT',i],'seed_'))
            assert int(decision['extra_calls'])<=2
            if events[i-1]['new_event']=='0':assert int(decision['extra_calls'])==0
        extra=[r for (a,i,j),r in candidates.items() if a==arm and j>0]
        times=[r for r in rows('RUNTIME_AND_MEMORY.csv') if r['arm']==arm]
        arm_stats[arm]=dict(candidate_available_events=sum(int(r['seeds_generated'])>0 for r in subset),
            seeds_generated=sum(int(r['seeds_generated']) for r in subset),extra_aligns=sum(int(r['extra_calls']) for r in subset),
            distinct_alternatives=sum(int(r['distinct_alternatives']) for r in subset),
            eligible_distinct_alternatives=sum(int(r['eligible_alternatives']) for r in subset),
            eligible_extra_results_before_dedup=sum(r['eligible']=='1' for r in extra),
            supported_recoveries=sum(r['recovery_success']=='1' for r in subset),
            accepted_nominal=sum(r['final_status']=='ACCEPT_NOMINAL' for r in subset),
            no_acceptable_measurement=sum(r['final_status']=='NO_ACCEPTABLE_MEASUREMENT' for r in subset),
            extra_native_per_call_ms=stats([val(r,'align_ms') for r in extra]),
            phase_wall_ms=stats([val(r,'phase_wall_ms') for r in times]),
            event_phase_wall_ms=stats([val(r,'phase_wall_ms') for r in times if int(r['scan_index']) in [60,61,62,63,64]]))
    for p in rows('PAIRWISE_EVIDENCE.csv'):
        arm,i,j,k=p['arm'],int(p['scan_index']),int(p['j']),int(p['k'])
        assert abs(lp_min([b-a for a,b in zip(lossmap[arm,i,j],lossmap[arm,i,k])])-val(p,'D_j_k'))<1e-12
    excluded=[]
    for (arm,i,j),c in candidates.items():
        if j>0 and c['eligible']=='1' and int(c['duplicate_of'])>=0:
            k=int(c['duplicate_of'])
            excluded.append(dict(arm=arm,scan_index=i,candidate_id=j,duplicate_of=k,
                D_candidate_vs_representative=lp_min([b-a for a,b in zip(lossmap[arm,i,j],lossmap[arm,i,k])]),
                D_representative_vs_candidate=lp_min([b-a for a,b in zip(lossmap[arm,i,k],lossmap[arm,i,j])]),
                role='OFFLINE_EXCLUDED_PAIR_DIAGNOSTIC_NOT_USED_FOR_SELECTION'))
    with (DOC/'EXCLUDED_CANDIDATE_EVIDENCE.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['arm','scan_index','candidate_id','duplicate_of','D_candidate_vs_representative','D_representative_vs_candidate','role'],lineterminator='\n')
        writer.writeheader();writer.writerows(excluded)
    seed_count=0;precision=[]
    for r in rows('SEED_COMPARISON.csv'):
        if r['seed_id']=='NA':continue
        arm,i,j=r['arm'],int(r['scan_index']),int(r['seed_id']);seed_count+=1
        seed=matrix(r,'seed_');native=matrix(r,'native_seed_')
        assert np.array_equal(seed.astype(np.float32).astype(float),native)
        assert np.array_equal(seed,matrix(candidates[arm,i,j],'seed_'))
        center=matrix(common['CONTROL_SPLIT',i],'final_' if arm==ARMS[0] else 'seed_')
        t,angle=distance(seed,center)
        assert t<=.30+1e-12 and angle<=3+1e-10
        nt,nr=distance(native,center)
        precision.append(dict(arm=arm,scan_index=i,seed_id=j,double_translation_m=t,double_rotation_deg=angle,
            native_translation_m=nt,native_rotation_deg=nr,native_within_strict_caps=nt<=.30 and nr<=3))
        if arm==ARMS[1]:
            assert t<=.30 and angle<=3 and distance(native,center)[0]<=.30 and distance(native,center)[1]<=3
            xi=np.array([val(r,k) for k in ['rho_x','rho_y','rho_z','phi_x','phi_y','phi_z']]);phi=xi[3:]
            hat=np.zeros((4,4));hat[:3,3]=xi[:3];hat[:3,:3]=[[0,-phi[2],phi[1]],[phi[2],0,-phi[0]],[-phi[1],phi[0],0]]
            independent=center@expm(float(r['sign'])*val(r,'scale')*hat)
            assert np.linalg.norm(independent-seed)<1e-10,'Independent matrix exponential seed mismatch'
            relative=np.linalg.inv(center)@matrix(common['CONTROL_SPLIT',i],'final_')
            assert np.linalg.norm(expm(hat)-relative)<1e-5
            assert abs(np.linalg.norm(expm(hat)-relative)-val(r,'reconstruction_error'))<1e-10
    extra=sum(s['extra_aligns'] for s in arm_stats.values());assert seed_count==extra==12<=20
    with (DOC/'SEED_PRECISION_AUDIT.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(precision[0]),lineterminator='\n');writer.writeheader();writer.writerows(precision)
    assert [int(r['budget_used']) for r in rows('ADMISSION_AND_SELECTION.csv')]==list(np.cumsum([int(r['extra_calls']) for r in rows('ADMISSION_AND_SELECTION.csv')]))
    runtime=rows('RUNTIME_AND_MEMORY.csv');assert len(runtime)==300
    completion=json.loads((DOC/'raw/completion.json').read_text());assert completion['returncode']==0 and 'after_gate' in completion
    freeze_data=json.loads((DOC/'raw/freeze.json').read_text())
    summary=dict(task='P62-R11-PRESERVATION-RETRY-AND-ALGORITHM-RESUME',START_SHA='10edcc0703129a3c5444c672c5b7cbb938bf3f74',
        actual_run_CODE_SHA=freeze_data['CODE_SHA'],analysis_CODE_SHA=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
        preservation='PRESERVATION_PASS',frames=100,full_native=100,split_native=100,extra_native=extra,total_native=200+extra,budget=20,
        old_trigger_count=0,natural_events=[60,61,62,63,64],arms=arm_stats,
        H1='SUPPORTED_IN_THIS_WINDOW: availability expanded1/5 to5/5; four no-weak-direction events received seeds',
        H2='NOT_DEMONSTRATED_FOR_PROPOSED: all10 extra results deduplicate to nominal',
        H3='ONE_ELIGIBLE_RAW_DUPLICATE_OBSERVED; no eligible physically distinct alternative after frozen dedup',H4='NOT_DEMONSTRATED: no accepted replacement',H5='PASS:12/20 extras',
        result='CANDIDATE_AVAILABILITY_EXPANDED_BUT_NO_SUPPORTED_RECOVERY',novelty='NOT_ESTABLISHED',
        peak_process_RSS_MiB=max(val(r,'process_high_water_rss_KiB') for r in runtime)/1024,
        native_process_wall_s=completion['native_process_wall_s'],map_runtime=rows('MAP_RUNTIME.csv')[0],
        full_native_ms=stats([val(c,'align_ms') for (arm,i),c in common.items() if arm=='CONTROL_FULL']),
        split_native_ms=stats([val(c,'align_ms') for (arm,i),c in common.items() if arm=='CONTROL_SPLIT']),
        common_phase_wall_ms=stats([val(r,'phase_wall_ms') for r in runtime if r['arm']=='COMMON']),
        source_preprocess_ms=stats([val(r,'preprocess_ms') for r in runtime if r['arm']=='COMMON']),
        timing_quantile='numpy linear/type7; descriptive only',after_gate=completion['after_gate'],
        independent_matrix_exponential_checks=10,unit_tests='2 actual C++ zero-align executables plus offline verification',
        excluded_eligible_candidate_diagnostics=excluded,
        seed_gate=dict(proposed_double_and_native_float='PASS',baseline_double='PASS',
            baseline_native_float_roundoff_exceedances=sum(not r['native_within_strict_caps'] for r in precision if r['arm']==ARMS[0]),
            baseline_native_float_max_rotation_deg=max(r['native_rotation_deg'] for r in precision if r['arm']==ARMS[0]),
            note='Frozen legacy generator unchanged; one float representation exceeds3deg by about3.42e-8deg. Retained, not silently clamped/re-run.'),
        pre_native_storage_blocked=True,pre_native_calls_at_block=0,actual_native_subprocess_executions=1,
        boundaries=dict(GT_reads=0,EKF_runs=0,ROS_nodes=0,feedback=0,forced_events=0,third_generator=0,production_changes=0,new_trajectory_estimates=0),
        limitations=['Same-record map and fixed historical causal initials; no independent GT/accuracy claim',
            'Candidate observations are not closed-loop trajectories',
            'Global process high-water RSS cannot distinguish independent arm memory peaks',
            'Ordinary robust singleton abstention is not evidence of candidate discrimination',
            'A raw formally eligible candidate at64 is discarded by frozen physical dedup; its offline paired advantage is still negative',
            'Baseline single distinct result lies near1deg dedup boundary; not a basin proof'],
        engineering_status='COMPLETED_WITH_LEGACY_FLOAT_SEED_CAP_DEVIATION',
        unique_NEXT='Ask research controller to define a bounded feasibility-preserving candidate-pool contract for eligible raw duplicate64/1 discarded against illegal nominal; retain dedup distances and D>0.02, note its diagnostic D is negative, and do not run or promise recovery now')
    save('results.json',summary);print(json.dumps(summary,ensure_ascii=False,indent=2))


class MathTests(unittest.TestCase):
    def test_lp(self):
        for n in range(4,9):
            self.assertAlmostEqual(lp_min([.02]*n),.02)
            d=np.linspace(-.3,.4,n);q=n//2;expected=2/n*sum(d[:q])+(1-2/n*q)*d[q]
            self.assertAlmostEqual(lp_min(d),expected)
    def test_exponential(self):
        a=np.zeros((4,4));a[:3,:3]=[[0,-np.pi/2,0],[np.pi/2,0,0],[0,0,0]];a[0,3]=1
        t=expm(a);self.assertAlmostEqual(t[0,3],2/np.pi);self.assertAlmostEqual(t[1,3],2/np.pi)


if __name__=='__main__':
    import sys
    if not __debug__:raise RuntimeError('Assertions required')
    if '--self-test' in sys.argv:unittest.main(argv=['analyze.py'])
    else:main()
