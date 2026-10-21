#!/usr/bin/env python3
"""Read-only numerical verification of the completed offline C++ decision replay."""
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import statistics

DOC=Path(__file__).resolve().parent
R11=Path('/home/jian/livox_ws/p62_r11_cause_aware_recovery_ws/docs/p62_r11_cause_aware_recovery')

def rows(path):
    with Path(path).open() as f:return list(csv.DictReader(f))
def save(name,value):
    with (DOC/name).open('x') as f:json.dump(value,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
def identity(p):
    p=Path(p);return dict(path=str(p),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
def stats(values):
    values=sorted(values);assert values and all(math.isfinite(v) for v in values)
    x=.95*(len(values)-1);a=int(x);p95=values[a]+(x-a)*(values[min(a+1,len(values)-1)]-values[a])
    return dict(n=len(values),mean=statistics.mean(values),p95=p95,max=max(values))
def lp_vertex(d):
    n=len(d);cap=2/n;q=n//2;out=[]
    for full in itertools.combinations(range(n),q):
        v=cap*sum(d[i] for i in full)
        if n%2:out.extend(v+(1-cap*q)*d[i] for i in range(n) if i not in full)
        else:out.append(v)
    return min(out)

def main():
    before=json.loads((DOC/'ASSET_IDENTITY_BEFORE.json').read_text())
    after=json.loads((DOC/'ASSET_IDENTITY_AFTER.json').read_text())
    receipt=json.loads((DOC/'REPLAY_RECEIPT.json').read_text())
    assert after['status']=='R12_FROZEN_INPUTS_AND_PRODUCTION_UNCHANGED' and not after['differences']
    assert after['stable']==before['stable'] and receipt['status']=='OFFLINE_REPLAY_COMPLETED'
    for old in before['inputs']:assert identity(old['path'])==old
    for old in receipt['outputs']:
        assert identity(old['path'])==old
        original=Path(old['path'])
        target=DOC/original.name if original.parent.name=='formal' else DOC/'raw'/original.name
        assert identity(target)['sha256']==old['sha256']
    raw=rows(R11/'CANDIDATE_POOL.csv');mapping=rows(DOC/'RAW_TO_GROUP_MAPPING.csv');decisions=rows(DOC/'ACCEPTANCE_COMPARISON.csv')
    groups=rows(DOC/'REPRESENTATIVE_COMPARISON.csv');pairs=rows(DOC/'PAIRWISE_EVIDENCE.csv');runtime=rows(DOC/'RUNTIME_AND_MEMORY.csv')
    assert len(raw)==len(mapping)==212 and len(decisions)==len(runtime)==200
    assert len({r['native_call_id'] for r in raw})==112
    bykey={(r['arm'],r['scan_index'],r['candidate_id']):r for r in raw}
    for m in mapping:
        r=bykey[m['arm'],m['scan_index'],m['candidate_id']]
        assert all(r[k]==m[k] for k in ['raw_index','native_call_id','candidate_id'])
        assert r['duplicate_of']==m['old_duplicate_of'] and r['eligible']==m['raw_eligible']
        if m['raw_eligible']=='1':assert m['representative_eligible']=='1'
    block_rows=rows(R11/'BLOCK_LOSSES.csv')
    losses={(r['arm'],r['scan_index'],r['candidate_id'],r['block_id']):r for r in block_rows}
    original_pairs={(r['arm'],r['scan_index'],r['j'],r['k']):float(r['D_j_k']) for r in rows(R11/'PAIRWISE_EVIDENCE.csv')}
    for pair in pairs:
        d=[]
        for b in range(8):
            j=losses[pair['arm'],pair['scan_index'],pair['j'],str(b)]
            k=losses[pair['arm'],pair['scan_index'],pair['k'],str(b)]
            assert j['active']==k['active']
            if j['active']=='1':d.append(float(k['loss'])-float(j['loss']))
            else:assert j['loss']==k['loss']=='NA'
        assert len(d)==int(pair['B']) and abs(lp_vertex(d)-float(pair['D_j_k']))<1e-15
        if pair['variant']=='CONTROL':assert float(pair['D_j_k'])==original_pairs[pair['arm'],pair['scan_index'],pair['j'],pair['k']]
    assert sum(r['variant']=='CONTROL' for r in pairs)==len(original_pairs)
    arm_stats={}
    for arm in ['BASELINE_WEAK','PROPOSED_DISAGREEMENT']:
        frame=[r for r in decisions if r['arm']==arm];mm=[r for r in mapping if r['arm']==arm];gg=[r for r in groups if r['arm']==arm]
        assert [int(r['scan_index']) for r in frame]==list(range(1,101))
        assert [int(r['raw_index']) for r in frame]==list(range(10641,10741))
        assert all(r['FEASIBLE_REPRESENTATIVE_PRESERVED']=='1' for r in frame)
        arm_stats[arm]=dict(raw_rows=len(mm),raw_eligible=sum(int(r['raw_eligible']) for r in mm),
            old_discarded_eligible=sum(int(r['old_eligible_discarded']) for r in frame),
            old_eligible_representatives=sum(int(r['raw_eligible'])-int(r['old_eligible_discarded']) for r in frame),
            new_eligible_representatives=sum(int(r['new_eligible_representatives']) for r in frame),
            non_nominal_physical_groups=sum(int(r['PHYSICALLY_DISTINCT_FROM_NOMINAL']) for r in gg),
            robust_supported=sum(int(r['ROBUST_EVIDENCE_SUPPORTED']) for r in frame),
            accepted_replacements=sum(int(r['ACCEPTED_REPLACEMENT']) for r in frame),
            old_accept_nominal=sum(r['old_status']=='ACCEPT_NOMINAL' for r in frame),
            new_accept_nominal=sum(r['new_status']=='ACCEPT_NOMINAL' for r in frame),
            old_rejections=sum(r['old_status']=='NO_ACCEPTABLE_MEASUREMENT' for r in frame),
            new_rejections=sum(r['new_status']=='NO_ACCEPTABLE_MEASUREMENT' for r in frame),
            final_acceptance_changes=sum(r['old_selected']!=r['new_selected'] or r['old_status']!=r['new_status'] for r in frame),
            native_rank_changes=sum(r['old_native']!=r['new_native'] for r in frame),
            mean_rank_changes=sum(r['old_mean']!=r['new_mean'] for r in frame),
            robust_rank_changes=sum(r['old_robust']!=r['new_robust'] for r in frame))
    counter=[r for r in decisions if r['arm']=='PROPOSED_DISAGREEMENT' and r['scan_index']=='64'][0]
    assert counter['new_representative_ids']=='1' and counter['new_evidence_ids']=='0|1'
    assert counter['new_status']=='NO_ACCEPTABLE_MEASUREMENT' and counter['new_native']==counter['new_mean']=='1'
    d64=[r for r in pairs if r['variant']=='PROPOSED' and r['arm']=='PROPOSED_DISAGREEMENT' and r['scan_index']=='64' and r['j']=='1' and r['k']=='0'][0]
    assert abs(float(d64['D_j_k'])-(-.001052113786269879))<1e-15
    assert sum(v['raw_eligible'] for v in arm_stats.values())==191
    assert sum(v['old_discarded_eligible'] for v in arm_stats.values())==1
    assert sum(v['accepted_replacements'] for v in arm_stats.values())==0
    result=dict(task='P62-R12-FEASIBILITY-PRESERVING-CANDIDATE-POOL',START_SHA='169771c332cf5e00cc1b532421db1b0be70dc370',
        actual_run_CODE_SHA=receipt['actual_run_CODE_SHA'],actual_CXX_build_CODE_SHA=json.loads((DOC/'BUILD_PROVENANCE.json').read_text())['source_CODE_SHA'],
        analysis_CODE_SHA=subprocess.check_output(['git','rev-parse','HEAD'],cwd=DOC.parents[1],text=True).strip(),
        result='POOL_FEASIBILITY_REPAIRED_BUT_SUPPORTED_RECOVERY_NOT_ESTABLISHED',frames=100,arms=arm_stats,
        arm_candidate_rows=212,unique_native_outputs_reused=112,new_physical_candidates=0,
        pairwise_independent_checks=len(pairs),counterexample64=dict(representative=1,raw_nominal_anchor=0,D_1_0=float(d64['D_j_k']),accepted_replacement=False),
        engineering=dict(A1='PASS',A2='PASS',A3='PASS',A4='PASS: original duplicate()',A5='PASS: original paired(), strict >.02',
            A6='PASS',A7=after['status'],A8='PASS:27 C++ checks and5 Python mock checks'),
        science=dict(S1='SUPPORTED: original feasible row1 preserved as representative at64',S2='NONE: no optimizer run or new candidate',
            S3='NOT_ESTABLISHED: previously excluded pair made explicit; no new independent evidence',S4='UNCHANGED:95 nominal/5 rejected per arm'),
        runtime=dict(control_ms=stats([float(r['control_ms']) for r in runtime]),proposed_ms=stats([float(r['proposed_ms']) for r in runtime]),
            peak_replay_process_RSS_MiB=max(float(r['process_high_water_RSS_KiB']) for r in runtime)/1024,
            process_wall_s=receipt['process_exit']['wall_s'],quantile='linear/type7; descriptive, offline decision cost only'),
        boundaries=dict(new_NDT_align=0,rosbag_play=0,EKF_runs=0,GT_data_reads=0,new_trajectory_estimates=0,production_code_changes=0,
            new_seed_generation=0,repeat_Bundle_preservation=0,formal_offline_replay_processes=1),
        novelty='NOT_ESTABLISHED',push_executed=False,
        retained_history='R10 asset drift remains failed; R11 baseline float3.000000034180631deg deviation retained, not retroactively fixed',
        limitations=['Known frozen counterexample; not blind/generalization or accuracy evaluation',
            'Structural sectors are correlated, not independent truth',
            'Fixed-anchor greedy groups do not imply pairwise member or promoted-representative proximity',
            'Representative promotion is not a different NDT basin',
            'Runtime excludes NDT, geometry optimization and physical localization'],
        unique_NEXT='Controller to pre-register one bounded mechanism for physically different NDT hypotheses together with an independent correctness evaluation contract; no further guard/dedup-threshold tuning or experiment now')
    save('results.json',result)
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
