#!/usr/bin/env python3
"""Post-freeze exploratory evaluation only; no GT and no threshold fitting."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time

import numpy as np
import p9_r4_contract as c
from p9_r6_nearoptimal import OUT, R5, START, GAMMA, LIB, LIB_SHA, tensor
import p9_r2b_nonoracle_evidence as g

CASES=(2932,3813,3929,2398,1944,3783,2374,184,655,3200,2675,2889,3899,2483,4110,542,3829)
CONTROLS=(3147,3951,167)


def verify_freeze():
    receipt=json.loads((OUT/'evidence_freeze.json').read_text())
    c.require(receipt['state']=='EVIDENCE_FROZEN_BEFORE_LABELS' and receipt['start_sha']==START
        and receipt['gamma']==GAMMA and not receipt['ORACLE_LABELS_LOADED_BY_EVIDENCE']
        and not receipt['GT_LOADED'],'invalid blind freeze')
    for name,sha in receipt['artifacts'].items():c.require(c.digest(OUT/name)==sha,'evidence changed: '+name)
    for name,sha in receipt['input_sha256'].items():c.require(c.pinned(c.ROOT/name,START)==sha,'input changed')
    for name,sha in receipt['frozen_source_sha256'].items():c.require(c.pinned(c.HERE/name,START)==sha,'helper changed')
    c.require(c.digest(c.HERE/'p9_r6_nearoptimal.py')==receipt['code_sha256'] and
        c.digest(OUT/'THEORY.md')==receipt['theory_sha256'] and c.digest(LIB)==LIB_SHA,'code/chart/theory changed')
    return receipt


def auc(positive,negative):
    a=np.asarray(positive);b=np.asarray(negative)
    c.require(len(a)>0 and len(b)>0,'missing class')
    return float(np.mean((a[:,None]>b[None,:])+.5*(a[:,None]==b[None,:])))


def distribution(values):
    v=np.asarray(values,float)
    return dict(mean=float(v.mean()),median=float(np.median(v)),p05=float(np.quantile(v,.05)),
        p95=float(np.quantile(v,.95)),minimum=float(v.min()),maximum=float(v.max()),zero_count=int(sum(v==0)))


def evaluate():
    start=time.perf_counter();receipt=verify_freeze()
    # First label access occurs only after all frozen artifacts and code pass above.
    inputs=(c.OUT/'oracle/oracle_labels.csv',c.OUT/'evaluation/candidate_per_frame.csv',
            R5/'full263_strict_candidate.csv')
    hashes={str(p.relative_to(c.ROOT)):c.pinned(p,START) for p in inputs}
    labels={int(r['transaction_id']):r['label'] for r in c.read_csv(inputs[0])}
    c.require(Counter(labels.values())=={'MAJOR':40,'NO_MAJOR':56},'cohort label count mismatch')
    old={'B12':{int(r['frame']):int(r['strict_candidate_count']) for r in c.read_csv(inputs[1])},
         'FULL263':{int(r['frame']):int(r['strict_candidate_count']) for r in c.read_csv(inputs[2])}}
    rows=c.read_csv(OUT/'frame_level_evidence.csv');parity=[];stats=[];distributions=[];transition=[];controls=[]
    for method,counts in (('B12',(17,3)),('FULL263',(30,5))):
        selected=[r for r in rows if r['method']==method]
        c.require(len(selected)==96 and {int(r['frame']) for r in selected}==set(labels),'evidence cohort mismatch')
        byclass={label:[r for r in selected if labels[int(r['frame'])]==label] for label in ('MAJOR','NO_MAJOR')}
        for r in selected:
            tx=int(r['frame']);n=int(r['strict_count'])
            c.require(n==old[method][tx],'STRICT per-frame parity fail')
            parity.append(dict(frame=tx,method=method,label=labels[tx],expected=old[method][tx],actual=n,parity='PASS'))
        actual=tuple(sum(int(r['strict_count'])>0 for r in byclass[label]) for label in ('MAJOR','NO_MAJOR'))
        c.require(actual==counts,'STRICT aggregate parity fail')
        predicates={'STRICT':lambda r:int(r['strict_count'])>0,'TYPE_A':lambda r:r['event_type']=='TYPE_A',
                    'TYPE_B':lambda r:r['event_type']=='TYPE_B','UNION':lambda r:r['event_type']!='NONE'}
        for event,predicate in predicates.items():
            pos=sum(predicate(r) for r in byclass['MAJOR']);neg=sum(predicate(r) for r in byclass['NO_MAJOR'])
            stats.append(dict(method=method,event=event,major_positive=pos,major_total=40,no_major_positive=neg,
                no_major_total=56,sensitivity_proxy=pos/40,specificity_proxy=1-neg/56,
                false_alert_rate_proxy=neg/56,balanced_accuracy_event=(pos/40+1-neg/56)/2,
                interpretation='EXPLORATORY_ONLY_NOT_CONFIRMATORY'))
        for score in ('U_near','U_disfavored'):
            values={label:[float(r[score]) for r in group] for label,group in byclass.items()}
            distributions.append(dict(method=method,score=score,AUC=auc(values['MAJOR'],values['NO_MAJOR']),
                MAJOR=distribution(values['MAJOR']),NO_MAJOR=distribution(values['NO_MAJOR'])))
        for r in selected:
            tx=int(r['frame']);detail=dict(frame=tx,method=method,label=labels[tx],old_strict_positive=int(int(r['strict_count'])>0),
                event_type=r['event_type'],eligible_clusters=int(r['eligible_cluster_count']),
                separated_clusters=int(r['separated_cluster_count']),eligible_outside_members=int(r['eligible_outside_member_count']),
                single_separated_terminal=int(int(r['eligible_outside_member_count'])==1),
                multiple_separated_clusters=int(int(r['eligible_cluster_count'])>1),
                boundary_clusters=int(r['boundary_cluster_count']),suppressed_clusters=int(r['suppressed_cluster_count']),
                nominal_near=int(r['nominal_near']),U_near=float(r['U_near']),U_disfavored=float(r['U_disfavored']))
            if tx in CASES+CONTROLS:transition.append(detail)
            if labels[tx]=='NO_MAJOR':controls.append(detail)
    c.write_csv(OUT/'strict_parity.csv',parity)
    c.write_csv(OUT/'major_no_major_statistics.csv',stats)
    c.write_csv(OUT/'r5_failure_case_transition.csv',transition)
    c.write_csv(OUT/'no_major_safety.csv',controls)
    results=dict(task='PAPER-P9-R6-NONLOCAL-AMBIGUITY-ADMISSION-FORMULATION',
        status='POSTHOC_FORMULATION_DEVELOPMENT',confirmatory=False,start_sha=START,branch=c.BRANCH,
        NEW_NDT_CALLS=0,BASELINE_REPLAY_CALLS=0,VISUAL_EXTRACTION=0,GT_LOADED=False,
        gamma=GAMMA,EPS_SCORE=c.EPS_SCORE,statistics=stats,score_distributions=distributions,
        transitions=transition,controls=controls,offline_build_wall_seconds=receipt['offline_build_wall_seconds'],
        evaluation_wall_seconds=time.perf_counter()-start,evidence_freeze_sha256=c.digest(OUT/'evidence_freeze.json'),
        label_input_sha256=hashes,R4_FINAL_RESULT='HELDOUT_LIDAR_CANDIDATE_GENERATOR_NOT_GENERALIZED',
        R5_FINAL_RESULT='MIXED_CANDIDATE_BOTTLENECK')
    c.save_json(OUT/'exploratory_evaluation.json',results)
    print(json.dumps(dict(statistics=stats,score_distributions=distributions),indent=2))


def self_test():
    c.require(auc([1,1],[1,1])==.5 and auc([2,3],[0,1])==1 and auc([0],[1])==0,'AUC tie/sign failure')
    c.require(auc([0,1],[0,2])==.375,'AUC mixed ties failure')
    print('P9_R6_EXPLORATORY_STATISTICS_SELF_TEST=PASS')


def report(result):
    lines=['# R6 nonlocal admission formulation development','',
        'FINAL_RESULT = `'+result['FINAL_RESULT']+'`','',
        'NEXT = `'+result['NEXT']+'`','',
        'POSTHOC FORMULATION DEVELOPMENT. EXPLORATORY ONLY, NOT CONFIRMATORY.',
        'R4 FAIL and R5 MIXED remain unchanged. No new held-out claim is made.','',
        '## Frozen rule and information boundary','',
        'gamma=0.05. Near-optimal: (S_star-S_k)/N <= 0.05*max(1,abs(S_star/N)).',
        'Each pool supplies its own S_star=max(S0, observed representative scores).',
        'TYPE A requires T0 in band; TYPE B requires T0 outside band. Both require',
        'a near-optimal non-nominal representative separated from T0 by >0.2m OR >2deg.',
        'The original converged-only complete-link memberships and maximum-score representatives',
        'are reused byte-pinned and verified; mixed inside/outside clusters are flagged, not changed.',
        'T0 uses its original S0, not the nominal cluster representative score.',
        'A_near and A_disfavored remain separate sums of xi xi^T in the frozen P9 spatial chart.',
        'They are directional evidence tensors, not covariances or probabilities.',
        'The read guard and evidence_freeze.json record the label-blind build; labels are opened',
        'only by the separate post-freeze evaluator. No canonical basin pose/ID or GT is read.','',
        '## Event rates','',
        '| Pool | Event | MAJOR | NO_MAJOR | Sensitivity proxy | Specificity proxy |',
        '| --- | --- | ---: | ---: | ---: | ---: |']
    for r in result['statistics']:
        lines.append(f"| {r['method']} | {r['event']} | {r['major_positive']}/40 | {r['no_major_positive']}/56 | "
                     f"{r['sensitivity_proxy']:.6f} | {r['specificity_proxy']:.6f} |")
    lines+=['','Original STRICT is exact: B12 17/40 and 3/56; FULL263 30/40 and 5/56.',
        'NO_MAJOR is only the frozen oracle proxy, not proof of no optimizer ambiguity.',
        'Thus false-alert rate means NO_MAJOR event rate, not independently verified false positives.','',
        '## Separate scalar channels (exploratory only)','',
        '| Pool | Score | AUC | MAJOR mean/median | NO_MAJOR mean/median |',
        '| --- | --- | ---: | --- | --- |']
    for r in result['score_distributions']:
        lines.append(f"| {r['method']} | {r['score']} | {r['AUC']:.9f} | {r['MAJOR']['mean']:.6f} / "
            f"{r['MAJOR']['median']:.6f} | {r['NO_MAJOR']['mean']:.6f} / {r['NO_MAJOR']['median']:.6f} |")
    lines+=['','P05/P95, minima/maxima and zero masses are in results.json.',
        'No U_near+U_disfavored fused score or fitted scalar threshold is evaluated.','',
        '## R5 failures and NO_MAJOR controls','',
        '| Frame | B12 event | B12 eligible clusters | FULL263 event | FULL263 eligible clusters |',
        '| --- | --- | ---: | --- | ---: |']
    by={(r['frame'],r['method']):r for r in result['transitions']}
    for tx in CASES+CONTROLS:
        a=by[tx,'B12'];b=by[tx,'FULL263']
        lines.append(f"| {tx} | {a['event_type']} | {a['eligible_clusters']} | {b['event_type']} | {b['eligible_clusters']} |")
    lines+=['','For the 17 ID-disagreement frames, B12 transitions are: `'+json.dumps(result['failure_case_counts'],sort_keys=True)+'`.',
        'Boundary/suppression flags, raw supporting member counts and one-versus-multiple cluster',
        'indicators are retained in r5_failure_case_transition.csv and no_major_safety.csv.',
        'Distinct terminal clusters are not certified independent attractors.',
        '2932 still has no B12 event and has a boundary-straddling suppressed cluster.',
        '3829 retains a suppression flag but now has TYPE A evidence from other unchanged representatives.',
        'The pool-dependent A/B changes in the controls are caused by different observed S_star and',
        'terminal sets; FULL263 never supplies B12 with a best score or extra candidate.','',
        'All originally STRICT-positive NO_MAJOR transitions:','',
        '| Pool | Frame | New event | Eligible clusters | Eligible outside members |',
        '| --- | --- | --- | ---: | ---: |']
    lines += [f"| {v['method']} | {v['frame']} | {v['event_type']} | {v['eligible_clusters']} | {v['eligible_outside_members']} |"
              for v in result['strict_positive_control_transitions']]
    lines += ['', 'NO_MAJOR positive support structure: `'+json.dumps(result['no_major_support_structure'],sort_keys=True)+'`.', '',
        '## Descriptive conclusion, not a new acceptance gate','',
        result['classification_basis'],
        'This PROMISING classification is limited to fixed B12 formulation development. R6 specified',
        'no numerical acceptance gate; no new one is invented. U_near AUC is below 0.80, and none of',
        'the old R4 gates are reinterpreted or declared passed.',
        'B12 union false-alert proxy rises from 3/56 to 12/56. FULL263 union reaches 34/56 NO_MAJOR',
        '(60.7%); therefore FULL263 binary admission is not a successful safety discriminator.',
        'More sampling changes S_star, event type, and the number of near-optimal clusters. Neither',
        'coverage nor the unnormalized tensor is budget-invariant. Larger FULL263 evidence must not',
        'be interpreted as calibrated confidence or an online improvement.',
        'The sole next step is validation on a new dataset with the same frozen formulation. No',
        'gamma tuning on these frames, pose switching, EKF or fusion is justified.','',
        '## Cost and verification','',
        f"Blind hash/load and evidence build wall: {result['offline_build_wall_seconds']:.6f} s.",
        f"Post-freeze evaluation wall: {result['evaluation_wall_seconds']:.6f} s.",
        'Cached-cluster verification/evidence times are in runtime_breakdown.csv and results.json.',
        'No cluster recomputation was performed; these are not production online timings.',
        'Historical B12 NDT alignment remains about827ms/frame. Admission does not solve low-compute feasibility.',
        'NEW_NDT_CALLS=0; BASELINE_REPLAY_CALLS=0; VISUAL_EXTRACTION=0; GT_LOADED=NO.',
        'Release/P9 and numerical/isolation checks are recorded in verification/.',
        'The containing commit supplies end_sha; R4/R5 archives are unchanged.','']
    return '\n'.join(lines)


def finalize():
    verify_freeze()
    result=json.loads((OUT/'exploratory_evaluation.json').read_text())
    result.update(FINAL_RESULT='NEAROPTIMAL_ADMISSION_PROMISING_EXPLORATORY',
        NEXT='CROSS_DATASET_NEAROPTIMAL_ADMISSION_VALIDATION',
        classification_basis='B12 TYPE A has 27/40 MAJOR versus 10/56 NO_MAJOR events, with U_near AUC '
        '0.777679 and higher MAJOR amplitudes. TYPE B is uncommon (3/40 and 2/56), not the dominant '
        'event. This is a directional exploratory signal, offset by increased proxy false alerts '
        'and severe FULL263 binary saturation. It motivates new-data validation only, not established usability.')
    result['failure_case_counts']=dict(Counter(r['event_type'] for r in result['transitions'] if r['method']=='B12' and r['frame'] in CASES))
    result['strict_positive_control_transitions']=[r for r in result['controls'] if r['old_strict_positive']]
    result['no_major_support_structure']={method:dict(
        single_outside_terminal=sum(v['single_separated_terminal'] for v in result['controls'] if v['method']==method),
        one_eligible_cluster=sum(v['eligible_clusters']==1 for v in result['controls'] if v['method']==method),
        multiple_eligible_clusters=sum(v['multiple_separated_clusters'] for v in result['controls'] if v['method']==method))
        for method in ('B12','FULL263')}
    times=c.read_csv(OUT/'runtime_breakdown.csv')
    result['runtime']={method:{key:dict(total_seconds=sum(float(r[key]) for r in times if r['method']==method),
        **distribution([float(r[key]) for r in times if r['method']==method]))
        for key in ('cached_partition_verification_seconds','representative_verification_and_evidence_seconds')}
        for method in ('B12','FULL263')}
    c.require(report(result)==report(json.loads(json.dumps(result,sort_keys=True))), 'report depends on JSON key order')
    c.save_json(OUT/'results.json',result)
    (OUT/'REPORT.md').write_text(report(result))
    c.save_json(OUT/'artifact_hashes.json',dict(artifacts={str(p.relative_to(OUT)):c.digest(p) for p in OUT.rglob('*')
        if p.is_file() and p.name!='artifact_hashes.json'},sources={str(p.relative_to(c.ROOT)):c.digest(p)
        for p in (Path(__file__).resolve(),c.HERE/'p9_r6_nearoptimal.py')}))
    audit()


def audit():
    verify_freeze()
    r=json.loads((OUT/'results.json').read_text());h=json.loads((OUT/'artifact_hashes.json').read_text())
    files={str(p.relative_to(OUT)):p for p in OUT.rglob('*') if p.is_file() and p.name!='artifact_hashes.json'}
    c.require(set(files)==set(h['artifacts']),'artifact inventory mismatch')
    for name,p in files.items():c.require(c.digest(p)==h['artifacts'][name],'artifact hash mismatch')
    for name,sha in h['sources'].items():c.require(c.digest(c.ROOT/name)==sha,'source hash mismatch')
    for name,sha in r['label_input_sha256'].items():c.require(c.pinned(c.ROOT/name,START)==sha,'label input changed')
    c.require((OUT/'REPORT.md').read_text()==report(r),'REPORT/results mismatch')
    for name,key in (('major_no_major_statistics.csv','statistics'),('r5_failure_case_transition.csv','transitions'),
                     ('no_major_safety.csv','controls')):
        c.require(c.read_csv(OUT/name)==[{k:str(v) for k,v in row.items()} for row in r[key]],'CSV/JSON mismatch: '+name)
    frames=c.read_csv(OUT/'frame_level_evidence.csv');candidates=c.read_csv(OUT/'nearoptimal_candidates.csv')
    labels={int(x['transaction_id']):x['label'] for x in c.read_csv(c.OUT/'oracle/oracle_labels.csv')}
    c.require(len(frames)==192 and len({(f['frame'],f['method']) for f in frames})==192,'frame inventory mismatch')
    for f in frames:
        cs=[v for v in candidates if v['frame']==f['frame'] and v['method']==f['method']]
        vs=[g.vector(v['xi'],6) for v in cs if v['near_eligible']=='1']
        for event,channel in (('TYPE_A','near'),('TYPE_B','disfavored')):
            a,u,t=tensor(vs if f['event_type']==event else [])
            c.require(a==f['A_'+channel] and u==float(f['U_'+channel]) and t==float(f['trace_'+channel]),'tensor CSV mismatch')
        c.require(len(vs)==int(f['eligible_cluster_count']),'eligible cluster count mismatch')
    for s in r['statistics']:
        selected=[f for f in frames if f['method']==s['method']]
        def positive(f):
            return int(f['strict_count'])>0 if s['event']=='STRICT' else f['event_type']!='NONE' if s['event']=='UNION' else f['event_type']==s['event']
        for label,key in (('MAJOR','major'),('NO_MAJOR','no_major')):
            c.require(sum(positive(f) for f in selected if labels[int(f['frame'])]==label)==s[key+'_positive'],'event count mismatch')
    for s in r['score_distributions']:
        values={label:[float(f[s['score']]) for f in frames if f['method']==s['method'] and labels[int(f['frame'])]==label]
                for label in ('MAJOR','NO_MAJOR')}
        c.require(auc(values['MAJOR'],values['NO_MAJOR'])==s['AUC'],'AUC CSV mismatch')
        c.require(all(distribution(v)==s[label] for label,v in values.items()),'distribution mismatch')
    print('P9_R6_CSV_JSON_TENSOR_HASH_AUDIT=PASS')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('evaluate','self-test','finalize','audit'))
    args=parser.parse_args()
    {'evaluate':evaluate,'self-test':self_test,'finalize':finalize,'audit':audit}[args.action]()
