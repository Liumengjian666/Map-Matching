#!/usr/bin/env python3
"""Summarize R21 sealed evaluation; never access original reference values."""
import csv,hashlib,json,re
from pathlib import Path
from collections import defaultdict
import numpy as np
HERE=Path(__file__).resolve().parents[1]
R20=HERE.parent/'p62_r20_quality_gated_recovery'
D='SINGLE_FRAME_RELATIVE_ADMISSION';E='TWO_FRAME_SHADOW_CONFIRMATION'
def rows(p):
    with p.open(newline='') as f:return list(csv.DictReader(f))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def dist(values):
    x=np.asarray(values,dtype=float)
    return {'n':len(x),'mean':float(x.mean()),'p95':float(np.percentile(x,95)),'max':float(x.max())} if len(x) else {'n':0}
def ident(r):return r['role'],r['arm'],r['policy'],r['frame']
def main():
    summary=json.loads((HERE/'EVALUATION_SUMMARY.json').read_text());identity=json.loads((HERE/'INPUT_IDENTITY.json').read_text())
    native=rows(HERE/'CAUSAL_RESULTS.csv');cand=rows(HERE/'RELATIVE_ADMISSION_LOG.csv');refs=rows(HERE/'REFERENCE_ERROR.csv');rt=rows(HERE/'RUNTIME_AND_MEMORY.csv');calls=rows(HERE/'ALIGN_CALLS.csv')
    old=rows(R20/'CAUSAL_RESULTS.csv');fields=[f'output{i}{j}' for i in range(4) for j in range(4)]
    equal_r20={}
    for role in ('WEAK','STRONG'):
        for arm in ('C0','C3'):
            baseline=[r for r in old if (r['role'],r['arm'],r['policy'])==(role,arm,'QUALITY_TRIGGERED_MULTISTART')]
            for p in (D,E):
                cur=[r for r in native if (r['role'],r['arm'],r['policy'])==(role,arm,p)]
                equal_r20[f'{role}/{arm}/{p}']=all(a['frame']==b['frame'] and all(a[f]==b[f] for f in fields) for a,b in zip(baseline,cur))
    refmap={(ident(r),r['estimate']):r for r in refs};byframe=defaultdict(list)
    for c in cand:byframe[ident(c)].append(c)
    chain=[]
    for r in native:
        if r['role']!='STRONG' or r['arm']!='C3':continue
        err=refmap[(ident(r),'CAUSAL_OUTPUT')]
        chain.append({k:r[k] for k in ('policy','frame','raw_ordinal','stamp_ns','prediction_mode','history_last_stamp_ns','quality_event','budget_before','extra_aligns','budget_after','selected_id','admission_reason','accepted_observation','pending_in','pending_created','pending_out','confirmation_attempted','confirmation_success','confirmation_failed')})
        chain[-1].update({'output_reference_translation_m':err['translation_error_m'],'output_reference_rotation_deg':err['rotation_error_deg'],
            'candidates_json':json.dumps([{k:c[k] for k in ('candidate_id','seed_kind','native_score','overlap_0p5','heldout_mean','basic_quality','quality_gate_accepted','pareto_dominates','chosen')} for c in byframe[ident(r)]],separators=(',',':'))})
    with (HERE/'STRONG_C3_COMPLETE_CHAIN.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(chain[0]),lineterminator='\n');w.writeheader();w.writerows(chain)
    assert len(chain)==120
    process={}
    for role in ('WEAK','STRONG'):
        text=(HERE/role.lower()/'FORMAL_RUN_01.log').read_text()
        m=re.search(r'R21_NATIVE_COMPLETE .*wall_ms=([0-9.]+) peak_rss_KiB=([0-9]+)',text)
        process[role]={'wall_ms':float(m[1]),'peak_rss_KiB':int(m[2])}
    extra=[r for r in calls if r['kind']!='NOMINAL_FULL'];confirm=[r for r in calls if r['kind']=='CONFIRMATION_FULL']
    summary.update({'status':'SINGLE_FRAME_RELATIVE_ADMISSION_TARGET_MET_ON_DEVELOPMENT_SET; TEMPORAL_CONFIRMATION_BENEFIT_NOT_ESTABLISHED',
        'start_sha':identity['start_sha'],'code_sha':identity['code_sha'],'binary_sha256':identity['binary_sha256'],
        'branch':'research/p62-r21-relative-temporal-admission','native_output_seal_commit':'ac27755',
        'end_sha_location':'Final enclosing Git commit reported in handoff; no self-referential hash',
        'new_native_calls':{'nominal':480,'search':len(extra)-len(confirm),'confirmation':len(confirm),'total':len(calls),'budget':576,'split':0,'ekf':0,'historical_rerun':0},
        'r20_multistart_exact_output_reproduction':equal_r20,
        'all_frames_wall_ms':dist([float(r['frame_wall_ms']) for r in rt if r['frame']!='0']),
        'event_frames_wall_ms':dist([float(r['frame_wall_ms']) for r in rt if r['event']=='1']),
        'extra_align_total_ms':sum(float(r['align_ms']) for r in extra),'confirmation_align_total_ms':sum(float(r['align_ms']) for r in confirm),
        'processes':process,'map_init_ms':{r['role']:float(r['map_init_ms']) for r in rt if r['frame']=='0'},
        'mechanism':{'direct_relative_admissions':sum(r['admission_reason']=='RELATIVE_ONLY_ADMISSION' for r in native),
            'pending_created':sum(r['pending_created']=='1' for r in native),'confirmation_attempts':len(confirm),
            'confirmation_success':sum(r['confirmation_success']=='1' for r in native),'confirmation_failed':sum(r['confirmation_failed']=='1' for r in native)},
        'hypotheses':{'H1':'SUPPORTED_ON_THIS_DEVELOPMENT_CASE: one direct relative admission improves strong C3 causal trajectory and meets both targets',
            'H2':'NOT_ESTABLISHED: E has higher strong C3 translation/rotation RMSE than D and no reduction of bad-output frames',
            'H3':'OBSERVED_COST_AND_DELAY: one extra confirmation align, 100.502016ms scan delay and one pending-frame fallback; confirmed relative pose still has 6.959934deg reference error'},
        'innovation':'NOT_ESTABLISHED; retain simple D prototype, do not promote temporal confirmation',
        'next':'Freeze D as the simpler baseline and stop E development; ask the research lead to decide a separately preregistered independent confirmation and real LiDAR-IMU causal evaluation, without executing it here',
        'preservation':{'online_inputs_unchanged':True,'production_snapshot_unchanged':True,'historical_archives_modified':False},
        'key_sha256':{p:sha(HERE/p) for p in ['METHOD.md','INPUT_IDENTITY.json','NATIVE_OUTPUT_FREEZE.json','CAUSAL_RESULTS.csv','RELATIVE_ADMISSION_LOG.csv','SHADOW_CONFIRMATION_LOG.csv','REFERENCE_ERROR.csv']}})
    with (HERE/'results.json').open('w') as f:json.dump(summary,f,indent=2,allow_nan=False);f.write('\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,7),sharex='col')
    policies=('BASELINE','QUALITY_GATE_ONLY','QUALITY_TRIGGERED_MULTISTART',D,E)
    labels=('R19 baseline','R20 gate only','R20 multistart','R21 direct relative','R21 temporal confirmation')
    for col,role in enumerate(('WEAK','STRONG')):
        for p,label in zip(policies,labels):
            a=[r for r in refs if r['role']==role and r['arm']=='C3' and r['policy']==p and r['estimate']=='CAUSAL_OUTPUT']
            for row,field in enumerate(('translation_error_m','rotation_error_deg')):
                axes[row,col].plot([int(r['frame']) for r in a],[float(r[field]) for r in a],label=label,alpha=.9)
        axes[0,col].set_title(role+' C3 (+8 deg anchor)');axes[1,col].set_xlabel('Evaluation frame (1-60)')
        for ax in axes[:,col]:ax.grid(alpha=.25)
    axes[0,0].set_ylabel('Reference-relative translation (m)');axes[1,0].set_ylabel('Reference-relative rotation (deg)')
    axes[0,0].legend(fontsize=7);fig.suptitle('R21 developmental comparison; map-anchored conditional benchmark reference');fig.tight_layout()
    fig.savefig(HERE/'C3_REFERENCE_ERRORS.png',dpi=150);plt.close(fig)
    print('R21_SUMMARY',summary['status'],summary['new_native_calls'])
if __name__=='__main__':main()
