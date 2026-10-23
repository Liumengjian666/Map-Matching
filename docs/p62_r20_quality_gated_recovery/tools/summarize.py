#!/usr/bin/env python3
"""Summarize sealed/evaluated R20 data; never access raw benchmark reference."""
import csv,hashlib,json,re
from pathlib import Path
import numpy as np
from baseline_stats import distribution

HERE=Path(__file__).resolve().parents[1]
REPO=HERE.parents[1]
OLD=HERE.parent/'p62_r19_native_ndt_basin_stress_test'
def rows(path):
    with path.open(newline='') as f:return list(csv.DictReader(f))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    evaluation=json.loads((HERE/'EVALUATION_SUMMARY.json').read_text())
    identity=json.loads((HERE/'INPUT_IDENTITY.json').read_text())
    seal=json.loads((HERE/'NATIVE_OUTPUT_FREEZE.json').read_text())
    causal=rows(HERE/'CAUSAL_RESULTS.csv');runtime=rows(HERE/'RUNTIME_AND_MEMORY.csv');calls=rows(HERE/'ALIGN_CALLS.csv')
    candidate=rows(HERE/'EVENT_AND_CANDIDATE_LOG.csv');ce=rows(HERE/'REFERENCE_CANDIDATE_ERROR.csv')
    index=lambda r:(r['role'],r['arm'],r['policy'],r['frame'],r['candidate_id'])
    errors={index(r):r for r in ce}
    g=[r for r in causal if r['role']=='STRONG' and r['arm']=='C3' and r['policy']=='QUALITY_GATE_ONLY']
    m=[r for r in causal if r['role']=='STRONG' and r['arm']=='C3' and r['policy']=='QUALITY_TRIGGERED_MULTISTART']
    matrix_fields=[f'{prefix}{i}{j}' for prefix in ('tp','raw','selected','output') for i in range(4) for j in range(4)]
    exact=all(a['frame']==b['frame'] and all(a[k]==b[k] for k in matrix_fields) for a,b in zip(g,m))
    assert len(g)==len(m)==60
    failure=[c for c in candidate if c['role']=='STRONG' and c['arm']=='C3' and c['candidate_id']!='0']
    low=[{**c,**errors[index(c)]} for c in failure if float(errors[index(c)]['translation_error_m'])<=.3 and float(errors[index(c)]['rotation_error_deg'])<=3]
    both_better=0
    for c in failure:
        err=errors[index(c)]; nk=index(c)[:-1]+('0',); nominal=errors[nk]
        both_better+=float(err['translation_error_m'])<float(nominal['translation_error_m']) and float(err['rotation_error_deg'])<float(nominal['rotation_error_deg'])
    typical=next(c for c in failure if c['raw_ordinal']=='10973' and c['candidate_id']=='2')
    typical_err=errors[index(typical)]
    process={}
    for role in ('WEAK','STRONG'):
        log=(HERE/role.lower()/'FORMAL_RUN_01.log').read_text()
        match=re.search(r'R20_NATIVE_COMPLETE .*wall_ms=([0-9.]+) peak_rss_KiB=([0-9]+)',log)
        process[role]={'wall_ms':float(match[1]),'peak_rss_KiB':int(match[2])}
    frame_rt=[r for r in runtime if r['frame']!='0']
    extra=[r for r in calls if r['kind']=='EXTRA_FULL']
    result=dict(evaluation)
    result.update({
      'status':'DEVELOPMENT_SET_MIXED_CAUSAL_EFFECT; STRONG_C3_RECOVERY_TARGET_NOT_MET',
      'start_sha':identity['start_sha'],'code_sha':identity['code_sha'],
      'branch':'research/p62-r20-quality-gated-recovery',
      'binary_sha256':identity['binary_sha256'],
      'native_seal_commit':'a794bd2','evaluation_code_commit':'f588758',
      'git_end_identity':'Final enclosing Git commit, reported in handoff; not self-hashed inside this file',
      'strong_c3_gate_vs_multistart_all_poses_exactly_equal':exact,
      'strong_c3_extra_failure':{'candidates':len(failure),
          'all_basic_quality_pass':all(c['basic_quality']=='1' for c in failure),
          'all_rejected_by_heldout_gate':all(c['quality_gate_accepted']=='0' and float(c['heldout_mean'])>.30 for c in failure),
          'heldout_mean':distribution([c['heldout_mean'] for c in failure]),
          'overlap':distribution([c['overlap_0p5'] for c in failure]),
          'reference_low_le_0p3m_3deg_count':len(low),
          'extra_candidates_improve_both_components_vs_nominal':both_better,
          'illustrative_rejected_extra':{'raw_ordinal':10973,'candidate_id':2,'heldout_mean':float(typical['heldout_mean']),
             'overlap':float(typical['overlap_0p5']),'native_score':float(typical['native_score']),
             'reference_translation_m':float(typical_err['translation_error_m']),'reference_rotation_deg':float(typical_err['rotation_error_deg'])}},
      'native_calls':{'nominal':480,'extra':len(extra),'total':len(calls),'budget':528,'split':0,'ekf':0,'baseline_rerun':0},
      'extra_ndt_total_ms':sum(float(r['align_ms']) for r in extra),
      'runtime_overall':{'frame_wall_ms':distribution([r['frame_wall_ms'] for r in frame_rt]),
          'searched_event_frame_wall_ms':distribution([r['frame_wall_ms'] for r in frame_rt if int(r['extra_aligns'])>0]),
          'processes':process,'map_init_ms':{r['role']:float(r['map_init_ms']) for r in runtime if r['frame']=='0'}},
      'preservation':{'frozen_inputs_unchanged':seal['all_frozen_inputs_unchanged'],
          'production_snapshot_unchanged':seal['stable_snapshot_unchanged'],'historical_files_modified':False},
      'hypotheses':{
          'H1':'SUPPORTED_ON_DEVELOPMENT_DATA: 21/21 historical suspect alerts plus 14/459 other alerts; early alarms in causal C3',
          'H2':'SCENE_DEPENDENT: strong C3 improves, weak C3 gate-only freezes all 60 outputs and worsens greatly',
          'H3':'PARTIAL: weak C3 multistart improves vs gate-only and baseline; strong C3 adds no trajectory benefit and misses both targets'},
      'novelty':'NOT_ESTABLISHED; simple threshold and fixed multi-start are comparison baselines',
      'next':'Submit the observed fallback lock and strong-C3 rejection evidence to the research lead to decide whether a separately preregistered causal prediction/recovery experiment is warranted; retain gate-only and fixed multistart as baselines; no further run in R20',
      'key_sha256':{'input_identity':sha(HERE/'INPUT_IDENTITY.json'),'native_output_freeze':sha(HERE/'NATIVE_OUTPUT_FREEZE.json'),
          'causal_results':sha(HERE/'CAUSAL_RESULTS.csv'),'candidate_log':sha(HERE/'EVENT_AND_CANDIDATE_LOG.csv'),
          'reference_error':sha(HERE/'REFERENCE_ERROR.csv'),'method':sha(HERE/'METHOD.md')}
    })
    with (HERE/'results.json').open('w') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    # Standalone research figure, generated only from post-freeze evaluation CSVs.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    refs=rows(HERE/'REFERENCE_ERROR.csv')
    fig,axes=plt.subplots(2,2,figsize=(12,7),sharex='col')
    policies=['BASELINE','QUALITY_GATE_ONLY','QUALITY_TRIGGERED_MULTISTART']
    labels=['R19 baseline','Quality gate only','Quality + multistart']
    for col,role in enumerate(('WEAK','STRONG')):
        for policy,label in zip(policies,labels):
            rs=[r for r in refs if r['role']==role and r['arm']=='C3' and r['policy']==policy and r['estimate']=='CAUSAL_OUTPUT']
            axes[0,col].plot([int(r['frame']) for r in rs],[float(r['translation_error_m']) for r in rs],label=label)
            axes[1,col].plot([int(r['frame']) for r in rs],[float(r['rotation_error_deg']) for r in rs],label=label)
        axes[0,col].set_title(role+' C3, +8 deg at anchor')
        axes[1,col].set_xlabel('Evaluation frame (1-60)')
        axes[0,col].set_ylabel('Reference-relative translation (m)')
        axes[1,col].set_ylabel('Reference-relative rotation (deg)')
        for ax in axes[:,col]:ax.grid(alpha=.25)
    axes[0,0].legend(fontsize=8)
    fig.suptitle('R20 development-set causal results; map-anchored benchmark reference')
    fig.tight_layout();fig.savefig(HERE/'C3_REFERENCE_ERRORS.png',dpi=150);plt.close(fig)
    print('R20_SUMMARY',result['status'],'native_calls',len(calls),'strong_c3_equal',exact)
if __name__=='__main__':main()
