"""Posthoc P18 evaluation: frozen output gate, original Control transform."""
import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path
import numpy as np

SCRIPT=Path(__file__).resolve().parent
sys.path.insert(0,str(SCRIPT.parent/'p10_corridor01_deterministic_oosm'))
import evaluate_frozen_v2 as frozen
G=frozen.geometry


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1<<20),b''):h.update(block)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--freeze',type=Path,required=True)
    p.add_argument('--gt',type=Path,required=True);p.add_argument('--extrinsics',type=Path,required=True)
    p.add_argument('--visual-applications',type=Path,
                   help='Optional application ledger; defaults to the historical P18/P19 location')
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    receipt=json.loads(args.freeze.read_text())
    if receipt['GT_LOADED'] is not False or receipt['protocol']!='P18_PRE_GT_OUTPUT_FREEZE':
        raise ValueError('NOT_A_PRE_GT_FREEZE')
    historical=args.repo/'docs/p10_corridor01_deterministic_oosm_replay_r1'
    application_path=args.visual_applications or args.run.parent.parent/'visual_full_audit/visual_application_summary.csv'
    required=[args.run/name for name in ('ndt_diagnostics.csv','ekf_lineage.csv','deskew.csv',
              'oosm.csv','oosm_fixed_lag.csv','execution.json','rosparams.yaml',
              'ekf_runtime.csv','resources.csv','outputs.bag')]
    required += [historical/'posthoc_metrics.json',historical/'runs/control_v2/ndt_diagnostics.csv',
                 historical/'runs/control_v2/ekf_lineage.csv',application_path]
    if not {str(x.resolve()) for x in required}<=set(receipt['files_sha256']):
        raise ValueError('PRE_GT_FREEZE_REQUIRED_FILE_MISSING')
    for path,digest in receipt['files_sha256'].items():
        if sha(path)!=digest:raise ValueError('FROZEN_ARTIFACT_CHANGED:'+path)
    if sha(args.gt)!='3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03':
        raise ValueError('GT_HASH_MISMATCH')
    if sha(args.extrinsics)!='59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d':
        raise ValueError('EXTRINSICS_HASH_MISMATCH')
    gt_t,gt_p=G.load_gt(args.gt)
    T_il=G.load_t_imu_lidar(args.extrinsics)
    contract=json.loads((historical/'posthoc_metrics.json').read_text())
    origin=contract['evaluation_origin_unix_s']
    rf=np.array(contract['fit_transform_rotation']);tf=np.array(contract['fit_transform_translation_m'])
    windows={'3_35s':(3,35),'0_10s':(0,10),'6_12s':(6,12),'26_30s':(26,30),'30_35s':(30,35)}
    groups={};summary=[];sample_rows=[];paired_times={}
    applications=frozen.read_csv(application_path)
    applied_times={float(r['relative_time_s'])+origin for r in applications if r['applied']=='1'}
    methods={'Control':historical/'runs/control_v2','Visual':args.run}
    for method,root in methods.items():
        diag=frozen.read_csv(root/'ndt_diagnostics.csv')
        highrate,lineage=frozen.state_trajectory(root/'ekf_lineage.csv')
        components={label:frozen.measurement_trajectory(diag,prefix,T_il,origin)
                    for label,prefix in [('Initial_Guess','initial_guess_'),('Raw_NDT','raw_ndt_'),('Final_Used','final_used_')]}
        components['EKF_HighRate']=highrate
        for component,records in components.items():
            pairs=frozen.pair_with_gt(records,gt_t,gt_p,origin,35.)
            times,et,er,_=frozen.summarize_window(pairs,rf,tf)
            if method=='Control':paired_times[component]=times
            elif not np.array_equal(times,paired_times[component]):
                raise ValueError('CONTROL_VISUAL_PAIRED_TIMESTAMPS_DIFFER:'+component)
            result={}
            for window,(start,end) in windows.items():
                mask=(times>=start)&(times<=end)
                if not np.any(mask):raise ValueError('EMPTY_EVALUATION_WINDOW')
                metrics=G.summarize(et[mask],er[mask]);result[window]=metrics
                summary.append(dict(method=method,component=component,window=window,count=int(mask.sum()),
                    translation_rmse_m=metrics['translation_m']['rmse'],translation_p95_m=metrics['translation_m']['p95'],
                    translation_max_m=metrics['translation_m']['max'],rotation_rmse_deg=metrics['rotation_deg']['rmse'],
                    rotation_p95_deg=metrics['rotation_deg']['p95'],rotation_max_deg=metrics['rotation_deg']['max']))
            if component=='EKF_HighRate':
                result['persistent_drift']={str(x):frozen.persistent(times,et,x) for x in (.5,1.,2.)}
            groups.setdefault(method,{})[component]=result
            for i,t in enumerate(times):
                sample_rows.append(dict(method=method,component=component,relative_time_s=float(t),
                    translation_error_m=float(et[i]),rotation_error_deg=float(er[i])))
        # Measurement comparison conditioned on actual visual application,
        # not GT-selected frames. Histories differ across methods.
        for component in ('Initial_Guess','Raw_NDT','Final_Used'):
            records=components[component]
            paired=frozen.pair_with_gt(records,gt_t,gt_p,origin,35.)
            times,et,er,_=frozen.summarize_window(paired,rf,tf)
            masks={'applied_frames':np.array([p[0]+origin in applied_times for p in paired]),
                   'not_applied_frames':np.array([p[0]+origin not in applied_times for p in paired])}
            for label,mask in masks.items():
                mask=mask&(times>=3)&(times<=35)
                if np.any(mask):groups[method][component][label]=G.summarize(et[mask],er[mask])
    comparisons=[]
    for component in groups['Control']:
        for window in windows:
            for unit in ('translation_m','rotation_deg'):
                for metric in ('rmse','p95','max'):
                    a=groups['Control'][component][window][unit][metric]
                    b=groups['Visual'][component][window][unit][metric]
                    comparisons.append(dict(component=component,window=window,unit=unit,metric=metric,
                        control=a,visual=b,delta=b-a,improvement_percent=(a-b)/a*100 if a else None,
                        improved=b<a))
    visual_diag=frozen.read_csv(args.run/'ndt_diagnostics.csv')
    on=[r for r in visual_diag if r['initial_guess_source']=='p4_visual_translation_imu_rotation']
    paired_initial={}
    for label,prefix in [('visual_on','initial_guess_'),('same_history_visual_off','imu_rotation_guess_')]:
        pairs=frozen.pair_with_gt(frozen.measurement_trajectory(on,prefix,T_il,origin),gt_t,gt_p,origin,35.)
        times,et,er,_=frozen.summarize_window(pairs,rf,tf)
        mask=(times>=3)&(times<=35);paired_initial[label]=G.summarize(et[mask],er[mask])
    primary=[r for r in comparisons if r['window']=='3_35s' and r['unit']=='translation_m' and r['metric']=='rmse']
    hypotheses={r['component']:r['improved'] for r in primary}
    if all(hypotheses[k] for k in ('Initial_Guess','Raw_NDT','EKF_HighRate')):result='VISUAL_CAUSAL_BENEFIT_OBSERVED'
    elif hypotheses['Raw_NDT'] and not hypotheses['EKF_HighRate']:result='RAW_NDT_BENEFIT_NOT_PROPAGATED'
    elif hypotheses['Initial_Guess'] and not hypotheses['Raw_NDT']:result='INITIAL_GUESS_BENEFIT_ONLY'
    else:result='VISUAL_BENEFIT_NOT_ESTABLISHED'
    args.output.mkdir(parents=True,exist_ok=True)
    for name,records in [('method_summary.csv',summary),('accuracy_comparison.csv',comparisons),('per_sample_errors.csv',sample_rows)]:
        with (args.output/name).open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(records[0]),lineterminator='\n');w.writeheader();w.writerows(records)
    out=dict(FINAL_RESULT=result,GT_LOADED=True,evaluation_contract=contract['evaluation_contract'],
             origin=origin,fit_rotation=rf.tolist(),fit_translation=tf.tolist(),methods=groups,
             hypotheses_primary_translation_RMSE=hypotheses,same_history_initial_pair=paired_initial,
             notes=['Offline vision CSV, not a realtime frontend','Histories differ: paired trajectory errors do not isolate per-frame causal contributions',
                    'CAUSAL label is the contract-defined aggregate controlled-switch replay outcome, not proof of a framewise initial-to-raw-to-EKF causal chain',
                    'Hypotheses classify RMSE; P95/max and safety are reported separately; no statistical-significance claim'])
    (args.output/'posthoc_metrics.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(dict(FINAL_RESULT=result,primary=primary,same_history_initial_pair=paired_initial),indent=2))


if __name__=='__main__':main()
