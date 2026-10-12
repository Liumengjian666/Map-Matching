"""P21 posthoc metrics with original Control transform; no alignment fitting."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
SCRIPTS=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(SCRIPTS/'p17_visual'))
from evaluate_p18 import sha,G,frozen
sys.path.insert(0,str(SCRIPTS/'p19_visual_tail'))
from compare import rows
sys.path.insert(0,str(SCRIPTS/'p19_visual_tail'))
from compare import write_csv
from analyze import state_pose,error


def evaluate(repo,run,freeze,gt,extrinsics,output):
    receipt=json.loads(freeze.read_text())
    if receipt['protocol']!='P21_PRE_GT_OUTPUT_FREEZE' or receipt['GT_LOADED'] is not False:
        raise ValueError('WRONG_P21_FREEZE')
    historical=repo/'docs/p10_corridor01_deterministic_oosm_replay_r1'
    baseline=repo/'docs/p19_r1_visual_replication_tail_risk/runs/repeat_visual'
    required=[p/name for p in (run,baseline) for name in ('ndt_diagnostics.csv','ekf_lineage.csv','rosparams.yaml','execution.json')]
    required+=[run/name for name in ('outputs.bag','deskew.csv','oosm.csv','oosm_fixed_lag.csv','resources.csv')]
    required+=[historical/'posthoc_metrics.json']
    required+=[historical/'runs/control_v2'/name for name in ('ndt_diagnostics.csv','ekf_lineage.csv')]
    if not {str(p.resolve()) for p in required}<=set(receipt['files_sha256']):raise ValueError('REQUIRED_FREEZE_FILE_MISSING')
    for path,digest in receipt['files_sha256'].items():
        if sha(path)!=digest:raise ValueError('FROZEN_OUTPUT_CHANGED:'+path)
    if sha(gt)!='3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03':raise ValueError('GT_SHA')
    if sha(extrinsics)!='59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d':raise ValueError('EXT_SHA')
    gt_t,gt_p=G.load_gt(gt);til=G.load_t_imu_lidar(extrinsics)
    contract=json.loads((historical/'posthoc_metrics.json').read_text());origin=contract['evaluation_origin_unix_s']
    rf=np.array(contract['fit_transform_rotation']);tf=np.array(contract['fit_transform_translation_m'])
    roots={'Control_V2':historical/'runs/control_v2','P19_Visual':baseline,'P21_EKF_Prior':run}
    windows={'3_35s':(3,35),'6_12s':(6,12),'26_30s':(26,30),'30_35s':(30,35),'32_35s':(32,35),'34_35s':(34,35)}
    summary=[];samples=[];tail=[];peaks=[];peak_stages=[];metrics={};all_errors={};motion=[];safety={};pair_times={};group_rows=[]
    experimental_diag=rows(run/'ndt_diagnostics.csv')
    source_by_scan={float(r['lidar_header_stamp']):('VISUAL_APPLIED' if r['visual_applied']=='1' else
        'EKF_TRANSLATION_APPLIED' if r['ekf_translation_applied']=='1' else 'NDT_HISTORY_FALLBACK') for r in experimental_diag}
    experimental_high={float(r['state_stamp']):float(r['last_ndt_observation_time']) for r in rows(run/'ekf_lineage.csv') if r['event_type']=='IMU_HIGH_RATE_PUBLISH'}
    for method,root in roots.items():
        diag=rows(root/'ndt_diagnostics.csv');high,events=frozen.state_trajectory(root/'ekf_lineage.csv')
        components={name:frozen.measurement_trajectory(diag,prefix,til,origin) for name,prefix in
                    [('Initial_Guess','initial_guess_'),('Raw_NDT','raw_ndt_'),('Final_Used','final_used_')]}
        components['EKF_HighRate']=high
        metrics[method]={}
        for name,records in components.items():
            pairs=frozen.pair_with_gt(records,gt_t,gt_p,origin,35)
            times,et,er,_=frozen.summarize_window(pairs,rf,tf)
            if method=='Control_V2':pair_times[name]=times
            elif not np.array_equal(times,pair_times[name]):raise ValueError('PAIRED_TIMESTAMPS_CHANGED:'+name)
            all_errors[(method,name)]=(times,et,er)
            metrics[method][name]={}
            for window,(start,end) in windows.items():
                mask=(times>=start)&(times<=end)
                result=G.summarize(et[mask],er[mask]);metrics[method][name][window]=result
                row=dict(method=method,component=name,window=window,count=int(mask.sum()),
                    translation_rmse_m=result['translation_m']['rmse'],translation_p95_m=result['translation_m']['p95'],translation_max_m=result['translation_m']['max'],
                    rotation_rmse_deg=result['rotation_deg']['rmse'],rotation_p95_deg=result['rotation_deg']['p95'],rotation_max_deg=result['rotation_deg']['max'])
                summary.append(row)
                if name=='EKF_HighRate':tail.append(dict(**row,over_1m=int(sum(et[mask]>1)),over_2m=int(sum(et[mask]>2)),over_5m=int(sum(et[mask]>5))))
            if name=='EKF_HighRate':
                metrics[method][name]['persistent']={str(t):frozen.persistent(times,et,t) for t in (.5,1.,2.)}
            # Identical timestamp cohorts defined by P21's actual source, never GT.
            labels=[source_by_scan.get(experimental_high.get(float(t+origin)) if name=='EKF_HighRate' else float(t+origin),'NDT_HISTORY_FALLBACK') for t in times]
            for source in ('VISUAL_APPLIED','EKF_TRANSLATION_APPLIED','NDT_HISTORY_FALLBACK'):
                mask=(times>=3)&(times<=35)&np.array([x==source for x in labels])
                if not mask.any():
                    group_rows.append(dict(method=method,component=name,source=source,count=0,
                        translation_rmse_m=None,translation_p95_m=None,translation_max_m=None,
                        rotation_rmse_deg=None,rotation_p95_deg=None,rotation_max_deg=None))
                else:
                    result=G.summarize(et[mask],er[mask])
                    group_rows.append(dict(method=method,component=name,source=source,count=int(mask.sum()),
                        translation_rmse_m=result['translation_m']['rmse'],translation_p95_m=result['translation_m']['p95'],translation_max_m=result['translation_m']['max'],
                        rotation_rmse_deg=result['rotation_deg']['rmse'],rotation_p95_deg=result['rotation_deg']['p95'],rotation_max_deg=result['rotation_deg']['max']))
            for t,e,a in zip(times,et,er):samples.append(dict(method=method,component=name,relative_time_s=float(t),translation_error_m=float(e),rotation_error_deg=float(a)))
        jumps=[(b[0],float(np.linalg.norm(b[1][0]-a[1][0]))) for a,b in zip(high,high[1:])]
        rotation_jumps=[G.rotation_angle_deg(a[1][1].T@b[1][1]) for a,b in zip(high,high[1:])]
        safety[method]=dict(max_jump_m=max(v for t,v in jumps),max_tail_jump_m=max(v for t,v in jumps if 32<=t-origin<=35),
            max_rotation_jump_deg=max(rotation_jumps),over_20deg=sum(v>20 for v in rotation_jumps),
            over_2m=sum(v>2 for t,v in jumps),nonfinite=0)
        ndt_motion=[];prev=None
        for d in diag:
            stamp=float(d['lidar_header_stamp']);rel=stamp-origin
            raw=G.lidar_pose_to_imu(dict(zip(('position','rotation'),frozen.pose_from_fields(d,'raw_ndt_'))),til)
            initial=G.lidar_pose_to_imu(dict(zip(('position','rotation'),frozen.pose_from_fields(d,'initial_guess_'))),til)
            used=G.lidar_pose_to_imu(dict(zip(('position','rotation'),frozen.pose_from_fields(d,'final_used_'))),til)
            if prev is not None and 32<=rel<=35:
                dt=stamp-prev['stamp'];vn=(used[0]-prev['used'])/dt
                vi=(initial[0]-prev['initial'])/dt;vr=(raw[0]-prev['raw'])/dt
                a,b=[G.interpolate_gt(gt_t,gt_p,stamp+x) for x in (-.05,.05)]
                vg=rf.T@(b[0]-a[0])/.1
                release=next(r for r in events if r['event_type']=='OOSM_REPLAY_COMPLETE' and float(r['ndt_measurement_stamp'])==stamp)
                gr0,gr1=[G.interpolate_gt(gt_t,gt_p,float(release['state_stamp'])+x) for x in (-.05,.05)]
                vg_release=rf.T@(gr1[0]-gr0[0])/.1
                gp0,gp1=[G.interpolate_gt(gt_t,gt_p,t)[0] for t in (prev['stamp'],stamp)]
                vg_interval=rf.T@(gp1-gp0)/dt
                ndt_motion.append(dict(stamp=stamp,ndt_vx=vn[0],initial_vx=vi[0],raw_vx=vr[0],GT_vx=vg[0],
                    GT_interval_vx=vg_interval[0],GT_release_vx=vg_release[0],release_vx=float(release['v_x']),
                    release_vx_error_sq=(float(release['v_x'])-vg_release[0])**2,
                    raw_minus_initial=np.linalg.norm(raw[0]-initial[0]),one_iteration=d['ndt_iterations']=='1',fitness=float(d['ndt_fitness']),
                    t_limit=d['translation_limited']=='1',r_limit=d['rotation_limited']=='1'))
            prev=dict(stamp=stamp,initial=initial[0],raw=raw[0],used=used[0])
        motion.append(dict(method=method,count=len(ndt_motion),**{k:float(np.mean([r[k] for r in ndt_motion])) for k in
            ('ndt_vx','initial_vx','raw_vx','GT_vx','GT_interval_vx','GT_release_vx','release_vx','raw_minus_initial','fitness')},
            release_vx_error_rmse=float(np.sqrt(np.mean([r['release_vx_error_sq'] for r in ndt_motion]))),
            one_iteration=sum(r['one_iteration'] for r in ndt_motion),translation_limited=sum(r['t_limit'] for r in ndt_motion),rotation_limited=sum(r['r_limit'] for r in ndt_motion)))
    comparisons=[]
    for name in metrics['P19_Visual']:
        for window in windows:
            for unit in ('translation_m','rotation_deg'):
                for statistic in ('rmse','p95','max'):
                    a=metrics['P19_Visual'][name][window][unit][statistic];b=metrics['P21_EKF_Prior'][name][window][unit][statistic]
                    comparisons.append(dict(component=name,window=window,unit=unit,metric=statistic,baseline=a,experiment=b,
                        delta=b-a,change_percent=(b-a)/a*100,count=metrics['P21_EKF_Prior'][name][window][unit]['count'],improved=b<a))
    for peak_method in ('P19_Visual','P21_EKF_Prior'):
        times,et,er=all_errors[(peak_method,'EKF_HighRate')];mask=times>=3
        index=np.flatnonzero(mask)[np.argmax(et[mask])];stamp=float(times[index]+origin)
        for method in ('P19_Visual','P21_EKF_Prior'):
            root=roots[method];events=rows(root/'ekf_lineage.csv');diag=rows(root/'ndt_diagnostics.csv')
            high=next(r for r in events if r['event_type']=='IMU_HIGH_RATE_PUBLISH' and float(r['state_stamp'])==stamp)
            obs=float(high['last_ndt_observation_time']);d=next(r for r in diag if float(r['lidar_header_stamp'])==obs)
            last=max(float(r['lidar_header_stamp']) for r in diag if r['initial_guess_source']=='p4_visual_translation_imu_rotation' and float(r['lidar_header_stamp'])<=stamp)
            peaks.append(dict(peak_method=peak_method,method=method,peak_stamp=stamp,relative_time_s=stamp-origin,
                **error(stamp,state_pose(high),gt_t,gt_p,rf,tf),latest_ndt_stamp=obs,latest_ndt_frame=d['frame_index'],
                latest_visual_stamp=last,visual_age_s=stamp-last,direct_visual=d['initial_guess_source']=='p4_visual_translation_imu_rotation'))
            for component,prefix in [('Initial_Guess','initial_guess_'),('Raw_NDT','raw_ndt_'),('Final_Used','final_used_')]:
                pose=G.lidar_pose_to_imu(dict(zip(('position','rotation'),frozen.pose_from_fields(d,prefix))),til)
                peak_stages.append(dict(peak_method=peak_method,method=method,peak_stamp=stamp,component=component,
                    component_stamp=obs,**error(obs,pose,gt_t,gt_p,rf,tf),iterations=d['ndt_iterations'],fitness=d['ndt_fitness'],
                    translation_limited=d['translation_limited'],rotation_limited=d['rotation_limited']))
            corrected=next(r for r in events if r['event_type']=='CORRECTED_PUBLISH' and float(r['last_ndt_observation_time'])==obs)
            for component,state in [('EKF_Corrected',corrected),('EKF_HighRate',high)]:
                actual_stamp=float(state['state_stamp'])
                peak_stages.append(dict(peak_method=peak_method,method=method,peak_stamp=stamp,component=component,
                    component_stamp=actual_stamp,**error(actual_stamp,state_pose(state),gt_t,gt_p,rf,tf),
                    iterations=d['ndt_iterations'],fitness=d['ndt_fitness'],translation_limited=d['translation_limited'],rotation_limited=d['rotation_limited']))
    experiment=metrics['P21_EKF_Prior']['EKF_HighRate']['3_35s']['translation_m']
    screen=dict(max_below_Control=experiment['max']<=5.667156999562539,
        rmse_within_5percent=experiment['rmse']<=1.05*1.2481083256675167,
        p95_within_5percent=experiment['p95']<=1.05*2.552406529827499)
    on=[r for r in experimental_diag if r['ekf_translation_applied']=='1']
    same_history={}
    for label,prefix in [('original_history_guess','imu_rotation_guess_'),('EKF_initial','initial_guess_'),('Raw_after_EKF_initial','raw_ndt_')]:
        pairs=frozen.pair_with_gt(frozen.measurement_trajectory(on,prefix,til,origin),gt_t,gt_p,origin,35)
        times,et,er,_=frozen.summarize_window(pairs,rf,tf)
        mask=(times>=3)&(times<=35)
        same_history[label]=G.summarize(et[mask],er[mask])
    output.mkdir(parents=True,exist_ok=False)
    for name,data in [('method_summary.csv',summary),('accuracy_comparison.csv',comparisons),('per_sample_errors.csv',samples),
                      ('tail_window_summary.csv',tail),('tail_peak_events.csv',peaks),('tail_peak_stages.csv',peak_stages),
                      ('ndt_motion_summary.csv',motion),('source_conditioned_accuracy.csv',group_rows)]:write_csv(output/name,data)
    result=dict(GT_LOADED=True,evaluation_contract=contract['evaluation_contract'],origin=origin,fit_rotation=rf.tolist(),fit_translation=tf.tolist(),
        methods=metrics,motion=motion,safety=safety,peaks=peaks,engineering_screen=screen,
        source_conditioned=group_rows,same_history_EKF_application=same_history,
        note='P21 source cohorts paired by time; different feedback histories, not independent per-frame causal effects')
    (output/'posthoc_metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(primary={m:metrics[m]['EKF_HighRate']['3_35s'] for m in roots},motion=motion,screen=screen),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for key in ('repo','run','freeze','gt','extrinsics','output'):p.add_argument('--'+key,type=Path,required=True)
    evaluate(**vars(p.parse_args()))
