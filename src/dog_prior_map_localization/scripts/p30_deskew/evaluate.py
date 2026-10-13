#!/usr/bin/env python3
"""Freeze complete P30 outputs first; evaluate only with the unchanged V2 GT contract."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'p29_feedback'))
from prepare import rows, sha, write_csv
from analyze import frozen, G, summary

def ndt_durations(root):
    # Production publishes timings as DiagnosticArray KeyValue, while the
    # determinism CSV deliberately excludes these wall-clock fields.
    import rosbag
    values={k:[] for k in ('align_time_ms','localization_time_ms')}
    with rosbag.Bag(str(root/'outputs.bag')) as bag:
        for _,message,_ in bag.read_messages(topics=['/dog_livo/diagnostics']):
            for status in message.status:
                fields={v.key:v.value for v in status.values}
                for key in values:
                    if key in fields: values[key].append(float(fields[key]))
    if any(len(v)!=396 or not np.isfinite(v).all() for v in values.values()):
        raise ValueError('P30_NDT_TIMING_MEASUREMENT_COUNT_OR_FINITE_FAILED')
    return values

def freeze(repo):
    cache=repo/'.p30_r1_scan_local_deskew';target=cache/'pre_gt_freeze.json'
    if target.exists(): raise ValueError('P30_OUTPUT_FREEZE_ALREADY_EXISTS')
    for stage in ('short','full'):
        gate=json.loads((cache/('gate_'+stage+'.json')).read_text())
        if gate['status']!='PASS' or gate['GT_LOADED'] is not False:
            raise ValueError('P30_GATE_NOT_PASSED')
    paths=[p for directory in ('runs','sources') for p in (cache/directory).rglob('*') if p.is_file()]
    paths+=list(cache.glob('*.yaml'))+[cache/'input_freeze.json',cache/'gate_short.json',cache/'gate_full.json']
    data=dict(protocol='P30_PRE_GT_OUTPUT_FREEZE',GT_LOADED=False,completed=True,
              files_sha256={str(p):sha(p) for p in sorted(paths)})
    target.write_text(json.dumps(data,indent=2)+'\n')
    print('P30_PRE_GT_OUTPUT_FREEZE_PASS')

def main(repo):
    cache=repo/'.p30_r1_scan_local_deskew'
    receipt=json.loads((cache/'pre_gt_freeze.json').read_text())
    if receipt['protocol']!='P30_PRE_GT_OUTPUT_FREEZE' or receipt['GT_LOADED'] is not False or not receipt['completed']:
        raise ValueError('P30_PRE_GT_FREEZE_REQUIRED')
    input_freeze=json.loads((cache/'input_freeze.json').read_text())
    for inventory in (input_freeze['files_sha256'],receipt['files_sha256']):
        for path,digest in inventory.items():
            if sha(path)!=digest: raise ValueError('P30_FREEZE_CHANGED:'+path)
    contract=json.loads((repo/'docs/p10_corridor01_deterministic_oosm_replay_r1/posthoc_metrics.json').read_text())
    origin=contract['evaluation_origin_unix_s'];R=np.array(contract['fit_transform_rotation']);t=np.array(contract['fit_transform_translation_m'])
    gt=repo/'.p18_map_recovery/posthoc_inputs/frozen.YOuGEy/corridor01_gt.txt';ext=gt.parent/'corridor01_extrinsics.yaml'
    if sha(gt)!='3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03' or sha(ext)!='59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d':
        raise ValueError('P30_GT_OR_EXTRINSIC_CHANGED')
    gt_times,gt_poses=G.load_gt(gt);Til=G.load_t_imu_lidar(ext)
    roots={m:repo/'.p29_r1_fixed_source_feedback/runs'/m for m in ('A','B')};roots['C']=cache/'runs/full'
    metrics=[];rpes=[];samples=[];runtime=[];resources=[];divergence=[];stability=[]
    for method,root in roots.items():
        high=frozen.state_trajectory(root/'ekf_lineage.csv')[0];ndt=rows(root/'ndt_diagnostics.csv')
        layers={'EKF_HighRate':high}
        for name,prefix in [('Initial_Guess','initial_guess_'),('Raw_NDT','raw_ndt_'),('Final_Used','final_used_')]:
            layers[name]=frozen.measurement_trajectory(ndt,prefix,Til,origin)
        for layer,trajectory in layers.items():
            pairs=frozen.pair_with_gt(trajectory,gt_times,gt_poses,origin,35)
            times,et,er,_=frozen.summarize_window(pairs,R,t)
            for window,lo,hi in [('3_35',3,35),('26_35',26,35),('34_35',34,35)]:
                mask=(times>=lo)&(times<=hi);tr,rr=summary(et[mask]),summary(er[mask])
                metrics.append(dict(method=method,layer=layer,window=window,count=int(mask.sum()),
                    translation_rmse_m=tr['rmse'],translation_p95_m=tr['p95'],translation_max_m=tr['maximum'],
                    rotation_rmse_deg=rr['rmse'],rotation_p95_deg=rr['p95'],rotation_max_deg=rr['maximum'],
                    over_1m=int((et[mask]>1).sum()),over_2m=int((et[mask]>2).sum()),over_5m=int((et[mask]>5).sum())))
            if layer=='EKF_HighRate':
                for i in np.flatnonzero(times>=3):
                    samples.append(dict(method=method,stamp=origin+times[i],relative_s=times[i],translation_error_m=et[i],rotation_error_deg=er[i]))
                te=[];re=[];interval=[]
                for i,record in enumerate(pairs):
                    if times[i]<3:continue
                    j=int(np.searchsorted(times,times[i]+1))
                    if j>=len(pairs) or abs(times[j]-times[i]-1)>0.0051:continue
                    pe,Re=record[1];pg,Rg=record[2];qe,Qe=pairs[j][1];qg,Qg=pairs[j][2]
                    te.append(np.linalg.norm(Re.T@(qe-pe)-Rg.T@(qg-pg)))
                    re.append(G.rotation_angle_deg((Rg.T@Qg).T@(Re.T@Qe)));interval.append(times[j]-times[i])
                tr,rr=summary(te),summary(re)
                rpes.append(dict(method=method,count=len(te),nominal_sec=1.,actual_min_sec=min(interval),actual_max_sec=max(interval),
                    translation_rmse_m=tr['rmse'],translation_p95_m=tr['p95'],translation_max_m=tr['maximum'],
                    rotation_rmse_deg=rr['rmse'],rotation_p95_deg=rr['p95'],rotation_max_deg=rr['maximum']))
                positions=np.array([p[1][0] for p in pairs]);jumps=np.linalg.norm(np.diff(positions,axis=0),axis=1)
                peak=int(np.argmax(et));stability.append(dict(method=method,max_error_time_s=float(times[peak]),
                    max_error_m=float(et[peak]),max_adjacent_jump_m=float(jumps.max()),jumps_gt2m=int((jumps>2).sum()),
                    persistent_1m=frozen.persistent(times,et,1),persistent_2m=frozen.persistent(times,et,2)))
        for file,field,label in [('deskew.csv','deskew_processing_ms','Deskew'),('msckf_diagnostics.csv','update_ms','MSCKF_processing')]:
            records=rows(root/file)
            if label=='MSCKF_processing':records=[r for r in records if r['status']=='APPLIED_REPROJECTION_SELECTED_GAIN']
            values=np.array([float(r[field]) for r in records])
            if not len(values):
                runtime.append(dict(method=method,stage=label,count=0,mean_ms=None,p95_ms=None,max_ms=None,timing_measured=False))
                continue
            runtime.append(dict(method=method,stage=label,count=len(values),mean_ms=float(values.mean()),p95_ms=float(np.percentile(values,95)),max_ms=float(values.max())))
        for key,values in ndt_durations(root).items():
            values=np.asarray(values)
            runtime.append(dict(method=method,stage='NDT_align' if key=='align_time_ms' else 'NDT_callback',count=len(values),
                mean_ms=float(values.mean()),p95_ms=float(np.percentile(values,95)),max_ms=float(values.max())))
        execution=json.loads((root/'execution.json').read_text())
        for name,resource in execution['resources'].items():
            resources.append(dict(method=method,process=name,peak_rss_bytes=resource['peak_rss_bytes'],cpu_sec=resource['cpu_sec'],replay_wall_sec=execution['wall_sec']))
    # The actual P29 B reference is recomputed with the identical frozen transform.
    prior=json.loads((repo/'docs/p29_r1_fixed_source_feedback/results.json').read_text())
    for m in ('A','B'):
        old=next(r for r in prior['alignment_metrics'] if r['method']==m and r['component']=='EKF' and r['window']=='3_35' and r['transform']=='FORMAL_SHARED_CONTROL_V2')
        actual=next(r for r in metrics if r['method']==m and r['layer']=='EKF_HighRate' and r['window']=='3_35')
        if any(abs(float(old[k])-float(actual[k]))>1e-10 for k in ('translation_rmse_m','translation_p95_m','translation_max_m')):
            raise ValueError('P29_REFERENCE_EVALUATION_CHANGED')
    comparisons=[]
    for b in [r for r in metrics if r['method']=='B']:
        c=next(r for r in metrics if r['method']=='C' and r['layer']==b['layer'] and r['window']==b['window'])
        for key in ('translation_rmse_m','translation_p95_m','translation_max_m','rotation_rmse_deg','rotation_p95_deg','rotation_max_deg'):
            comparisons.append(dict(layer=b['layer'],window=b['window'],metric=key,B=b[key],C=c[key],
                delta=c[key]-b[key],relative_percent=(c[key]-b[key])/b[key]*100,count=b['count']))
    db,dc=[rows(root/'ndt_diagnostics.csv') for root in (roots['B'],roots['C'])]
    for label,prefix in [('Initial','initial_guess_'),('Raw','raw_ndt_'),('Used','final_used_')]:
        for b,c in zip(db,dc):
            bp,br=frozen.pose_from_fields(b,prefix);cp,cr=frozen.pose_from_fields(c,prefix)
            bp,br=G.lidar_pose_to_imu(dict(position=bp,rotation=br),Til);cp,cr=G.lidar_pose_to_imu(dict(position=cp,rotation=cr),Til)
            dp=float(np.linalg.norm(bp-cp));dr=float(np.degrees((Rotation.from_matrix(br).inv()*Rotation.from_matrix(cr)).magnitude()))
            if dp>1e-9 or dr>1e-8:
                divergence.append(dict(layer=label,frame_index=b['frame_index'],stamp=b['lidar_header_stamp'],translation_delta_m=dp,rotation_delta_deg=dr));break
    for b,c in zip(db,dc):
        if b['cloud_hash']!=c['cloud_hash']:
            ns=next(r['scan_ns'] for r in rows(roots['C']/'mature_diagnostics.csv') if r['frame_index']==c['frame_index'])
            divergence.append(dict(layer='Source_hash',frame_index=c['frame_index'],scan_ns=ns,B_hash=b['cloud_hash'],C_hash=c['cloud_hash']));break
    hb,hc=[[r for r in rows(root/'ekf_lineage.csv') if r['event_type']=='IMU_HIGH_RATE_PUBLISH'] for root in (roots['B'],roots['C'])]
    for b,c in zip(hb,hc):
        dp=np.linalg.norm([float(c[k])-float(b[k]) for k in ('p_x','p_y','p_z')])
        br=G.quat_to_rotation(np.array([float(b[k]) for k in ('q_x','q_y','q_z','q_w')]))
        cr=G.quat_to_rotation(np.array([float(c[k]) for k in ('q_x','q_y','q_z','q_w')]))
        dr=float(np.degrees((Rotation.from_matrix(br).inv()*Rotation.from_matrix(cr)).magnitude()))
        if dp>1e-9 or dr>1e-8:
            divergence.append(dict(layer='EKF_HighRate',stamp=b['state_stamp'],translation_delta_m=float(dp),rotation_delta_deg=dr));break
    for name,data in [('accuracy_metrics',metrics),('accuracy_comparison',comparisons),('rpe_summary',rpes),('per_sample_errors',samples),
                      ('runtime_summary',runtime),('resource_summary',resources),('first_divergence',divergence)]:
        write_csv(cache/(name+'.csv'),data)
    b=next(r for r in metrics if r['method']=='B' and r['layer']=='EKF_HighRate' and r['window']=='3_35')
    c=next(r for r in metrics if r['method']=='C' and r['layer']=='EKF_HighRate' and r['window']=='3_35')
    full_gate=json.loads((cache/'gate_full.json').read_text())
    boundary_improved=all(r['local_boundary_residual_m_max']<r['history_boundary_residual_m_max'] and
        r['local_boundary_residual_deg_max']<r['history_boundary_residual_deg_max'] for r in full_gate['motion'])
    result=dict(TASK='P30-R1-SCAN-LOCAL-IMU-DESKEW-FEEDBACK-ISOLATION',GT_LOADED=True,
        START_SHA=input_freeze['START_SHA'],math_contract='PASS',short=json.loads((cache/'gate_short.json').read_text()),
        full=full_gate,metrics=metrics,comparisons=comparisons,rpe=rpes,stability=stability,
        runtime=runtime,resources=resources,first_divergence=divergence,shared_Control_V2_transform=True,
        full_replay_count=1,short_replay_count=1,gt_parameter_search=False,unique_root_cause_proven=False,
        primary_RMSE_improved=c['translation_rmse_m']<b['translation_rmse_m'],
        same_current_history_boundary_residual_reduced=boundary_improved,
        interpretation_limit='Trajectory construction includes removal of history resets AND alignment to existing mature ZOH convention; no pure visual-rewrite-only attribution.',
        FINAL_RESULT=('DESKEW_FEEDBACK_ISOLATION_EFFECTIVE' if boundary_improved else 'DESKEW_ISOLATION_UNSUPPORTED')
            if c['translation_rmse_m']<b['translation_rmse_m'] else 'DESKEW_ISOLATION_VALID_BUT_NO_ACCURACY_GAIN')
    (cache/'analysis_results.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(FINAL_RESULT=result['FINAL_RESULT'],EKF_primary={m:next(r for r in metrics if r['method']==m and r['layer']=='EKF_HighRate' and r['window']=='3_35') for m in ('A','B','C')},rpe=rpes,stability=stability),indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--freeze',action='store_true')
    a=p.parse_args();freeze(a.repo) if a.freeze else main(a.repo)
