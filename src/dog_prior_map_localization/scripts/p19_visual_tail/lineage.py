"""Sensor-keyed Initial/Raw/Used/EKF history diagnostics; no counterfactual."""
import argparse
import json
from pathlib import Path
import numpy as np
from compare import rows, write_csv, pose_delta
from analyze import frozen, G, error, state_pose


def vector(row, prefix):
    return np.array([float(row[prefix+k]) for k in ('tx','ty','tz')])


def inspect(repo, gt, extrinsics, output):
    gt_t,gt_p=G.load_gt(gt);til=G.load_t_imu_lidar(extrinsics)
    old=repo/'docs/p10_corridor01_deterministic_oosm_replay_r1'
    contract=json.loads((old/'posthoc_metrics.json').read_text())
    origin=contract['evaluation_origin_unix_s'];rf=np.array(contract['fit_transform_rotation'])
    tf=np.array(contract['fit_transform_translation_m'])
    roots={'Control':old/'runs/control_v2','Visual':repo/'.p18_map_recovery/runs/visual_full'}
    diags={m:rows(p/'ndt_diagnostics.csv') for m,p in roots.items()}
    events={m:rows(p/'ekf_lineage.csv') for m,p in roots.items()}
    def shared_times(tables,key,expected):
        stamps=[[float(r[key]) for r in table] for table in tables]
        if any(len(t)!=expected or not np.isfinite(t).all() or any(b<=a for a,b in zip(t,t[1:])) for t in stamps):
            raise ValueError('INCOMPLETE_OR_UNORDERED:'+key)
        if stamps[0]!=stamps[1]:raise ValueError('UNPAIRED:'+key)
    shared_times(list(diags.values()),'lidar_header_stamp',396)
    evidence=[];first={};jumps={};metrics={};stage_poses={}
    for method,root in roots.items():
        es=events[method];ds=diags[method];used=[];last_applied=None;prev_meas=None;previous_stamp=None
        for i,d in enumerate(ds):
            stamp=float(d['lidar_header_stamp']);rel=stamp-origin
            applied=d['initial_guess_source']=='p4_visual_translation_imu_rotation'
            if applied:last_applied=stamp
            stages={}
            for label,prefix in [('Initial','initial_guess_'),('Raw','raw_ndt_'),('Used','final_used_')]:
                stages[label]=G.lidar_pose_to_imu(dict(zip(('position','rotation'),frozen.pose_from_fields(d,prefix))),til)
            if i:stages['BeforeVisual']=G.lidar_pose_to_imu(dict(zip(('position','rotation'),frozen.pose_from_fields(d,'imu_rotation_guess_'))),til)
            stage_poses[(method,i)]=stages
            used.append((stamp,stages['Used']))
            cor=next(r for r in es if r['event_type']=='NDT_CORRECTION_APPLIED' and float(r['ndt_measurement_stamp'])==stamp)
            rollback=next(r for r in es if r['event_type']=='OOSM_ROLLBACK' and float(r['ndt_measurement_stamp'])==stamp)
            release=next(r for r in es if r['event_type']=='OOSM_REPLAY_COMPLETE' and float(r['ndt_measurement_stamp'])==stamp)
            corrected=next(r for r in es if r['event_type']=='CORRECTED_PUBLISH' and float(r['last_ndt_observation_time'])==stamp)
            v_before=np.array([float(cor['v_'+k]) for k in 'xyz'])
            v_ndt=(stages['Used'][0]-prev_meas)/(stamp-previous_stamp) if i else None
            v_mix=.4*v_before+.6*v_ndt if i else v_before
            r=dict(method=method,frame=int(d['frame_index']),scan_stamp=stamp,relative_time_s=rel,
                applied=applied,last_visual_age_s=stamp-last_applied if last_applied else None,
                iterations=int(d['ndt_iterations']),fitness=float(d['ndt_fitness']),converged=d['ndt_has_converged'],
                translation_limited=d['translation_limited'],rotation_limited=d['rotation_limited'],
                cloud_hash=d['cloud_hash'],cloud_size_raw=d['cloud_size_raw'],cloud_size_after_filter=d['cloud_size_after_filter'],
                guess_to_raw_translation_m=float(np.linalg.norm(vector(d,'raw_ndt_')-vector(d,'initial_guess_'))),
                used_jump_m=float(np.linalg.norm(vector(d,'final_used_')-vector(d,'previous_pose_'))) if i else None,
                raw_to_used_translation_m=float(np.linalg.norm(vector(d,'raw_ndt_')-vector(d,'final_used_'))),
                rollback_stamp=float(rollback['state_stamp']),release_stamp=float(corrected['state_stamp']),
                replay_imu_count=int(release['replay_imu_count']))
            if 0<=rel<=35:
                for label,pose in stages.items():
                    value=error(stamp,pose,gt_t,gt_p,rf,tf)
                    r[label+'_error_m']=value['translation_error_m'];r[label+'_rotation_error_deg']=value['rotation_error_deg']
                    for k,p in zip('xyz',pose[0]):r[label+'_imu_map_'+k]=float(p)
                for label,event in [('Rollback',rollback),('Correction',cor),('Corrected',corrected)]:
                    pose=state_pose(event);r[label+'_error_m']=error(float(event['state_stamp']),pose,gt_t,gt_p,rf,tf)['translation_error_m']
                for k in 'xyz':
                    r['v_before_'+k]=float(cor['v_'+k]);r['v_ndt_'+k]=float(v_ndt['xyz'.index(k)]) if i else None
                    r['v_mix_'+k]=float(v_mix['xyz'.index(k)]);r['v_release_'+k]=float(corrected['v_'+k])
                a,b=[G.interpolate_gt(gt_t,gt_p,stamp+delta) for delta in (-.05,.05)]
                velocity=rf.T@(b[0]-a[0])/.1
                for k,v in zip('xyz',velocity):r['GT_map_velocity_'+k]=float(v)
                evidence.append(r)
            prev_meas=stages['Used'][0];previous_stamp=stamp
        high=[r for r in es if r['event_type']=='IMU_HIGH_RATE_PUBLISH']
        jump_values=[(float(b['state_stamp']),np.linalg.norm(state_pose(b)[0]-state_pose(a)[0])) for a,b in zip(high,high[1:])]
        jumps[method]=dict(maximum_m=float(max(x[1] for x in jump_values)),
            max_32_35_m=float(max(x[1] for x in jump_values if 32<=x[0]-origin<=35)),
            count_over_2m=int(sum(x[1]>2 for x in jump_values)))
    for i,(a,b) in enumerate(zip(diags['Control'],diags['Visual'])):
        if 'source_hash' not in first and a['cloud_hash']!=b['cloud_hash']:
            first['source_hash']=dict(frame=i+1,stamp=float(a['lidar_header_stamp']),
                relative_time_s=float(a['lidar_header_stamp'])-origin,control=a['cloud_hash'],visual=b['cloud_hash'])
        for stage in ('Initial','Raw','Used'):
            x,y=[stage_poses[(m,i)][stage] for m in ('Control','Visual')]
            # Robust tiny-angle check uses quaternion difference instead of acos.
            dt,dr=pose_delta([*x[0],*G.rotation_to_quaternion(x[1])],[*y[0],*G.rotation_to_quaternion(y[1])])
            for threshold in (1e-9,.12,.5,1.):
                key=stage+'_'+str(threshold)
                if key not in first and (dt>threshold or (threshold==1e-9 and dr>1e-8)):
                    first[key]=dict(frame=i+1,stamp=float(a['lidar_header_stamp']),relative_time_s=float(a['lidar_header_stamp'])-origin,
                                    separation_m=dt,separation_deg=dr,visual_direct=b['initial_guess_source']=='p4_visual_translation_imu_rotation')
    hc,hv=[[r for r in events[m] if r['event_type']=='IMU_HIGH_RATE_PUBLISH'] for m in ('Control','Visual')]
    shared_times([hc,hv],'state_stamp',8004)
    for i,(a,b) in enumerate(zip(hc,hv)):
        x,y=state_pose(a),state_pose(b)
        dt,dr=pose_delta([*x[0],*G.rotation_to_quaternion(x[1])],[*y[0],*G.rotation_to_quaternion(y[1])])
        for threshold in (1e-9,.12,.5,1.):
            key='EKF_'+str(threshold)
            if key not in first and (dt>threshold or (threshold==1e-9 and dr>1e-8)):
                first[key]=dict(index=i+1,stamp=float(a['state_stamp']),relative_time_s=float(a['state_stamp'])-origin,
                    separation_m=dt,separation_deg=dr)
    for window,(start,end) in {'6_12s':(6,12),'applied_primary':(3,35),'32_35s':(32,35)}.items():
        selected=[r for r in evidence if r['method']=='Visual' and start<=r['relative_time_s']<=end and
            (r['applied'] if window!='32_35s' else True)]
        metrics[window]=dict(count=len(selected),raw_improves_initial=sum(r['Raw_error_m']<r['Initial_error_m'] for r in selected),
            raw_worsens_initial=sum(r['Raw_error_m']>r['Initial_error_m'] for r in selected),
            before_visual_rmse_m=float(np.sqrt(np.mean([r['BeforeVisual_error_m']**2 for r in selected]))),
            initial_rmse_m=float(np.sqrt(np.mean([r['Initial_error_m']**2 for r in selected]))),
            raw_rmse_m=float(np.sqrt(np.mean([r['Raw_error_m']**2 for r in selected]))),
            one_iteration=sum(r['iterations']==1 for r in selected))
    # All diagnostic frames have i>0; uniform columns can be generated directly.
    output.mkdir(parents=True,exist_ok=False)
    write_csv(output/'causal_lineage_summary.csv',evidence)
    result=dict(first_divergence=first,jumps=jumps,visual_application_layers=metrics,
        velocity_bookkeeping='NDT_CORRECTION_APPLIED is before velocity_blend; v_mix=.4*v_before+.6*v_ndt; release includes actual IMU replay',
        limitation='Separate causal histories; no controlled same-state optimizer or velocity intervention in this task')
    (output/'lineage_metrics.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for flag in ('repo','gt','extrinsics','output'):p.add_argument('--'+flag,type=Path,required=True)
    inspect(**vars(p.parse_args()))
