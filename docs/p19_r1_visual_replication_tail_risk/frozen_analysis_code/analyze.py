"""Bounded posthoc P19 tail analysis, using the unchanged P10 transform."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from compare import rows, write_csv, pose_delta

SCRIPTS=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(SCRIPTS/'p10_corridor01_deterministic_oosm'))
import evaluate_frozen_v2 as frozen
G=frozen.geometry
sys.path.insert(0,str(SCRIPTS/'p17_visual'))
from evaluate_p18 import sha


def error(stamp, pose, gt_t, gt_p, rf, tf):
    gt=G.interpolate_gt(gt_t,gt_p,stamp)
    if gt is None:raise ValueError('MISSING_GT:'+str(stamp))
    et,er=G.errors_for([pose],[gt],rf,tf)
    e=pose[0]-rf.T@(gt[0]-tf)
    if abs(np.linalg.norm(e)-et[0])>1e-8:raise ValueError('MAP_ERROR_TRANSFORM')
    return dict(translation_error_m=float(et[0]),rotation_error_deg=float(er[0]),
                map_error_x_m=float(e[0]),map_error_y_m=float(e[1]),map_error_z_m=float(e[2]))


def state_pose(row):
    return np.array([float(row[k]) for k in ('p_x','p_y','p_z')]), G.quat_to_rotation(
        [float(row[k]) for k in ('q_x','q_y','q_z','q_w')])


def statistics(data):
    t=np.array([r['translation_error_m'] for r in data]);rot=np.array([r['rotation_error_deg'] for r in data])
    return dict(count=len(t),translation_rmse_m=float(np.sqrt(np.mean(t*t))),
        translation_p95_m=float(np.percentile(t,95)),translation_max_m=float(t.max()),
        rotation_rmse_deg=float(np.sqrt(np.mean(rot*rot))),rotation_p95_deg=float(np.percentile(rot,95)),
        rotation_max_deg=float(rot.max()),over_1m=int(sum(t>1)),over_2m=int(sum(t>2)),over_5m=int(sum(t>5)))


def analyze(repo, repeat, freeze, gt, extrinsics, output):
    receipt=json.loads(freeze.read_text())
    if receipt['GT_LOADED'] is not False:raise ValueError('NO_PRE_GT_FREEZE')
    for name,digest in receipt['files_sha256'].items():
        if sha(name)!=digest:raise ValueError('FROZEN_FILE_CHANGED:'+name)
    if sha(gt)!='3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03':raise ValueError('GT_SHA')
    if sha(extrinsics)!='59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d':raise ValueError('EXT_SHA')
    gt_t,gt_p=G.load_gt(gt);til=G.load_t_imu_lidar(extrinsics)
    historical=repo/'docs/p10_corridor01_deterministic_oosm_replay_r1'
    contract=json.loads((historical/'posthoc_metrics.json').read_text())
    origin=contract['evaluation_origin_unix_s'];rf=np.array(contract['fit_transform_rotation'])
    tf=np.array(contract['fit_transform_translation_m'])
    roots={'Control':historical/'runs/control_v2','P18_Visual':repo/'.p18_map_recovery/runs/visual_full',
           'P19_Repeat':repeat}
    state_rows={};diagnostics={};errors={};per_state=[];summaries=[]
    windows={'3_35s':(3,35),'6_12s':(6,12),'26_30s':(26,30),'30_35s':(30,35),'32_35s':(32,35)}
    for method,root in roots.items():
        state_rows[method]=[r for r in rows(root/'ekf_lineage.csv') if r['event_type']=='IMU_HIGH_RATE_PUBLISH']
        diagnostics[method]=rows(root/'ndt_diagnostics.csv')
        data=[]
        for r in state_rows[method]:
            stamp=float(r['state_stamp']);rel=stamp-origin
            if not 0<=rel<=35:continue
            p=state_pose(r);er=error(stamp,p,gt_t,gt_p,rf,tf)
            # Offline central GT motion tangent, not a sensor input/Hessian direction.
            a,b=[G.interpolate_gt(gt_t,gt_p,stamp+d) for d in (-.05,.05)]
            tangent=rf.T@(b[0]-a[0]) if a is not None and b is not None else np.zeros(3)
            e=np.array([er['map_error_'+axis+'_m'] for axis in 'xyz'])
            norm=np.linalg.norm(tangent)
            along=float(e@(tangent/norm)) if norm>1e-6 else None
            lateral=float(np.sqrt(max(0.,e@e-along*along))) if along is not None else None
            er.update(method=method,stamp=stamp,relative_time_s=rel,
                longitudinal_error_m=along,lateral_error_m=lateral,
                latest_ndt_stamp=float(r['last_ndt_observation_time']))
            data.append(er)
            if 26<=rel<=35:per_state.append(er)
        errors[method]=data
        for window,(start,end) in windows.items():
            summaries.append(dict(method=method,window=window,**statistics([r for r in data if start<=r['relative_time_s']<=end])))
        for start in range(26,35):
            summaries.append(dict(method=method,window=f'{start}_{start+1}s',**statistics(
                [r for r in data if start<=r['relative_time_s']<start+1])))
    for method in ('P18_Visual','P19_Repeat'):
        if [r['stamp'] for r in errors[method]]!=[r['stamp'] for r in errors['Control']]:raise ValueError('UNPAIRED_STATES')
    peaks=[];peak_detail={}
    for peak_method in ('Control','P18_Visual'):
        peak=max([r for r in errors[peak_method] if r['relative_time_s']>=3],key=lambda r:r['translation_error_m'])
        stamp=peak['stamp'];event={}
        for method in roots:
            state=next(r for r in state_rows[method] if float(r['state_stamp'])==stamp)
            same=next(r for r in errors[method] if r['stamp']==stamp)
            obs=float(state['last_ndt_observation_time'])
            ndt=next(r for r in diagnostics[method] if float(r['lidar_header_stamp'])==obs)
            latest=max(r for r in (float(d['lidar_header_stamp']) for d in diagnostics[method]
                if d['initial_guess_source']=='p4_visual_translation_imu_rotation') if r<=stamp) if method!='Control' else None
            erows=rows(roots[method]/'ekf_lineage.csv')
            corrected=next(r for r in erows if r['event_type']=='CORRECTED_PUBLISH' and float(r['ndt_measurement_stamp'])==obs)
            e=dict(peak_method=peak_method,method=method,peak_stamp=stamp,relative_time_s=stamp-origin,
                latest_accepted_ndt_stamp=obs,scan_relative_time_s=obs-origin,
                corrected_state_stamp=float(corrected['state_stamp']),
                direct_visual=ndt['initial_guess_source']=='p4_visual_translation_imu_rotation',
                last_visual_stamp=latest,last_visual_age_s=stamp-latest if latest is not None else None,
                fitness=float(ndt['ndt_fitness']),iterations=int(ndt['ndt_iterations']),
                translation_limited=ndt['translation_limited'],rotation_limited=ndt['rotation_limited'])
            for component,prefix in [('Initial_Guess','initial_guess_'),('Raw_NDT','raw_ndt_'),('Final_Used','final_used_')]:
                pose=G.lidar_pose_to_imu(dict(zip(('position','rotation'),frozen.pose_from_fields(ndt,prefix))),til)
                value=error(obs,pose,gt_t,gt_p,rf,tf)
                peaks.append(dict(**e,component=component,component_stamp=obs,
                    imu_map_x=pose[0][0],imu_map_y=pose[0][1],imu_map_z=pose[0][2],**value))
            for component,row in [('EKF_Corrected',corrected),('EKF_HighRate',state)]:
                pose=state_pose(row);actual_stamp=float(row['state_stamp'])
                peaks.append(dict(**e,component=component,component_stamp=actual_stamp,
                    imu_map_x=pose[0][0],imu_map_y=pose[0][1],imu_map_z=pose[0][2],
                    **error(actual_stamp,pose,gt_t,gt_p,rf,tf)))
            event[method]=dict(**e,peak_error=same,high_rate_position=state_pose(state)[0].tolist(),
                              raw_lidar_position=[float(ndt['raw_ndt_'+k]) for k in ('tx','ty','tz')])
        peak_detail[peak_method]=event
    output.mkdir(parents=True,exist_ok=False)
    write_csv(output/'tail_error_timeline.csv',per_state)
    write_csv(output/'tail_window_summary.csv',summaries)
    write_csv(output/'tail_peak_events.csv',peaks)
    result=dict(GT_LOADED=True,origin=origin,peaks=peak_detail,window_metrics=summaries,
        motion_axis='offline GT central 0.1s tangent transported to map; not Hessian weak direction')
    (output/'tail_metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(peak_detail,indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for flag in ('repo','repeat','freeze','gt','extrinsics','output'):p.add_argument('--'+flag,type=Path,required=True)
    a=p.parse_args();analyze(**vars(a))
