"""No-GT sensor-keyed velocity intervention and first-propagation evidence."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
import yaml
from audit import paired,rows,pose_delta,SCRIPTS
sys.path.insert(0,str(SCRIPTS/'p10_corridor01_deterministic_oosm'))
import evaluate_frozen_v2 as frozen
G=frozen.geometry
sys.path.insert(0,str(SCRIPTS/'p19_visual_tail'))
from compare import write_csv


def velocity_mix(before,ndt,beta,eligible=True):
    return (1-beta)*before+beta*ndt if eligible else before.copy()


def state(row):
    return [float(row[k]) for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w')]


def before_intervention(row,first_scan,first_release):
    event=row['event_type']
    if event=='IMU_HIGH_RATE_PUBLISH':return float(row['state_stamp'])<first_release
    if event=='CORRECTED_PUBLISH':return float(row['last_ndt_observation_time'])<first_scan
    # Later OOSM events may carry earlier historical state timestamps.
    obs=float(row['ndt_measurement_stamp'])
    return obs<=first_scan if event in ('OOSM_ROLLBACK','NDT_CORRECTION_APPLIED') else obs<first_scan


def used_imu(row,params):
    cfg=params['deskew'];ril=np.array(cfg['T_imu_lidar_rotation_row_major']).reshape(3,3)
    ril=G.quat_to_rotation(G.rotation_to_quaternion(ril))
    p,r=frozen.pose_from_fields(row,'final_used_')
    return p-(r@ril.T)@np.array(cfg['T_imu_lidar_translation_m'])


def inspect(repo,run,output):
    roots={'blend_0.6':repo/'docs/p19_r1_visual_replication_tail_risk/runs/repeat_visual','blend_0.0':run}
    ds={m:rows(p/'ndt_diagnostics.csv') for m,p in roots.items()}
    es={m:rows(p/'ekf_lineage.csv') for m,p in roots.items()}
    paired(*ds.values(),'lidar_header_stamp',396)
    for event in ('OOSM_ROLLBACK','NDT_CORRECTION_APPLIED','OOSM_REPLAY_COMPLETE'):
        a,b=[[r for r in es[m] if r['event_type']==event] for m in roots]
        paired(a,b,'ndt_measurement_stamp',396)
        for method in roots:
            observations=[r['ndt_measurement_stamp'] for r in es[method] if r['event_type']==event]
            if observations!=[r['lidar_header_stamp'] for r in ds[method]]:raise ValueError('LINEAGE_OBSERVATION_ASSOCIATION')
    records=[];first={};origin=1517157224.188979
    for method,root in roots.items():
        params=yaml.safe_load((root/'rosparams.yaml').read_text());beta=params['ndt_observation']['velocity_blend']
        lookup={(r['event_type'],r['ndt_measurement_stamp']):r for r in es[method] if r['event_type'] in ('NDT_CORRECTION_APPLIED','OOSM_ROLLBACK','OOSM_REPLAY_COMPLETE')}
        corrected={r['last_ndt_observation_time']:r for r in es[method] if r['event_type']=='CORRECTED_PUBLISH'}
        last_position=None
        for i,d in enumerate(ds[method]):
            if not np.isfinite(float(d['ndt_fitness'])):raise ValueError('NONFINITE_NDT_FITNESS')
            stamp=d['lidar_header_stamp'];cor=lookup[('NDT_CORRECTION_APPLIED',stamp)]
            release=lookup[('OOSM_REPLAY_COMPLETE',stamp)];pos=used_imu(d,params)
            dt=float(stamp)-float(cor['last_ndt_observation_time']);eligible=1e-3<dt<1.
            if eligible and last_position is None:raise ValueError('MISSING_PREVIOUS_MEASUREMENT')
            vn=(pos-last_position)/dt if eligible else np.zeros(3)
            vn[2]*=params['ndt_observation']['z_apply_ratio']
            before=np.array([float(cor['v_'+k]) for k in 'xyz']);mixed=velocity_mix(before,vn,beta,eligible)
            record=dict(method=method,frame=i+1,scan_stamp=float(stamp),relative_time_s=float(stamp)-origin,
                rollback_stamp=float(cor['state_stamp']),release_stamp=float(release['state_stamp']),
                dt_s=dt,eligible=eligible,beta=beta,applied_visual=d['initial_guess_source']=='p4_visual_translation_imu_rotation',
                cloud_hash=d['cloud_hash'],iterations=int(d['ndt_iterations']),fitness=float(d['ndt_fitness']),
                translation_limited=d['translation_limited'],rotation_limited=d['rotation_limited'])
            for k,j in zip('xyz',range(3)):
                record['ndt_v_'+k]=float(vn[j]);record['before_v_'+k]=float(before[j]);record['computed_mix_v_'+k]=float(mixed[j])
                record['release_v_'+k]=float(release['v_'+k]);record['used_imu_'+k]=float(pos[j])
                record['initial_lidar_'+k]=float(d['initial_guess_t'+k]);record['raw_lidar_'+k]=float(d['raw_ndt_t'+k])
            records.append(record);last_position=pos
    eligible=[r for r in records if r['method']=='blend_0.6' and r['eligible']]
    first_frame=eligible[0]['frame'];first_release=eligible[0]['release_stamp']
    pair=[next(r for r in records if r['method']==m and r['frame']==first_frame) for m in roots]
    expected=np.array([pair[1]['computed_mix_v_'+k]-pair[0]['computed_mix_v_'+k] for k in 'xyz'])
    actual=np.array([pair[1]['release_v_'+k]-pair[0]['release_v_'+k] for k in 'xyz'])
    residual=float(np.linalg.norm(actual-expected))
    if residual>1e-10:raise ValueError('FIRST_RELEASE_VELOCITY_DOES_NOT_MATCH_INTERVENTION')
    first['first_eligible_mix']=dict(frame=first_frame,stamp=pair[0]['scan_stamp'],release_stamp=first_release,
        beta06=pair[0],beta0=pair[1],release_delta_matches_mix_delta_mps=residual)
    prefix_checks=0
    for event in ('IMU_HIGH_RATE_PUBLISH','CORRECTED_PUBLISH','OOSM_ROLLBACK','NDT_CORRECTION_APPLIED','OOSM_REPLAY_COMPLETE'):
        a,b=[[r for r in es[m] if r['event_type']==event] for m in roots]
        if len(a)!=len(b):raise ValueError('EVENT_COUNT')
        for x,y in zip(a,b):
            if x['state_stamp']!=y['state_stamp']:raise ValueError('EVENT_TIME_CHANGED')
            if event in ('OOSM_ROLLBACK','NDT_CORRECTION_APPLIED','OOSM_REPLAY_COMPLETE') and x['state_now_before_ndt']!=y['state_now_before_ndt']:
                raise ValueError('EVENT_PROCESSING_NODE_CHANGED')
            dt,dr=pose_delta(state(x),state(y));dv=float(np.linalg.norm([float(y['v_'+k])-float(x['v_'+k]) for k in 'xyz']))
            before_mix=before_intervention(x,pair[0]['scan_stamp'],first_release)
            if before_mix:
                prefix_checks+=1
                if dt>1e-9 or dr>1e-8 or dv!=0:raise ValueError('UNEXPLAINED_PREINTERVENTION_DIFFERENCE:'+event)
            for kind,changed,amplitude in [('pose',dt>1e-9 or dr>1e-8,dt),('velocity',dv!=0,dv)]:
                key=event+'_'+kind
                if key not in first and changed:
                    process_stamp=float(x['state_now_before_ndt'])
                    observation_stamp=float(x['ndt_measurement_stamp'])
                    first[key]=dict(stamp=float(x['state_stamp']),relative_time_s=float(x['state_stamp'])-origin,
                        processing_state_stamp=process_stamp if np.isfinite(process_stamp) else float(x['state_stamp']),
                        observation_stamp=observation_stamp if np.isfinite(observation_stamp) else float(x['last_ndt_observation_time']),
                        translation_difference_m=dt,rotation_difference_deg=dr,velocity_difference_mps=dv)
    for i,(a,b) in enumerate(zip(*ds.values())):
        if i+1<=first_frame:
            if a['cloud_hash']!=b['cloud_hash']:raise ValueError('SOURCE_CHANGED_BEFORE_INTERVENTION')
        for label,prefix in [('Initial','initial_guess_'),('Raw','raw_ndt_'),('Used','final_used_')]:
            dt,dr=pose_delta([a[prefix+k] for k in ('tx','ty','tz','qx','qy','qz','qw')],
                            [b[prefix+k] for k in ('tx','ty','tz','qx','qy','qz','qw')])
            if i+1<=first_frame and (dt>1e-9 or dr>1e-8):raise ValueError('NDT_CHANGED_BEFORE_INTERVENTION')
            if label not in first and (dt>1e-9 or dr>1e-8):first[label]=dict(frame=i+1,stamp=float(a['lidar_header_stamp']),
                relative_time_s=float(a['lidar_header_stamp'])-origin,translation_difference_m=dt,rotation_difference_deg=dr)
        if 'source_hash' not in first and a['cloud_hash']!=b['cloud_hash']:
            first['source_hash']=dict(frame=i+1,stamp=float(a['lidar_header_stamp']),relative_time_s=float(a['lidar_header_stamp'])-origin,
                baseline_hash=a['cloud_hash'],experiment_hash=b['cloud_hash'])
    desk=[rows(p/'deskew.csv') for p in roots.values()]
    for i,(a,b) in enumerate(zip(*desk)):
        fields=('point_displacement_mean_m','point_displacement_p95_m','max_velocity_norm_mps')
        changed=[k for k in fields if a[k]!=b[k]]
        if changed:
            first['Deskew']=dict(frame=i+1,stamp=float(a['reference_stamp']),scan_end_stamp=float(a['scan_end_stamp']),
                relative_time_s=float(a['reference_stamp'])-origin,changed_fields=changed,
                baseline={k:a[k] for k in changed},experiment={k:b[k] for k in changed});break
    output.mkdir(parents=True,exist_ok=False);write_csv(output/'velocity_lineage.csv',records)
    write_csv(output/'first_divergence.csv',[dict(kind=k,details_json=json.dumps(v)) for k,v in first.items()])
    result=dict(VELOCITY_INTERVENTION='VELOCITY_INTERVENTION_CONFIRMED',GT_LOADED=False,first=first,
        preintervention_checks=prefix_checks,first_release_residual_mps=residual,
        mixed_velocity_scope='computed from recorded pre-blend inputs and unchanged C++ formula; release velocity is observed after replay',
        deskew_time_note='Scan-start may precede the intervention release; scan-end and actual processing must be considered')
    (output/'mechanism.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for key in ('repo','run','output'):p.add_argument('--'+key,type=Path,required=True)
    inspect(**vars(p.parse_args()))
