#!/usr/bin/env python3
"""P30 numerical/point gates. This module never opens GT or runs localization."""
import argparse
import json
import struct
import sys
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'p29_feedback'))
from prepare import REQUESTS, rows, sha, write_csv
from analyze import cloud_array, normalized_params, frozen, G
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'p19_visual_tail'))
from compare import schedule_valid

def jump_stats(events):
    values=np.array([[float(r[k]) for k in ('p_x','p_y','p_z')] for r in events])
    jumps=np.linalg.norm(np.diff(values,axis=0),axis=1)
    return dict(max_adjacent_jump_m=float(jumps.max()),jumps_gt2m=int((jumps>2).sum()))

def inspect(repo,stage):
    cache=repo/'.p30_r1_scan_local_deskew';run=cache/'runs'/stage
    old=repo/'.p29_r1_fixed_source_feedback/runs/B'
    receipt=json.loads((run/'execution.json').read_text())
    if receipt['protocol']!='P30_R1_SCAN_LOCAL_IMU_DESKEW' or not receipt['completed']:
        raise ValueError('P30_RUN_NOT_COMPLETE')
    for path,digest in json.loads((cache/'input_freeze.json').read_text())['files_sha256'].items():
        if sha(path)!=digest: raise ValueError('P30_FROZEN_INPUT_CHANGED:'+path)
    params=yaml.safe_load((run/'rosparams.yaml').read_text())
    before=yaml.safe_load((old/'rosparams.yaml').read_text())
    x=normalized_params(params,run);y=normalized_params(before,old)
    if x['deskew'].pop('experimental_scan_local_imu_enable',False) is not True:
        raise ValueError('P30_FLAG_NOT_ENABLED')
    x['output'].pop('experimental_scan_local_motion_csv',None)
    if x!=y: raise ValueError('P30_UNAUTHORIZED_PARAMETERS_DIFFER')
    diags=rows(run/'ndt_diagnostics.csv');deskew=rows(run/'deskew.csv')
    mature=rows(run/'mature_diagnostics.csv');old_ndt=rows(old/'ndt_diagnostics.csv')
    high=[r for r in rows(run/'ekf_lineage.csv') if r['event_type']=='IMU_HIGH_RATE_PUBLISH']
    old_high=[r for r in rows(old/'ekf_lineage.csv') if r['event_type']=='IMU_HIGH_RATE_PUBLISH']
    if [r['state_stamp'] for r in high]!=[r['state_stamp'] for r in old_high[:len(high)]]:
        raise ValueError('P30_HIGH_RATE_TIMELINE_CHANGED')
    if [r['scan_ns'] for r in mature]!=[r['scan_ns'] for r in rows(old/'mature_diagnostics.csv')[:len(mature)]]:
        raise ValueError('P30_SCAN_TIMELINE_CHANGED')
    if not all(np.isfinite([float(r[k]) for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w')]).all() for r in high):
        raise ValueError('P30_NONFINITE_STATE')
    if len(deskew)!=len(diags) or any(r['status']!='PUBLISHED' or r['post_scan_end_imu_used']!='0' or
        float(r['max_imu_source_stamp'])>float(r['scan_end_stamp']) for r in deskew):
        raise ValueError('P30_DESKEW_TIME_OR_COMPLETENESS_FAILED')
    expected=sum(float(r['scan_end_stamp'])<=float(high[-1]['state_stamp']) for r in rows(old/'deskew.csv'))
    if len(diags)!=expected: raise ValueError('P30_COVERED_SCAN_MISSING')
    lag=rows(run/'oosm_fixed_lag.csv')
    if any(not schedule_valid(r) for r in lag): raise ValueError('P30_FIXED_LAG_INVALID')
    applied={r['observation_id'] for r in lag if r['status']=='APPLIED'}
    due={r['observation_id'] for r in lag if r['status']=='QUEUED' and float(r['deadline_stamp'])<=float(high[-1]['state_stamp'])}
    if applied!=due: raise ValueError('P30_DUE_NDT_TRANSACTIONS_LOST')
    if any('LATE' in r['status'] or 'NONFINITE' in r['status'] or 'INCOMPLETE' in r['status'] for r in lag):
        raise ValueError('P30_OOSM_ERROR')
    sqrt=rows(run/'sqrt_covariance_summary.csv')
    if any(int(r['invalid']) for r in sqrt): raise ValueError('P30_INVALID_COVARIANCE_ROOT')
    updates=rows(run/'msckf_diagnostics.csv')
    actual=sum(r['status']=='APPLIED_REPROJECTION_SELECTED_GAIN' for r in updates)
    if actual==0: raise ValueError('P30_NO_REAL_VISUAL_UPDATE')
    jump=jump_stats(high);old_jump=jump_stats(old_high[:len(high)])
    if jump['jumps_gt2m']>old_jump['jumps_gt2m']: raise ValueError('P30_NEW_GT2M_STATE_JUMP')
    if stage=='full' and (len(diags)!=396 or len(applied)!=396 or len(high)!=8004):
        raise ValueError('P30_FULL_FRAME_COUNT_FAILED')

    selected=[q for q in REQUESTS if int(q.split(':')[0])<=len(diags)]
    source=[];pose_compare=[]
    ext=repo/'.p18_map_recovery/posthoc_inputs/frozen.YOuGEy/corridor01_extrinsics.yaml'
    if sha(ext)!='59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d':
        raise ValueError('P30_CALIBRATION_CHANGED')
    Til=G.load_t_imu_lidar(ext)
    for request in selected:
        frame,ns=request.split(':');stem=cache/'sources'/stage/('frame_'+frame)
        meta=json.loads(stem.with_suffix('.source.json').read_text());data=stem.with_suffix('.xyz').read_bytes()
        h=1469598103934665603
        for byte in struct.pack('<IIII',meta['width'],meta['height'],int(meta['is_dense']),meta['points'])+data:
            h=((h^byte)*1099511628211)&((1<<64)-1)
        row=diags[int(frame)-1];hash_value=format(h,'016x')
        if meta['scan_ns']!=ns or mature[int(frame)-1]['scan_ns']!=ns or hash_value!=meta['stable_hash'] or hash_value!=row['cloud_hash'] or sha(stem.with_suffix('.xyz'))!=meta['xyz_sha256'] or len(data)!=meta['points']*12:
            raise ValueError('P30_SOURCE_CAPTURE_INVALID:'+request)
        poses=np.fromfile(stem.with_suffix('.poses.f64'),dtype='<f8')
        if poses.size!=80 or not np.isfinite(poses).all(): raise ValueError('P30_CAPTURED_POSE_INVALID')
        xc=np.frombuffer(data,dtype='<f4').reshape(-1,3).astype(float)
        xb=np.fromfile(repo/'.p29_r1_fixed_source_feedback/sources/B'/('frame_'+frame+'.xyz'),dtype='<f4').reshape(-1,3).astype(float)
        nearest=np.r_[cKDTree(xb).query(xc)[0],cKDTree(xc).query(xb)[0]]
        source.append(dict(frame_index=frame,scan_ns=ns,C_hash=hash_value,B_hash=old_ndt[int(frame)-1]['cloud_hash'],
            C_points=meta['points'],xyz_sha256=meta['xyz_sha256'],hash_validation='PASS',
            source_distance_method='symmetric_nearest_neighbor_NOT_ordered_pair',
            nearest_rms_m=float(np.sqrt(np.mean(nearest**2))),nearest_p95_m=float(np.percentile(nearest,95)),nearest_max_m=float(nearest.max())))
        for prefix in ('initial_guess_','raw_ndt_','final_used_'):
            a=frozen.pose_from_fields(old_ndt[int(frame)-1],prefix);b=frozen.pose_from_fields(row,prefix)
            a=G.lidar_pose_to_imu(dict(position=a[0],rotation=a[1]),Til);b=G.lidar_pose_to_imu(dict(position=b[0],rotation=b[1]),Til)
            pose_compare.append(dict(frame_index=frame,scan_ns=ns,layer=prefix,
                reference_point='IMU_origin',BC_translation_m=float(np.linalg.norm(a[0]-b[0])),
                BC_rotation_deg=G.rotation_angle_deg(a[1].T@b[1]),B_fitness=old_ndt[int(frame)-1]['ndt_fitness'],
                C_fitness=row['ndt_fitness'],B_iterations=old_ndt[int(frame)-1]['ndt_iterations'],C_iterations=row['ndt_iterations']))
    write_csv(cache/('source_comparison_'+stage+'.csv'),source)
    write_csv(cache/('pose_comparison_'+stage+'.csv'),pose_compare)

    import rosbag
    clouds={}
    for name,root in [('A',repo/'.p29_r1_fixed_source_feedback/runs/A'),('B',old),('C',run)]:
        with rosbag.Bag(str(root/'deskew_selected.bag')) as bag:
            clouds[name]={msg.header.stamp.to_nsec():msg for _,msg,_ in bag.read_messages()}
    point_rows=[]
    for request in selected:
        frame,ns=request.split(':');arrays={m:cloud_array(c[int(ns)]) for m,c in clouds.items()}
        if any(not np.array_equal(a['time'],arrays['C']['time']) for a in arrays.values()):
            raise ValueError('P30_ORDERED_POINT_TIMES_DIFFER')
        for left,right in [('A','B'),('B','C'),('A','C')]:
            a,b=[np.column_stack([arrays[m][k] for k in ('x','y','z')]).astype(float) for m in (left,right)]
            if a.shape!=b.shape: raise ValueError('P30_POINT_COUNT_CHANGED')
            d=np.linalg.norm(a-b,axis=1)
            point_rows.append(dict(frame_index=frame,scan_ns=ns,comparison=left+right,points=len(d),
                rms_m=float(np.sqrt(np.mean(d*d))),p95_m=float(np.percentile(d,95)),max_m=float(d.max())))
    write_csv(cache/('point_comparison_'+stage+'.csv'),point_rows)
    motion=rows(run/'scan_local_motion.csv');motion_summary=[]
    for request in selected:
        frame,ns=request.split(':');g=[r for r in motion if r['scan_ns']==ns]
        if not g or any(float(r['anchor_state_stamp'])>float(r['scan_start']) or
            float(r['imu_source_stamp'])>float(r['node_stamp']) or
            float(r['max_imu_source_stamp'])>float(r['scan_end']) for r in g):
            raise ValueError('P30_MOTION_DIAGNOSTIC_TIME_CONTRACT')
        output=dict(frame_index=frame,scan_ns=ns,nodes=len(g),comparison='local_vs_legacy_same_CURRENT_C_history_NOT_P29_reconstruction')
        for field in ('history_relative_delta_m','history_relative_delta_deg','history_boundary_residual_m',
                      'history_boundary_residual_deg','local_boundary_residual_m','local_boundary_residual_deg'):
            values=np.array([float(r[field]) for r in g]);values=values[np.isfinite(values)]
            output[field+'_max']=float(values.max()) if len(values) else None
        motion_summary.append(output)
    write_csv(cache/('motion_summary_'+stage+'.csv'),motion_summary)
    result=dict(protocol='P30_NUMERICAL_RUNTIME_GATE',GT_LOADED=False,stage=stage,status='PASS',
        NDT=len(diags),converged=sum(r['ndt_has_converged']=='1' for r in diags),OOSM_APPLIED=len(applied),
        HighRate=len(high),MSCKF=actual,captured=len(selected),root_invalid=0,nonfinite=0,future_imu=0,
        timestamp_parity=True,physical_parameter_parity=True,jumps=jump,reference_jumps=old_jump,
        source=source,points=point_rows,motion=motion_summary)
    (cache/('gate_'+stage+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--stage',choices=['short','full'],required=True)
    inspect(**vars(p.parse_args()))
