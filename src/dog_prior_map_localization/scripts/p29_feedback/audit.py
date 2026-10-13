#!/usr/bin/env python3
"""Admit real captures and compare P28 physical outputs before any GT use."""
import argparse
import json
import struct
import sys
from pathlib import Path
import numpy as np
from prepare import REQUESTS, rows, sha, write_csv
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'p19_visual_tail'))
from compare import Checks, pose_delta, schedule_valid


def audit(repo, method):
    cache=repo/'.p29_r1_fixed_source_feedback'; run=cache/'runs'/method
    old=repo/'docs/p28_r2_mature_visual_module/runs'/method
    receipt=json.loads((run/'execution.json').read_text())
    if not receipt['completed'] or receipt['protocol']!='P29_R1_SOURCE_CAPTURE':
        raise ValueError('CAPTURE_RUN_INCOMPLETE')
    a,b=rows(old/'ndt_diagnostics.csv'),rows(run/'ndt_diagnostics.csv')
    c=Checks()
    prefixes=('baseline_guess_','imu_rotation_guess_','initial_guess_',
              'previous_pose_','delta_pose_','raw_ndt_','used_before_step_limit_','final_used_')
    keys={p+k for p in prefixes for k in ('tx','ty','tz','qx','qy','qz','qw')}
    ignore={'ros_now','wall_time','ndt_align_complete_wall_sec','ndt_odom_publish_wall_sec',
            'local_imu_history_first_stamp','local_imu_history_last_stamp','pending_lidar_count'}
    c.table('NDT',a,b,396,ignore|keys)
    for i,(x,y) in enumerate(zip(a,b),1):
        for prefix in prefixes:
            left,right=[[float(r[prefix+k]) for k in ('tx','ty','tz','qx','qy','qz','qw')] for r in (x,y)]
            if i==1 and prefix in ('previous_pose_','delta_pose_','imu_rotation_guess_') and np.isnan(left).all() and np.isnan(right).all():
                continue
            dt,dr=pose_delta(left,right)
            c.check('NDT',i,x['lidar_header_stamp'],prefix+'position_m',0,dt,1e-9)
            c.check('NDT',i,x['lidar_header_stamp'],prefix+'rotation_deg',0,dr,1e-8)
    for name,label,excluded in [('deskew.csv','Deskew',{'deskew_processing_ms'}),('oosm.csv','OOSM',set())]:
        c.table(label,rows(old/name),rows(run/name),396,excluded)
    for root,label in ((old,'P28'),(run,'P29')):
        lag=rows(root/'oosm_fixed_lag.csv')
        if any(not schedule_valid(r) for r in lag):
            raise ValueError('FIXED_LAG_SENSOR_TIME_CONTRACT_FAILED')
        if sum(r['status']=='APPLIED' for r in lag)!=396:
            raise ValueError('FIXED_LAG_COMPLETENESS_FAILED')
    c.table('FixedLag',[r for r in rows(old/'oosm_fixed_lag.csv') if r['status']=='APPLIED'],
            [r for r in rows(run/'oosm_fixed_lag.csv') if r['status']=='APPLIED'],396,
            {'arrival_state_stamp','arrival_wall_sec','processing_wall_sec'})
    for event,expected in [('IMU_HIGH_RATE_PUBLISH',8004),('CORRECTED_PUBLISH',396)]:
        left,right=[[r for r in rows(root/'ekf_lineage.csv') if r['event_type']==event] for root in (old,run)]
        c.table(event,left,right,expected,{'event_index','event_ros_stamp','p_x','p_y','p_z','q_x','q_y','q_z','q_w'})
        for i,(x,y) in enumerate(zip(left,right),1):
            dt,dr=pose_delta([x[k] for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w')],
                             [y[k] for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w')])
            c.check(event,i,x['state_stamp'],'position_m',0,dt,1e-9)
            c.check(event,i,x['state_stamp'],'rotation_deg',0,dr,1e-8)
    ma,mb=rows(old/'msckf_diagnostics.csv'),rows(run/'msckf_diagnostics.csv')
    c.table('MSCKF',ma,mb,len(ma),{'update_ms'})
    # Ordered little-endian XYZ are checked independently of the noncrypto hash.
    summary=[]; cases=[]
    diag={int(r['frame_index']):r for r in b}
    mature={int(r['frame_index']):r for r in rows(run/'mature_diagnostics.csv')}
    for request in REQUESTS:
        frame,ns=request.split(':'); prefix=cache/'sources'/method/('frame_'+frame)
        meta=json.loads(prefix.with_suffix('.source.json').read_text())
        xyz=prefix.with_suffix('.xyz').read_bytes(); pose=np.fromfile(prefix.with_suffix('.poses.f64'),dtype='<f8')
        h=1469598103934665603
        for byte in struct.pack('<IIII',meta['width'],meta['height'],int(meta['is_dense']),meta['points'])+xyz:
            h=((h^byte)*1099511628211)&((1<<64)-1)
        row=diag[int(frame)]; stable=format(h,'016x')
        valid=(meta['frame_index']==int(frame) and meta['scan_ns']==ns and mature[int(frame)]['scan_ns']==ns
               and len(xyz)==meta['xyz_bytes']==meta['points']*12 and meta['points']==int(row['cloud_size_after_filter'])
               and stable==meta['stable_hash']==row['cloud_hash'] and sha(prefix.with_suffix('.xyz'))==meta['xyz_sha256']
               and pose.size==80 and np.isfinite(pose).all())
        if not valid:
            raise ValueError('SOURCE_CAPTURE_HASH_OR_POSE_FAILED:'+request)
        summary.append(dict(method=method,frame_index=frame,scan_ns=ns,points=meta['points'],
            saved_hash=stable,online_hash=row['cloud_hash'],p28_hash=a[int(frame)-1]['cloud_hash'],
            xyz_sha256=meta['xyz_sha256'],source_status='PASS',p28_hash_match=stable==a[int(frame)-1]['cloud_hash']))
        cases.append(' '.join(map(str,(frame,ns,meta['points'],meta['width'],meta['height'],int(meta['is_dense']),stable,
            row['ndt_iterations'],row['ndt_has_converged'],row['ndt_fitness'],row['translation_limited'],row['rotation_limited'],
            mature[int(frame)]['selected_candidate']))))
    passive=json.loads((run/'deskew_capture.json').read_text())
    if passive['captured']!=8 or {r['scan_ns'] for r in passive['records']}!={r.split(':')[1] for r in REQUESTS}:
        raise ValueError('SELECTED_DESKEW_CAPTURE_INCOMPLETE')
    write_csv(cache/('source_hash_parity_'+method+'.csv'),summary)
    write_csv(cache/('p28_replay_parity_'+method+'.csv'),list(c.summary.values()))
    (cache/('cases_'+method+'.txt')).write_text('\n'.join(cases)+'\n')
    result=dict(method=method,GT_LOADED=False,captured=8,source_capture='PASS',
                p28_parity='PASS' if not c.failures else 'FAILED_NOT_EXACT_REPLAY',
                first_differences=c.failures[:10],failure_count=len(c.failures))
    (cache/('capture_gate_'+method+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--method',choices=['A','B'],required=True)
    audit(**vars(p.parse_args()))
