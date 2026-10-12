"""Admit actual captured clouds and compare the full P19 physical baseline; no GT."""
import argparse
import json
import struct
import sys
from pathlib import Path
import numpy as np
from prepare_capture import sha
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'p19_visual_tail'))
from compare import Checks,rows,pose_delta,schedule_valid,write_csv


def gate(repo):
    repo=repo.resolve();cache=repo/'.p23_r1_budgeted_coupled_subspace_refinement'
    run=cache/'runs/capture_visual';old=repo/'docs/p19_r1_visual_replication_tail_risk/runs/repeat_visual'
    output=cache/'capture_gate';output.mkdir(exist_ok=False)
    a,b=rows(old/'ndt_diagnostics.csv'),rows(run/'ndt_diagnostics.csv')
    c=Checks();prefixes=('baseline_guess_','imu_rotation_guess_','initial_guess_','previous_pose_',
        'delta_pose_','raw_ndt_','used_before_step_limit_','final_used_')
    pose_keys={prefix+key for prefix in prefixes for key in ('tx','ty','tz','qx','qy','qz','qw')}
    timing={'ros_now','wall_time','ndt_align_complete_wall_sec','ndt_odom_publish_wall_sec',
            'local_imu_history_first_stamp','local_imu_history_last_stamp','pending_lidar_count'}
    projected=[{key:r[key] for key in a[0]} for r in b]
    c.table('NDT',a,projected,396,timing|pose_keys)
    for i,(left,right) in enumerate(zip(a,b),1):
        for prefix in prefixes:
            x,y=[[float(r[prefix+key]) for key in ('tx','ty','tz','qx','qy','qz','qw')] for r in (left,right)]
            if np.isnan(x).all() and np.isnan(y).all() and i==1:continue
            dt,dr=pose_delta(x,y)
            c.check('NDT',i,left['lidar_header_stamp'],prefix+'position_m',0,dt,1e-9)
            c.check('NDT',i,left['lidar_header_stamp'],prefix+'rotation_deg',0,dr,1e-8)
    for name,stage,excluded in [('deskew.csv','Deskew',{'deskew_processing_ms'}),('oosm.csv','OOSM',set())]:
        c.table(stage,rows(old/name),rows(run/name),396,excluded)
    for root,label in [(old,'P19'),(run,'P23')]:
        lag=rows(root/'oosm_fixed_lag.csv')
        for status in ('QUEUED','APPLIED'):
            c.check('FixedLag',0,None,label+'_'+status,396,sum(r['status']==status for r in lag))
        c.check('FixedLag',0,None,label+'_late_or_other',0,sum(r['status'] not in ('QUEUED','APPLIED') for r in lag))
        for i,row in enumerate(lag,1):c.check('FixedLag',i,row['ndt_stamp'],label+'_deadline',True,schedule_valid(row))
    c.table('FixedLag',[r for r in rows(old/'oosm_fixed_lag.csv') if r['status']=='APPLIED'],
            [r for r in rows(run/'oosm_fixed_lag.csv') if r['status']=='APPLIED'],396,
            {'arrival_state_stamp','arrival_wall_sec','processing_wall_sec'})
    ea,eb=rows(old/'ekf_lineage.csv'),rows(run/'ekf_lineage.csv')
    for event,expected in [('IMU_HIGH_RATE_PUBLISH',8004),('CORRECTED_PUBLISH',396),
                          ('OOSM_ROLLBACK',396),('NDT_CORRECTION_APPLIED',396),('OOSM_REPLAY_COMPLETE',396)]:
        left,right=[[r for r in table if r['event_type']==event] for table in (ea,eb)]
        c.table(event,left,right,expected,{'event_index','event_ros_stamp','p_x','p_y','p_z','q_x','q_y','q_z','q_w'})
        for i,(x,y) in enumerate(zip(left,right),1):
            c.check(event,i,y['state_stamp'],'velocity_finite',True,
                    np.isfinite([float(y[key]) for key in ('v_x','v_y','v_z')]).all().item())
            dt,dr=pose_delta([x[key] for key in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w')],
                             [y[key] for key in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w')])
            c.check(event,i,x['state_stamp'],'position_m',0,dt,1e-9)
            c.check(event,i,x['state_stamp'],'rotation_deg',0,dr,1e-8)
    c.check('Completeness',0,None,'visual',169,sum(r['visual_applied']=='1' for r in b))
    c.check('Completeness',0,None,'converged',396,sum(r['ndt_has_converged']=='1' for r in b))
    c.check('Completeness',0,None,'P21_applications',0,sum(r['ekf_translation_applied']=='1' for r in b))
    for i,row in enumerate(b,1):
        c.check('Completeness',i,row['lidar_header_stamp'],'overflow',0,int(row['pending_overflow_count']))
    receipt=json.loads((run/'execution.json').read_text())
    c.check('Completeness',0,None,'runner_completed',True,receipt['completed'])
    freeze=json.loads((cache/'runtime_input_freeze.json').read_text())
    source_rows=[]
    by_frame={r['frame_index']:r for r in b}
    for chosen in freeze['selection']:
        frame=chosen['frame_index'];prefix=cache/'sources'/('frame_'+frame)
        metadata=json.loads(prefix.with_suffix('.source.json').read_text())
        xyz=prefix.with_suffix('.xyz').read_bytes();pose_path=prefix.with_suffix('.poses.f64')
        pose=np.fromfile(pose_path,dtype='<f8')
        h=1469598103934665603
        header=struct.pack('<IIII',metadata['width'],metadata['height'],int(metadata['is_dense']),metadata['points'])
        for byte in header+xyz:h=((h^byte)*1099511628211)&((1<<64)-1)
        actual=format(h,'016x');diag=by_frame[frame]
        valid=(str(metadata['frame_index'])==frame and metadata['scan_ns']==chosen['scan_ns']
            and len(xyz)==metadata['xyz_bytes']==metadata['points']*12
            and metadata['points']==int(diag['cloud_size_after_filter'])
            and actual==metadata['stable_hash']==diag['cloud_hash']
            and sha(prefix.with_suffix('.xyz'))==metadata['xyz_sha256']
            and pose.size==80 and np.isfinite(pose).all())
        source_rows.append(dict(frame_index=frame,scan_ns=metadata['scan_ns'],points=metadata['points'],
            online_hash=diag['cloud_hash'],saved_hash=actual,xyz_sha256=metadata['xyz_sha256'],
            xyz_bytes=len(xyz),poses_sha256=sha(pose_path),status='PASS' if valid else 'FAIL'))
    write_csv(output/'source_capture_summary.csv',source_rows)
    write_csv(output/'source_hash_parity.csv',source_rows)
    write_csv(output/'baseline_parity.csv',list(c.summary.values()))
    status='SOURCE_CAPTURE_PASS' if sum(r['status']=='PASS' for r in source_rows)>=4 else 'SOURCE_CAPTURE_INSUFFICIENT'
    result=dict(SOURCE_CAPTURE=status,BASELINE_PARITY='PASS' if not c.failures else 'SOURCE_CAPTURE_PARITY_FAILED',
                captured=len(source_rows),valid=sum(r['status']=='PASS' for r in source_rows),
                failures=c.failures[:20],failure_count=len(c.failures),GT_LOADED=False)
    (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if c.failures or status!='SOURCE_CAPTURE_PASS':raise ValueError('P23_A_GATE_FAILED')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--repo',type=Path,required=True)
    gate(**vars(parser.parse_args()))
