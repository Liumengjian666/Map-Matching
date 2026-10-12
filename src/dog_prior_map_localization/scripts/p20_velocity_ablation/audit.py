"""P20 pre-GT completeness and fixed sensor-time contract (not pose parity)."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
import yaml
from run_guarded import parameter_check
SCRIPTS=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(SCRIPTS/'p19_visual_tail'))
from compare import rows, schedule_valid, pose_delta
sys.path.insert(0,str(SCRIPTS/'p17_visual'))
from evaluate_p18 import sha


def paired(a,b,key,count):
    aa,bb=[[r[key] for r in table] for table in (a,b)]
    if len(aa)!=count or len(bb)!=count or aa!=bb or len(set(aa))!=count:
        raise ValueError('SENSOR_TIME_OR_COUNT_CONTRACT:'+key)
    values=np.array(aa,dtype=float)
    if not np.isfinite(values).all() or not np.all(np.diff(values)>0):raise ValueError('NONFINITE_OR_UNORDERED:'+key)


def audit(repo,run,output):
    baseline=repo/'docs/p19_r1_visual_replication_tail_risk/runs/repeat_visual'
    a,b=[rows(p/'ndt_diagnostics.csv') for p in (baseline,run)]
    paired(a,b,'lidar_header_stamp',396)
    ea,eb=[rows(p/'ekf_lineage.csv') for p in (baseline,run)]
    for event,count in [('IMU_HIGH_RATE_PUBLISH',8004),('CORRECTED_PUBLISH',396)]:
        paired([r for r in ea if r['event_type']==event],[r for r in eb if r['event_type']==event],'state_stamp',count)
    expected=[r['lidar_header_stamp'] for r in b]
    for event in ('OOSM_ROLLBACK','NDT_CORRECTION_APPLIED','OOSM_REPLAY_COMPLETE','CORRECTED_PUBLISH'):
        selected=[r for r in eb if r['event_type']==event]
        key='last_ndt_observation_time' if event=='CORRECTED_PUBLISH' else 'ndt_measurement_stamp'
        if [r[key] for r in selected]!=expected:raise ValueError('LINEAGE_OBSERVATION_ASSOCIATION:'+event)
    for r in eb:
        if not all(np.isfinite(float(r[k])) for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w','v_x','v_y','v_z')):
            raise ValueError('NONFINITE_STATE')
        if abs(np.linalg.norm([float(r['q_'+k]) for k in ('x','y','z','w')])-1)>1e-5:
            raise ValueError('INVALID_STATE_QUATERNION')
    for left,right in zip(a,b):
        if not np.isfinite(float(right['ndt_fitness'])):raise ValueError('NONFINITE_NDT_FITNESS')
        for key in ('frame_index','cloud_seq','cloud_size_raw','ndt_has_converged','initial_guess_source','initial_guess_reason'):
            if left[key]!=right[key]:raise ValueError('SCAN_VISUAL_CONTRACT:'+key)
        for prefix in ('initial_guess_','raw_ndt_','final_used_'):
            value=[right[prefix+k] for k in ('tx','ty','tz','qx','qy','qz','qw')]
            values=np.array(value,dtype=float)
            if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:])-1)>1e-5:
                raise ValueError('NONFINITE_OR_INVALID_NDT_POSE')
    fa,fb=[rows(p/'oosm_fixed_lag.csv') for p in (baseline,run)]
    if len(fb)!=792 or any(r['status'] not in ('QUEUED','APPLIED') or not schedule_valid(r) for r in fb):
        raise ValueError('FIXED_LAG_FAILURE')
    for status in ('QUEUED','APPLIED'):
        left,right=[[r for r in table if r['status']==status] for table in (fa,fb)]
        paired(left,right,'ndt_stamp',396)
        for x,y in zip(left,right):
            keys=('observation_id','deadline_stamp')+(('release_state_stamp','replay_imu_count','correction_applied') if status=='APPLIED' else ())
            if any(x[k]!=y[k] for k in keys):raise ValueError('FIXED_RELEASE_NODE_CHANGED')
    oosm=rows(run/'oosm.csv')
    previous_oosm=rows(baseline/'oosm.csv')
    paired(previous_oosm,oosm,'ndt_stamp',396)
    if len(oosm)!=396 or any(r['oosm_result']!='APPLIED' for r in oosm):raise ValueError('OOSM_INCOMPLETE')
    for x,y in zip(previous_oosm,oosm):
        if any(x[k]!=y[k] for k in ('state_now_stamp','rollback_stamp','replay_imu_count')):
            raise ValueError('OOSM_TIME_NODE_CHANGED')
    deskew=rows(run/'deskew.csv')
    previous_deskew=rows(baseline/'deskew.csv')
    for key in ('scan_start_stamp','scan_end_stamp','reference_stamp'):
        paired(previous_deskew,deskew,key,396)
    for x,y in zip(previous_deskew,deskew):
        if any(x[k]!=y[k] for k in ('scan_index','point_count_in','point_count_out')):
            raise ValueError('DESKEW_INPUT_COUNT_OR_ORDER')
    if len(deskew)!=396 or any(r['status']!='PUBLISHED' or r['current_scan_ndt_leakage']!='0' or r['post_scan_end_imu_used']!='0' for r in deskew):
        raise ValueError('DESKEW_CONTRACT')
    params=yaml.safe_load((run/'rosparams.yaml').read_text())
    parameter_check(yaml.safe_load((baseline/'rosparams.yaml').read_text()),params)
    receipt=json.loads((run/'execution.json').read_text());old=json.loads((baseline/'execution.json').read_text())
    if not receipt['completed'] or receipt['binaries']!=old['binaries'] or any(receipt[k]!=old[k] for k in ('input_sha256','visual_sha256')):
        raise ValueError('RUN_RECEIPT_CONTRACT')
    guard=json.loads((run/'parameter_parity.json').read_text())
    if guard['result']!='PARAMETER_PARITY_PASS' or not guard['checked_before_playback']:raise ValueError('PREPLAYBACK_GUARD')
    count=sum(r['initial_guess_source']=='p4_visual_translation_imu_rotation' for r in b)
    if count!=169:raise ValueError('VISUAL_APPLICATION_COUNT')
    result=dict(EXPERIMENT_CONTRACT='EXPERIMENT_CONTRACT_PASS',GT_LOADED=False,
        NDT=396,NDT_converged=396,OOSM_APPLIED=396,EKF_Corrected=396,EKF_HighRate=8004,visual_applied=count,
        nonfinite=0,late=0,actual_beta=params['ndt_observation']['velocity_blend'],
        note='Post-intervention poses/source hashes may differ; fixed raw inputs and sensor-time scheduling must not')
    output.mkdir(parents=True,exist_ok=False);(output/'completeness.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for key in ('repo','run','output'):p.add_argument('--'+key,type=Path,required=True)
    audit(**vars(p.parse_args()))
