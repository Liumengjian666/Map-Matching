"""P21 prefix parity and sensor-time provenance checks; never opens GT."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
import yaml
from run_guarded import check_parameters,SCRIPTS
sys.path.insert(0,str(SCRIPTS/'p19_visual_tail'))
from compare import Checks,rows,pose_delta,schedule_valid,write_csv


def basic(run,phase):
    diag=rows(run/'ndt_diagnostics.csv');events=rows(run/'ekf_lineage.csv')
    counts=dict(NDT=len(diag),OOSM=sum(r['oosm_result']=='APPLIED' for r in rows(run/'oosm.csv')),
        HighRate=sum(r['event_type']=='IMU_HIGH_RATE_PUBLISH' for r in events),
        Corrected=sum(r['event_type']=='CORRECTED_PUBLISH' for r in events),
        Visual=sum(r['initial_guess_source']=='p4_visual_translation_imu_rotation' for r in diag),
        EKF=sum(r['ekf_translation_applied']=='1' for r in diag))
    expected=dict(NDT=396,OOSM=396,HighRate=8004,Corrected=396,Visual=169) if phase=='full' else dict(
        NDT=183,OOSM=182,HighRate=3698,Corrected=182,Visual=106)
    if any(counts[k]!=v for k,v in expected.items()):raise ValueError('P21_FRAME_CONTRACT:'+str(counts))
    for prefix in ('initial_guess_','raw_ndt_','final_used_'):
        if not all(np.isfinite([float(r[prefix+k]) for k in ('tx','ty','tz','qx','qy','qz','qw')]).all() for r in diag):
            raise ValueError('NONFINITE_NDT')
    if any(r['ndt_has_converged']!='1' or r['pending_overflow_count']!='0' or not np.isfinite(float(r['ndt_fitness'])) for r in diag):
        raise ValueError('NDT_OR_PENDING_CONTRACT')
    if any(not np.isfinite([float(r[k]) for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w','v_x','v_y','v_z')]).all() for r in events):
        raise ValueError('NONFINITE_STATE')
    for r in rows(run/'oosm_fixed_lag.csv'):
        if r['status'] not in ('QUEUED','APPLIED') or not schedule_valid(r):raise ValueError('FIXED_LAG_FAILURE')
    receipt=json.loads((run/'execution.json').read_text())
    if not receipt['completed']:raise ValueError('RUN_INCOMPLETE')
    return diag,events,counts


def check(repo,run,phase,output):
    baseline=repo/'docs/p19_r1_visual_replication_tail_risk/runs/repeat_visual'
    diag,events,counts=basic(run,phase)
    check_parameters(yaml.safe_load((baseline/'rosparams.yaml').read_text()),yaml.safe_load((run/'rosparams.yaml').read_text()),phase!='off')
    output.mkdir(parents=True,exist_ok=False)
    if phase=='off':
        c=Checks();old=rows(baseline/'ndt_diagnostics.csv')[:180]
        prefixes=('baseline_guess_','imu_rotation_guess_','initial_guess_','previous_pose_','delta_pose_',
                  'raw_ndt_','used_before_step_limit_','final_used_')
        poses={p+k for p in prefixes for k in ('tx','ty','tz','qx','qy','qz','qw')}
        timing={'ros_now','wall_time','ndt_align_complete_wall_sec','ndt_odom_publish_wall_sec',
                'local_imu_history_first_stamp','local_imu_history_last_stamp','pending_lidar_count'}
        c.table('NDT',old,[{k:r[k] for k in old[0]} for r in diag[:180]],180,timing|poses)
        for i,(a,b) in enumerate(zip(old,diag),1):
            for p in prefixes:
                aa,bb=[[float(r[p+k]) for k in ('tx','ty','tz','qx','qy','qz','qw')] for r in (a,b)]
                if np.isnan(aa).all() and np.isnan(bb).all():continue
                dt,dr=pose_delta(aa,bb)
                c.check('NDT',i,a['lidar_header_stamp'],p+'position_m',0,dt,1e-9)
                c.check('NDT',i,a['lidar_header_stamp'],p+'rotation_deg',0,dr,1e-8)
        last=float(rows(baseline/'oosm.csv')[179]['state_now_stamp'])
        old_events=rows(baseline/'ekf_lineage.csv')
        pose_keys={'p_x','p_y','p_z','q_x','q_y','q_z','q_w'}
        for event in ('IMU_HIGH_RATE_PUBLISH','CORRECTED_PUBLISH'):
            a,b=[[r for r in table if r['event_type']==event and float(r['state_stamp'])<=last] for table in (old_events,events)]
            c.table(event,a,b,len(a),{'event_index','event_ros_stamp'}|pose_keys)
            for i,(x,y) in enumerate(zip(a,b),1):
                dt,dr=pose_delta([x[k] for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w')],
                    [y[k] for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w')])
                c.check(event,i,x['state_stamp'],'position_m',0,dt,1e-9)
                c.check(event,i,x['state_stamp'],'rotation_deg',0,dr,1e-8)
        for file,stage,exclude in [('deskew.csv','Deskew',{'deskew_processing_ms'}),('oosm.csv','OOSM',set())]:
            c.table(stage,rows(baseline/file)[:180],rows(run/file)[:180],180,exclude)
        a,b=[[r for r in rows(root/'oosm_fixed_lag.csv') if r['status']=='APPLIED'][:180] for root in (baseline,run)]
        c.table('FixedLag',a,b,180,{'arrival_state_stamp','arrival_wall_sec','processing_wall_sec'})
        write_csv(output/'default_off_parity.csv',list(c.summary.values()))
        result=dict(DEFAULT_OFF_PARITY='DEFAULT_OFF_PARITY_PASS' if not c.failures else 'DEFAULT_OFF_PARITY_FAILED',
            counts=counts,failures=c.failures[:20],failure_count=len(c.failures),GT_LOADED=False)
        (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
        if c.failures:raise ValueError('DEFAULT_OFF_PARITY_FAILED:'+str(c.failures[:1]))
    else:
        samples=rows(run/'ndt_diagnostics.csv.ekf_samples.csv')
        if any(r['status']!='ACCEPTED_CURRENT_EKF_SAMPLE' for r in samples):raise ValueError('CAUSAL_STREAM_REJECTED')
        stamps=[int(r['sample_ns']) for r in samples]
        if len(stamps)!=counts['HighRate'] or any(b<=a for a,b in zip(stamps,stamps[1:])):
            raise ValueError('CAUSAL_STREAM_INCOMPLETE_OR_AMBIGUOUS')
        sample_lookup={int(r['sample_ns']):r for r in samples}
        high=[r for r in events if r['event_type']=='IMU_HIGH_RATE_PUBLISH']
        if len(high)!=len(samples):raise ValueError('SOURCE_LINEAGE_COUNT')
        for sample,state in zip(samples,high):
            if abs(int(sample['sample_ns'])/1e9-float(state['state_stamp']))>3e-7:
                raise ValueError('SAMPLE_LINEAGE_TIME')
            last=float(state['last_ndt_observation_time'])
            if last>float(state['state_stamp']) or (last>0 and last+.15>float(state['state_stamp'])+3e-7):
                raise ValueError('FUTURE_NDT_CORRECTION_IN_SAMPLE')
        availability=[]
        for r in diag:
            s,t,w=[int(r[k]) for k in ('ekf_sample_ns','ekf_target_ns','ekf_watermark_ns')]
            if r['visual_applied']=='1' and r['ekf_translation_applied']=='1':raise ValueError('VISUAL_OVERRIDDEN')
            if r['ekf_translation_applied']=='1':
                if not s or s>t or t-s>20000000 or w<t:raise ValueError('CAUSAL_SAMPLE_TIME_CONTRACT')
                available=[x for x in stamps if x<=t and float(sample_lookup[x]['callback_wall_sec'])<=float(r['ekf_application_wall_sec'])]
                if not available or s!=max(available):raise ValueError('ARRIVAL_ORDER_SAMPLE_SELECTION')
                if abs((t-s)/1e6-float(r['ekf_prediction_age_ms']))>1e-9:raise ValueError('SAMPLE_AGE_LEDGER')
                if pose_delta([r['initial_guess_'+k] for k in ('tx','ty','tz','qx','qy','qz','qw')],
                    [r['imu_rotation_guess_'+k] for k in ('tx','ty','tz','qx','qy','qz','qw')])[1]>1e-8:
                    raise ValueError('EKF_REPLACED_IMU_ROTATION')
            availability.append(dict(scan_ns=t,sample_ns=s,watermark_ns=w,age_ms=r['ekf_prediction_age_ms'],
                visual=r['visual_applied'],ekf_applied=r['ekf_translation_applied'],reason=r['ekf_translation_reason'],
                translation_change_m=r['ekf_translation_change_m'],query_ms=r['ekf_prediction_query_ms']))
        if not counts['EKF']:raise ValueError('EKF_FALLBACK_NOT_AVAILABLE')
        write_csv(output/'ekf_prediction_availability.csv',availability)
        reasons={reason:sum(r['ekf_translation_reason']==reason for r in diag) for reason in {r['ekf_translation_reason'] for r in diag}}
        ages=[float(r['ekf_prediction_age_ms']) for r in diag if r['ekf_translation_applied']=='1']
        result=dict(CAUSAL_PREDICTION_CONTRACT='CAUSAL_PREDICTION_CONTRACT_PASS',
            SHORT_TEST='SHORT_EKF_FALLBACK_PASS' if phase=='short' else 'FULL_COMPLETENESS_PASS',
            counts=counts,reasons=reasons,age_ms=dict(mean=float(np.mean(ages)),p95=float(np.percentile(ages,95)),max=max(ages)),
            future_used=0,unverified_provenance=0,nonfinite=0,late=0,GT_LOADED=False)
        (output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('repo','run','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--phase',choices=('off','short','full'),required=True)
    check(**vars(p.parse_args()))
