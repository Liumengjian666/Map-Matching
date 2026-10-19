#!/usr/bin/python3
"""Compare existing Raw rotations with interval-only gyro increments, no trajectory."""
import bisect,csv,json,subprocess,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from check_assets import HERE,REPO,sha
from gyro_math import angle_deg,independent_angle_deg,check_rotation,imu_relative_rotation,integrate_zoh,classify,samples_from_records


def write_csv(name,rows):
    with (HERE/name).open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)


def main():
    started=time.monotonic();pin=json.loads((HERE/'inputs_frozen.json').read_text())
    assert sha(HERE/'PRE_REGISTRATION.md')==pin['pre_registration_sha256']
    for a in pin['source_assets']:assert sha(a['path'])==a['sha256'],a['path']
    raw=json.loads((HERE/'gyro_samples.json').read_text());assert raw['inputs_frozen_sha256']==sha(HERE/'inputs_frozen.json')
    assert raw['topic']=='/imu/data' and raw['frame_id']=='imu_link' and raw['after_window_sample_decoded'] is False
    sample_error=None
    try:samples=samples_from_records(raw['samples'])
    except ValueError as exc:samples=[];sample_error=str(exc)
    stamps=[s[0] for s in samples];monotonic=all(a<b for a,b in zip(stamps,stamps[1:]))
    start=pin['intervals'][0]['t_start_ns'];end=pin['intervals'][-1]['t_end_ns']
    in_window=[t for t in stamps if start<=t<=end]
    positive=[b-a for a,b in zip(in_window,in_window[1:]) if b>a]
    median=float(np.median(positive)) if positive else float('nan')
    TIL=np.array(pin['T_I_L']).reshape(4,4);RIL=check_rotation(TIL[:3,:3])
    bias=pin['gyro_bias_I_radps'];poses={}
    for key,value in pin['raw_pose_strings'].items():
        pose=np.array(json.loads(value),dtype=float)
        assert pose.shape==(7,) and np.isfinite(pose).all() and abs(np.linalg.norm(pose[3:])-1)<1e-8
        poses[int(key)]=check_rotation(Rotation.from_quat(pose[3:]).as_matrix())
    assert sorted(poses)==list(range(5154,5166))
    assert [i['scan_curr'] for i in pin['intervals']]==list(range(5155,5166))
    rows=[];coverage=[];segments=[];residuals=[];valid=[]
    for interval in pin['intervals']:
        a=interval['t_start_ns'];b=interval['t_end_ns'];prev=interval['scan_prev'];curr=interval['scan_curr']
        # Take only nearest head plus samples <=b. Later samples in the total
        # window cannot enter this interval's integral or its gap statistics.
        lo=bisect.bisect_right(stamps,a)-1 if monotonic else -1
        hi=bisect.bisect_right(stamps,b) if monotonic else 0
        causal=samples[lo:hi] if lo>=0 else []
        row=dict(scan_prev=prev,scan_curr=curr,t_start_ns=a,t_end_ns=b,dt_s=(b-a)*1e-9,
            imu_sample_count=len(causal),median_imu_dt_ms=median*1e-6,max_imu_gap_ms='NA',
            coverage_status='INVALID',raw_rotation_angle_deg='NA',gyro_rotation_angle_deg='NA',angular_residual_deg='NA',
            independent_logm_residual_deg='NA',independent_agreement_deg='NA',
            raw_pose_rotation_correction_deg=interval['raw_pose_rotation_correction_deg'],
            bias_source='R6 initial_state.gyro_bias_I fullprecision fixed; rad/s in imu_link',
            extrinsic_source='R6 T_I_L + identical R8 fullprecision YAML; L->I',validity_reason='NOT_COMPUTED')
        cov=dict(scan_prev=prev,scan_curr=curr,t_start_ns=a,t_end_ns=b,
            first_sample_ns=causal[0][0] if causal else 'NA',last_sample_ns=causal[-1][0] if causal else 'NA',
            imu_sample_count=len(causal),median_dt_ns=median,gap_limit_ns=3*median,max_gap_ns='NA',
            coverage_start_ns='NA',coverage_end_ns='NA',head_age_ns='NA',tail_age_ns='NA',
            after_end_samples_used=False,coverage_status='INVALID',validity_reason='NOT_COMPUTED')
        if causal:
            gaps=[v[0]-u[0] for u,v in zip(causal,causal[1:])]+[a-causal[0][0],b-causal[-1][0]]
            row['max_imu_gap_ms']=max(gaps)*1e-6;cov['max_gap_ns']=max(gaps)
            cov['head_age_ns']=a-causal[0][0];cov['tail_age_ns']=b-causal[-1][0]
        try:
            if sample_error:raise ValueError(sample_error)
            if not monotonic:raise ValueError('NONMONOTONIC_OR_DUPLICATE')
            G,info=integrate_zoh(causal,a,b,bias,median)
            D=imu_relative_rotation(poses[prev],poses[curr],RIL)
            E=check_rotation(G.T@D);error=angle_deg(E)
            independent=independent_angle_deg(E)
            if abs(error-independent)>1e-6:raise ValueError('INDEPENDENT_RESIDUAL_MISMATCH')
            row.update(coverage_status='VALID',max_imu_gap_ms=info['max_gap_ns']*1e-6,
                raw_rotation_angle_deg=angle_deg(D),gyro_rotation_angle_deg=angle_deg(G),angular_residual_deg=error,
                independent_logm_residual_deg=independent,independent_agreement_deg=abs(error-independent),validity_reason='PASS')
            cov.update(max_gap_ns=info['max_gap_ns'],coverage_start_ns=a,coverage_end_ns=b,
                head_age_ns=info['start_head_age_ns'],tail_age_ns=info['end_hold_age_ns'],coverage_status='VALID',validity_reason='PASS')
            segments.append(dict(scan_prev=prev,scan_curr=curr,Delta_R_gyro=G.tolist(),Delta_R_raw=D.tolist(),
                E_R=E.tolist(),integration=info))
            residuals.append(error);valid.append(True)
        except ValueError as exc:
            row['validity_reason']=cov['validity_reason']=str(exc);residuals.append(float('nan'));valid.append(False)
        rows.append(row);coverage.append(cov)
    result=classify(residuals,valid);v=np.array([x for x,ok in zip(residuals,valid) if ok])
    summary=dict(result=result,valid_intervals=sum(valid),target_intervals=11,median_imu_dt_ns=median,
        median_residual_deg=float(np.median(v)) if len(v) else None,max_residual_deg=float(np.max(v)) if len(v) else None,
        descriptive_p95_linear_deg=float(np.percentile(v,95)) if len(v) else None,p95_is_decision_gate=False,
        consecutive_exceedance_pairs=[[[5154+j,5155+j],[5155+j,5156+j]] for j in range(10) if valid[j] and valid[j+1] and residuals[j]>5 and residuals[j+1]>5],
        max_independent_agreement_deg=max((float(x['independent_agreement_deg']) for x in rows if x['coverage_status']=='VALID'),default=None),
        bias_fixed_radps=bias,T_I_L=pin['T_I_L'],extrinsic_det=float(np.linalg.det(RIL)),
        extrinsic_orthogonality_error=float(np.linalg.norm(RIL.T@RIL-np.eye(3))),
        code_sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
        script_sha256=sha(Path(__file__)),elapsed_sec=time.monotonic()-started,
        raw_pose_strings_sha256=sha(HERE/'inputs_frozen.json'),gt_reads=0,ros_localization_nodes=0,rosbag_play=0,
        new_ndt_align=0,ekf_runs=0,new_localization_trajectory_estimates=0)
    write_csv('interval_rotation_audit.csv',rows);write_csv('imu_coverage_audit.csv',coverage)
    with (HERE/'integration_details.json').open('x') as f:json.dump(segments,f,indent=2);f.write('\n')
    with (HERE/'numerical_results.json').open('x') as f:json.dump(summary,f,indent=2);f.write('\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
