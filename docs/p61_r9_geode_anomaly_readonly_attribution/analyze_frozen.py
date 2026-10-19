#!/usr/bin/python3
"""Bounded attribution metrics from existing poses and geometry, no optimization."""
import csv,json,math,bisect
from decimal import Decimal
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from asset_gate import HERE,REPO,R8,RUN,MAP,map_xyz,sha
from freeze_analysis_inputs import SELECTED,PREDECESSORS,PHASES
from analysis_math import transform,inverse,increment,residual,pose_difference


def read(path):
    with Path(path).open() as f:return list(csv.DictReader(f))


def write(name,rows):
    with (HERE/name).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)


def native(ns):return float(Decimal(ns)/Decimal(10**9))


def from_lineage(row):return transform([float(row[k]) for k in ('p_x','p_y','p_z','q_x','q_y','q_z','q_w')])


def main():
    pin=json.loads((HERE/'analysis_inputs_frozen.json').read_text());assert pin['selected_indices']==SELECTED
    assert json.loads((HERE/'asset_gate_before.json').read_text())['status']=='PASS'
    # Require identical input bytes, not just previously successful gate status.
    for a in pin['input_assets']:assert sha(a['path'])==a['sha256'],a['path']
    keep=set(SELECTED+PREDECESSORS)
    frames={int(x['scan_index']):x for x in read(R8/'frame_results.csv') if int(x['scan_index']) in keep}
    scans={int(x['scan_index']):x for x in read(R8/'scan_manifest.csv') if int(x['scan_index']) in keep}
    end_to_index={int(x['end_ns']):i for i,x in scans.items()}
    geo={end_to_index[int(x['scan_ns'])]:x for x in read(R8/'geometry.csv') if int(x['scan_ns']) in end_to_index}
    desks={int(x['scan_index'])+2985:x for x in read(R8/'deskew.csv') if int(x['scan_index'])+2985 in keep}
    point_time=json.loads((HERE/'point_time_records.json').read_text())
    imu=read(RUN/'input/imu.csv');imu_ns=[int(x['stamp_ns']) for x in imu]
    live=read(R8/'live_trajectories.csv')
    predictions={int(x['stamp_ns']):x for x in live if x['stream']=='PREDICTION' and int(x['stamp_ns']) in end_to_index}
    target_native={native(int(scans[i]['end_ns'])):i for i in keep}
    events={}
    for x in read(RUN/'formal/ekf_lineage.csv'):
        t=float(x['ndt_measurement_stamp'])
        if t in target_native:
            k=(target_native[t],x['event_type']);assert k not in events
            events[k]=x
    init=yaml.safe_load((RUN/'input/initialization.yaml').read_text())
    ext=init['deskew'];TIL=np.eye(4);TIL[:3,:3]=np.array(ext['T_imu_lidar_rotation_row_major']).reshape(3,3);TIL[:3,3]=ext['T_imu_lidar_translation_m']
    xyz,_=map_xyz(MAP);tree=cKDTree(xyz);bbox=(xyz.min(0),xyz.max(0))
    poses={i:{key:transform(json.loads(row[name])) for key,name in [('initial','initial_pose'),('raw','raw_ndt_pose'),('used','used_ndt_pose'),('fused','ekf_final_pose')]} for i,row in frames.items()}
    anomaly=[];motions=[];density=[];audits=[]
    for i in SELECTED:
        row=frames[i];s=scans[i];g=geo[i];d=desks[i];end=int(s['end_ns']);P=poses[i];old=poses[i-1]
        stamp=native(end);rb=events[(i,'OOSM_ROLLBACK')];ap=events[(i,'NDT_CORRECTION_APPLIED')];rp=events[(i,'OOSM_REPLAY_COMPLETE')]
        tags='|'.join(name for name,(lo,hi) in PHASES.items() if lo<=i<=hi)
        a=dict(scan_index=i,phases=tags,header_ns=s['stamp_ns'],end_ns=end,source_hash=row['source_hash'],source_points=row['source_points'],
            source='R8 actual recorded geometry.csv and pose CSV; not reconstructed',fitness=g['fitness'],iterations=g['iterations'],
            iteration_limit_reached=int(int(g['iterations'])>=40),ndt_converged=g['ndt_converged'],
            translation_limited=row['translation_limited'],rotation_limited=row['rotation_limited'])
        for name in ('initial','raw','used'):
            for metric in ('nn_median_m','nn_p90_m','overlap_0p5m','overlap_2m','overlap_8m'):a[name+'_'+metric]=g[name+'_'+metric]
            a[name+'_pose_xyz_quat_xyzw']=json.dumps(json.loads(row[{'initial':'initial_pose','raw':'raw_ndt_pose','used':'used_ndt_pose'}[name]]))
        a['previous_used_pose_xyz_quat_xyzw']=frames[i-1]['used_ndt_pose'];a['fused_pose_xyz_quat_xyzw']=row['ekf_final_pose']
        for label,A,B in [('raw_vs_initial',P['initial'],P['raw']),('raw_vs_previous_used',old['used'],P['raw']),
                          ('used_vs_previous_used',old['used'],P['used']),('used_vs_raw',P['raw'],P['used'])]:
            a[label+'_translation_m'],a[label+'_rotation_deg']=pose_difference(A,B)
        a['raw_minus_initial_overlap_0p5m']=float(g['raw_overlap_0p5m'])-float(g['initial_overlap_0p5m'])
        a['raw_minus_used_overlap_0p5m']=float(g['raw_overlap_0p5m'])-float(g['used_overlap_0p5m'])
        # Check limiter's algebraic effect against recorded poses, without
        # constructing any replacement Used pose or counterfactual trajectory.
        raw_step=a['raw_vs_previous_used_translation_m'];used_step=a['used_vs_previous_used_translation_m']
        a['translation_step_rule_error_m']=abs(used_step-min(raw_step,.5))
        v=P['raw'][:3,3]-old['used'][:3,3];w=P['used'][:3,3]-old['used'][:3,3]
        a['translation_step_collinearity_cross_m2']=float(np.linalg.norm(np.cross(v,w)))
        raw_rot=a['raw_vs_previous_used_rotation_deg'];used_rot=a['used_vs_previous_used_rotation_deg']
        a['rotation_step_rule_error_deg']=abs(used_rot-min(raw_rot,5.))
        rv=Rotation.from_matrix(old['used'][:3,:3].T@P['raw'][:3,:3]).as_rotvec()
        uv=Rotation.from_matrix(old['used'][:3,:3].T@P['used'][:3,:3]).as_rotvec()
        a['rotation_step_axis_cross_rad2']=float(np.linalg.norm(np.cross(rv,uv)))
        a['geometry_target_pointset']='R8 actual two-pass voxelized Target318377; frozen geometry.csv'
        anomaly.append(a)
        dp,dr,E,log=residual(old['initial'],P['initial'],old['raw'],P['raw'])
        motion=dict(scan_index=i,previous_scan_index=i-1,previous_end_ns=scans[i-1]['end_ns'],end_ns=end,
                    fused_previous_actual_stamp_ns=frames[i-1]['corrected_publish_ns'],fused_actual_stamp_ns=row['corrected_publish_ns'],
                    coordinate_chart='Delta=Tprev^-1*Tcur; E=DeltaPrediction^-1*DeltaRaw; log=[rho_m,phi_rad]',
                    data_source='existing R8 poses only')
        for name,T in [('prediction',dp),('raw',dr),('used',increment(old['used'],P['used'])),('fused',increment(old['fused'],P['fused']))]:
            motion[name+'_increment_translation_m'],motion[name+'_increment_rotation_deg']=pose_difference(np.eye(4),T)
        motion.update({k:float(v) for k,v in zip(('residual_rho_x_m','residual_rho_y_m','residual_rho_z_m','residual_phi_x_rad','residual_phi_y_rad','residual_phi_z_rad'),log)})
        motion.update(residual_rho_norm_m=float(np.linalg.norm(log[:3])),residual_phi_deg=float(np.rad2deg(np.linalg.norm(log[3:]))),
            residual_transform_translation_norm_m=float(np.linalg.norm(E[:3,3])))
        motions.append(motion)
        for name in ('initial','raw','used'):
            p=P[name][:3,3];indices2=tree.query_ball_point(p,2.);indices5=tree.query_ball_point(p,5.)
            q=xyz[indices5];span=np.ptp(q,axis=0) if len(q) else ['NA_NO_POINTS']*3
            outside=np.maximum(np.maximum(bbox[0]-p,p-bbox[1]),0)
            density.append(dict(scan_index=i,end_ns=end,pose_kind=name,frame='camera_init',center_xyz_m=json.dumps(p.tolist()),
                map_points_within_2m=len(indices2),map_points_within_5m=len(indices5),center_nn_m=float(tree.query(p)[0]),
                span_5m_x_m=span[0],span_5m_y_m=span[1],span_5m_z_m=span[2],center_distance_outside_global_aabb_m=float(np.linalg.norm(outside)),
                nn_median_m=g[name+'_nn_median_m'],nn_p90_m=g[name+'_nn_p90_m'],overlap_0p5m=g[name+'_overlap_0p5m'],
                overlap_2m=g[name+'_overlap_2m'],overlap_8m=g[name+'_overlap_8m'],
                density_pointset='R5 frozen PCD321053 original map points; not rebuilt R8 Target',
                correspondence_pointset='R8 recorded filtered Source to two-pass voxelized Target318377',
                source='new fixed-radius frozen-map counts; original R8 source-to-map NN metrics',
                limitation='center density is not full-FOV coverage, uniqueness, or true-pose validation'))
        raw=point_time['frames'][str(i)];assert raw['header_ns']==int(s['stamp_ns']) and raw['point_time_last_ns']==end
        assert raw['frame_id']==s['frame_id']=='livox_frame'
        corr_pose=P['fused'];rp_lidar=from_lineage(rp)@TIL
        mi=P['used']@inverse(TIL);roundtrip=mi@TIL
        rt_m,rt_deg=pose_difference(P['used'],roundtrip)
        rb_m,rb_deg=pose_difference(P['initial'],from_lineage(rb)@TIL)
        applied_m,applied_deg=pose_difference(mi,from_lineage(ap))
        pub_m,pub_deg=pose_difference(corr_pose,rp_lidar)
        head_index=bisect.bisect_right(imu_ns,end)-1;head=imu[head_index];head_ns=imu_ns[head_index]
        assert head['frame_id']=='imu_link'
        pred=predictions[end];predpose=transform([float(pred[k]) for k in ('px','py','pz','qx','qy','qz','qw')])
        match_prediction=pose_difference(predpose,P['initial'])
        audit=dict(scan_index=i,header_ns=raw['header_ns'],offset_min_ns=raw['offset_min_ns'],offset_max_ns=raw['offset_max_ns'],
            earliest_point_ns=raw['point_time_first_ns'],scan_end_ns=end,point_count=raw['point_count'],input_frame=raw['frame_id'],
            deskew_start_sec=d['scan_start_stamp'],deskew_end_sec=d['scan_end_stamp'],deskew_reference_sec=d['reference_stamp'],
            ndt_measurement_ns=row['ndt_measurement_ns'],oosm_correction_sec=ap['state_stamp'],rollback_state_sec=rb['state_stamp'],
            replay_end_sec=rp['state_stamp'],watermark_ns=row['current_watermark_ns'],max_imu_source_stamp=d['max_imu_source_stamp'],
            last_original_imu_at_or_before_end_ns=head_ns,imu_frame_id=head['frame_id'],raw_oosm_rollback_sec=row['rollback_sec'],
            post_scan_end_imu_used=d['post_scan_end_imu_used'],replay_imu_count=row['replay_imu_count'],oosm_status=row['oosm_status'],
            next_prediction_ns=predictions[int(scans[i+1]['end_ns'])]['stamp_ns'] if i+1 in SELECTED else 'NA_BOUNDARY_SCOPE',
            native_representation_error_ns=str(Decimal.from_float(stamp)*10**9-Decimal(end)),
            input_point_time_min_sec=d['point_time_min_sec'],input_point_time_max_sec=d['point_time_max_sec'],
            csv_point_time_convention=d['point_time_convention'],label_audit='INPUT_OFFSETS_MISLABELED_AS_OUTPUT_REFERENCE',
            roundtrip_translation_m=rt_m,roundtrip_rotation_deg=rt_deg,
            roundtrip_matrix_frobenius_error=float(np.linalg.norm(P['used']-roundtrip)),
            initial_vs_rollback_lidar_m=rb_m,initial_vs_rollback_lidar_deg=rb_deg,
            used_measurement_vs_applied_imu_m=applied_m,used_measurement_vs_applied_imu_deg=applied_deg,
            fused_vs_replay_lidar_m=pub_m,fused_vs_replay_lidar_deg=pub_deg,
            recorded_prediction_vs_ndt_guess_m=match_prediction[0],recorded_prediction_vs_ndt_guess_deg=match_prediction[1],
            rotation_determinant=float(np.linalg.det(P['used'][:3,:3])),rotation_orthogonality_error=float(np.linalg.norm(P['used'][:3,:3].T@P['used'][:3,:3]-np.eye(3))),
            frames_contract='map=camera_init; source/pose child=livox_frame; external Xsens=imu_link',
            output_frame_id_evidence='source assignments + frozen config; actual output header IDs not saved in R8 CSV')
        checks=[float(d['scan_start_stamp'])==native(int(s['stamp_ns'])),float(d['scan_end_stamp'])==float(d['reference_stamp'])==stamp,
                int(row['ndt_measurement_ns'])==end,float(ap['state_stamp'])==float(ap['ndt_measurement_stamp'])==stamp,
                float(rb['state_stamp'])==stamp,float(d['max_imu_source_stamp'])<=stamp,d['post_scan_end_imu_used']=='0',
                float(rp['state_stamp'])==native(int(row['current_watermark_ns'])),row['oosm_status']=='APPLIED',
                float(row['rollback_sec'])==native(head_ns),float(d['max_imu_source_stamp'])<=native(head_ns),
                int(raw['offset_max_ns'])==int(round(float(d['point_time_max_sec'])*1e9)),
                int(raw['offset_min_ns'])==int(round(float(d['point_time_min_sec'])*1e9)),
                native(int(scans[i+1]['end_ns']))>float(rp['state_stamp']) if i+1 in SELECTED else True]
        audit['time_record_contract']='PASS' if all(checks) else 'FAIL'
        audits.append(audit)
    write('anomaly_frame_table.csv',anomaly);write('candidate_increment_consistency.csv',motions)
    write('map_support_summary.csv',density);write('time_coordinate_audit.csv',audits)
    summary=dict(selected_count=len(SELECTED),predecessors=PREDECESSORS,phases={},
        label_problem_count=len(audits),time_record_failures=[x['scan_index'] for x in audits if x['time_record_contract']!='PASS'],
        max_roundtrip_m=max(x['roundtrip_translation_m'] for x in audits),max_roundtrip_deg=max(x['roundtrip_rotation_deg'] for x in audits),
        max_initial_vs_rollback_m=max(x['initial_vs_rollback_lidar_m'] for x in audits),max_initial_vs_rollback_deg=max(x['initial_vs_rollback_lidar_deg'] for x in audits),
        max_used_vs_applied_imu_m=max(x['used_measurement_vs_applied_imu_m'] for x in audits),max_used_vs_applied_imu_deg=max(x['used_measurement_vs_applied_imu_deg'] for x in audits),
        max_fused_vs_replay_lidar_m=max(x['fused_vs_replay_lidar_m'] for x in audits),max_fused_vs_replay_lidar_deg=max(x['fused_vs_replay_lidar_deg'] for x in audits),
        max_roundtrip_matrix_frobenius_error=max(x['roundtrip_matrix_frobenius_error'] for x in audits),
        extrinsic_det=float(np.linalg.det(TIL[:3,:3])),extrinsic_orth_error=float(np.linalg.norm(TIL[:3,:3].T@TIL[:3,:3]-np.eye(3))),
        source_cloud_status='NO_SAVED_DESKEW_SOURCE; recorded R8 geometry used; no Source reconstruction',
        gt_reads=0,new_ndt_aligns=0,new_ekf_runs=0,localization_replays=0)
    for name,(lo,hi) in PHASES.items():
        chosen=[x for x in anomaly if lo<=x['scan_index']<=hi];mo=[x for x in motions if lo<=x['scan_index']<=hi]
        def distribution(values):
            v=np.array(values,dtype=float);return dict(min=float(v.min()),median=float(np.median(v)),max=float(v.max()))
        summary['phases'][name]=dict(count=len(chosen),header_first_ns=chosen[0]['header_ns'],end_last_ns=chosen[-1]['end_ns'],
            overlap={kind:distribution([float(x[kind+'_overlap_0p5m']) for x in chosen]) for kind in ('initial','raw','used')},
            rho_m=distribution([x['residual_rho_norm_m'] for x in mo]),phi_deg=distribution([x['residual_phi_deg'] for x in mo]),
            density_5m={kind:distribution([x['map_points_within_5m'] for x in density if x['pose_kind']==kind and lo<=x['scan_index']<=hi]) for kind in ('initial','raw','used')},
            limiter_translation=sum(x['translation_limited']=='1' for x in chosen),limiter_rotation=sum(x['rotation_limited']=='1' for x in chosen))
    (HERE/'metric_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
