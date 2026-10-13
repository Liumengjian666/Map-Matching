#!/usr/bin/env python3
"""Finite P29 posthoc analysis. Never launches localization or changes a result."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from prepare import REQUESTS, rows, sha, write_csv
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'p10_corridor01_deterministic_oosm'))
import evaluate_frozen_v2 as frozen
G=frozen.geometry


def matrix(row,prefix):
    return np.array([float(row[prefix+str(i)+str(j)]) for i in range(4) for j in range(4)]).reshape(4,4)


def pose_difference(a,b):
    return float(np.linalg.norm(a[:3,3]-b[:3,3])),float(np.degrees(
        (Rotation.from_matrix(a[:3,:3]).inv()*Rotation.from_matrix(b[:3,:3])).magnitude()))


def imu_matrix(T,extrinsic):
    p,R=G.lidar_pose_to_imu(dict(position=T[:3,3],rotation=Rotation.from_matrix(T[:3,:3]).as_matrix()),extrinsic)
    result=np.eye(4);result[:3,3]=p;result[:3,:3]=R
    return result


def summary(values):
    values=np.asarray(values)
    return dict(rmse=float(np.sqrt(np.mean(values**2))),p95=float(np.percentile(values,95)),maximum=float(values.max()))


def normalized_params(params,run_root):
    p=dict(params);p.pop('run_id',None);p.pop('roslaunch',None)
    def normalize(v):
        if isinstance(v,dict):return {k:normalize(x) for k,x in v.items()}
        if isinstance(v,list):return [normalize(x) for x in v]
        if isinstance(v,str):return v.replace(str(run_root),'<RUN>')
        return v
    p=normalize(p)
    for key in ('experimental_ndt_source_capture_enable','experimental_ndt_source_capture_directory',
                'experimental_ndt_source_capture_requests'):
        p['output'].pop(key,None)
    return p


def timeline(cache,origin,start_ns,extrinsic):
    runs={m:cache/'runs'/m for m in ('A','B')}
    updates=[r for r in rows(runs['B']/'msckf_diagnostics.csv') if r['status']=='APPLIED_REPROJECTION_SELECTED_GAIN']
    releases={str(int(r['observation_id']) & ~(1<<62)):r for r in rows(runs['B']/'oosm_fixed_lag.csv')
              if r['status']=='MSCKF_APPLIED_REPROJECTION_SELECTED_GAIN'}
    for update in updates:
        release=releases[update['image_ns']]
        update['release_state_stamp']=release['release_state_stamp']
        update['processing_wall_sec']=release['processing_wall_sec']
        if float(release['release_state_stamp'])<float(release['deadline_stamp']):
            raise ValueError('VISUAL_RELEASE_BEFORE_SENSOR_DEADLINE')
    event=dict(event='FIRST_MSCKF_MEASUREMENT_TIME_NOT_RELEASE',identity=updates[0]['image_ns'],
               sensor_stamp_s=int(updates[0]['image_ns'])/1e9,
               replay_relative_s=(int(updates[0]['image_ns'])-start_ns)/1e9,
               evaluation_relative_s=int(updates[0]['image_ns'])/1e9-origin,
               translation_delta_m=float(updates[0]['correction_m']),
               rotation_delta_deg=float(updates[0]['correction_rad'])*180/np.pi)
    release=float(updates[0]['release_state_stamp'])
    events=[event,dict(event='FIRST_MSCKF_RELEASE_STATE_TIME',identity=updates[0]['image_ns'],
        sensor_stamp_s=release,replay_relative_s=release-start_ns/1e9,evaluation_relative_s=release-origin,
        translation_delta_m=0.,rotation_delta_deg=event['rotation_delta_deg'])]
    high={m:[r for r in rows(root/'ekf_lineage.csv') if r['event_type']=='IMU_HIGH_RATE_PUBLISH'] for m,root in runs.items()}
    for x,y in zip(high['A'],high['B']):
        if x['state_stamp']!=y['state_stamp']:raise ValueError('AB_STATE_TIME_DIFFER')
        from scipy.spatial.transform import Rotation as R
        dp=np.linalg.norm([float(x[k])-float(y[k]) for k in ('p_x','p_y','p_z')])
        dq=float(np.degrees((R.from_quat([float(x[k]) for k in ('q_x','q_y','q_z','q_w')]).inv()*
                            R.from_quat([float(y[k]) for k in ('q_x','q_y','q_z','q_w')])).magnitude()))
        if dp>1e-9 or dq>1e-8:
            t=float(x['state_stamp'])
            events.append(dict(event='FIRST_HIGH_RATE_STATE_DIFFERENCE',identity=x['high_rate_publish_seq'],
                sensor_stamp_s=t,replay_relative_s=t-start_ns/1e9,evaluation_relative_s=t-origin,
                translation_delta_m=float(dp),rotation_delta_deg=dq));break
    da,db=[rows(root/'deskew.csv') for root in runs.values()]
    observed=('point_displacement_mean_m','point_displacement_median_m','point_displacement_p95_m','point_displacement_max_m')
    for x,y in zip(da,db):
        if any(abs(float(x[k])-float(y[k]))>1e-12 for k in observed):
            t=float(x['reference_stamp'])
            events.append(dict(event='FIRST_DESKEW_DIFFERENCE_SCAN_REFERENCE_NOT_EXECUTION',identity=x['scan_index'],sensor_stamp_s=t,
                replay_relative_s=t-start_ns/1e9,evaluation_relative_s=t-origin,
                translation_delta_m=abs(float(x['point_displacement_mean_m'])-float(y['point_displacement_mean_m'])),
                rotation_delta_deg=None));break
    diagnostics={m:rows(root/'ndt_diagnostics.csv') for m,root in runs.items()}
    source_diff=[i+1 for i,(a,b) in enumerate(zip(diagnostics['A'],diagnostics['B'])) if a['cloud_hash']!=b['cloud_hash']]
    for label,prefix,threshold in [('FIRST_INITIAL_DIFFERENCE','initial_guess_',1e-9),
                                   ('FIRST_RAW_DIFFERENCE','raw_ndt_',1e-9),
                                   ('FIRST_RAW_GE_0_1M','raw_ndt_',0.1)]:
        for x,y in zip(diagnostics['A'],diagnostics['B']):
            px,rx=frozen.pose_from_fields(x,prefix);py,ry=frozen.pose_from_fields(y,prefix)
            lidar_dt=float(np.linalg.norm(px-py))
            px,rx=G.lidar_pose_to_imu(dict(position=px,rotation=rx),extrinsic)
            py,ry=G.lidar_pose_to_imu(dict(position=py,rotation=ry),extrinsic)
            dp=float(np.linalg.norm(px-py));dr=float(np.degrees((Rotation.from_matrix(rx).inv()*Rotation.from_matrix(ry)).magnitude()))
            if dp>=threshold or (threshold==1e-9 and dr>1e-8):
                t=float(x['lidar_header_stamp'])
                events.append(dict(event=label,identity=x['frame_index'],sensor_stamp_s=t,replay_relative_s=t-start_ns/1e9,
                    evaluation_relative_s=t-origin,translation_delta_m=dp,rotation_delta_deg=dr,
                    reference_point='IMU_origin',lidar_translation_delta_m=lidar_dt));break
    write_csv(cache/'feedback_timeline.csv',events)
    return dict(events=events,source_different_frames=len(source_diff),first_source_different_frame=source_diff[0],
                visual_updates=len(updates)),diagnostics,updates


def cloud_array(msg):
    needed=('x','y','z','time');fields={f.name:f for f in msg.fields}
    if any(k not in fields or fields[k].datatype!=7 for k in needed) or msg.is_bigendian:
        raise ValueError('DESKEW_POINT_ENCODING_NOT_SUPPORTED')
    dtype=np.dtype(dict(names=needed,formats=['<f4']*4,
        offsets=[fields[k].offset for k in needed],itemsize=msg.point_step))
    return np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
        strides=(msg.row_step,msg.point_step)).reshape(-1)


def cloud_feedback(cache,repo,updates):
    import rosbag
    wanted={int(r.split(':')[1]):int(r.split(':')[0]) for r in REQUESTS}
    clouds={}
    for method in ('A','B'):
        with rosbag.Bag(str(cache/'runs'/method/'deskew_selected.bag')) as bag:
            clouds[method]={msg.header.stamp.to_nsec():msg for _,msg,_ in bag.read_messages()}
    raw={}
    input_bag=repo/'.p10_corridor01_deterministic_oosm_replay_r1/input_v2/corridor01_p10_causal_startup5_eval35_fixedlag_v2.bag'
    with rosbag.Bag(str(input_bag)) as bag:
        for _,msg,_ in bag.read_messages(topics=['/p10/corridor01/raw_timed_points']):
            if msg.header.stamp.to_nsec() in wanted:raw[msg.header.stamp.to_nsec()]=msg
    records=[];rigid=[]
    for ns,frame in wanted.items():
        a,b,r=[cloud_array(group[ns]) for group in (clouds['A'],clouds['B'],raw)]
        if a.shape!=b.shape or a.shape!=r.shape or not np.array_equal(a['time'],b['time']) or not np.array_equal(a['time'],r['time']):
            raise ValueError('RAW_DESKEW_POINT_CORRESPONDENCE_FAILED')
        xa=np.column_stack([a[k] for k in ('x','y','z')]).astype(float)
        xb=np.column_stack([b[k] for k in ('x','y','z')]).astype(float)
        xr=np.column_stack([r[k] for k in ('x','y','z')]).astype(float)
        differences=np.linalg.norm(xa-xb,axis=1);da=np.linalg.norm(xa-xr,axis=1);db=np.linalg.norm(xb-xr,axis=1)
        lo,hi=float(r['time'].min()),float(r['time'].max())
        active=[int(u['image_ns']) for u in updates if ns<=int(u['image_ns'])<=ns+round(hi*1e9)]
        released=[u['image_ns'] for u in updates if ns/1e9<=float(u['release_state_stamp'])<=ns/1e9+hi]
        deskew_row=rows(cache/'runs/B/deskew.csv')[frame-1]
        within_available=[u['image_ns'] for u in updates if int(u['image_ns']) in active
                          and float(u['release_state_stamp'])<=float(deskew_row['history_last_stamp'])]
        records.append(dict(frame_index=frame,scan_ns=str(ns),raw_point_count=len(r),
            correspondence='EXACT_ORDER_AND_POINT_TIMES',point_delta_rms_m=float(np.sqrt(np.mean(differences**2))),
            point_delta_p95_m=float(np.percentile(differences,95)),point_delta_max_m=float(differences.max()),
            changed_xyz_points=int(np.sum(np.any(xa!=xb,axis=1))),
            A_deskew_displacement_mean_m=float(da.mean()),B_deskew_displacement_mean_m=float(db.mean()),
            visual_image_times_within_scan='|'.join(map(str,active)),
            visual_updates_released_during_scan='|'.join(released),
            within_scan_images_released_by_deskew_horizon='|'.join(within_available)))
        # Diagnostic only: fixed first/last 2 ms cohorts, not internal snapshots.
        # Residual quantifies the within-cohort motion approximation.
        for label,mask in [('FIRST_2MS',r['time']<=lo+.002),('LAST_2MS',r['time']>=hi-.002)]:
            for method,points in [('A',xa),('B',xb)]:
                x,y=xr[mask],points[mask]
                if len(x)<4:raise ValueError('RIGID_DIAGNOSTIC_COHORT_TOO_SMALL')
                R=G.position_kabsch_rotation(x,y);t=y.mean(axis=0)-R@x.mean(axis=0)
                residual=np.linalg.norm((R@x.T).T+t-y,axis=1)
                rigid.append(dict(frame_index=frame,method=method,cohort=label,points=len(x),
                    approximation='2ms_cohort_rigid_fit_NOT_state_history_reconstruction',
                    rotation_deg=G.rotation_angle_deg(R),translation_m=float(np.linalg.norm(t)),
                    fit_residual_rms_m=float(np.sqrt(np.mean(residual**2)))))
    write_csv(cache/'deskew_feedback_summary.csv',records)
    write_csv(cache/'intra_scan_rigid_fit.csv',rigid)
    return records


def main(repo):
    cache=repo/'.p29_r1_fixed_source_feedback'
    receipt=json.loads((cache/'pre_gt_freeze.json').read_text())
    if receipt['protocol']!='P29_PRE_GT_OUTPUT_FREEZE' or receipt['GT_LOADED'] is not False or not receipt['completed']:
        raise ValueError('PRE_GT_FREEZE_REQUIRED')
    input_receipt=json.loads((cache/'input_freeze.json').read_text())
    for path,digest in input_receipt['files_sha256'].items():
        if sha(path)!=digest:raise ValueError('FROZEN_INPUT_CHANGED:'+path)
    for path,digest in receipt['files_sha256'].items():
        if sha(path)!=digest:raise ValueError('FROZEN_OUTPUT_CHANGED:'+path)
    history=repo/'docs/p10_corridor01_deterministic_oosm_replay_r1/posthoc_metrics.json'
    contract=json.loads(history.read_text());origin=contract['evaluation_origin_unix_s']
    common_R=np.array(contract['fit_transform_rotation']);common_t=np.array(contract['fit_transform_translation_m'])
    ext=repo/'.p18_map_recovery/posthoc_inputs/frozen.YOuGEy/corridor01_extrinsics.yaml'
    if sha(ext)!='59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d':
        raise ValueError('EXTRINSIC_FREEZE_FAILED')
    Til=G.load_t_imu_lidar(ext)  # Calibration only, not GT.
    start_ns=int(rows(repo/'docs/p28_r2_mature_visual_module/runs/A/mature_diagnostics.csv')[0]['scan_ns'])
    feedback,diags,updates=timeline(cache,origin,start_ns,Til)
    clouds=cloud_feedback(cache,repo,updates)
    parameter={}
    for method in ('A','B'):
        old=repo/'docs/p28_r2_mature_visual_module/runs'/method;new=cache/'runs'/method
        x,y=[yaml.safe_load((root/'rosparams.yaml').read_text()) for root in (old,new)]
        xx=normalized_params(x,Path(x['output']['ndt_diagnostics_csv_path']).parent)
        yy=normalized_params(y,Path(y['output']['ndt_diagnostics_csv_path']).parent)
        parameter[method]=dict(physical_parameters_equal=xx==yy,source_capture_enabled=y['output']['experimental_ndt_source_capture_enable'],
                              requests=y['output']['experimental_ndt_source_capture_requests'])
        if xx!=yy:raise ValueError('UNAUTHORIZED_RUNTIME_PARAMETER_DIFFERENCE')
    (cache/'parameter_parity.json').write_text(json.dumps(parameter,indent=2)+'\n')
    # All prescribed choices and native outputs above are immutable before GT.
    gt=repo/'.p18_map_recovery/posthoc_inputs/frozen.YOuGEy/corridor01_gt.txt';ext=gt.parent/'corridor01_extrinsics.yaml'
    if sha(gt)!='3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03' or sha(ext)!='59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d':
        raise ValueError('GT_OR_EXTRINSIC_FREEZE_FAILED')
    gt_times,gt_poses=G.load_gt(gt);Til=G.load_t_imu_lidar(ext)
    pairs=rows(cache/'offline/fixed_source_pairs.csv');evaluated=[]
    ledger=rows(cache/'offline/align_ledger.csv')
    if len(pairs)!=32 or len({(r['frame_index'],r['source'],r['initial']) for r in pairs})!=32 or sum(r['status']=='COMPLETED' for r in ledger)!=32:
        raise ValueError('ACTUAL_32_CELL_ALIGN_BUDGET_CONTRACT_FAILED')
    for row in pairs:
        record={key:row[key] for key in ('frame_index','scan_ns','source','initial','iterations','converged','fitness',
            'objective_initial','objective_raw','objective_used','align_ms','translation_limited','rotation_limited')}
        stamp=int(row['scan_ns'])/1e9;reference=G.interpolate_gt(gt_times,gt_poses,stamp)
        record['relative_time_s']=stamp-origin;record['primary_sample']=3<=stamp-origin<=35
        for prefix,name in [('initial_m','initial'),('raw_m','raw'),('used_m','used')]:
            T=matrix(row,prefix);pose=G.lidar_pose_to_imu(dict(position=T[:3,3],rotation=Rotation.from_matrix(T[:3,:3]).as_matrix()),Til)
            et,er=G.errors_for([pose],[reference],common_R,common_t)
            record[name+'_error_m']=float(et[0]);record[name+'_rotation_error_deg']=float(er[0])
        evaluated.append(record)
    write_csv(cache/'paired_accuracy.csv',evaluated)
    effects=[];initial=[]
    for request in REQUESTS:
        frame,ns=request.split(':');group={(r['source'],r['initial']):r for r in pairs if r['frame_index']==frame}
        errs={(r['source'],r['initial']):r for r in evaluated if r['frame_index']==frame}
        lidar={k:matrix(v,'raw_m') for k,v in group.items()}
        raw={k:imu_matrix(T,Til) for k,T in lidar.items()};aa,ab,ba,bb=[raw[k] for k in [('A','A'),('A','B'),('B','A'),('B','B')]]
        dt,dr=pose_difference(imu_matrix(matrix(group['A','A'],'initial_m'),Til),imu_matrix(matrix(group['B','B'],'initial_m'),Til))
        effect=dict(frame_index=frame,scan_ns=ns,reference_point='IMU_origin',initial_AB_translation_m=dt,initial_AB_rotation_deg=dr,
            source_effect_IA_m=pose_difference(aa,ba)[0],source_effect_IB_m=pose_difference(ab,bb)[0],
            initial_effect_SA_m=pose_difference(aa,ab)[0],initial_effect_SB_m=pose_difference(ba,bb)[0],
            observed_AB_raw_m=pose_difference(aa,bb)[0],
            observed_AB_raw_lidar_m=pose_difference(lidar['A','A'],lidar['B','B'])[0],
            raw_translation_interaction_m=float(np.linalg.norm(bb[:3,3]-ba[:3,3]-ab[:3,3]+aa[:3,3])))
        for tag,left,right in [('source_IA',('A','A'),('B','A')),('source_IB',('A','B'),('B','B')),
                               ('initial_SA',('A','A'),('A','B')),('initial_SB',('B','A'),('B','B'))]:
            effect[tag+'_raw_error_delta_m']=float(errs[right]['raw_error_m'])-float(errs[left]['raw_error_m'])
        effects.append(effect);initial.append({key:effect[key] for key in ('frame_index','scan_ns','initial_AB_translation_m','initial_AB_rotation_deg')})
    write_csv(cache/'fixed_source_effects.csv',effects);write_csv(cache/'initial_guess_comparison.csv',initial)
    high={m:frozen.state_trajectory(cache/'runs'/m/'ekf_lineage.csv')[0] for m in ('A','B')}
    aligned_pairs={m:frozen.pair_with_gt(records,gt_times,gt_poses,origin,35) for m,records in high.items()}
    own={m:frozen.fit_prefix(p,3)[1:3] for m,p in aligned_pairs.items()}
    align_rows=[];rpes=[]
    for method in ('A','B'):
        for label,R,t in [('FORMAL_SHARED_CONTROL_V2',common_R,common_t),
                          ('DIAGNOSTIC_INDEPENDENT_PREFIX',*own[method]),('DIAGNOSTIC_SHARED_A_PREFIX',*own['A'])]:
            for component,records in [('EKF',high[method]),('Raw_NDT',frozen.measurement_trajectory(diags[method],'raw_ndt_',Til,origin))]:
                p=frozen.pair_with_gt(records,gt_times,gt_poses,origin,35);times,et,er,_=frozen.summarize_window(p,R,t)
                for name,lo,hi in [('3_35',3,35),('26_35',26,35),('34_35',34,35)]:
                    mask=(times>=lo)&(times<=hi)
                    sr,se=summary(et[mask]),summary(er[mask])
                    align_rows.append(dict(method=method,component=component,transform=label,window=name,count=int(mask.sum()),
                        translation_rmse_m=sr['rmse'],translation_p95_m=sr['p95'],translation_max_m=sr['maximum'],
                        rotation_rmse_deg=se['rmse'],rotation_p95_deg=se['p95'],rotation_max_deg=se['maximum']))
        p=aligned_pairs[method];times=np.array([x[0] for x in p]);te=[];re=[];intervals=[]
        for i,record in enumerate(p):
            if times[i]<3:continue
            j=int(np.searchsorted(times,times[i]+1.0))
            if j>=len(p) or abs(times[j]-times[i]-1)>0.0051:continue
            intervals.append(times[j]-times[i])
            pe,Re=record[1];pg,Rg=record[2];qe,Qe=p[j][1];qg,Qg=p[j][2]
            te.append(np.linalg.norm(Re.T@(qe-pe)-Rg.T@(qg-pg)))
            re.append(G.rotation_angle_deg((Rg.T@Qg).T@(Re.T@Qe)))
        st,sr=summary(te),summary(re)
        rpes.append(dict(method=method,nominal_delta_sec=1.0,actual_delta_min_sec=float(min(intervals)),
                        actual_delta_max_sec=float(max(intervals)),count=len(te),translation_rmse_m=st['rmse'],
                        translation_p95_m=st['p95'],translation_max_m=st['maximum'],rotation_rmse_deg=sr['rmse']))
    write_csv(cache/'alignment_effect_summary.csv',align_rows);write_csv(cache/'rpe_summary.csv',rpes)
    transforms=dict(formal_A_B_rotation_difference_deg=0.,formal_A_B_translation_difference_m=0.,
        diagnostic_A_B_rotation_difference_deg=G.rotation_angle_deg(own['A'][0].T@own['B'][0]),
        diagnostic_A_B_translation_difference_m=float(np.linalg.norm(own['A'][1]-own['B'][1])),
        diagnostic_A_R=own['A'][0].tolist(),diagnostic_A_t=own['A'][1].tolist(),
        diagnostic_B_R=own['B'][0].tolist(),diagnostic_B_t=own['B'][1].tolist())
    (cache/'alignment_transforms.json').write_text(json.dumps(transforms,indent=2)+'\n')
    runtimes=[]
    for method in ('A','B'):
        receipt=json.loads((cache/'runs'/method/'execution.json').read_text())
        runtimes.append(dict(stage='capture_replay',method=method,count=396,mean_ms=None,p95_ms=None,
            max_ms=None,wall_sec=receipt['wall_sec'],peak_rss_bytes=receipt['resources']['dog_prior_map_ekf_node_cpp']['peak_rss_bytes'],
            cpu_sec=receipt['resources']['dog_prior_map_ekf_node_cpp']['cpu_sec']))
    durations=[float(r['align_ms']) for r in pairs]
    runtimes.append(dict(stage='native_offline_align',method='all_32',count=32,mean_ms=float(np.mean(durations)),
        p95_ms=float(np.percentile(durations,95)),max_ms=float(max(durations)),wall_sec=None,peak_rss_bytes=None,cpu_sec=None))
    write_csv(cache/'runtime_summary.csv',runtimes)
    temporal_bounds=dict(first_visual_measurement_ns=updates[0]['image_ns'],
        first_visual_release_stamp=updates[0]['release_state_stamp'],
        first_changed_high_rate_stamp=next(e['sensor_stamp_s'] for e in feedback['events'] if e['event']=='FIRST_HIGH_RATE_STATE_DIFFERENCE'),
        first_affected_scan_start=rows(cache/'runs/B/deskew.csv')[6]['scan_start_stamp'],
        first_affected_scan_end=rows(cache/'runs/B/deskew.csv')[6]['scan_end_stamp'],
        first_affected_ndt_ros_now=diags['B'][6]['ros_now'],
        note='Image measurement time, release/high-rate visibility, scan reference and processing watermark are distinct clocks; scan_start is not execution time.')
    result=dict(TASK='P29-R1-FIXED-SOURCE-NDT-FEEDBACK-ISOLATION',GT_LOADED=True,
        SOURCE_CAPTURE='PASS',P28_PARITY={m:json.loads((cache/('capture_gate_'+m+'.json')).read_text())['p28_parity'] for m in ('A','B')},
        full_aligns=32,optimizer_parity=rows(cache/'offline/optimizer_parity.csv'),feedback=feedback,
        source_pointwise=clouds,effects=effects,paired_accuracy=evaluated,alignment=transforms,temporal_bounds=temporal_bounds,
        alignment_metrics=align_rows,rpe=rpes,parameter_parity=parameter,
        source_reconstruction_used=False,runtime_math_changed=False,candidate_recovery_changed=False)
    (cache/'analysis_results.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(result='P29_FINITE_ANALYSIS_COMPLETE',full_aligns=32,events=feedback['events'],
        effects=effects,alignment=transforms,rpe=rpes),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);main(**vars(p.parse_args()))
