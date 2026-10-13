Adversarial review. Find what is wrong with this artifact. Assume the author is overconfident. Do NOT validate or summarize. Report only substantiated required issues, or explicitly say no substantive issue.

Do NOT execute commands, read other files, call tools, access network, modify files, run ROS or run NDT. Use only this supplied artifact. No user/session history is provided.

CONTRACT: Eight prescribed sensor frames, exact integer ns identities; real captured A/B NDT Source, identical runtime binaries and inputs, 32 native align calls total. 2x2 raw optimizer effects must isolate Source at fixed Initial and Initial at fixed Source. All primary pose differences and GT errors use IMU origin; a separately named LiDAR diagnostic column is permitted. Derived Used uses PA for Source A and PB for Source B, so cross-source Used differences are NOT pure Source effects. Sensor image time, OOSM release, scan start/reference, processing clocks are distinct. Deskew cohorts are approximate rigid fits, NOT state history reconstruction. Formal A/B metrics share frozen Control V2 transform; separate 0-3s fitted transforms are diagnostic only. RPE is nominal 1s and actual accepted intervals are recorded. No strict local-minimum or unique causal root claim can follow merely from hasConverged. No GT-based frame/parameter selection. Review the corrected origin handling, math, row pairing, causal qualifiers and budget. No raw GT/images/clouds/bags or secrets are supplied.

ARTIFACT:

### src/dog_prior_map_localization/scripts/p29_feedback/analyze.py
```
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

```

### src/dog_prior_map_localization/scripts/p29_feedback/offline_pair.cpp
```
// Fixed 2x2 experiment: existing native PCL NDT, no new search or feedback.
#include "../p23_budgeted_coupled/frozen_ndt_rules.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/observable_pcl_ndt.hpp"
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/resource.h>
#include <fcntl.h>
#include <unistd.h>
using Cloud=pcl::PointCloud<pcl::PointXYZ>;
using Ndt=dog_prior_map_fastlio2_frontend_exp::ObservablePclNdt<pcl::PointXYZ>;
using Clock=std::chrono::steady_clock;
struct Case {
  int frame,points,width,height,dense,iterations,converged,lt,lr,selected;
  uint64_t ns,hash; double fitness;
};
std::vector<Case> cases(const std::string& path) {
  std::ifstream in(path); std::vector<Case> out; Case c;
  while(in>>c.frame>>c.ns>>c.points>>c.width>>c.height>>c.dense>>std::hex>>c.hash>>std::dec
        >>c.iterations>>c.converged>>c.fitness>>c.lt>>c.lr>>c.selected)out.push_back(c);
  if(out.size()!=8)throw std::runtime_error("P29_EIGHT_CASES_REQUIRED");
  return out;
}
std::vector<Eigen::Matrix4d> poses(const std::string& path) {
  std::ifstream in(path,std::ios::binary); std::vector<Eigen::Matrix4d> out(5);
  for(auto& T:out)for(int i=0;i<4;++i)for(int j=0;j<4;++j)
    if(!in.read(reinterpret_cast<char*>(&T(i,j)),sizeof(double)))throw std::runtime_error("POSE_READ_FAILED");
  for(const auto& T:out)if(!T.allFinite())throw std::runtime_error("NONFINITE_INPUT_POSE");
  return out;
}
Cloud::Ptr source(const std::string& path,const Case& c) {
  std::ifstream in(path,std::ios::binary); Cloud::Ptr out(new Cloud); float p[3];
  while(in.read(reinterpret_cast<char*>(p),sizeof(p)))out->push_back(pcl::PointXYZ(p[0],p[1],p[2]));
  out->width=c.width;out->height=c.height;out->is_dense=c.dense;
  if(out->size()!=static_cast<std::size_t>(c.points)||p23::FrozenNdtRules::stableCloudHash(out)!=c.hash)
    throw std::runtime_error("FIXED_SOURCE_HASH_FAILED");
  return out;
}
double angle(const Eigen::Matrix4d& a,const Eigen::Matrix4d& b) {
  return Eigen::AngleAxisd(Eigen::Quaterniond(a.block<3,3>(0,0)).normalized().conjugate()*
    Eigen::Quaterniond(b.block<3,3>(0,0)).normalized()).angle()*180/M_PI;
}
void matrix(std::ostream& out,const Eigen::Matrix4d& T) {
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<T(i,j);
}
int main(int argc,char** argv) {
  if(argc!=3){std::cerr<<"map.pcd P29-cache\n";return 2;}
  int calls=0;
  try {
    std::string root=argv[2],out=root+"/offline";
    int fd=::open((out+"/started").c_str(),O_CREAT|O_EXCL|O_WRONLY,0600);
    if(fd<0)throw std::runtime_error("OFFLINE_ALREADY_ATTEMPTED");::close(fd);
    auto A=cases(root+"/cases_A.txt"),B=cases(root+"/cases_B.txt");
    p23::FrozenNdtRules rules;Cloud::Ptr raw(new Cloud),finite(new Cloud);
    if(pcl::io::loadPCDFile(argv[1],*raw))throw std::runtime_error("MAP_READ_FAILED");
    for(const auto& p:*raw)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))finite->push_back(p);
    p23::finalizeCloud(finite);
    const auto map=rules.voxelDown(finite,.15,.15,0),target=rules.voxelDown(map,.15,.15,0);
    if(target->size()!=226164)throw std::runtime_error("TARGET_POINTS_CHANGED");
    Ndt ndt;ndt.setInputTarget(target);ndt.setResolution(.8);ndt.setStepSize(.08);
    ndt.setTransformationEpsilon(.001);ndt.setMaximumIterations(40);
    std::ofstream results(out+"/fixed_source_pairs.csv"),parity(out+"/optimizer_parity.csv"),ledger(out+"/align_ledger.csv");
    if(!results||!parity||!ledger)throw std::runtime_error("OUTPUT_OPEN_FAILED");
    results<<std::setprecision(17);parity<<std::setprecision(17);
    results<<"frame_index,scan_ns,source,initial,source_hash,points,iterations,converged,fitness,objective_initial,objective_raw,objective_used,align_ms,translation_limited,rotation_limited,online_selected_candidate,candidate_replacement_executed";
    for(const auto prefix:{"initial_m","raw_m","used_m"})for(int i=0;i<4;++i)for(int j=0;j<4;++j)results<<','<<prefix<<i<<j;
    results<<'\n';
    parity<<"frame_index,method,raw_dt_m,raw_dr_deg,used_dt_m,used_dr_deg,fitness_delta,discrete_match,status\n";
    ledger<<"call,frame,source,initial,status\n";
    for(int n=0;n<8;++n) {
      if(A[n].frame!=B[n].frame||A[n].ns!=B[n].ns)throw std::runtime_error("CASE_IDENTITIES_DIFFER");
      const std::string name="/frame_"+std::to_string(A[n].frame);
      const auto PA=poses(root+"/sources/A"+name+".poses.f64"),PB=poses(root+"/sources/B"+name+".poses.f64");
      const auto SA=source(root+"/sources/A"+name+".xyz",A[n]),SB=source(root+"/sources/B"+name+".xyz",B[n]);
      for(int s=0;s<2;++s)for(int i=0;i<2;++i) {
        const Case& c=s?B[n]:A[n]; const auto& history=s?PB:PA; const auto& init=i?PB[1]:PA[1];
        ndt.setInputSource(s?SB:SA);rules.previous_pose_=history[0];
        if(++calls>32)throw std::runtime_error("ALIGN_BUDGET_EXCEEDED");
        ledger<<calls<<','<<c.frame<<','<<(s?'B':'A')<<','<<(i?'B':'A')<<",STARTED\n";ledger.flush();
        Cloud aligned;auto start=Clock::now();ndt.align(aligned,init.cast<float>());
        const double ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        const Eigen::Matrix4d result=ndt.getFinalTransformation().cast<double>();
        const double fitness=ndt.getFitnessScore();const int iterations=ndt.getFinalNumIteration();
        const bool converged=ndt.hasConverged();Eigen::Matrix4d used=result;bool lt=false,lr=false;
        rules.limitNdtStep(result,used,lt,lr);
        const auto cloud=s?SB:SA;
        const double vi=ndt.dynamicScore(cloud,init.cast<float>()),vr=ndt.dynamicScore(cloud,result.cast<float>()),vu=ndt.dynamicScore(cloud,used.cast<float>());
        if(!result.allFinite()||!used.allFinite()||!std::isfinite(fitness)||!std::isfinite(vi)||!std::isfinite(vr)||!std::isfinite(vu))
          throw std::runtime_error("NONFINITE_OFFLINE_RESULT");
        results<<c.frame<<','<<c.ns<<','<<(s?'B':'A')<<','<<(i?'B':'A')<<','<<std::hex<<c.hash<<std::dec<<','<<c.points
          <<','<<iterations<<','<<converged<<','<<fitness<<','<<vi<<','<<vr<<','<<vu<<','<<ms<<','<<lt<<','<<lr<<','<<c.selected<<",0";
        matrix(results,init);matrix(results,result);matrix(results,used);results<<'\n';results.flush();
        ledger<<calls<<','<<c.frame<<','<<(s?'B':'A')<<','<<(i?'B':'A')<<",COMPLETED\n";ledger.flush();
        if(s==i) {
          const double dt=(result.block<3,1>(0,3)-history[3].block<3,1>(0,3)).norm(),dr=angle(result,history[3]);
          const double ut=(used.block<3,1>(0,3)-history[4].block<3,1>(0,3)).norm(),ur=angle(used,history[4]);
          const bool discrete=iterations==c.iterations&&converged==bool(c.converged)&&lt==bool(c.lt)&&lr==bool(c.lr);
          const bool same=dt<=1e-5&&dr<=1e-4&&ut<=1e-5&&ur<=1e-4&&discrete&&std::abs(fitness-c.fitness)<=1e-6*std::max(1.,std::abs(c.fitness));
          const char* status=c.selected>=0?"ONLINE_MULTI_SELECTED":(same?"PASS":"FAIL");
          parity<<c.frame<<','<<(s?'B':'A')<<','<<dt<<','<<dr<<','<<ut<<','<<ur<<','<<fitness-c.fitness<<','<<discrete<<','<<status<<'\n';parity.flush();
          if(c.selected<0&&!same)throw std::runtime_error("NATIVE_OPTIMIZER_PARITY_FAILED");
        }
      }
    }
    rusage usage;getrusage(RUSAGE_SELF,&usage);
    std::ofstream resource(out+"/resource_summary.csv");
    resource<<"target_points,target_hash,full_align_calls,objective_value_calls,peak_rss_kib\n"<<target->size()<<','
      <<std::hex<<rules.stableCloudHash(target)<<std::dec<<','<<calls<<','<<calls*3<<','<<usage.ru_maxrss<<'\n';
    std::cout<<"P29_OFFLINE_PASS frames=8 full_aligns="<<calls<<'\n';return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<" full_aligns="<<calls<<'\n';return 1;}
}

```

### src/dog_prior_map_localization/scripts/p29_feedback/test_contract.py
```
"""Small contract/numerical tests; no extra native align and no GT selection."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from prepare import REQUESTS, rows
from analyze import pose_difference, imu_matrix

repo=Path(__file__).resolve().parents[4]
cache=repo/'.p29_r1_fixed_source_feedback'
assert len(REQUESTS)==8 and len({r.split(':')[1] for r in REQUESTS})==8
assert all(str(int(r.split(':')[1]))==r.split(':')[1] for r in REQUESTS)
# Exact uint64 identity must survive JSON roundtrip, never float conversion.
assert json.loads(json.dumps(REQUESTS))==REQUESTS
for method in ('A','B'):
    gate=json.loads((cache/('capture_gate_'+method+'.json')).read_text())
    assert gate['captured']==8 and gate['source_capture']=='PASS' and gate['p28_parity']=='PASS'
pairs=rows(cache/'offline/fixed_source_pairs.csv')
assert len(pairs)==32 and len({(r['frame_index'],r['source'],r['initial']) for r in pairs})==32
for request in REQUESTS:
    frame,ns=request.split(':'); group=[r for r in pairs if r['frame_index']==frame]
    assert len(group)==4 and {r['scan_ns'] for r in group}=={ns}
    for source in ('A','B'):
        assert len({r['source_hash'] for r in group if r['source']==source})==1
    for initial in ('A','B'):
        columns=[k for k in group[0] if k.startswith('initial_m')]
        assert len({tuple(r[k] for k in columns) for r in group if r['initial']==initial})==1
assert all(r['status']=='PASS' for r in rows(cache/'offline/optimizer_parity.csv'))
assert sum(r['status']=='STARTED' for r in rows(cache/'offline/align_ledger.csv'))==32
assert sum(r['status']=='COMPLETED' for r in rows(cache/'offline/align_ledger.csv'))==32
A=np.eye(4);B=np.eye(4);B[:3,3]=[1,2,3]
T=np.eye(4);T[:3,:3]=Rotation.from_rotvec([.2,-.1,.3]).as_matrix();T[:3,3]=[4,5,6]
assert pose_difference(A,A)==(0.,0.)
assert abs(pose_difference(A,B)[0]-pose_difference(T@A,T@B)[0])<1e-12
lever=np.eye(4);lever[0,3]=.1
L0=np.eye(4);L1=np.eye(4);L1[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
assert pose_difference(L0,L1)[0]==0.
assert abs(pose_difference(imu_matrix(L0,lever),imu_matrix(L1,lever))[0]-np.sqrt(.02))<1e-12
print('P29 contracts PASS: exact IDs, 16 real Sources, 32 cells, 16 self-parities, shared-transform isometry; no extra align')

```

### .p29_r1_fixed_source_feedback/fixed_source_effects.csv
```
frame_index,scan_ns,reference_point,initial_AB_translation_m,initial_AB_rotation_deg,source_effect_IA_m,source_effect_IB_m,initial_effect_SA_m,initial_effect_SB_m,observed_AB_raw_m,observed_AB_raw_lidar_m,raw_translation_interaction_m,source_IA_raw_error_delta_m,source_IB_raw_error_delta_m,initial_SA_raw_error_delta_m,initial_SB_raw_error_delta_m
7,1517157219794140100,IMU_origin,0.0,0.0,0.006110443246214823,0.006110443246214823,0.0,0.0,0.006110443246214823,0.005970160681774045,0.0,0.0005574073003646457,0.0005574073003646457,0.0,0.0
193,1517157238552942991,IMU_origin,0.12812463657649323,0.14138263500927792,0.020715836723907987,0.015149249305383618,0.11381228505634182,0.10810319825379983,0.128736897478747,0.1287255558910394,0.006161967602530084,0.002766969004012576,0.0042792418337290195,0.02393439837488026,0.025446671204596705
270,1517157246318675041,IMU_origin,0.3896524589425089,0.2754052940142546,0.005572231504668942,0.0063077833744192985,0.3665636153869383,0.3603422625417164,0.36386581424771464,0.36385087461481325,0.006500760060454746,0.001236687541196324,0.002421989410087977,0.03900856567323696,0.040193867542128614
320,1517157251361368895,IMU_origin,0.21218408340849484,0.17680635372334533,0.009993977029889172,0.0019673533357252144,0.21260210090414214,0.22282221463556537,0.21330722151358394,0.21331445676591923,0.010435796298176137,0.008578607579410313,0.0010001156455616833,-0.14356197278548488,-0.1511404647193335
340,1517157253378448009,IMU_origin,0.40002262742343153,0.06023478419653722,0.0012297616062056315,0.004266143193578941,0.4014922140544969,0.3986491399914801,0.3982304486712386,0.39823494970058926,0.003539742769403789,3.108953930874847e-05,0.0013685608431799334,-0.3534186871491467,-0.3520812158452755
347,1517157254084407091,IMU_origin,0.516181234978513,0.2568736339072247,0.006533213660145487,0.005076837756303092,0.5309607849827008,0.5406676569567295,0.5342155956777652,0.5342243284072125,0.010134814682035918,0.005743746479960787,-0.0045298982621244654,-0.4919299100836265,-0.5022035548257118
365,1517157255899825096,IMU_origin,1.1593326809357216,3.1018930745064646,0.1499904552615166,0.14142314167984546,1.0332361514311634,1.0201850322484094,1.1029850243950483,1.1140350081987358,0.23902582497439115,-0.01423971355407705,0.12932686482215772,0.7306361685606904,0.8742027469369251
396,1517157259026242018,IMU_origin,6.170020379588524,0.04832419509847991,0.0057525393354260455,0.002467064029090333,6.166841081056018,6.164143845856324,6.169228720184917,6.16927818415539,0.004087675407186316,0.004585798605642566,0.0024164401246018485,6.124053348838293,6.121883990357253

```

### .p29_r1_fixed_source_feedback/paired_accuracy.csv
```
frame_index,scan_ns,source,initial,iterations,converged,fitness,objective_initial,objective_raw,objective_used,align_ms,translation_limited,rotation_limited,relative_time_s,primary_sample,initial_error_m,initial_rotation_error_deg,raw_error_m,raw_rotation_error_deg,used_error_m,used_rotation_error_deg
7,1517157219794140100,A,A,1,1,0.54586112293040578,1829.6324356916102,1856.8057040650549,1856.8057040650549,31.342677999999999,0,0,-4.394838809967041,False,0.06353698601096265,3.833602305483373,0.07738121041706632,4.084105163550919,0.07738121041706632,4.084105163550919
7,1517157219794140100,A,B,1,1,0.54586112293040578,1829.6324356916102,1856.8057040650549,1856.8057040650549,4.2574839999999998,0,0,-4.394838809967041,False,0.06353698601096265,3.833602305483373,0.07738121041706632,4.084105163550919,0.07738121041706632,4.084105163550919
7,1517157219794140100,B,A,1,1,0.5721197360115704,1813.3113147759309,1824.7593741687729,1824.7593741687729,4.1872930000000004,0,0,-4.394838809967041,False,0.06353698601096265,3.833602305483373,0.07793861771743096,3.9879678542772954,0.07793861771743096,3.9879678542772954
7,1517157219794140100,B,B,1,1,0.5721197360115704,1813.3113147759309,1824.7593741687729,1824.7593741687729,4.2808109999999999,0,0,-4.394838809967041,False,0.06353698601096265,3.833602305483373,0.07793861771743096,3.9879678542772954,0.07793861771743096,3.9879678542772954
193,1517157238552942991,A,A,1,1,0.015874639528660087,2407.4210310902513,2413.8303749249521,2413.8303749249521,4.7496590000000003,0,0,14.363964080810547,True,0.3786566633825516,2.2657410968369858,0.3720666138825963,2.3710534740854285,0.3720666138825963,2.3710534740854285
193,1517157238552942991,A,B,1,1,0.015779012479296008,2403.3237687772107,2406.2229558165459,2406.2229558165459,4.7126429999999999,0,0,14.363964080810547,True,0.40258488313260654,2.366205472014202,0.3960010122574766,2.435481861059526,0.3960010122574766,2.435481861059526
193,1517157238552942991,B,A,1,1,0.016422708026792472,2414.6668795802007,2411.7440151956366,2411.7440151956366,4.7145979999999996,0,0,14.363964080810547,True,0.3786566633825516,2.2657410968369858,0.3748335828866089,2.4518076127678095,0.3748335828866089,2.4518076127678095
193,1517157238552942991,B,B,1,1,0.016634743265387573,2411.5411222168041,2409.4446564951104,2409.4446564951104,10.461667,0,0,14.363964080810547,True,0.40258488313260654,2.366205472014202,0.4002802540912056,2.6077898325182516,0.4002802540912056,2.6077898325182516
270,1517157246318675041,A,A,1,1,1.1676896172819604,2298.4846719797083,2302.3111302795492,2302.3111302795492,4.6402169999999998,0,0,22.129696130752563,True,0.6444769017961202,2.0436012267987893,0.635439585881129,2.360594906946965,0.635439585881129,2.360594906946965
270,1517157246318675041,A,B,1,1,1.1687933053427539,2317.3520191513994,2318.1077343423303,2306.3894102813033,4.6576069999999996,1,0,22.129696130752563,True,0.6816164481235979,2.265676033352831,0.674448151554366,2.5306308152263517,0.6264284778359047,2.5306308152263517
270,1517157246318675041,B,A,1,1,1.1660222390422132,2294.2199623311353,2299.1457071353016,2299.1457071353016,4.589791,0,0,22.129696130752563,True,0.6444769017961202,2.0436012267987893,0.6366762734223254,2.3099125684093615,0.6366762734223254,2.3099125684093615
270,1517157246318675041,B,B,1,1,1.1671532898436714,2319.4485060416396,2318.1098805464285,2318.1098805464285,4.6398669999999997,0,0,22.129696130752563,True,0.6816164481235979,2.265676033352831,0.676870140964454,2.435053620300382,0.676870140964454,2.435053620300382
320,1517157251361368895,A,A,1,1,0.013509092677704757,2433.6939695872952,2434.9198777438892,2434.9198777438892,4.7558049999999996,0,0,27.17238998413086,True,1.2201455281861373,1.5519159832843197,1.206562582312835,1.4146725247271548,1.206562582312835,1.4146725247271548
320,1517157251361368895,A,B,1,1,0.013169155216793505,2445.5682262040232,2446.0466835039488,2446.0466835039488,4.7088720000000004,0,0,27.17238998413086,True,1.075052316193929,1.7008068909627734,1.06300060952735,1.498038011673439,1.06300060952735,1.498038011673439
320,1517157251361368895,B,A,1,1,0.015055740683305235,2426.8350863350765,2427.2964973087032,2428.4153831157028,4.6768140000000002,1,0,27.17238998413086,True,1.2201455281861373,1.5519159832843197,1.2151411898922453,1.4148828106875138,1.196706534626767,1.4148828106875138
320,1517157251361368895,B,B,1,1,0.014593757997225631,2437.9101024823863,2438.5180142724557,2438.5180142724557,4.6813539999999998,0,0,27.17238998413086,True,1.075052316193929,1.7008068909627734,1.0640007251729118,1.5584544078018643,1.0640007251729118,1.5584544078018643
340,1517157253378448009,A,A,1,1,0.021517521532306481,2332.4679824773434,2337.2713922167422,2337.2713922167422,4.6578169999999997,0,0,29.1894690990448,True,2.0269701562640634,1.2009857252981906,2.0315806797687634,1.1703973926892215,2.0315806797687634,1.1703973926892215
340,1517157253378448009,A,B,1,1,0.020843880423245927,2333.5097834199264,2336.208450281355,2336.208450281355,4.6330229999999997,0,0,29.1894690990448,True,1.6770388788250554,1.1868227826304067,1.6781619926196167,1.1567656934922144,1.6781619926196167,1.1567656934922144
340,1517157253378448009,B,A,1,1,0.021515045904213496,2338.6466327951234,2342.5770435626396,2347.7962794049786,4.5780570000000003,1,0,29.1894690990448,True,2.0269701562640634,1.2009857252981906,2.031611769308072,1.1633847185870567,1.8420308811039838,1.1633847185870567
340,1517157253378448009,B,B,1,1,0.021020490436692074,2338.4561237113658,2342.4690357542736,2342.4690357542736,4.646363,0,0,29.1894690990448,True,1.6770388788250554,1.1868227826304067,1.6795305534627967,1.1559228312684577,1.6795305534627967,1.1559228312684577
347,1517157254084407091,A,A,1,1,0.027795946462740955,2320.1639628939415,2319.1744508760898,2319.1744508760898,4.5784070000000003,0,0,29.89542818069458,True,2.6315830199323353,1.2892933248941698,2.6380449051031745,1.2483976919617916,2.6380449051031745,1.2483976919617916
347,1517157254084407091,A,B,1,1,0.025552648763675408,2322.8333344806938,2324.3521498174678,2324.3521498174678,4.5258159999999998,0,0,29.89542818069458,True,2.1541566460680435,1.2525761020457076,2.146114995019548,1.249337226055533,2.146114995019548,1.249337226055533
347,1517157254084407091,B,A,1,1,0.027219538288934149,2316.8888170969099,2315.4181980559588,2309.6726872088184,4.5895820000000001,1,0,29.89542818069458,True,2.6315830199323353,1.2892933248941698,2.6437886515831353,1.2515664781108025,2.288488483174376,1.2515664781108025
347,1517157254084407091,B,B,1,1,0.025391176912240292,2321.0398172883088,2322.1106957503739,2322.1106957503739,4.540832,0,0,29.89542818069458,True,2.1541566460680435,1.2525761020457076,2.1415850967574235,1.2544414314185455,2.1415850967574235,1.2544414314185455
365,1517157255899825096,A,A,20,1,0.050393765326480204,576.5988959373334,2126.9186370470716,733.52653779515617,41.508844000000003,1,1,31.710846185684204,True,3.840749162056737,14.654517923661386,4.092360670776336,1.5951578232171177,3.7428923842774298,10.657729721084705
365,1517157255899825096,A,B,4,1,2.4974580247644917,592.54191797155545,716.1429785088161,729.32159669719658,7.9320680000000001,1,0,31.710846185684204,True,4.7572054542685205,15.134259307007936,4.822996839337026,12.175152433797008,3.9667239337603903,12.175152433797008
365,1517157255899825096,B,A,19,1,0.048823748900608081,592.71590550868723,2167.6930031706711,655.71400430336246,36.400548000000001,1,1,31.710846185684204,True,3.840749162056737,14.654517923661386,4.078120957222259,1.463726899821471,4.097163821593635,10.846262166248534
365,1517157255899825096,B,B,6,1,2.0627359386965898,607.67084942840813,717.08373715666255,715.80943012938803,11.113220999999999,1,0,31.710846185684204,True,4.7572054542685205,15.134259307007936,4.952323704159184,11.17062871928013,4.8130104447533135,11.17062871928013
396,1517157259026242018,A,A,1,1,0.045240001568464715,2210.7240335542665,2214.0756945194248,2214.0756945194248,4.4612119999999997,0,0,34.837263107299805,True,6.384909921993716,1.9132501715292234,6.38005075993062,1.9445296186192693,6.38005075993062,1.9445296186192693
396,1517157259026242018,A,B,1,1,0.043409200444304068,2342.9395852153511,2351.8328639199217,2231.7935997406144,4.5431369999999998,1,0,34.837263107299805,True,12.513619767054871,1.8661973466612385,12.504104108768914,1.8269312209741935,6.57263077561759,1.8269312209741935
396,1517157259026242018,B,A,1,1,0.04605034653552658,2207.9803710579231,2207.623928356184,2301.2640060850463,12.152118,1,0,34.837263107299805,True,6.384909921993716,1.9132501715292234,6.384636558536263,1.894671392727835,11.534175712284997,1.894671392727835
396,1517157259026242018,B,B,1,1,0.042024299484201035,2323.7088205552395,2336.092501081268,2336.092501081268,4.4693839999999998,0,0,34.837263107299805,True,12.513619767054871,1.8661973466612385,12.506520548893516,1.857105484116993,12.506520548893516,1.857105484116993

```

### .p29_r1_fixed_source_feedback/alignment_effect_summary.csv
```
method,component,transform,window,count,translation_rmse_m,translation_p95_m,translation_max_m,rotation_rmse_deg,rotation_p95_deg,rotation_max_deg
A,EKF,FORMAL_SHARED_CONTROL_V2,3_35,6396,1.9769247258305394,4.88159712856743,6.485653811199304,2.7373268403968707,3.6163259692404637,14.232664228630636
A,EKF,FORMAL_SHARED_CONTROL_V2,26_35,1799,3.6130715198591594,6.249051036543529,6.485653811199304,3.482010602660322,8.665038415945332,14.232664228630636
A,EKF,FORMAL_SHARED_CONTROL_V2,34_35,200,6.187017887558957,6.466967562272133,6.485653811199304,1.6303757351393555,2.4287847794853943,2.5536964612807944
A,Raw_NDT,FORMAL_SHARED_CONTROL_V2,3_35,316,1.9378304304179956,4.773930885561753,6.38005075993062,2.476912815417123,3.4232085051038554,14.702781142996084
A,Raw_NDT,FORMAL_SHARED_CONTROL_V2,26_35,88,3.5558154077630517,6.109239594545557,6.38005075993062,2.745379557101412,3.2510785896907217,14.702781142996084
A,Raw_NDT,FORMAL_SHARED_CONTROL_V2,34_35,9,6.1119817200729045,6.362408010993711,6.38005075993062,1.494974232550772,2.0829271894139167,2.1751922366105987
A,EKF,DIAGNOSTIC_INDEPENDENT_PREFIX,3_35,6396,1.9739256004331744,4.877099538088656,6.481476299335189,2.676645997024591,3.5288142841096812,14.167617414979553
A,EKF,DIAGNOSTIC_INDEPENDENT_PREFIX,26_35,1799,3.608487434776634,6.24483420078421,6.481476299335189,3.4404089300220426,8.691078678047681,14.167617414979553
A,EKF,DIAGNOSTIC_INDEPENDENT_PREFIX,34_35,200,6.182818240467873,6.462788454921726,6.481476299335189,1.545831309912506,2.378171766111721,2.4737747428230388
A,Raw_NDT,DIAGNOSTIC_INDEPENDENT_PREFIX,3_35,316,1.9348070394070849,4.769475892788901,6.375831603378091,2.4097212377123407,3.3646546674273683,14.688299310079776
A,Raw_NDT,DIAGNOSTIC_INDEPENDENT_PREFIX,26_35,88,3.551184711275052,6.105024788382541,6.375831603378091,2.695635465437494,3.30954341489116,14.688299310079776
A,Raw_NDT,DIAGNOSTIC_INDEPENDENT_PREFIX,34_35,9,6.107756011085469,6.3581903310238435,6.375831603378091,1.4092344576184392,2.00442000922337,2.1184650064283677
A,EKF,DIAGNOSTIC_SHARED_A_PREFIX,3_35,6396,1.9739256004331744,4.877099538088656,6.481476299335189,2.676645997024591,3.5288142841096812,14.167617414979553
A,EKF,DIAGNOSTIC_SHARED_A_PREFIX,26_35,1799,3.608487434776634,6.24483420078421,6.481476299335189,3.4404089300220426,8.691078678047681,14.167617414979553
A,EKF,DIAGNOSTIC_SHARED_A_PREFIX,34_35,200,6.182818240467873,6.462788454921726,6.481476299335189,1.545831309912506,2.378171766111721,2.4737747428230388
A,Raw_NDT,DIAGNOSTIC_SHARED_A_PREFIX,3_35,316,1.9348070394070849,4.769475892788901,6.375831603378091,2.4097212377123407,3.3646546674273683,14.688299310079776
A,Raw_NDT,DIAGNOSTIC_SHARED_A_PREFIX,26_35,88,3.551184711275052,6.105024788382541,6.375831603378091,2.695635465437494,3.30954341489116,14.688299310079776
A,Raw_NDT,DIAGNOSTIC_SHARED_A_PREFIX,34_35,9,6.107756011085469,6.3581903310238435,6.375831603378091,1.4092344576184392,2.00442000922337,2.1184650064283677
B,EKF,FORMAL_SHARED_CONTROL_V2,3_35,6396,3.1047078020521264,8.799511452051483,12.883491211847893,2.8194987233560593,3.698021031690448,14.678118768362761
B,EKF,FORMAL_SHARED_CONTROL_V2,26_35,1799,5.7826010092926445,11.834723627255505,12.883491211847893,3.722440834847461,11.052596493504174,14.678118768362761
B,EKF,FORMAL_SHARED_CONTROL_V2,34_35,200,11.756361532744783,12.791086510576646,12.883491211847893,1.625251316530799,2.4173056922656846,2.4772935448992444
B,Raw_NDT,FORMAL_SHARED_CONTROL_V2,3_35,316,3.0144807702257377,8.446812980287714,12.506520548893516,2.6311098547344165,3.4465033831526055,14.010028489881662
B,Raw_NDT,FORMAL_SHARED_CONTROL_V2,26_35,88,5.638746136183931,11.482516987726285,12.506520548893516,3.227398048694625,5.164626572724883,14.010028489881662
B,Raw_NDT,FORMAL_SHARED_CONTROL_V2,34_35,9,11.572118710285265,12.420412155160012,12.506520548893516,1.4839387890970084,1.9651070279063538,2.0371080570989997
B,EKF,DIAGNOSTIC_INDEPENDENT_PREFIX,3_35,6396,3.1006414618334768,8.793780466029494,12.878527884092867,2.7590589983589355,3.606082925357774,14.651128852840895
B,EKF,DIAGNOSTIC_INDEPENDENT_PREFIX,26_35,1799,5.776542503872911,11.829592266757587,12.878527884092867,3.682519135075227,10.999327901386396,14.651128852840895
B,EKF,DIAGNOSTIC_INDEPENDENT_PREFIX,34_35,200,11.751264207291099,12.786143811445674,12.878527884092867,1.5365901493737992,2.3600882420533993,2.4025139192248055
B,Raw_NDT,DIAGNOSTIC_INDEPENDENT_PREFIX,3_35,316,3.0103482476679586,8.441071483277593,12.501506684510497,2.5641512469765866,3.319017536288811,13.983305407260955
B,Raw_NDT,DIAGNOSTIC_INDEPENDENT_PREFIX,26_35,88,5.632592154297217,11.477364788350137,12.501506684510497,3.179263407345609,5.167393627599537,13.983305407260955
B,Raw_NDT,DIAGNOSTIC_INDEPENDENT_PREFIX,34_35,9,11.566978941037778,12.415382798118609,12.501506684510497,1.3925953116955179,1.8950151311349503,1.9974613242353676
B,EKF,DIAGNOSTIC_SHARED_A_PREFIX,3_35,6396,3.1025898571747788,8.796163019952663,12.880408636296192,2.7604694784012804,3.6099971204472485,14.640673874209659
B,EKF,DIAGNOSTIC_SHARED_A_PREFIX,26_35,1799,5.779344731260477,11.83157259936425,12.880408636296192,3.681404144839605,10.98927271740566,14.640673874209659
B,EKF,DIAGNOSTIC_SHARED_A_PREFIX,34_35,200,11.75323008941281,12.788012696721408,12.880408636296192,1.5409478258710516,2.366465445091988,2.4094350619149396
B,Raw_NDT,DIAGNOSTIC_SHARED_A_PREFIX,3_35,316,3.0123385962888887,8.443471477403824,12.503416818572315,2.5661255060928925,3.3191706224636524,13.973068361268693
B,Raw_NDT,DIAGNOSTIC_SHARED_A_PREFIX,26_35,88,5.635449644066654,11.479361666963054,12.503416818572315,3.179059392873298,5.163729509598657,13.973068361268693
B,Raw_NDT,DIAGNOSTIC_SHARED_A_PREFIX,34_35,9,11.568970088682422,12.417302030608848,12.503416818572315,1.3970964573396714,1.9000581057493937,2.0042778546146782

```

### .p29_r1_fixed_source_feedback/feedback_timeline.csv
```
event,identity,sensor_stamp_s,replay_relative_s,evaluation_relative_s,translation_delta_m,rotation_delta_deg,reference_point,lidar_translation_delta_m
FIRST_MSCKF_MEASUREMENT_TIME_NOT_RELEASE,1517157219654659032,1517157219.654659,0.465680122,-4.534319877624512,0.0,1.3321125036452401,,
FIRST_MSCKF_RELEASE_STATE_TIME,1517157219654659032,1517157219.809216,0.6202371120452881,-4.379762887954712,0.0,1.3321125036452401,,
FIRST_HIGH_RATE_STATE_DIFFERENCE,125,1517157219.809216,0.6202371120452881,-4.379762887954712,0.00271514816457544,1.3321125036452395,,
FIRST_DESKEW_DIFFERENCE_SCAN_REFERENCE_NOT_EXECUTION,7,1517157219.79414,0.605161190032959,-4.394838809967041,5.092158879110853e-05,,,
FIRST_INITIAL_DIFFERENCE,8,1517157219.895,0.7060210704803467,-4.293978929519653,0.012073523762787324,0.14327302521044846,IMU_origin,0.01194268905623078
FIRST_RAW_DIFFERENCE,7,1517157219.79414,0.605161190032959,-4.394838809967041,0.006110443246214831,0.14327298695805,IMU_origin,0.005970160681774045
FIRST_RAW_GE_0_1M,193,1517157238.552943,19.363964080810547,14.363964080810547,0.128736897478747,0.3102512933139131,IMU_origin,0.1287255558910394

```

### .p29_r1_fixed_source_feedback/deskew_feedback_summary.csv
```
frame_index,scan_ns,raw_point_count,correspondence,point_delta_rms_m,point_delta_p95_m,point_delta_max_m,changed_xyz_points,A_deskew_displacement_mean_m,B_deskew_displacement_mean_m,visual_image_times_within_scan,visual_updates_released_during_scan,within_scan_images_released_by_deskew_horizon
7,1517157219794140100,28874,EXACT_ORDER_AND_POINT_TIMES,0.00018026553389596879,0.0002965847350188987,0.00031253647603021663,28865,0.09409900551825956,0.0941499270618968,1517157219858260648,1517157219654659032,
193,1517157238552942991,29094,EXACT_ORDER_AND_POINT_TIMES,0.020040834733771714,0.03295908060213183,0.034681801771669185,29093,0.1536953417042516,0.1710073791568102,1517157238636635155,1517157238432892571,
270,1517157246318675041,28994,EXACT_ORDER_AND_POINT_TIMES,0.023111754647051574,0.03810136011340613,0.04008729597226047,28993,0.15676753609713592,0.17672903292658781,1517157246417431316,1517157246213997251,
320,1517157251361368895,29075,EXACT_ORDER_AND_POINT_TIMES,0.014191177996884944,0.02336235984751328,0.024604882215429343,29074,0.17782010810324214,0.16567945114470142,1517157251429501671,1517157251224841793,
340,1517157253378448009,29040,EXACT_ORDER_AND_POINT_TIMES,0.0025074480768528156,0.004130193970566772,0.004346182220144999,29039,0.15505587806620313,0.15301261464986135,1517157253468053841,1517157253264358001,
347,1517157254084407091,29001,EXACT_ORDER_AND_POINT_TIMES,0.0043514657014961655,0.0071666486330641595,0.007542712354658534,29000,0.1756243142045552,0.17226715952770807,1517157254160622809,1517157253956922385,
365,1517157255899825096,28963,EXACT_ORDER_AND_POINT_TIMES,0.15679001975661355,0.25866198531206913,0.27222718432687615,28962,0.1966650720312983,0.22072404135624948,,,
396,1517157259026242018,29030,EXACT_ORDER_AND_POINT_TIMES,0.09983815248661643,0.16427336765348582,0.1728493521273076,29029,0.1732110321282628,0.2567265805993351,,1517157258886444472,

```

### .p29_r1_fixed_source_feedback/rpe_summary.csv
```
method,nominal_delta_sec,actual_delta_min_sec,actual_delta_max_sec,count,translation_rmse_m,translation_p95_m,translation_max_m,rotation_rmse_deg
A,1.0,1.004863977432251,1.004960060119629,6192,0.45618793812785935,1.0264137717125634,1.9682441731319977,2.9215305742556543
B,1.0,1.004863977432251,1.004960060119629,6192,0.8934845299346678,2.590852611635792,2.9306170403967435,3.0816458893390943

```
