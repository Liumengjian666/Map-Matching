"""Read actual IMU CDR and model native timestamps; no ROS, NDT, UKF or test GT."""
import csv, hashlib, json, subprocess, shutil
from pathlib import Path
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from imu_geometry import R_L_I, P_L_I, convert
A=Path(__file__).resolve().parents[1];W=A.parents[1]
R8=W/'docs/p63_r8_hdl_indoor_hard_holdout'
ROOT=Path('/home/jian/rosbag/public_datasets/koide_p62_r29')
HDL=Path('/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/src/hdl_localization')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(name,x): (A/name).write_text(json.dumps(x,indent=2)+'\n')
def csvout(name,rows):
    with (A/name).open('w',newline='') as s:
        wr=csv.DictWriter(s,fieldnames=list(rows[0]));wr.writeheader();wr.writerows(rows)
F=json.loads((R8/'INPUT_FREEZE.json').read_text());B=json.loads((R8/'BASELINE_BUILD.json').read_text())
assert shutil.disk_usage(W).free>=20*1024**3
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=HDL,text=True).strip()=='de2dc7769aa2412876d05ae6161408d3295b6a3c'
assert not subprocess.check_output(['git','status','--porcelain'],cwd=HDL)
for p,v in B['native_binaries'].items():assert sha(p)==v['sha256']
assert sha(F['map_path'])==F['map_sha256']
assert F['target_points']==93223 and F['target_xyz_sha256']=='166c4d2a4f5d24a4c79d4c90a8a275e0fe905fcc9650d559aabb775780e81654'
for f in F['frames']:assert sha(f['path'])==f['sha256']
manifest=ROOT/'indoor_hard_01_adapter/manifest.json';M=json.loads(manifest.read_text());imu_path=manifest.parent/'imu.csv'
rows=list(csv.DictReader(imu_path.open()));times=np.array([int(r['sec'])*10**9+int(r['nsec']) for r in rows],dtype=np.int64)
assert len(times)==44398 and np.all(np.diff(times)>0)
anchor=F['anchor_stamp_ns'];end=F['frames'][-1]['stamp_ns'];begin=int(np.searchsorted(times,anchor,side='right'));stop=int(np.searchsorted(times,end,side='right'))
assert begin>0
store=get_typestore(Stores.ROS2_HUMBLE);cdr_check=[]
with Reader(Path(M['source_metadata']).parent) as reader:
    c=next(c for c in reader.connections if c.topic=='/imu');assert c.msgcount==44398
    for idx,(_,bag_ns,cdr) in enumerate(reader.messages(connections=[c])):
        if idx>=stop:break
        msg=store.deserialize_cdr(cdr,c.msgtype);stamp=int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)
        vals=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
        assert stamp==times[idx] and msg.header.frame_id=='imu_link'
        assert np.array_equal(vals,[float(rows[idx][k]) for k in ['gx','gy','gz','ax','ay','az']]) and np.isfinite(vals).all()
        cdr_check.append({'id':idx,'stamp_ns':stamp,'bag_stamp_ns':int(bag_ns),'cdr_sha256':hashlib.sha256(cdr).hexdigest(),**dict(zip(['gx','gy','gz','ax','ay','az'],vals))})
csvout('evidence/RAW_IMU_VERIFIED.csv',cdr_check)
adapted=[]
for i in range(begin,stop):
    force=np.array([float(rows[i][k]) for k in ['ax','ay','az']]);gyro=np.array([float(rows[i][k]) for k in ['gx','gy','gz']]);old=np.array([float(rows[i-1][k]) for k in ['gx','gy','gz']]);dt=(int(times[i])-int(times[i-1]))/1e9
    out,w,lever=convert(force,gyro,old,dt)
    adapted.append({'id':i,'stamp_ns':int(times[i]),'previous_stamp_ns':int(times[i-1]),'dt_s':dt,**dict(zip(['ax','ay','az'],out)),**dict(zip(['gx','gy','gz'],w)),**dict(zip(['lever_ax','lever_ay','lever_az'],lever))})
csvout('evidence/CAUSAL_TRANSFORM_PREVIEW.csv',adapted)
# Structural clock analysis only: never call the real estimator. First predict
# sets init_stamp, cool time updates prev_stamp without propagation; correct()
# leaves prev_stamp unchanged. Do not equate this replay with actual consumption.
prev=None;init=None;pointer=begin;last_scan=anchor;timeline=[]
for f in F['frames']:
    s=f['stamp_ns'];batch=[];integrated=[];overlap=0
    while pointer<stop and int(times[pointer])<=s:
        ts=int(times[pointer]);batch.append(ts)
        if init is None:init=ts
        active=prev is not None and ts-init>=2*10**9 and ts!=prev
        if active:
            integrated.append((ts-prev)/1e9)
            overlap+=max(0,min(ts,last_scan)-prev)
        prev=ts;pointer+=1
    timeline.append({'raw_ordinal':f['raw_ordinal'],'scan_stamp_ns':s,'prior_correction_stamp_ns':last_scan,'scheduled_imu_calls':len(batch),'model_active_predict_calls':len(integrated),'last_imu_stamp_ns':prev,'scan_minus_last_imu_ns':s-prev,'model_integrated_s':sum(integrated),'model_post_correction_overlap_ns':overlap,'batch_max_dt_s':max([(batch[i]-batch[i-1])/1e9 for i in range(1,len(batch))],default=0)})
    last_scan=s
assert pointer==stop
csvout('evidence/NATIVE_TIME_PREFLIGHT.csv',timeline)
stat=lambda x:dict(zip(['min','mean','p95','max'],[float(np.min(x)),float(np.mean(x)),float(np.percentile(x,95)),float(np.max(x))]))
used=times[begin:stop];dt=np.diff(times[begin-1:stop])/1e9
force=np.array([[float(rows[i][k]) for k in ['ax','ay','az']] for i in range(begin,stop)])
gyro=np.array([[float(rows[i][k]) for k in ['gx','gy','gz']] for i in range(begin,stop)])
gaps=[{'previous_stamp_ns':int(times[i-1]),'stamp_ns':int(times[i]),'dt_s':(int(times[i])-int(times[i-1]))/1e9} for i in range(begin,stop) if times[i]-times[i-1]>50_000_000]
files=[HDL/'apps/hdl_localization_nodelet.cpp',HDL/'src/hdl_localization/pose_estimator.cpp',HDL/'include/hdl_localization/pose_estimator.hpp',HDL/'include/hdl_localization/pose_system.hpp',W/'docs/p62_r29_full_sequence_benchmark/sources/zenodo_record.json',W/'docs/p62_r29_full_sequence_benchmark/sources/glim_config_sensors.jsonc',Path('/opt/ros/noetic/share/sensor_msgs/msg/Imu.msg'),imu_path,manifest,R8/'INPUT_FREEZE.json']
anchor_force=np.array(F['T_map_lidar_anchor'])[:3,:3]@R_L_I@np.array([float(rows[begin-1][k]) for k in ['ax','ay','az']])
result={'stage':'ZERO_NDT_INPUT_PREFLIGHT','native_align_calls':0,'actual_ukf_predict_calls':0,'test_reference_read':False,'source_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),'raw_imu_total':len(rows),'cdr_verified_count':len(cdr_check),'scheduled_imu_count':stop-begin,'anchor_stamp_ns':anchor,'first_scheduled_imu_stamp_ns':int(used[0]),'last_scheduled_imu_stamp_ns':int(used[-1]),'last_scan_stamp_ns':end,'imu_dt_s':stat(dt),'gaps_over_50ms':gaps,'raw_acc_norm_m_s2':stat(np.linalg.norm(force,axis=1)),'raw_gyro_norm_rad_s':stat(np.linalg.norm(gyro,axis=1)),'scan_last_imu_age_ms':stat([x['scan_minus_last_imu_ns']/1e6 for x in timeline]),'zero_new_imu_raws':[x['raw_ordinal'] for x in timeline if not x['scheduled_imu_calls']],'zero_active_predict_raws':[x['raw_ordinal'] for x in timeline if not x['model_active_predict_calls']],'model_active_predict_calls':sum(x['model_active_predict_calls'] for x in timeline),'model_cooltime_or_first_skipped_calls':stop-begin-sum(x['model_active_predict_calls'] for x in timeline),'model_corrected_time_overlap_s':sum(x['model_post_correction_overlap_ns'] for x in timeline)/1e9,'anchor_only_world_force_m_s2':anchor_force.tolist(),'T_L_I_rotation':R_L_I.tolist(),'p_L_I':P_L_I.tolist(),'lever_arm_norm_m':float(np.linalg.norm(P_L_I)),'lever_correction_m_s2':stat([np.linalg.norm([x[k] for k in ['lever_ax','lever_ay','lever_az']]) for x in adapted]),'source_files_sha256':{str(p):sha(p) for p in files},'native_binaries':B['native_binaries'],'free_bytes':shutil.disk_usage(W).free,'timing_decision':'PENDING_USER_CLARIFICATION_NATIVE_APPROXIMATION_NOT_SILENTLY_ACCEPTED'}
save('PREFLIGHT.json',result)
print(json.dumps({k:result[k] for k in ['scheduled_imu_count','model_active_predict_calls','imu_dt_s','scan_last_imu_age_ms','zero_new_imu_raws','zero_active_predict_raws','model_corrected_time_overlap_s','lever_correction_m_s2']},indent=2))
