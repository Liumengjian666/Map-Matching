#!/usr/bin/env python3
"""Extract a preselected real 100-scan prefix and causal recorded initial guesses."""
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
import rosbag

DOC=Path(__file__).resolve().parent
OUT=Path('/home/jian/livox_ws/p62_r6_dog_shadow_runs/input')
STABLE=Path('/home/jian/livox_ws/dog_visual_loc_ws')
BAG=Path('/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag')
MAP=Path('/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd')
INITIALS=Path('/home/jian/rosbag/loop2/final_delivery_run_A_20260922/ndt_determinism.csv')


def identity(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
    return {'path':str(path),'bytes':path.stat().st_size,'sha256':digest.hexdigest()}


def main():
    OUT.mkdir(parents=True,exist_ok=False)
    pkg=STABLE/'src/dog_prior_map_localization'
    protected=[p for root in ['src','include','config','launch'] for p in (pkg/root).rglob('*')
               if p.is_file() and p.suffix in ('.cpp','.hpp','.h','.yaml','.launch')]
    stable={'head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=STABLE,text=True).strip(),
            'status':subprocess.check_output(['git','status','--porcelain=v1'],cwd=STABLE,text=True),
            'files':[identity(p) for p in sorted(protected)]}
    with (DOC/'stable_before.json').open('x') as handle:json.dump(stable,handle,indent=2)
    with INITIALS.open() as stream:
        reader=csv.DictReader(stream)
        initials=[next(reader) for _ in range(100)]
    scans=[];imus=[]
    with rosbag.Bag(str(BAG),'r') as bag:
        for topic,msg,record in bag.read_messages(topics=['/livox/lidar','/livox/imu']):
            if topic=='/livox/imu':
                imus.append([msg.header.stamp.to_nsec(),record.to_nsec(),msg.header.frame_id,
                    msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,
                    msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
                continue
            index=len(scans)+1
            old=initials[index-1]
            header=msg.header.stamp.to_nsec()
            if int(old['frame_index'])!=index or int(old['cloud_seq'])!=msg.header.seq:
                raise RuntimeError('Recorded initial/source sequence mismatch')
            if abs(float(old['lidar_header_stamp'])-header/1e9)>5.12e-7:
                raise RuntimeError('Recorded header differs beyond float representation tolerance')
            raw=np.array([(p.x,p.y,p.z,p.offset_time) for p in msg.points],
                dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('offset_ns','<u4')])
            if msg.point_num!=len(raw) or msg.header.frame_id!='livox_frame' or not len(raw):
                raise RuntimeError('Unexpected real LiDAR contract')
            low=int(raw['offset_ns'].min());high=int(raw['offset_ns'].max())
            path=OUT/('scan_%03d.xyz_offset.bin'%index)
            with path.open('xb') as handle:handle.write(raw.tobytes())
            pose=[float(old['initial_guess_'+k]) for k in ['tx','ty','tz','qx','qy','qz','qw']]
            if not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:])-1)>1e-5:
                raise RuntimeError('Invalid frozen common prediction')
            scans.append({'scan_index':index,'seq':msg.header.seq,'header_ns':header,
                'point_start_ns':header+low,'point_end_ns':header+high,'reference_ns':header+round(high/2),
                'raw_points':len(raw),'timebase':msg.timebase,'frame_id':msg.header.frame_id,
                'raw_serialized_sha256':hashlib.sha256(msg._buff if hasattr(msg,'_buff') else raw.tobytes()).hexdigest(),
                'serialized_hash_method':'xyz_f32_and_raw_offset_u32_only_not_full_ros_message',
                'raw_xyz_offset_file':str(path),'raw_sha256':identity(path)['sha256'],
                'expected_processed_fnv':old['cloud_hash'],'expected_processed_points':int(old['cloud_size_after_filter']),
                'initial_source':old['prediction_source'],'local_imu_prior_used':old['local_imu_prior_used'],
                **dict(zip(['tx','ty','tz','qx','qy','qz','qw'],pose))})
            if index==100:break
    if len(scans)!=100:raise RuntimeError('Real 100-scan prefix missing')
    if not all(imus[j][0]>imus[j-1][0] for j in range(1,len(imus))):
        raise RuntimeError('Nonmonotonic real IMU')
    if not all(r[2]=='livox_frame' and np.isfinite(r[3:]).all() for r in imus):
        raise RuntimeError('Invalid IMU fields')
    with (OUT/'scans.csv').open('x') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(scans[0]));writer.writeheader();writer.writerows(scans)
    with (OUT/'imu.csv').open('x') as handle:
        writer=csv.writer(handle);writer.writerow(['stamp_ns','record_ns','frame_id','wx_rad_s','wy_rad_s','wz_rad_s','ax_m_s2','ay_m_s2','az_m_s2']);writer.writerows(imus)
    manifest={'task':'P62-R6','selection':'First100 raw LiDAR messages, fixed before optimization; no reselection',
        'bag':identity(BAG),'map':identity(MAP),'recorded_causal_initials':identity(INITIALS),
        'raw_scans':scans,'scans_csv':identity(OUT/'scans.csv'),'imu_csv':identity(OUT/'imu.csv'),
        'imu_samples':len(imus),'topic':'/livox/lidar','type':'livox_ros_driver2/CustomMsg',
        'time':'header_ns + uint32 offset_time_ns; timebase recorded, not substituted',
        'deskew':False,'reference':'original delivery midpoint','motion_evidence_beta':0,
        'first_scan_imu_start_bracket':False,'first_initial':'production nonGT current_pose; not claimed as IMU-integrated',
        'map_independence':'Existing loop2 FAST-LIVO2 map; same-record engineering dependence, not independent prior-map validation',
        'source_only_fields_read_from_old_log':'initial_guess, sequence, time, processed_count/hash; not old Raw/Used/reference errors',
        'GT_read':False,'stable_snapshot':'stable_before.json'}
    with (DOC/'REAL_INPUT_MANIFEST.json').open('x') as handle:json.dump(manifest,handle,indent=2);handle.write('\n')
    print('FROZEN real_scans=100 imu_samples=%d'%len(imus),flush=True)


if __name__=='__main__':main()
