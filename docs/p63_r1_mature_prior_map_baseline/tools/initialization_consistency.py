#!/usr/bin/env python3
"""Bounded, read-only first10-scan initialization reconstruction. No new estimation/run."""
from pathlib import Path
import rosbag,json,numpy as np,re
P=Path(__file__).resolve().parents[1];D=Path('/home/jian/livox_ws/p63_r1_super_lio_data/loop2_smoke_01')
clouds=[];imu=[];end=None;total=0
with rosbag.Bag('/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag','r') as bag:
 for topic,m,t in bag.read_messages(topics=['/livox/lidar','/livox/imu']):
  if topic=='/livox/imu':imu.append((m.header.stamp.to_sec(),[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]))
  elif len(clouds)<10:
   kept=[x for x in m.points[::3] if (x.tag&0x30) in [0,0x10] and .3**2<x.x*x.x+x.y*x.y+x.z*x.z<100.**2]
   assert kept;end=m.header.stamp.to_sec()+kept[-1].offset_time*1e-9;clouds.append(m.header.stamp.to_nsec());total+=len(kept)
  if len(clouds)==10 and topic=='/livox/imu' and m.header.stamp.to_sec()>end:break
sample=[x for x in imu if x[0]<=end];acc=np.array([x[1] for x in sample]);gyro=np.array([x[2] for x in sample]);mean=acc.mean(axis=0)
text=re.sub(r'\x1b\[[0-9;]*m','',(D/'ROSLAUNCH.log').read_text());lines=text.splitlines();start=next(i for i,x in enumerate(lines) if 'super_lio_reloc.cpp:241]' in x);matrix=np.array([[float(v) for v in x.split()] for x in lines[start+1:start+5]])
world_specific_force=matrix[:3,:3]@mean;unit=world_specific_force/np.linalg.norm(world_specific_force);angle=float(np.degrees(np.arccos(np.clip(unit[2],-1,1))))
r={'scope':'read-only original initializer reconstruction;not independent calibration or GT','input_scans':len(clouds),'kept_initialization_points':total,'matches_actual_initializer_point_count':total==58407,'IMU_count':len(sample),'mean_raw_accel':mean.tolist(),'std_raw_accel':acc.std(axis=0).tolist(),'mean_gyro_radps':gyro.mean(axis=0).tolist(),'std_gyro_radps':gyro.std(axis=0).tolist(),'logged_ICP_T_map_imu_rounded':matrix.tolist(),'specific_force_in_map_direction':unit.tolist(),'angle_to_author_positive_map_Z_deg':angle,'predicted_stationary_acceleration_with_author_gravity_mps2':(unit*9.7946+np.array([0,0,-9.7946])).tolist(),'log_matrix_precision':'author stdout6-digit float,diagnostic only','conclusion':'large initialization/gravity disagreement under historical identity assumption;map axes/incorrect initialization branch/physical rotation not independently disambiguated;no transform fitted or applied'}
assert total==58407
(P/'evidence/INITIALIZATION_GRAVITY_DIAGNOSTIC.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
