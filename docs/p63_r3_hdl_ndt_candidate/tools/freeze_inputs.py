#!/usr/bin/env python3
"""Read only the fixed first400 scans, never inspect localization/reference errors."""
import csv,hashlib,io,json,struct,subprocess,sys,os
from pathlib import Path
import rosbag
A=Path(__file__).resolve().parents[1];W=A.parents[1]
B=Path('/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag');M=Path('/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd')
frames=[];imus=[];end=None
with rosbag.Bag(str(B),'r') as bag:
 for topic,m,t in bag.read_messages(topics=['/livox/lidar','/livox/imu']):
  ns=m.header.stamp.to_nsec()
  if topic=='/livox/imu':imus.append(ns)
  elif len(frames)<400:
   b=io.BytesIO();m.serialize(b)
   xyz=b''.join(struct.pack('<fff',p.x,p.y,p.z) for p in m.points)
   import math
   assert len(m.points)==m.point_num and all(math.isfinite(v) for p in m.points for v in (p.x,p.y,p.z))
   end=ns+max(p.offset_time for p in m.points)
   frames.append({'ordinal_1based':len(frames)+1,'stamp_ns':ns,'scan_end_max_offset_ns':end,'bag_ns':t.to_nsec(),'points':len(m.points),'raw_message_sha256':hashlib.sha256(b.getvalue()).hexdigest(),'xyz_sha256':hashlib.sha256(xyz).hexdigest(),'frame_id':m.header.frame_id})
  if len(frames)==400 and topic=='/livox/imu' and ns>end+20000000:break
assert len(frames)==400 and all(y['stamp_ns']>x['stamp_ns'] for x,y in zip(frames,frames[1:]))
r={'bag':str(B),'bag_bytes':B.stat().st_size,'archival_bag_sha256':'d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae','fresh_whole_bag_hash':False,'map':str(M),'map_sha256':hashlib.sha256(M.read_bytes()).hexdigest(),'map_bytes':M.stat().st_size,'original_map_points':459154,'first_scan_start_ns':frames[0]['stamp_ns'],'last_scan_end_ns':end,'IMU_in_interval_read_only_not_used':sum(frames[0]['stamp_ns']<=t<=end for t in imus),'frames':frames,'GT_read':False,'T_map_lidar_manual_initial':[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],'initial_source':'stable dog NDT source defaults p_=Zero,R_=Identity,lines1145-46;manual numerical map/LiDAR initialization,not physical IMU calibration','UTM_conversion':False,'IMU_prediction':False,'global_localization':False}
assert r['map_sha256']=='016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca'
(A/'WINDOW_IDENTITY.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps({k:v for k,v in r.items() if k!='frames'},indent=2))
