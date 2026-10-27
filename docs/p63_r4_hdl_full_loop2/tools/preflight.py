#!/usr/bin/env python3
"""Fresh full-bag and all-LiDAR identity verification; no pose/reference evaluation."""
import csv,hashlib,json,os,struct,subprocess,time
from pathlib import Path
import numpy as np,rosbag
A=Path(__file__).resolve().parents[1];W=A.parents[1];B=Path('/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag');M=Path('/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd')
t0=time.monotonic();st=B.stat();h=hashlib.sha256();done=0
with B.open('rb') as f:
 for block in iter(lambda:f.read(8*1024**2),b''):
  h.update(block);done+=len(block)
  if done//(2*1024**3)!=(done-len(block))//(2*1024**3):print('bag hash bytes',done,flush=True)
bagsha=h.hexdigest();assert bagsha=='d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae'
r3=json.loads((W/'docs/p63_r3_hdl_ndt_candidate/WINDOW_IDENTITY.json').read_text());old=json.loads((W/'docs/p63_r3_hdl_ndt_candidate/BUILD_AND_SOURCE_FREEZE.json').read_text())
for p,v in old['binaries'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==v['sha256']
for name,v in old['upstream'].items():
 p=Path('/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/src')/name
 assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=p,text=True).strip()==v['SHA']
 assert not subprocess.check_output(['git','--no-optional-locks','status','--porcelain'],cwd=p,text=True)
assert hashlib.sha256(M.read_bytes()).hexdigest()==r3['map_sha256']
dtype=np.dtype({'names':['offset','x','y','z','reflectivity','tag','line'],'formats':['<u4','<f4','<f4','<f4','u1','u1','u1'],'offsets':[0,4,8,12,16,17,18],'itemsize':19})
count=0;first=last=None;end=None;previous=None;frame_ids=set();points_min=10**9;points_max=0;prefix_ok=True
out=A/'INPUT_FRAMES.csv'
with rosbag.Bag(str(B),'r') as bag,out.open('w',newline='') as f:
 info=bag.get_type_and_topic_info();indexed_count=bag.get_message_count(topic_filters=['/livox/lidar']);bag_start_ns=int(round(bag.get_start_time()*1e9));bag_end_ns=int(round(bag.get_end_time()*1e9))
 writer=csv.writer(f,lineterminator='\n');writer.writerow(['ordinal_1based','stamp_ns','scan_end_ns','record_ns','original_seq','points','raw_sha256','transport_seq_sha256','xyz_sha256'])
 for topic,raw,record in bag.read_messages(topics=['/livox/lidar'],raw=True):
  if len(raw)==5:typ,payload,md5,position,cls=raw
  else:typ,(payload,md5,position),cls=raw
  assert typ=='livox_ros_driver2/CustomMsg' and md5=='e4d6829bdfe657cb6c21a746c86b21a6'
  seq,sec,nsec,length=struct.unpack_from('<IIII',payload,0);assert nsec<10**9
  ns=sec*10**9+nsec;frame=payload[16:16+length].decode();base=16+length
  timebase,point_num=struct.unpack_from('<QI',payload,base);array_num=struct.unpack_from('<I',payload,base+16)[0]
  assert point_num==array_num and point_num>0 and len(payload)==base+20+19*point_num
  pts=np.frombuffer(payload,dtype=dtype,count=point_num,offset=base+20);xyz=np.stack([pts[k] for k in ('x','y','z')],axis=1)
  assert np.isfinite(xyz).all() and frame=='livox_frame'
  assert previous is None or ns>previous;previous=ns;frame_ids.add(frame);count+=1
  first=ns if first is None else first;last=ns;end=ns+int(pts['offset'].max())
  rawsha=hashlib.sha256(payload).hexdigest();xyzsha=hashlib.sha256(xyz.tobytes()).hexdigest();transport=hashlib.sha256();transport.update(struct.pack('<I',count));transport.update(memoryview(payload)[4:])
  writer.writerow([count,ns,end,record.to_nsec(),seq,point_num,rawsha,transport.hexdigest(),xyzsha]);points_min=min(points_min,point_num);points_max=max(points_max,point_num)
  if count<=400:
   oldframe=r3['frames'][count-1];assert ns==oldframe['stamp_ns'] and rawsha==oldframe['raw_message_sha256'] and xyzsha==oldframe['xyz_sha256']
  if count%1000==0:print('verified LiDAR frames',count,flush=True)
 assert count==indexed_count==10905
 f.flush()  # The manifest must be complete before its on-disk digest is recorded.
 after=B.stat();assert (st.st_size,st.st_mtime_ns)==(after.st_size,after.st_mtime_ns)
 receipt={'status':'PASS_FRESH_ALL_LIDAR_PREFLIGHT','bag':str(B),'fresh_bag_sha256':bagsha,'bag_bytes':st.st_size,'bag_mtime_ns':st.st_mtime_ns,'indexed_lidar_count':indexed_count,'verified_lidar_count':count,'type':info.topics['/livox/lidar'].msg_type,'MD5':'e4d6829bdfe657cb6c21a746c86b21a6','current_bag_record_start_ns_rounded_from_rosbag_float':bag_start_ns,'current_bag_record_end_ns_rounded_from_rosbag_float':bag_end_ns,'first_lidar_header_ns':first,'last_lidar_header_ns':last,'last_lidar_max_offset_end_ns':end,'sensor_span_s':(end-first)/1e9,'frame_ids':sorted(frame_ids),'points_min':points_min,'points_max':points_max,'first400_exactly_match_R3':True,'map':str(M),'map_sha256':r3['map_sha256'],'source_R3_CODE_SHA':'eb6ff37ab791193ff1a803952d79e148b9ecea6c','four_upstream_pristine_and_original_binaries_match':True,'observation_origin':'first LiDAR header.stamp,not bag record start','observation_interval_rel_s':[600,650],'frames_manifest_sha256':hashlib.sha256(out.read_bytes()).hexdigest(),'preflight_wall_s':time.monotonic()-t0,'GT_or_old_trajectory_read':False}
 (A/'INPUT_IDENTITY.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(receipt,indent=2),flush=True)
