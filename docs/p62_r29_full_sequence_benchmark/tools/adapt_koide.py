#!/usr/bin/env python3
"""Deterministic ROS2 CDR -> XYZ/t=0 PCD + unchanged SI IMU CSV. No test GT."""
import argparse,csv,hashlib,json,os,time
from pathlib import Path
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores,get_typestore
P=Path(__file__).resolve().parents[1]
D=Path('/home/jian/rosbag/public_datasets/koide_p62_r29')
a=argparse.ArgumentParser();a.add_argument('sequence',choices=['indoor_easy_01','indoor_hard_01']);args=a.parse_args()
seq=args.sequence;root=D/seq
bags=list(root.rglob('metadata.yaml'));assert len(bags)==1,bags
out=D/(seq+'_adapter');out.mkdir(exist_ok=False);scans=out/'scans';scans.mkdir()
store=get_typestore(Stores.ROS2_HUMBLE)
counts={};frames=[];imus=[];cdr_hash={};position_hash=hashlib.sha256();imu_hash=hashlib.sha256();begin=time.monotonic()
with Reader(bags[0].parent) as reader,(out/'imu.csv').open('x',newline='') as f:
 writer=csv.writer(f);writer.writerow(['#id','sec','nsec','gx','gy','gz','ax','ay','az'])
 connections=[c for c in reader.connections if c.topic in ['/points2/decompressed','/imu']]
 assert {c.topic for c in connections}=={'/points2/decompressed','/imu'}
 meta=[{'topic':c.topic,'type':c.msgtype,'count':c.msgcount} for c in connections]
 (out/'bag_topics.json').write_text(json.dumps(meta,indent=2)+'\n')
 for connection,bagtime,data in reader.messages(connections=connections):
  topic=connection.topic;counts[topic]=counts.get(topic,0)+1
  if topic not in cdr_hash:cdr_hash[topic]=hashlib.sha256()
  cdr_hash[topic].update(data)
  msg=store.deserialize_cdr(data,connection.msgtype);stamp=int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)
  assert stamp>0
  if topic=='/imu':
   vals=[getattr(msg.angular_velocity,k) for k in 'xyz']+[getattr(msg.linear_acceleration,k) for k in 'xyz']
   assert np.isfinite(vals).all() and (not imus or stamp>imus[-1][0])
   row=[len(imus),stamp//10**9,stamp%10**9]+vals
   text=[str(x) for x in row];assert np.array_equal(np.array([float(x) for x in text[3:]]),np.array(vals))
   writer.writerow(text);imu_hash.update(np.array([stamp],dtype='<i8').tobytes()+np.array(vals,dtype='<f8').tobytes())
   imus.append((stamp,vals));continue
  assert not msg.is_bigendian and msg.point_step>0
  fields={x.name:x for x in msg.fields}
  assert all(k in fields and fields[k].datatype==7 and fields[k].count==1 for k in ['x','y','z'])
  assert not any(k in fields for k in ['t','time','timestamp']), 'point time exists: review explicit source semantics'
  dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[fields[k].offset for k in ['x','y','z']],'itemsize':msg.point_step})
  original=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
  cloud=np.zeros(msg.width*msg.height,dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('t','<u4')])
  for k in ['x','y','z']:cloud[k]=original[k].reshape(-1)
  assert not frames or stamp>frames[-1]['stamp_ns']
  fn=scans/('cloud_%d_%09d.pcd'%(stamp//10**9,stamp%10**9))
  header=('VERSION .7\nFIELDS x y z t\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT %d\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'%(msg.width,msg.height,len(cloud))).encode()
  st=os.statvfs(out);assert st.f_bavail*st.f_frsize>20*1024**3+cloud.nbytes+len(header)
  payload=cloud.tobytes()
  with fn.open('xb') as file:file.write(header);file.write(payload)
  check=np.fromfile(fn,dtype=cloud.dtype,offset=len(header));assert len(check)==len(cloud)
  # Bitwise equality, including original NaN payloads; do not reestimate coordinates.
  for k in ['x','y','z']:assert check[k].tobytes()==original[k].reshape(-1).tobytes()
  position_hash.update(check[['x','y','z']].tobytes())
  sha=hashlib.sha256(header+payload).hexdigest()
  frames.append({'raw_ordinal':len(frames)+1,'stamp_ns':stamp,'bag_stamp_ns':int(bagtime),'path':str(fn),'sha256':sha,'width':int(msg.width),'height':int(msg.height),'points':len(cloud),'frame_id':msg.header.frame_id,'header_bytes':len(header),'xyz_bitwise_equal':True,'finite_xyz':int(np.isfinite(check['x']).sum())})
  if len(frames)==1:(out/'first_cloud_definition.json').write_text(json.dumps({'frame_id':msg.header.frame_id,'header_stamp_ns':stamp,'bag_stamp_ns':int(bagtime),'point_step':int(msg.point_step),'fields':[{'name':x.name,'offset':int(x.offset),'datatype':int(x.datatype),'count':int(x.count)} for x in msg.fields],'timing_assumption':'instantaneous depth snapshot at header stamp; zero offsets added, not original per-point time'},indent=2)+'\n')
  if len(frames)%250==0:print(seq,'converted',len(frames),flush=True)
 assert all(counts.get(c.topic,0)==c.msgcount for c in connections)
assert len(imus)>1 and len(frames)>1
# Verify the emitted CSV's parsed measurements and integer times, not only a writer hash.
with (out/'imu.csv').open() as f:
 for row,expected in zip(list(csv.reader(f))[1:],imus):
  assert int(row[1])*10**9+int(row[2])==expected[0]
  assert np.array_equal(np.array([float(x) for x in row[3:]]),expected[1])
manifest={'sequence':seq,'source_metadata':str(bags[0]),'selected_message_counts':counts,'xyz_bitwise_roundtrip':'PASS_ALL_POINTS_ALL_SCANS','imu_exact_roundtrip':'PASS_ALL_USED_VALUES','cdr_topic_sha256':{k:v.hexdigest() for k,v in cdr_hash.items()},'converted_xyz_sha256':position_hash.hexdigest(),'imu_value_sha256':imu_hash.hexdigest(),'imu_csv':str(out/'imu.csv'),'imu_count':len(imus),'imu_start_ns':imus[0][0],'imu_end_ns':imus[-1][0],'max_imu_gap_ns':max(b[0]-a[0] for a,b in zip(imus,imus[1:])),'frames':frames,'seconds':time.monotonic()-begin,'gt_pose_values_read':False}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(P/'logs'/('adapter_'+seq+'.json')).write_text(json.dumps({k:v for k,v in manifest.items() if k!='frames'},indent=2)+'\n')
print('ADAPTER_PASS',seq,len(frames),len(imus),manifest['max_imu_gap_ns'],flush=True)
