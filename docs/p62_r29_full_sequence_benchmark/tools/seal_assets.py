#!/usr/bin/env python3
from pathlib import Path
import hashlib,json
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore,Stores
P=Path(__file__).resolve().parents[1];D=Path('/home/jian/rosbag/public_datasets/koide_p62_r29');records=[];definitions=[];store=get_typestore(Stores.ROS2_HUMBLE)
for seq in ['indoor_easy_01','indoor_hard_01']:
 metadata=list((D/seq).rglob('metadata.yaml'));assert len(metadata)==1
 for f in sorted(metadata[0].parent.glob('*')):
  if not f.is_file():continue
  h=hashlib.sha256()
  with f.open('rb') as stream:
   for b in iter(lambda:stream.read(1024*1024),b''):h.update(b)
  records.append({'path':str(f),'bytes':f.stat().st_size,'sha256':h.hexdigest()})
 with Reader(metadata[0].parent) as r:
  c=next(x for x in r.connections if x.topic=='/imu');connection,bag_stamp,data=next(r.messages(connections=[c]));m=store.deserialize_cdr(data,c.msgtype)
  definitions.append({'sequence':seq,'topic':c.topic,'type':c.msgtype,'original_frame_id':m.header.frame_id,'stamp_ns':int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec),'bag_stamp_ns':int(bag_stamp),'angular_velocity':[getattr(m.angular_velocity,k) for k in 'xyz'],'linear_acceleration':[getattr(m.linear_acceleration,k) for k in 'xyz'],'units':'ROS sensor_msgs/Imu SI: rad/s and m/s^2; no numeric scaling applied','runtime_alias':'benchmark_imu; axes unmodified'})
(P/'sources/EXTRACTED_BAG_IDENTITIES.json').write_text(json.dumps(records,indent=2)+'\n');(P/'sources/IMU_MESSAGE_DEFINITIONS.json').write_text(json.dumps(definitions,indent=2)+'\n')
print('Expanded ROS2 bags SHA sealed once; original IMU frames retained.')
