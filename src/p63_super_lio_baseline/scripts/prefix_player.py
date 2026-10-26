#!/usr/bin/env python3
"""Replay first400 original scans and genuine IMU in bag-record order; no pose input."""
import io,time,json,csv,hashlib,argparse,sys
from pathlib import Path
import rospy,rosbag
p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--output',required=True);a=p.parse_args()
rospy.init_node('p63_loop2_prefix_input');directory=Path(a.output)
ready=time.monotonic()+20
while not directory.is_dir():
 if rospy.is_shutdown() or time.monotonic()>ready:raise RuntimeError('qualified baseline output directory not ready')
 time.sleep(.05)
ledger=(directory/'INPUT_LEDGER.csv').open('x',newline='');w=csv.writer(ledger,lineterminator='\n');w.writerow(['topic','header_ns','bag_ns','bytes','sha256'])
counts={'/livox/lidar':0,'/livox/imu':0};publishers={};first_record=None;wall=None;end_ns=None
with rosbag.Bag(a.bag,'r') as b:
 for topic,msg,record in b.read_messages(topics=list(counts)):
  if rospy.is_shutdown():break
  if topic not in publishers:
   publishers[topic]=rospy.Publisher(topic,type(msg),queue_size=1000,tcp_nodelay=True)
  # Collect both topic message classes before starting, then rely on existing TCP queues.
  timeout=time.monotonic()+20
  while publishers[topic].get_num_connections()==0 and not rospy.is_shutdown():
   if time.monotonic()>timeout:raise RuntimeError('no original subscriber for '+topic)
   time.sleep(.01)
  if topic=='/livox/lidar' and counts[topic]>=400:continue
  if first_record is None:first_record=record.to_nsec();wall=time.monotonic()
  due=wall+(record.to_nsec()-first_record)/1e9
  while time.monotonic()<due and not rospy.is_shutdown():time.sleep(min(.01,due-time.monotonic()))
  if rospy.is_shutdown():break
  payload=io.BytesIO();msg.serialize(payload);raw=payload.getvalue()
  publishers[topic].publish(msg);counts[topic]+=1
  w.writerow([topic,msg.header.stamp.to_nsec(),record.to_nsec(),len(raw),hashlib.sha256(raw).hexdigest()]);ledger.flush()
  if topic=='/livox/lidar' and counts[topic]==400:end_ns=msg.header.stamp.to_nsec()+max(x.offset_time for x in msg.points)
  if end_ns is not None and topic=='/livox/imu' and msg.header.stamp.to_nsec()>end_ns+20000000:break
ledger.close();(directory/'INPUT_RECEIPT.json').write_text(json.dumps({'counts':counts,'last_scan_end_ns':end_ns,'wall_s':time.monotonic()-wall if wall else 0,'pose_messages_published':0,'timestamps_and_values':'original message fields, no shift or scaling'},indent=2)+'\n')
print('P63_INPUT_COMPLETE',counts,flush=True)
# Let the final TCP buffers drain without modifying source timestamps.
time.sleep(1)
