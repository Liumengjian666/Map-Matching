#!/usr/bin/env python3
"""Lossless finite-XYZ scan bridge. No filtering, rotation, deskew or IMU use."""
import csv,hashlib,io,os,time
from pathlib import Path
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2,PointField
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped
rospy.init_node('p63_livox_bridge')
directory=Path(rospy.get_param('~output_dir'));f=(directory/'ADAPTER.csv').open('x',newline='')
w=csv.writer(f,lineterminator='\n');w.writerow(['stamp_ns','scan_end_ns','points','raw_message_sha256','xyz_sha256','bridge_ms'])
pub=rospy.Publisher('/p63/livox_points',PointCloud2,queue_size=1)
pathpub=rospy.Publisher('/p63/path',RosPath,queue_size=1,latch=True);path=RosPath();path.header.frame_id='map'
def cloud(m):
 begin=time.perf_counter();n=len(m.points);a=np.empty((n,4),dtype='<f4')
 a[:,:3]=[(p.x,p.y,p.z) for p in m.points];a[:,3]=[p.reflectivity for p in m.points]
 # No implicit loss of invalid points or tag-based subsampling.
 out=PointCloud2();out.header=m.header;out.height=1;out.width=n;out.is_bigendian=False;out.is_dense=bool(np.isfinite(a[:,:3]).all());out.point_step=16;out.row_step=16*n
 out.fields=[PointField(k,4*i,PointField.FLOAT32,1) for i,k in enumerate(('x','y','z','intensity'))];out.data=a.tobytes()
 raw=io.BytesIO();m.serialize(raw);digest=hashlib.sha256(a[:,:3].copy().tobytes()).hexdigest()
 w.writerow([m.header.stamp.to_nsec(),m.header.stamp.to_nsec()+max((p.offset_time for p in m.points),default=0),n,hashlib.sha256(raw.getvalue()).hexdigest(),digest,(time.perf_counter()-begin)*1000]);f.flush();pub.publish(out)
def pose(m):
 p=PoseStamped();p.header=m.header;p.pose=m.pose.pose;path.header=m.header;path.poses.append(p);pathpub.publish(path)
rospy.Subscriber('/livox/lidar',CustomMsg,cloud,queue_size=1,buff_size=16777216)
rospy.Subscriber('/odom',Odometry,pose,queue_size=100)
rospy.spin();f.close()
