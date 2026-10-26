#!/usr/bin/env python3
"""Visualize sealed results only. No localizer nodelet, no align or filter updates."""
from pathlib import Path
import json,os,subprocess,signal,time
import numpy as np,rosbag,rospy,psutil
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import Path as RosPath,Odometry
from geometry_msgs.msg import PoseStamped,TransformStamped
from tf.transformations import quaternion_matrix
from tf2_ros import StaticTransformBroadcaster
A=Path(__file__).resolve().parents[1];D=Path('/home/jian/livox_ws/p63_r3_hdl_data/frozen_preview_01');D.mkdir(exist_ok=False)
r=json.loads(Path('/home/jian/livox_ws/p63_r3_hdl_data/trial_01/RUN_RECEIPT.json').read_text());n=json.loads((A/'evidence/MEASURED_SUMMARY.json').read_text());identity=json.loads((A/'WINDOW_IDENTITY.json').read_text())
xml='''<launch><node pkg="nodelet" type="nodelet" name="preview_manager" args="manager"/><node pkg="nodelet" type="nodelet" name="preview_globalmap" args="load hdl_localization/GlobalmapServerNodelet preview_manager"><param name="globalmap_pcd" value="'''+identity['map']+'''"/><param name="convert_utm_to_local" value="false"/><param name="downsample_resolution" value="0.1"/><remap from="/map_request/pcd" to="/preview_disabled/map_request"/></node><node pkg="rviz" type="rviz" name="recorded_output_preview" args="-d /home/jian/livox_ws/p63_r3_hdl_prior_map_ws/research/p63_hdl_candidate/rviz/hdl_prior.rviz"><env name="DISABLE_ROS1_EOL_WARNINGS" value="1"/></node></launch>'''
p=D/'frozen_preview.launch';p.write_text(xml)
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11634',ROS_IP='127.0.0.1');env=os.environ.copy();env.update(ROS_HOME=str(D/'ros_home'),ROS_LOG_DIR=str(D/'ros_logs'),LIBGL_ALWAYS_SOFTWARE='1')
f=(D/'ROSLAUNCH.log').open('x');proc=subprocess.Popen(['roslaunch',str(p)],env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
try:
 time.sleep(2);rospy.init_node('sealed_output_renderer',disable_signals=True)
 from nodelet.srv import NodeletList
 rospy.wait_for_service('/preview_manager/list',timeout=15);nodes=rospy.ServiceProxy('/preview_manager/list',NodeletList)().nodelets;assert nodes==['/preview_globalmap']
 stamp=rospy.Time(r['records'][-1]['stamp_ns']//10**9,r['records'][-1]['stamp_ns']%10**9)
 path=RosPath();path.header.stamp=stamp;path.header.frame_id='map'
 for row in r['records']:
  x=row['output_pose_xyzw'];p=PoseStamped();p.header.frame_id='map';p.header.stamp=rospy.Time(row['stamp_ns']//10**9,row['stamp_ns']%10**9)
  p.pose.position.x,p.pose.position.y,p.pose.position.z=x[:3];p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=x[3:];path.poses.append(p)
 pubpath=rospy.Publisher('/p63/path',RosPath,queue_size=1,latch=True);pubodom=rospy.Publisher('/odom',Odometry,queue_size=1,latch=True);pubcloud=rospy.Publisher('/aligned_points',PointCloud2,queue_size=1,latch=True)
 o=Odometry();o.header=path.header;o.child_frame_id='livox_frame';o.pose.pose=path.poses[-1].pose
 with rosbag.Bag(identity['bag'],'r') as b:
  for i,(_,m,_) in enumerate(b.read_messages(topics=['/livox/lidar'])):
   if i==399:break
 xyz=np.array([(p.x,p.y,p.z) for p in m.points],dtype=np.float32);rawpose=r['records'][-1]['raw_ndt_pose_xyzw'];R=quaternion_matrix(rawpose[3:])[:3,:3];world=xyz@R.T+np.array(rawpose[:3]);data=np.empty((len(xyz),4),'<f4');data[:,:3]=world;data[:,3]=[p.reflectivity for p in m.points]
 cloud=PointCloud2();cloud.header=path.header;cloud.height=1;cloud.width=len(xyz);cloud.point_step=16;cloud.row_step=16*len(xyz);cloud.is_dense=True;cloud.fields=[PointField(k,4*i,PointField.FLOAT32,1) for i,k in enumerate(('x','y','z','intensity'))];cloud.data=data.tobytes()
 tf=TransformStamped();tf.header=path.header;tf.child_frame_id='livox_frame';tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=r['records'][-1]['output_pose_xyzw'][:3];tf.transform.rotation=path.poses[-1].pose.orientation
 broadcaster=StaticTransformBroadcaster();broadcaster.sendTransform(tf)
 pubpath.publish(path);pubodom.publish(o);pubcloud.publish(cloud);time.sleep(4)
 rv=next(p for p in psutil.Process(proc.pid).children(recursive=True) if p.name()=='rviz')
 cap='/home/jian/livox_ws/p63_r1_super_lio_baseline_ws/docs/p63_r1_mature_prior_map_baseline/tools/capture_rviz.py'
 subprocess.run(['/usr/bin/python3',cap,'--pid',str(rv.pid),'--output',str(A/'FROZEN_OUTPUT_PREVIEW.png')],check=True,timeout=10)
 (A/'evidence/PREVIEW_RECEIPT.json').write_text(json.dumps({'type':'SEALED_OUTPUT_VISUALIZATION_ONLY','localizer_nodelets':nodes,'new_NDT_calls':0,'new_UKF_updates':0,'trajectory_poses':400,'scan_ordinal':400,'transform_source':'sealed final raw NDT pose;not a new calibration','GUI_only_fix':'DISABLE_ROS1_EOL_WARNINGS=1','original_live_capture':'Noetic modal warning,not a map view','preview_screenshot':'FROZEN_OUTPUT_PREVIEW.png'},indent=2)+'\n')
finally:
 os.killpg(proc.pid,signal.SIGINT)
 try:proc.wait(timeout=10)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
 f.close()
