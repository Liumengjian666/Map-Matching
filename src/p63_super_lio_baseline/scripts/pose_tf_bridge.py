#!/usr/bin/env python3
"""Visualization only: publish world->IMU TF from the author's IMU-frame odometry."""
import rospy
import numpy as np
from tf.transformations import quaternion_from_matrix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
rospy.init_node('p63_pose_tf_bridge');broadcaster=TransformBroadcaster()
static=StaticTransformBroadcaster();ext=rospy.get_param('/lio/extrinsic/lidar_imu')
m=np.eye(4);m[:3,:3]=np.array(ext[3:]).reshape(3,3,order='F');q=quaternion_from_matrix(m)
t=TransformStamped();t.header.stamp=rospy.Time.now();t.header.frame_id='p63_imu';t.child_frame_id='livox_frame'
t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=ext[:3]
t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=q
static.sendTransform(t)
def received(msg):
 t=TransformStamped();t.header=msg.header;t.child_frame_id='p63_imu'
 t.transform.translation.x=msg.pose.pose.position.x;t.transform.translation.y=msg.pose.pose.position.y;t.transform.translation.z=msg.pose.pose.position.z
 t.transform.rotation=msg.pose.pose.orientation;broadcaster.sendTransform(t)
rospy.Subscriber('/lio/odom',Odometry,received,queue_size=100);rospy.spin()
