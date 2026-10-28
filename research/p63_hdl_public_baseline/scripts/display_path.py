#!/usr/bin/python3
"""Read-only native /odom -> RViz path. No estimator commands or extra TF."""
import rospy
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped
rospy.init_node('p63_r5_path')
path=Path();pub=rospy.Publisher('/p63/path',Path,queue_size=1,latch=True)
def callback(message):
    path.header=message.header
    point=PoseStamped();point.header=message.header;point.pose=message.pose.pose
    path.poses.append(point);pub.publish(path)
rospy.Subscriber('/odom',Odometry,callback,queue_size=100)
rospy.spin()
