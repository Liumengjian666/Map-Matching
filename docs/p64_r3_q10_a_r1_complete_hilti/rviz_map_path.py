#!/usr/bin/python3
"""Load the frozen map and seed; publish the upstream odometry trajectory."""
from pathlib import Path
import sys
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as RosPath
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"p64_r3_q9_public_benchmark_fine_geometry"))
from stream_hdl import xyzmsg


def main():
    rospy.init_node("hilti_prior_map")
    publisher = rospy.Publisher("/globalmap",PointCloud2,queue_size=1,latch=True)
    anchor = rospy.Publisher("/initialpose",PoseWithCovarianceStamped,queue_size=1,latch=True)
    path_pub = rospy.Publisher("/hilti/trajectory",RosPath,queue_size=2,latch=True)
    trace=RosPath()
    trace.header.frame_id="map"

    def odometry(msg):
        state=PoseStamped(header=msg.header,pose=msg.pose.pose)
        trace.poses.append(state)
        trace.header.stamp=msg.header.stamp
        path_pub.publish(trace)

    sub=rospy.Subscriber("/odom",Odometry,odometry,queue_size=1500)
    publisher.publish(xyzmsg(Path(rospy.get_param("~map")),"map",0))
    # Use wall time so the map and known pose are delivered before /clock starts.
    deadline=time.monotonic()+40
    while not rospy.is_shutdown() and anchor.get_num_connections()==0:
        if time.monotonic()>deadline:
            raise TimeoutError("HDL initialpose subscription did not start")
        time.sleep(.05)
    seed=PoseWithCovarianceStamped()
    seed.header.frame_id="map"
    raw=np.loadtxt(rospy.get_param("~anchor"))
    if raw.shape!=(8,) or not np.isfinite(raw).all():
        raise ValueError("expected position + slot + xyzw")
    p,q=seed.pose.pose.position,seed.pose.pose.orientation
    p.x,p.y,p.z=raw[:3]
    q.x,q.y,q.z,q.w=raw[4:]
    anchor.publish(seed)
    rospy.loginfo("HILTI_REPLAY_READY")
    rospy.spin()
    sub.unregister()


if __name__=="__main__":
    main()
