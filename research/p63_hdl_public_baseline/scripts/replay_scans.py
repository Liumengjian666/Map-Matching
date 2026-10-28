#!/usr/bin/python3
"""Manual future RViz replay, fixed raw5–164; same exact data adapter, no reference."""
import json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'docs/p63_r5_hdl_public_baseline/tools'))
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from data import message
rospy.init_node('p63_r5_player')
freeze=json.loads(Path(rospy.get_param('~input_freeze')).read_text())
assert [f['raw_ordinal'] for f in freeze['frames']]==list(range(5,165))
seen=set()
rospy.Subscriber('/odom',Odometry,lambda m:seen.add(m.header.stamp.to_nsec()),queue_size=10)
pub=rospy.Publisher('/p63/koide_points',PointCloud2,queue_size=1)
rospy.wait_for_message('/globalmap',PointCloud2,timeout=30)
deadline=time.monotonic()+30
while pub.get_num_connections()<1 and not rospy.is_shutdown():
    assert time.monotonic()<deadline
    rospy.sleep(.05)
for frame in freeze['frames']:
    pub.publish(message(frame));deadline=time.monotonic()+30
    while frame['stamp_ns'] not in seen and not rospy.is_shutdown():
        assert time.monotonic()<deadline
        rospy.sleep(.01)
rospy.loginfo('Fixed 160-scan manual playback complete; no GT evaluation performed.')
