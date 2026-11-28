"""Build a ROS1 evidence bag with real sensor input and the recorded active path."""
import sys
from pathlib import Path
import rosbag
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from scipy.spatial.transform import Rotation

from data_flow import mat, rows
p=Path(__file__).resolve().parents[1]/"p64_r3_q10_a_r1_complete_hilti"
sys.path.insert(0,str(p))
from bag_from_prepared import make, stamp


def run(input_, trajectory, sensor_bag, result):
    if sensor_bag.exists() or result.exists():
        raise FileExistsError("select a new replay output")
    make(input_,sensor_bag)
    native=rows(trajectory)
    by_stamp={int(row["stamp_ns"]):row for row in native}
    count=0
    track=RosPath()
    track.header.frame_id="map"
    with rosbag.Bag(str(sensor_bag)) as source,rosbag.Bag(str(result),"w",compression="lz4") as output:
        for topic,msg,t in source.read_messages():
            output.write(topic,msg,t)
            if topic!="/velodyne_points":
                continue
            ns=msg.header.stamp.to_nsec()
            matrix=mat(by_stamp[ns],"state")
            pose=PoseStamped()
            pose.header.stamp=stamp(ns)
            pose.header.frame_id="map"
            p,q=pose.pose.position,pose.pose.orientation
            p.x,p.y,p.z=matrix[:3,3]
            q.x,q.y,q.z,q.w=Rotation.from_matrix(matrix[:3,:3]).as_quat()
            track.poses.append(pose)
            track.header.stamp=pose.header.stamp
            output.write("/q10b/icp_trajectory",track,t)
            count+=1
    assert count==len(native)


if __name__=="__main__":
    run(*(Path(x) for x in sys.argv[1:]))
