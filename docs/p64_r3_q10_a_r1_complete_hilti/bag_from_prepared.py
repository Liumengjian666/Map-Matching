"""Package the real adapter output for ROS1/clock playback and RViz."""
import argparse
from pathlib import Path
import sys

import rosbag
from sensor_msgs.msg import Imu

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"p64_r3_q9_public_benchmark_fine_geometry"))
from stream_hdl import stamp, xyzmsg


def make(folder, output):
    imu_rows=(folder/"imu.tsv").read_text().splitlines()
    imu_index=0
    with rosbag.Bag(str(output),"w",compression="lz4") as bag:
        for line in (folder/"scans.tsv").read_text().splitlines():
            value, name=line.split()
            epoch=int(value)
            while imu_index<len(imu_rows):
                ticks,*v=imu_rows[imu_index].split()
                i=int(ticks)
                if i>epoch:
                    break
                row=Imu()
                row.header.stamp=stamp(i)
                row.header.frame_id="base_link"
                (row.linear_acceleration.x,row.linear_acceleration.y,row.linear_acceleration.z,
                 row.angular_velocity.x,row.angular_velocity.y,row.angular_velocity.z)=map(float,v)
                bag.write("/gpsimu_driver/imu_data",row,stamp(i))
                imu_index+=1
            # Record slightly after IMU with the original scan stamp kept in the message.
            bag.write("/velodyne_points",xyzmsg(folder/name,"base_link",epoch),stamp(epoch+1))


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("folder",type=Path)
    parser.add_argument("out",type=Path)
    a=parser.parse_args()
    if a.out.exists():
        raise FileExistsError(a.out)
    make(a.folder,a.out)
