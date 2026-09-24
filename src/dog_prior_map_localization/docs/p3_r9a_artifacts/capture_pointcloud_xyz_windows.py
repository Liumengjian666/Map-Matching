#!/usr/bin/env python3
"""Capture ordered XYZ point arrays only inside a preselected ROS-time window.

This evaluation helper writes a compact binary stream plus a CSV index. It
does not alter messages or localization state. A record is [float64 stamp,
uint32 point_count, point_count x (float32 x,y,z)] in little-endian order.
"""

import argparse
import csv
import hashlib
import struct
from threading import Lock
from pathlib import Path

import numpy as np
import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--origin-stamp", type=float, required=True)
    parser.add_argument("--window-start", type=float, required=True)
    parser.add_argument("--window-end", type=float, required=True)
    parser.add_argument("--raw-topic", default="/velodyne_points")
    parser.add_argument("--deskew-topic", required=True)
    parser.add_argument("--output-prefix", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    if args.window_end <= args.window_start:
        parser.error("window-end must be greater than window-start")
    return args


def main():
    args = parse_args()
    rospy.init_node("p3_r9a_pointcloud_window_capture", anonymous=True)
    prefix = Path(args.output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    index_path = prefix.with_suffix(".csv")
    xyz_path = prefix.with_suffix(".xyzbin")
    index_file = index_path.open("w", newline="")
    xyz_file = xyz_path.open("wb")
    writer = csv.writer(index_file)
    writer.writerow(["topic", "stamp", "offset_sec", "point_count",
                     "message_sha256", "xyz_sha256", "xyz_file_offset"])
    output_topics = {args.raw_topic, args.deskew_topic}
    write_lock = Lock()

    def callback(topic):
        def capture(msg):
            stamp = msg.header.stamp.to_sec()
            offset = stamp - args.origin_stamp
            if offset < args.window_start or offset >= args.window_end:
                return
            if msg.width == 0 or msg.height == 0:
                rospy.logerr("empty point cloud at %.9f", stamp)
                return
            xyz = np.asarray(list(point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=False)), dtype="<f4")
            if xyz.ndim != 2 or xyz.shape[1] != 3:
                rospy.logerr("unexpected XYZ shape at %.9f: %s", stamp, xyz.shape)
                return
            payload = xyz.tobytes(order="C")
            message_hash = hashlib.sha256(bytes(msg.data)).hexdigest()
            xyz_hash = hashlib.sha256(payload).hexdigest()
            # rospy dispatches different subscriptions on different threads.
            # Serialize the shared binary offset, record and index-row writes
            # so each CSV offset names one complete [stamp,count,XYZ] record.
            with write_lock:
                file_offset = xyz_file.tell()
                xyz_file.write(struct.pack("<dI", stamp, xyz.shape[0]))
                xyz_file.write(payload)
                writer.writerow([topic, format(stamp, ".9f"), format(offset, ".9f"),
                                 xyz.shape[0], message_hash, xyz_hash, file_offset])
                index_file.flush()
                xyz_file.flush()
        return capture

    for topic in sorted(output_topics):
        rospy.Subscriber(topic, PointCloud2, callback(topic), queue_size=64,
                         tcp_nodelay=True)

    def close_files():
        index_file.close()
        xyz_file.close()

    rospy.on_shutdown(close_files)
    rospy.spin()


if __name__ == "__main__":
    main()
