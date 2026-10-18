#!/usr/bin/python3
"""Bounded raw-LiDAR engineering probe. No IMU, Deskew, EKF, or GT."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import threading
import time

import numpy as np
import rosbag


LABEL = "SAME_SEQUENCE_SELF_MAP_ENGINEERING_ONLY"


def prepare(bag_path, directory):
    directory = Path(directory)
    directory.mkdir(exist_ok=False)
    with rosbag.Bag(bag_path, "r") as bag:
        _, msg, _ = next(bag.read_messages(topics=["/velodyne_points"]))
    field = {f.name: f for f in msg.fields}
    if msg.is_bigendian or any(field[n].datatype != 7 for n in ("x", "y", "z")):
        raise ValueError("unsupported XYZ representation")
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                      "offsets": [field[n].offset for n in ("x", "y", "z")],
                      "itemsize": msg.point_step})
    cloud = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.row_step, msg.point_step))
    xyz = np.stack([cloud[n].reshape(-1) for n in ("x", "y", "z")], axis=1)
    xyz = xyz[np.isfinite(xyz).all(axis=1)].astype("<f4")
    pcd = directory / "first_scan_xyz.pcd"
    header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
              f"COUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {len(xyz)}\nDATA binary\n")
    with pcd.open("xb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(xyz.tobytes())
    manifest = {"label": LABEL, "map_source": "only original scan 1, raw XYZ",
                "bag": os.path.realpath(bag_path), "map_path": str(pcd),
                "map_sha256": hashlib.sha256(pcd.read_bytes()).hexdigest(),
                "map_points": len(xyz), "map_scan_stamp_ns": msg.header.stamp.to_nsec(),
                "source_data_sha256": hashlib.sha256(msg.data).hexdigest(),
                "T_map_lidar_scan1": np.eye(4).tolist(),
                "T_imu_lidar": None, "deskew": False, "ekf": False, "gt_used": False}
    with (directory / "map_manifest.json").open("x") as stream:
        json.dump(manifest, stream, indent=2)
    print(json.dumps(manifest, indent=2))


def load_frozen_map(map_directory):
    manifest = json.loads((Path(map_directory) / "map_manifest.json").read_text())
    frozen_map = Path(map_directory) / "first_scan_xyz.pcd"
    if frozen_map.resolve() != Path(manifest["map_path"]).resolve():
        raise RuntimeError("map manifest refers to another map path")
    if hashlib.sha256(frozen_map.read_bytes()).hexdigest() != manifest["map_sha256"]:
        raise RuntimeError("frozen first-scan map hash mismatch")
    return manifest


def publish(bag_path, directory, map_directory):
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    manifest = load_frozen_map(map_directory)
    rospy.init_node("ground_challenge_raw_registration_probe")
    directory = Path(directory)
    received = {}
    condition = threading.Condition()
    def callback(msg):
        with condition:
            received[msg.header.stamp.to_nsec()] = msg
            condition.notify_all()
    subscriber = rospy.Subscriber("/dog_livo/ndt_odom", Odometry, callback, queue_size=50)
    publisher = rospy.Publisher("/ground_challenge/raw_registration_probe", PointCloud2, queue_size=1)
    deadline = time.monotonic() + 90
    while publisher.get_num_connections() == 0:
        if rospy.is_shutdown() or time.monotonic() > deadline:
            raise RuntimeError("real NDT subscriber did not become ready")
        time.sleep(0.02)
    with (directory / "publisher_receipts.csv").open("x", newline="") as receipts, \
         (directory / "ndt_used_trajectory.csv").open("x", newline="") as trajectory, \
         rosbag.Bag(bag_path, "r") as bag:
        receipt = csv.writer(receipts)
        pose = csv.writer(trajectory)
        receipt.writerow(["original_scan_index", "stamp_ns", "input_data_sha256", "ack_wall_ms"])
        pose.writerow(["original_scan_index", "stamp_ns", "x", "y", "z", "qx", "qy", "qz", "qw"])
        done = 0
        for index, (_, msg, _) in enumerate(bag.read_messages(topics=["/velodyne_points"]), 1):
            if index == 1:
                if (msg.header.stamp.to_nsec() != manifest["map_scan_stamp_ns"] or
                        hashlib.sha256(msg.data).hexdigest() != manifest["source_data_sha256"]):
                    raise RuntimeError("input bag does not match the frozen first-scan map")
                continue
            if index > 21:
                break
            ns = msg.header.stamp.to_nsec()
            started = time.monotonic()
            publisher.publish(msg)
            with condition:
                while ns not in received:
                    remaining = 30 - (time.monotonic() - started)
                    if remaining <= 0 or rospy.is_shutdown():
                        raise RuntimeError(f"no real NDT output for original scan {index}")
                    condition.wait(min(remaining, 0.2))
                output = received.pop(ns)
            p, q = output.pose.pose.position, output.pose.pose.orientation
            receipt.writerow([index, ns, hashlib.sha256(msg.data).hexdigest(),
                              (time.monotonic() - started) * 1000])
            pose.writerow([index, ns, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            receipts.flush()
            trajectory.flush()
            done += 1
            print(f"RAW_REGISTRATION_ONLY_ACK scan={index} ns={ns}", flush=True)
        if done != 20:
            raise RuntimeError(f"expected 20 consecutive test frames, got {done}")
    _ = subscriber


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["prepare", "publish"])
    parser.add_argument("--bag", required=True)
    parser.add_argument("--directory", required=True)
    parser.add_argument("--map-directory")
    from rospy import myargv
    args = parser.parse_args(myargv()[1:])
    if args.mode == "prepare":
        prepare(args.bag, args.directory)
    else:
        if not args.map_directory:
            parser.error("publish requires --map-directory")
        publish(args.bag, args.directory, args.map_directory)
