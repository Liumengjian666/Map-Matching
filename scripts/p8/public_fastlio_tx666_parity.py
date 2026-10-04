#!/usr/bin/env python3
"""Relay Velodyne XYZIRT to FAST-LIO's public ns contract and capture TX666.

The public Velodyne ROS driver emits relative point time in seconds. The
unchanged FAST-LIO SubT config declares timestamp_unit=nanoseconds, so this
diagnostic relay changes only PointCloud2.time from seconds to nanoseconds.
It also captures the exact input transaction and FAST-LIO's full body-frame
deskewed publication for one requested scan.
"""

import argparse
import os
import time

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField


def field_by_name(msg, name):
    matches = [field for field in msg.fields if field.name == name]
    if len(matches) != 1:
        raise RuntimeError("expected exactly one PointCloud2 field: " + name)
    return matches[0]


def unpack_cloud(msg, include_ring=False):
    x_field = field_by_name(msg, "x")
    y_field = field_by_name(msg, "y")
    z_field = field_by_name(msg, "z")
    intensity_field = field_by_name(msg, "intensity")
    if any(f.datatype != PointField.FLOAT32 for f in
           (x_field, y_field, z_field, intensity_field)):
        raise RuntimeError("XYZI fields must be FLOAT32")
    dtype = ">f4" if msg.is_bigendian else "<f4"
    raw = memoryview(msg.data)
    count = int(msg.width) * int(msg.height)
    xyz = np.column_stack([
        np.ndarray((count,), dtype=dtype, buffer=raw, offset=f.offset,
                   strides=(msg.point_step,))
        for f in (x_field, y_field, z_field)
    ]).astype(np.float64)
    intensity = np.ndarray((count,), dtype=dtype, buffer=raw,
                           offset=intensity_field.offset,
                           strides=(msg.point_step,)).astype(np.float64)
    output = {"xyz": xyz, "intensity": intensity}
    if include_ring:
        ring = field_by_name(msg, "ring")
        if ring.datatype != PointField.UINT16:
            raise RuntimeError("ring field must be UINT16")
        ring_dtype = ">u2" if msg.is_bigendian else "<u2"
        output["ring"] = np.ndarray(
            (count,), dtype=ring_dtype, buffer=raw, offset=ring.offset,
            strides=(msg.point_step,)).astype(np.uint16)
    if any(field.name == "time" for field in msg.fields):
        time_field = field_by_name(msg, "time")
        if time_field.datatype != PointField.FLOAT32:
            raise RuntimeError("time field must be FLOAT32")
        output["time_s"] = np.ndarray(
            (count,), dtype=dtype, buffer=raw, offset=time_field.offset,
            strides=(msg.point_step,)).astype(np.float64)
    if any(field.name == "curvature" for field in msg.fields):
        curvature = field_by_name(msg, "curvature")
        output["curvature_ms"] = np.ndarray(
            (count,), dtype=dtype, buffer=raw, offset=curvature.offset,
            strides=(msg.point_step,)).astype(np.float64)
    return output


class CaptureRelay:
    def __init__(self, output_dir, scan_start_ns, scan_end_ns, tolerance_ns):
        self.output_dir = output_dir
        self.scan_start_ns = scan_start_ns
        self.scan_end_ns = scan_end_ns
        self.tolerance_ns = tolerance_ns
        self.raw_saved = False
        self.body_saved = False
        self.last_raw_header_ns = None
        self.last_body_stamp_ns = None
        self.pub = rospy.Publisher("/velodyne_points", PointCloud2,
                                   queue_size=4)
        self.raw_sub = rospy.Subscriber("/velodyne_points_raw", PointCloud2,
                                        self.raw_cb, queue_size=4,
                                        buff_size=64 * 1024 * 1024)
        self.body_sub = rospy.Subscriber("/cloud_registered_body", PointCloud2,
                                         self.body_cb, queue_size=4,
                                         buff_size=64 * 1024 * 1024)

    def write_npz(self, name, msg, arrays):
        path = os.path.join(self.output_dir, name)
        if os.path.exists(path):
            raise RuntimeError("refusing to overwrite " + path)
        np.savez_compressed(
            path,
            header_stamp_ns=np.int64(msg.header.stamp.to_nsec()),
            frame_id=np.asarray(msg.header.frame_id),
            width=np.int64(msg.width), height=np.int64(msg.height),
            point_step=np.int64(msg.point_step), **arrays)
        rospy.loginfo("saved %s with %d points", path, msg.width * msg.height)

    def raw_cb(self, msg):
        stamp_ns = msg.header.stamp.to_nsec()
        self.last_raw_header_ns = stamp_ns
        output = PointCloud2()
        output.header = msg.header
        output.height = msg.height
        output.width = msg.width
        output.fields = msg.fields
        output.is_bigendian = msg.is_bigendian
        output.point_step = msg.point_step
        output.row_step = msg.row_step
        output.is_dense = msg.is_dense
        data = bytearray(msg.data)
        time_field = field_by_name(msg, "time")
        if time_field.datatype != PointField.FLOAT32:
            raise RuntimeError("Velodyne point time must be FLOAT32 seconds")
        dtype = ">f4" if msg.is_bigendian else "<f4"
        times = np.ndarray((msg.width * msg.height,), dtype=dtype,
                           buffer=data, offset=time_field.offset,
                           strides=(msg.point_step,))
        if np.any(~np.isfinite(times)) or np.any(times < 0.0):
            raise RuntimeError("invalid Velodyne relative point time")
        if not self.raw_saved and stamp_ns == self.scan_start_ns:
            arrays = unpack_cloud(msg, include_ring=True)
            arrays["point_time_ns_from_header"] = np.rint(
                arrays.pop("time_s") * 1.0e9).astype(np.int64)
            self.write_npz("tx666_public_velodyne_raw.npz", msg, arrays)
            self.raw_saved = True
        times *= np.float32(1.0e9)
        output.data = bytes(data)
        self.pub.publish(output)

    def body_cb(self, msg):
        stamp_ns = msg.header.stamp.to_nsec()
        self.last_body_stamp_ns = stamp_ns
        if self.body_saved or msg.header.frame_id != "body":
            return
        if abs(stamp_ns - self.scan_end_ns) > self.tolerance_ns:
            return
        arrays = unpack_cloud(msg)
        if "curvature_ms" not in arrays:
            raise RuntimeError("body output is missing diagnostic curvature")
        arrays["point_time_ns_from_header"] = np.rint(
            arrays.pop("curvature_ms") * 1.0e6).astype(np.int64)
        self.write_npz("tx666_fastlio_body_deskew.npz", msg, arrays)
        self.body_saved = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--scan-start-ns", type=int, default=1517157286155932903)
    parser.add_argument("--scan-end-ns", type=int, default=1517157286256772352)
    parser.add_argument("--tolerance-ms", type=float, default=2.0)
    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    if os.listdir(args.output_dir):
        raise RuntimeError("output directory must be empty: " + args.output_dir)
    rospy.init_node("p8_fastlio_tx666_parity_relay", anonymous=False)
    node = CaptureRelay(args.output_dir, args.scan_start_ns,
                        args.scan_end_ns, int(args.tolerance_ms * 1.0e6))
    timeout_s = float(rospy.get_param("~timeout_s", 120.0))
    deadline = time.time() + timeout_s
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and time.time() < deadline:
        if node.raw_saved and node.body_saved:
            rospy.loginfo("both TX666 captures complete")
            return
        rate.sleep()
    raise RuntimeError(
        "capture timeout: raw_saved=%s body_saved=%s last_raw=%s last_body=%s" %
        (node.raw_saved, node.body_saved, node.last_raw_header_ns,
         node.last_body_stamp_ns))


if __name__ == "__main__":
    main()
