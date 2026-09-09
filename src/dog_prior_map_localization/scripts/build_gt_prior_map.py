#!/usr/bin/env python3
"""Build a voxelized prior PCD by projecting PointCloud2 scans with TUM ground truth."""

import argparse
from pathlib import Path

import numpy as np
import rosbag
from scipy.spatial.transform import Rotation, Slerp


POINT_FIELD_TYPES = {
    1: "i1", 2: "u1", 3: "<i2", 4: "<u2",
    5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8",
}


def cloud_xyz(msg):
    fields = {field.name: field for field in msg.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 does not contain x/y/z fields")
    endian = ">" if msg.is_bigendian else "<"
    formats = []
    offsets = []
    for name in ("x", "y", "z"):
        field = fields[name]
        fmt = POINT_FIELD_TYPES[field.datatype]
        if fmt[0] in "<>":
            fmt = endian + fmt[1:]
        formats.append(fmt)
        offsets.append(field.offset)
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": formats,
                      "offsets": offsets, "itemsize": msg.point_step})
    data = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    xyz = np.column_stack((data["x"], data["y"], data["z"])).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def voxel_down(points, voxel):
    if not len(points):
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def write_binary_pcd(path, points):
    points = np.asarray(points, dtype="<f4")
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        points.tofile(stream)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--ground-truth", required=True)
    parser.add_argument("--topic", default="/eve/lidar3d")
    parser.add_argument("--output", required=True)
    parser.add_argument("--voxel", type=float, default=0.15)
    parser.add_argument("--sensor-z", type=float, default=0.4612)
    parser.add_argument("--every-n", type=int, default=1)
    parser.add_argument("--chunk-scans", type=int, default=50)
    args = parser.parse_args()

    truth = np.loadtxt(args.ground_truth)
    truth_times = truth[:, 0]
    truth_positions = truth[:, 1:4]
    truth_rotations = Rotation.from_quat(truth[:, 4:8])
    slerp = Slerp(truth_times, truth_rotations)
    sensor_offset = np.array([0.0, 0.0, args.sensor_z])
    initial_rotation = truth_rotations[0]
    initial_sensor_position = truth_positions[0] + initial_rotation.apply(sensor_offset)
    initial_rotation_inverse = initial_rotation.inv()

    accumulated = np.empty((0, 3), dtype=np.float64)
    chunk = []
    used_scans = 0
    with rosbag.Bag(args.bag) as bag:
        for index, (_, msg, _) in enumerate(bag.read_messages(topics=[args.topic])):
            if index % max(1, args.every_n):
                continue
            stamp = msg.header.stamp.to_sec()
            if stamp < truth_times[0] or stamp > truth_times[-1]:
                continue
            rotation = slerp([stamp])[0]
            right = np.searchsorted(truth_times, stamp)
            left = max(0, right - 1)
            right = min(len(truth_times) - 1, right)
            span = truth_times[right] - truth_times[left]
            alpha = 0.0 if span <= 0 else (stamp - truth_times[left]) / span
            base_position = (1.0 - alpha) * truth_positions[left] + alpha * truth_positions[right]
            sensor_position = base_position + rotation.apply(sensor_offset)
            points_world = rotation.apply(cloud_xyz(msg)) + sensor_position
            chunk.append(initial_rotation_inverse.apply(points_world - initial_sensor_position))
            used_scans += 1
            if len(chunk) >= args.chunk_scans:
                accumulated = voxel_down(np.vstack([accumulated] + chunk), args.voxel)
                chunk.clear()
                print(f"processed_scans={used_scans} map_points={len(accumulated)}", flush=True)
    if chunk:
        accumulated = voxel_down(np.vstack([accumulated] + chunk), args.voxel)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    write_binary_pcd(output, accumulated)
    print(f"output={output} scans={used_scans} points={len(accumulated)}")


if __name__ == "__main__":
    main()
