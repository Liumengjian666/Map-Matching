#!/usr/bin/env python3
"""Export the exact raw VLP-16 scans used by Floor01 transactions.

The output keeps each source point in the original LiDAR frame and stores its
header-relative time offset. Deskew is deliberately left to the replay runner,
where it uses that run's own causal IMU/filter trajectory.
"""

import argparse
import csv
import math
import os
import sys

import numpy as np
import rosbag
from sensor_msgs.msg import PointField


POINT_FORMATS = {
    PointField.FLOAT32: np.dtype("<f4"),
    PointField.FLOAT64: np.dtype("<f8"),
    PointField.UINT32: np.dtype("<u4"),
    PointField.INT32: np.dtype("<i4"),
    PointField.UINT16: np.dtype("<u2"),
    PointField.INT16: np.dtype("<i2"),
    PointField.UINT8: np.dtype("u1"),
    PointField.INT8: np.dtype("i1"),
}


def scan_end_ns_from_header(start_ns, maximum_offset_seconds):
    # The live parser rounds the absolute long-double epoch expression. Doing
    # start_ns + round(offset) can differ by 1 ns for float32 point times.
    absolute = (np.longdouble(start_ns) +
                np.longdouble(maximum_offset_seconds) * np.longdouble(1_000_000_000))
    if (not np.isfinite(absolute) or
            absolute > np.longdouble(np.iinfo(np.uint64).max)):
        raise ValueError("scan-end timestamp overflow")
    return int(np.floor(absolute + np.longdouble(0.5)))


def row_views(cloud, time_field):
    if cloud.is_bigendian or cloud.width == 0 or cloud.height == 0:
        raise ValueError("raw cloud must be nonempty and little-endian")
    if cloud.row_step < cloud.width * cloud.point_step:
        raise ValueError("PointCloud2 row_step is shorter than its packed points")
    fields = {field.name: field for field in cloud.fields}
    for name in ("x", "y", "z", time_field):
        if name not in fields:
            raise ValueError("PointCloud2 missing required field: " + name)
        if fields[name].count != 1 or fields[name].datatype not in POINT_FORMATS:
            raise ValueError("unsupported PointCloud2 field: " + name)
        if fields[name].offset + POINT_FORMATS[fields[name].datatype].itemsize > cloud.point_step:
            raise ValueError("PointCloud2 field exceeds point_step: " + name)
    names, formats, offsets = [], [], []
    for name in ("x", "y", "z", time_field, "intensity"):
        field = fields.get(name)
        if field is None:
            continue
        if field.count != 1 or field.datatype not in POINT_FORMATS:
            raise ValueError("unsupported PointCloud2 field: " + name)
        names.append(name)
        formats.append(POINT_FORMATS[field.datatype])
        offsets.append(field.offset)
    dtype = np.dtype({"names": names, "formats": formats,
                      "offsets": offsets, "itemsize": cloud.point_step})
    data = memoryview(cloud.data)
    return [np.ndarray((cloud.width,), dtype=dtype, buffer=data,
                       offset=row * cloud.row_step)
            for row in range(cloud.height)]


def read_expected_scans(path):
    expected = {}
    with open(path, newline="") as stream:
        for row in csv.DictReader(stream):
            transaction = int(row["transaction_id"])
            stamp_ns = int(row["stamp_ns"])
            if transaction in expected or stamp_ns in expected:
                raise ValueError("duplicate expected transaction or scan-end stamp")
            expected[stamp_ns] = transaction
    if not expected or sorted(expected.values()) != list(range(1, len(expected) + 1)):
        raise ValueError("expected scan transactions must be contiguous from 1")
    return expected


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--scans-csv", required=True,
                        help="frozen P7 scans.csv containing transaction and scan-end stamps")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--topic", default="/velodyne_points")
    parser.add_argument("--time-field", default="time")
    parser.add_argument("--time-scale-seconds", type=float, default=1.0)
    parser.add_argument("--max-scan-duration-seconds", type=float, default=0.15)
    args = parser.parse_args()

    if not math.isfinite(args.time_scale_seconds) or args.time_scale_seconds <= 0:
        raise ValueError("time scale must be positive and finite")
    if not math.isfinite(args.max_scan_duration_seconds) or args.max_scan_duration_seconds <= 0:
        raise ValueError("maximum scan duration must be positive and finite")
    if os.path.exists(args.output_dir):
        raise ValueError("output directory already exists; refusing overwrite")

    expected = read_expected_scans(args.scans_csv)
    os.makedirs(args.output_dir)
    binary_path = os.path.join(args.output_dir, "raw_timed_points.bin")
    index_path = os.path.join(args.output_dir, "raw_timed_scans.csv")
    records = {}
    raw_messages = 0
    output_points = 0
    previous_offset = 0

    with open(binary_path, "xb") as binary, open(index_path, "x", newline="") as index_file:
        index = csv.writer(index_file, lineterminator="\n")
        index.writerow(["transaction_id", "scan_start_ns", "scan_end_ns",
                        "cloud_byte_offset", "cloud_point_count"])
        with rosbag.Bag(args.bag, "r") as bag:
            for _, cloud, _ in bag.read_messages(topics=[args.topic]):
                raw_messages += 1
                if cloud.header.stamp.is_zero():
                    continue
                rows = row_views(cloud, args.time_field)
                fields = {field.name: field for field in cloud.fields}
                if args.time_field not in fields:
                    continue
                max_offset_seconds = -math.inf
                point_count = 0
                for row in rows:
                    xyz_finite = np.isfinite(row["x"]) & np.isfinite(row["y"]) & np.isfinite(row["z"])
                    time_seconds = row[args.time_field].astype(np.float64) * args.time_scale_seconds
                    valid = xyz_finite & np.isfinite(time_seconds) & (time_seconds >= 0.0)
                    if "intensity" in row.dtype.names:
                        valid &= np.isfinite(row["intensity"])
                    if not valid.all():
                        raise ValueError("raw PointCloud2 contains a point rejected by the live parser")
                    row_max = float(np.max(time_seconds))
                    if row_max > args.max_scan_duration_seconds:
                        raise ValueError("point-time offset exceeds configured scan duration")
                    max_offset_seconds = max(max_offset_seconds, row_max)
                    point_count += len(row)
                if point_count == 0 or max_offset_seconds <= 0.0:
                    continue
                scan_start_ns = int(cloud.header.stamp.to_nsec())
                scan_end_ns = scan_end_ns_from_header(scan_start_ns, max_offset_seconds)
                transaction = expected.get(scan_end_ns)
                if transaction is None:
                    continue
                if transaction in records:
                    raise ValueError("multiple raw clouds map to one P7 scan transaction")
                if transaction != len(records) + 1:
                    raise ValueError("raw scan transactions are not contiguous in sensor order")

                for row in rows:
                    time_seconds = row[args.time_field].astype(np.float64) * args.time_scale_seconds
                    offsets_ns = np.floor(
                        time_seconds.astype(np.longdouble) * np.longdouble(1_000_000_000)
                        + np.longdouble(0.5)).astype(np.uint64)
                    if np.any(offsets_ns > np.iinfo(np.uint32).max):
                        raise ValueError("point-time offset does not fit the replay format")
                    packed = np.empty(len(row), dtype=np.dtype([
                        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("offset_ns", "<u4")]))
                    packed["x"] = row["x"]
                    packed["y"] = row["y"]
                    packed["z"] = row["z"]
                    packed["offset_ns"] = offsets_ns.astype(np.uint32)
                    binary.write(packed.tobytes(order="C"))

                records[transaction] = (scan_start_ns, scan_end_ns,
                                        previous_offset, point_count)
                index.writerow([transaction, scan_start_ns, scan_end_ns,
                                previous_offset, point_count])
                previous_offset += point_count * 16
                output_points += point_count

    missing = sorted(set(expected.values()) - set(records))
    if missing:
        raise ValueError("raw bag did not provide exact scan-end matches; missing tx " +
                         ",".join(map(str, missing[:20])))
    if len(records) != len(expected):
        raise ValueError("raw scan transaction count mismatch")
    print("RAW_TIMED_EXPORT_PASS")
    print("bag=" + os.path.realpath(args.bag))
    print("raw_messages=" + str(raw_messages))
    print("selected_transactions=" + str(len(records)))
    print("point_count=" + str(output_points))
    print("binary_bytes=" + str(previous_offset))
    print("index=" + index_path)
    print("binary=" + binary_path)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:  # produce a concise, auditable failure
        print("RAW_TIMED_EXPORT_FAIL: " + str(error), file=sys.stderr)
        sys.exit(1)
