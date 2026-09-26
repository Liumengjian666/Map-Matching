#!/usr/bin/env python3
"""Extract only the frozen P5-I1 sample clouds from an R10B topic bag."""

import argparse
import csv
import hashlib
import struct
from pathlib import Path

import rosbag
import sensor_msgs.point_cloud2 as pc2


REQUEST_TOPIC = "/dog_livo/ndt/scan_request"
FNV_OFFSET = 14695981039346656037
FNV_PRIME = 1099511628211
MASK64 = (1 << 64) - 1


def mix_bytes(value, data):
    for byte in data:
        value = ((value ^ byte) * FNV_PRIME) & MASK64
    return value


def mix_u32(value, number):
    return mix_bytes(value, struct.pack("<I", int(number) & 0xffffffff))


def mix_string(value, string):
    data = string.encode("utf-8")
    return mix_bytes(mix_u32(value, len(data)), data)


def request_cloud_hash(cloud):
    value = mix_string(FNV_OFFSET, cloud.header.frame_id)
    value = mix_u32(value, cloud.header.stamp.secs)
    value = mix_u32(value, cloud.header.stamp.nsecs)
    value = mix_u32(value, cloud.width)
    value = mix_u32(value, cloud.height)
    value = mix_u32(value, len(cloud.fields))
    for field in cloud.fields:
        value = mix_string(value, field.name)
        value = mix_u32(value, field.offset)
        value = mix_bytes(value, bytes([field.datatype]))
        value = mix_u32(value, field.count)
    value = mix_u32(value, cloud.point_step)
    value = mix_u32(value, cloud.row_step)
    value = mix_bytes(value, bytes([int(cloud.is_bigendian)]))
    value = mix_u32(value, len(cloud.data))
    return mix_bytes(value, cloud.data)


def write_xyz_pcd(path, cloud):
    count = int(cloud.width) * int(cloud.height)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\nDATA binary\n"
    ).encode("ascii")
    written = 0
    with path.open("wb") as stream:
        stream.write(header)
        for point in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=False):
            stream.write(struct.pack("<fff", *point))
            written += 1
    if written != count:
        raise RuntimeError(f"PointCloud2 count mismatch: header={count}, decoded={written}")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return count, digest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    with args.manifest.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    wanted = {int(row["transaction_id"]): row for row in rows}
    if len(wanted) != len(rows):
        raise RuntimeError("manifest transaction IDs are not unique")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    assets = []
    with rosbag.Bag(str(args.bag), "r") as bag:
        for _, request, _ in bag.read_messages(topics=[REQUEST_TOPIC]):
            transaction_id = int(request.transaction_id)
            if transaction_id not in wanted:
                continue
            row = wanted.pop(transaction_id)
            cloud = request.cloud_end_frame
            if cloud.header.frame_id != "cmu_sp1_velodyne":
                raise RuntimeError(f"cloud frame mismatch at transaction {transaction_id}")
            actual_hash = request_cloud_hash(cloud)
            if actual_hash != int(row["request_cloud_hash"]):
                raise RuntimeError(f"request cloud hash mismatch at transaction {transaction_id}")
            path = args.output_dir / f"{row['frame_id']}.pcd"
            count, digest = write_xyz_pcd(path, cloud)
            assets.append({
                "frame_id": row["frame_id"],
                "transaction_id": transaction_id,
                "request_cloud_hash": f"{actual_hash:016x}",
                "raw_point_count": count,
                "pcd_path": str(path),
                "pcd_sha256": digest,
            })
            if not wanted:
                break
    if wanted:
        raise RuntimeError(f"missing selected requests: {sorted(wanted)}")
    assets.sort(key=lambda row: row["transaction_id"])
    index_path = args.output_dir / "cloud_assets.csv"
    with index_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(assets[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(assets)
    print(f"EXTRACTED_CLOUDS={len(assets)}")
    print(f"REQUEST_CLOUD_HASH_MISMATCHES=0")
    print(f"ASSET_INDEX={index_path}")


if __name__ == "__main__":
    main()
