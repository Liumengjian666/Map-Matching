#!/usr/bin/env python3
"""Create a fixed-lag replay bag by appending genuine post-window IMU only.

The frozen V1 bag is never modified. Every existing message is copied in
record-time order; an already-verified raw IMU CSV supplies only the minimal
drain tail required to pass the final NDT observation's fixed-lag deadline.
"""

import argparse
import csv
import hashlib
import io
import json
import pathlib
import sys

import rosbag
import rospy
from sensor_msgs.msg import Imu


EXPECTED_V1_SHA256 = "f6e77c96d97402e551835e81a1ce8d780d075da2f18a9cb5e4fc0646628b86d6"
EXPECTED_IMU_CSV_SHA256 = "21b94c0cb0ade931db1586799aa022dff2ba20b9ab76dc1e29eac0e948e132e6"
V1_BAG = pathlib.Path(
    "/home/jian/livox_ws/dog_loc_paper_ws/.p10_corridor01_prospective_causal_replay_r1/input/"
    "corridor01_p10_causal_startup5_eval35_v1.bag"
)
IMU_CSV = pathlib.Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/"
    "p9_corridor01_raw_scanend_v1/imu.csv"
)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stamp_from_ns(stamp_ns):
    return rospy.Time(stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000)


def imu_from_row(row):
    msg = Imu()
    msg.header.stamp = stamp_from_ns(int(row["stamp_ns"]))
    msg.header.frame_id = "epson"
    msg.orientation_covariance[0] = -1.0
    msg.angular_velocity.x = float(row["gx"])
    msg.angular_velocity.y = float(row["gy"])
    msg.angular_velocity.z = float(row["gz"])
    msg.linear_acceleration.x = float(row["ax"])
    msg.linear_acceleration.y = float(row["ay"])
    msg.linear_acceleration.z = float(row["az"])
    return msg


def serialized_digest(msg):
    buff = io.BytesIO()
    msg.serialize(buff)
    return hashlib.sha256(buff.getvalue()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True, type=pathlib.Path)
    parser.add_argument("--fixed-lag-sec", default=0.15, type=float)
    parser.add_argument("--drain-margin-sec", default=0.05, type=float)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise RuntimeError("refusing_to_overwrite_output_directory")
    if not (args.fixed_lag_sec > 0 and args.drain_margin_sec >= 0):
        raise RuntimeError("invalid_lag_or_drain_margin")
    if sha256(V1_BAG) != EXPECTED_V1_SHA256:
        raise RuntimeError("frozen_v1_bag_hash_mismatch")
    if sha256(IMU_CSV) != EXPECTED_IMU_CSV_SHA256:
        raise RuntimeError("verified_raw_imu_csv_hash_mismatch")

    # First pass: prove the V1 chronology and derive its final observation
    # deadline without relying on rosbag record time as sensor time.
    cloud_count = 0
    imu_count = 0
    last_cloud_stamp_ns = -1
    last_imu_stamp_ns = -1
    original_last_record_ns = -1
    with rosbag.Bag(str(V1_BAG), "r") as source:
        for topic, msg, record_time in source.read_messages():
            record_ns = record_time.to_nsec()
            original_last_record_ns = max(original_last_record_ns, record_ns)
            if topic == "/p10/corridor01/raw_timed_points":
                cloud_count += 1
                last_cloud_stamp_ns = max(last_cloud_stamp_ns, msg.header.stamp.to_nsec())
            elif topic == "/imu/data":
                imu_count += 1
                last_imu_stamp_ns = max(last_imu_stamp_ns, msg.header.stamp.to_nsec())
    if cloud_count != 396 or imu_count != 7989 or last_cloud_stamp_ns <= 0:
        raise RuntimeError("frozen_v1_topic_count_or_stamp_contract_mismatch")
    lag_ns = int(round(args.fixed_lag_sec * 1_000_000_000))
    margin_ns = int(round(args.drain_margin_sec * 1_000_000_000))
    deadline_ns = last_cloud_stamp_ns + lag_ns
    target_ns = deadline_ns + margin_ns

    tail_rows = []
    with IMU_CSV.open(newline="") as stream:
        for row in csv.DictReader(stream):
            stamp_ns = int(row["stamp_ns"])
            if stamp_ns > last_imu_stamp_ns:
                tail_rows.append(row)
                if stamp_ns >= target_ns:
                    break
    if not tail_rows or int(tail_rows[-1]["stamp_ns"]) < target_ns:
        raise RuntimeError("raw_imu_csv_does_not_cover_fixed_lag_drain_target")
    tail_stamps = [int(row["stamp_ns"]) for row in tail_rows]
    if any(b <= a for a, b in zip(tail_stamps, tail_stamps[1:])):
        raise RuntimeError("raw_imu_tail_not_strictly_increasing")
    if tail_stamps[0] <= original_last_record_ns:
        raise RuntimeError("tail_record_time_would_not_append_after_frozen_bag")

    args.output_dir.mkdir(parents=True)
    output_bag = args.output_dir / "corridor01_p10_causal_startup5_eval35_fixedlag_v2.bag"
    source_prefix = {"/imu/data": hashlib.sha256(), "/p10/corridor01/raw_timed_points": hashlib.sha256()}
    output_prefix = {topic: hashlib.sha256() for topic in source_prefix}
    source_records = {topic: [] for topic in source_prefix}
    output_records = {topic: [] for topic in output_prefix}

    with rosbag.Bag(str(V1_BAG), "r") as source, rosbag.Bag(str(output_bag), "w") as output:
        imu_connection = None
        for topic, msg, record_time, connection_header in source.read_messages(
                return_connection_header=True):
            if topic not in source_prefix:
                raise RuntimeError("unexpected_topic_in_frozen_v1_bag:" + topic)
            if topic == "/imu/data" and imu_connection is None:
                imu_connection = connection_header
            digest = serialized_digest(msg)
            source_prefix[topic].update(digest.encode("ascii"))
            source_records[topic].append((record_time.to_nsec(), digest))
            output.write(topic, msg, t=record_time, connection_header=connection_header)
        if imu_connection is None:
            raise RuntimeError("missing_imu_connection_header")
        for row in tail_rows:
            msg = imu_from_row(row)
            record_time = msg.header.stamp
            output.write("/imu/data", msg, t=record_time, connection_header=imu_connection)

    # Re-read output and prove all V1 messages are the exact serialized prefix
    # with identical record timestamps. Additional records must be only IMU.
    output_imu_count = 0
    output_cloud_count = 0
    output_last_imu_stamp_ns = -1
    output_records = {topic: [] for topic in source_prefix}
    with rosbag.Bag(str(output_bag), "r") as output:
        for topic, msg, record_time in output.read_messages():
            if topic not in output_records:
                raise RuntimeError("unexpected_topic_in_fixed_lag_v2_bag:" + topic)
            digest = serialized_digest(msg)
            if len(output_records[topic]) < len(source_records[topic]):
                output_records[topic].append((record_time.to_nsec(), digest))
            elif topic != "/imu/data":
                raise RuntimeError("unexpected_appended_non_imu_message")
            if topic == "/p10/corridor01/raw_timed_points":
                output_cloud_count += 1
            elif topic == "/imu/data":
                output_imu_count += 1
                output_last_imu_stamp_ns = max(output_last_imu_stamp_ns, msg.header.stamp.to_nsec())
    for topic in source_records:
        if output_records[topic] != source_records[topic]:
            raise RuntimeError("frozen_v1_message_prefix_changed:" + topic)
    if output_cloud_count != cloud_count or output_imu_count != imu_count + len(tail_rows):
        raise RuntimeError("fixed_lag_v2_topic_count_mismatch")
    if output_last_imu_stamp_ns < target_ns:
        raise RuntimeError("fixed_lag_v2_output_does_not_reach_drain_target")

    manifest = {
        "protocol": "P10_CORRIDOR01_FIXED_SENSOR_TIME_OOSM_V2",
        "GT_LOADED": False,
        "fixed_lag_sec": args.fixed_lag_sec,
        "drain_margin_sec": args.drain_margin_sec,
        "input_v1": {"path": str(V1_BAG), "sha256": EXPECTED_V1_SHA256,
                     "cloud_messages": cloud_count, "imu_messages": imu_count},
        "imu_tail_source": {"path": str(IMU_CSV), "sha256": EXPECTED_IMU_CSV_SHA256,
                            "frame_id": "epson", "fields": ["ax", "ay", "az", "gx", "gy", "gz"]},
        "last_ndt_sensor_stamp_ns": last_cloud_stamp_ns,
        "last_fixed_lag_deadline_ns": deadline_ns,
        "drain_target_ns": target_ns,
        "v1_last_imu_sensor_stamp_ns": last_imu_stamp_ns,
        "appended_tail_messages": len(tail_rows),
        "tail_first_sensor_stamp_ns": tail_stamps[0],
        "tail_last_sensor_stamp_ns": tail_stamps[-1],
        "output": {"path": str(output_bag), "sha256": sha256(output_bag),
                   "cloud_messages": output_cloud_count,
                   "imu_messages": output_imu_count,
                   "last_imu_sensor_stamp_ns": output_last_imu_stamp_ns},
        "source_prefix_serialized_sha256": {
            topic: source_prefix[topic].hexdigest() for topic in source_prefix
        },
        "limitations": [
            "This is a new V2 replay input, not byte-identical to V1.",
            "All V1 messages are copied unchanged; only genuine IMU samples after the V1 end are appended.",
            "No scans, cloud points, map, configuration, or pre-existing IMU sample are changed."
        ]
    }
    manifest_path = args.output_dir / "fixed_lag_v2_input_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps(manifest, indent=2, sort_keys=True))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # print a concise auditable failure before exiting
        print("input_generation_failed:", exc, file=sys.stderr)
        raise
