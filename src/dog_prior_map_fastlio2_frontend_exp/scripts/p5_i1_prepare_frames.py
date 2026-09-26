#!/usr/bin/env python3
"""Freeze P5-I1 Floor01 frames from the captured R10B transaction bag.

This reads the bag directly (never plays it), records request/result metadata,
and selects fixed uniform/targeted cohorts before any GT is opened.
"""

import argparse
import csv
import hashlib
from pathlib import Path

import rosbag


DEFAULT_BAG = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/"
    "p3_r10b_fix1_floor01_full_rerun_20260926/floor01_fix1_runtime_topics.bag"
)
DEFAULT_MAP = Path("/tmp/floor01_candidates/floor01_h1_map.pcd")
EXPECTED_BAG_SHA = "860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db"
EXPECTED_MAP_SHA = "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570"
EVAL_ORIGIN_NS = 1660857393197807074
SEGMENTS = [
    ("0-50", 0.0, 50.0),
    ("50-100", 50.0, 100.0),
    ("100-150", 100.0, 150.0),
    ("150-200", 150.0, 200.0),
    ("200-250", 200.0, 250.0),
    ("250-300", 250.0, 300.0),
    ("300-350", 300.0, 350.0),
]
CROSSINGS = (("0p5m", 84.91932845115662), ("1m", 93.5928385257721),
             ("2m", 151.48321318626404), ("5m", 157.43361401557922))
REQUEST_TOPIC = "/dog_livo/ndt/scan_request"
RESULT_TOPIC = "/dog_livo/ndt/scan_result"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def pose_fields(pose):
    p, q = pose.position, pose.orientation
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def key(message):
    return (message.frontend_session_id, int(message.epoch), int(message.transaction_id))


def frozen_records(bag_path):
    requests, results = {}, {}
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, message, _ in bag.read_messages(topics=[REQUEST_TOPIC, RESULT_TOPIC]):
            target = requests if message._type.endswith("NdtScanRequest") else results
            item_key = key(message)
            if item_key in target:
                raise RuntimeError(f"duplicate transaction record: {item_key}")
            target[item_key] = message
    if len(requests) != 4127 or len(results) != 4127 or requests.keys() != results.keys():
        raise RuntimeError(f"unexpected request/result key counts: {len(requests)}/{len(results)}")

    records = []
    for item_key, request in requests.items():
        result = results[item_key]
        if request.map_frame != "floor01_map_h1" or request.lidar_frame != "cmu_sp1_velodyne":
            raise RuntimeError(f"request frame mismatch at transaction {request.transaction_id}")
        if request.cloud_end_frame.header.frame_id != request.lidar_frame:
            raise RuntimeError(f"cloud frame mismatch at transaction {request.transaction_id}")
        if result.disposition != result.SUCCESS or not result.pose_valid:
            raise RuntimeError(f"non-success saved transaction {request.transaction_id}")
        scan_ns = int(request.scan_end_ns)
        record = {
            "key": item_key,
            "transaction_id": int(request.transaction_id),
            "scan_end_ns": scan_ns,
            "time_s": (scan_ns - EVAL_ORIGIN_NS) / 1e9,
            "request_cloud_hash": int(request.request_cloud_hash),
            "ndt_source_cloud_hash": int(result.ndt_source_cloud_hash),
            "predicted_pose": pose_fields(request.predicted_map_T_lidar.pose),
            "raw_pose": pose_fields(result.raw_map_T_lidar.pose),
            "used_pose": pose_fields(result.used_map_T_lidar.pose),
            "fitness": float(result.fitness),
            "iterations": int(result.iterations),
            "converged": int(result.converged),
            "step_limited": int(result.step_limited),
            "cloud_width": int(request.cloud_end_frame.width),
            "cloud_height": int(request.cloud_end_frame.height),
            "cloud_point_step": int(request.cloud_end_frame.point_step),
        }
        records.append(record)
    return sorted(records, key=lambda row: (row["scan_end_ns"], row["transaction_id"]))


def nearest(rows, target_s, predicate=lambda row: True):
    candidates = [row for row in rows if predicate(row)]
    if not candidates:
        raise RuntimeError(f"no candidate scan for target time {target_s:.6f}s")
    return min(candidates, key=lambda row: (abs(row["time_s"] - target_s), row["scan_end_ns"]))


def select_frames(records):
    end_s = max(row["time_s"] for row in records)
    if end_s < 400.0:
        raise RuntimeError(f"runtime topic bag is unexpectedly short: {end_s:.3f}s")
    selected = {}

    def add(row, label):
        identity = row["key"]
        if identity not in selected:
            selected[identity] = {"record": row, "labels": set()}
        selected[identity]["labels"].add(label)

    for segment, lo, hi in SEGMENTS:
        for percent in (0.25, 0.50, 0.75):
            fraction_label = int(percent * 100)
            row = nearest(records, lo + percent * (hi - lo),
                          lambda item: lo <= item["time_s"] < hi)
            add(row, f"UNIFORM:{segment}:{fraction_label}%")
    for percent in (0.25, 0.50, 0.75):
        row = nearest(records, 350.0 + percent * (end_s - 350.0),
                      lambda item: item["time_s"] >= 350.0)
        add(row, f"UNIFORM:350-end:{int(percent * 100)}%")

    for name, crossing_s in CROSSINGS:
        before = nearest(records, crossing_s,
                         lambda item: item["time_s"] <= crossing_s)
        after = nearest(records, crossing_s,
                        lambda item: item["time_s"] >= crossing_s)
        add(before, f"TARGETED:{name}:before")
        add(after, f"TARGETED:{name}:after")

    baseline_targets = (("0-50", 0.25), ("0-50", 0.75),
                        ("150-200", 0.25), ("150-200", 0.75))
    for segment, fraction in baseline_targets:
        lo, hi = next((lo, hi) for name, lo, hi in SEGMENTS if name == segment)
        row = nearest(records, lo + fraction * (hi - lo),
                      lambda item: lo <= item["time_s"] < hi)
        add(row, f"BASELINE_GATE:{segment}:{int(fraction * 100)}%")
    row = nearest(records, 350.0 + 0.50 * (end_s - 350.0),
                  lambda item: item["time_s"] >= 350.0)
    add(row, "BASELINE_GATE:350-end:50%")

    frame_rows = []
    for index, entry in enumerate(sorted(selected.values(),
                                         key=lambda item: item["record"]["scan_end_ns"]), 1):
        record = entry["record"]
        labels = sorted(entry["labels"])
        segments = [label.split(":")[1] for label in labels if label.startswith("UNIFORM:")]
        row = {key: value for key, value in record.items() if key not in {"key", "predicted_pose", "raw_pose", "used_pose"}}
        row.update({
            "frame_id": f"F{index:03d}",
            "cohorts": ";".join(sorted({label.split(":")[0] for label in labels})),
            "selection_labels": ";".join(labels),
            "segment": segments[0] if segments else segment_name(record["time_s"]),
            "baseline_reproduction": int(any(label.startswith("BASELINE_GATE:") for label in labels)),
            "predicted_pose_xyz_q_xyzw": ";".join(map(str, record["predicted_pose"])),
            "saved_raw_pose_xyz_q_xyzw": ";".join(map(str, record["raw_pose"])),
            "saved_used_pose_xyz_q_xyzw": ";".join(map(str, record["used_pose"])),
        })
        frame_rows.append(row)
    return frame_rows


def segment_name(time_s):
    for name, lo, hi in SEGMENTS:
        if lo <= time_s < hi:
            return name
    return "350-end" if time_s >= 350.0 else "outside"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args()
    bag_sha, map_sha = sha256(args.bag), sha256(args.map)
    if bag_sha != EXPECTED_BAG_SHA or map_sha != EXPECTED_MAP_SHA:
        raise RuntimeError(f"input SHA mismatch: bag={bag_sha}, map={map_sha}")
    records = frozen_records(args.bag)
    rows = select_frames(records)
    for row in rows:
        row["input_bag_sha256"] = bag_sha
        row["input_map_sha256"] = map_sha
    fields = list(rows[0].keys())
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    with args.manifest.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    print(f"BAG_SHA256={bag_sha}")
    print(f"MAP_SHA256={map_sha}")
    print(f"NDT_REQUESTS={len(records)}")
    print(f"EVAL_TIME_RANGE={min(row['time_s'] for row in records):.9f},{max(row['time_s'] for row in records):.9f}")
    print(f"SELECTED_UNIQUE_FRAMES={len(rows)}")
    print(f"BASELINE_GATE_FRAMES={sum(row['baseline_reproduction'] for row in rows)}")
    print(f"MANIFEST={args.manifest}")


if __name__ == "__main__":
    main()
