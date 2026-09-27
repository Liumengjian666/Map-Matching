#!/usr/bin/env python3
"""Freeze P5-I2 diagnostic cohorts, then apply the frozen P3-R10C GT alignment."""

import argparse
import csv
import hashlib
import math
from decimal import Decimal
from pathlib import Path

import numpy as np
import rosbag
import yaml
from scipy.spatial.transform import Rotation, Slerp


ROOT = Path(__file__).resolve().parents[1]
P5_I1 = ROOT / "docs/p5_i1_ndt_mode_landscape"
DEFAULT_BAG = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/"
    "p3_r10b_fix1_floor01_full_rerun_20260926/floor01_fix1_runtime_topics.bag"
)
DEFAULT_MAP = Path("/tmp/floor01_candidates/floor01_h1_map.pcd")
GT_PATH = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/gt/floor01_gt.txt")
EXTRINSICS_PATH = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_extrinsics.yaml"
)
CORRECTED_PATH = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/"
    "p3_r10b_fix1_floor01_full_rerun_20260926/evaluation_inputs/corrected.csv"
)
EXPECTED = {
    "bag": "860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db",
    "map": "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570",
    "gt": "b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f",
    "extrinsics": "fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414",
    "corrected": "ff61f3fc72ec2b0c4c9e7a99f54e0866d696bb8999cf1f7a16001cacd3a26416",
}
EVAL_ORIGIN_NS = 1660857393197807074
EVAL_START = Decimal("1660857393.197807")
REQUEST_TOPIC = "/dog_livo/ndt/scan_request"
RESULT_TOPIC = "/dog_livo/ndt/scan_result"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def pose_from_ros(pose):
    p, q = pose.position, pose.orientation
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    out[:3, 3] = [p.x, p.y, p.z]
    return out


def pose_from_text(text):
    values = np.asarray([float(value) for value in text.split(";")])
    if values.size == 16:
        return values.reshape(4, 4)
    if values.size != 7:
        raise RuntimeError(f"pose requires 7 or 16 values, got {values.size}")
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    out[:3, 3] = values[:3]
    return out


def pose_text(pose):
    q = Rotation.from_matrix(pose[:3, :3]).as_quat()
    values = [*pose[:3, 3], *q]
    return ";".join(format(float(value), ".16g") for value in values)


def matrix_text(pose):
    return ";".join(format(float(value), ".16g") for value in pose.reshape(-1))


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
            if target is requests:
                request = message
                cloud = request.cloud_end_frame
                if request.map_frame != "floor01_map_h1" or request.lidar_frame != "cmu_sp1_velodyne":
                    raise RuntimeError(f"request frame mismatch at transaction {request.transaction_id}")
                if cloud.header.frame_id != request.lidar_frame:
                    raise RuntimeError(f"cloud frame mismatch at transaction {request.transaction_id}")
                target[item_key] = {
                    "key": item_key,
                    "transaction_id": int(request.transaction_id),
                    "scan_end_ns": int(request.scan_end_ns),
                    "request_cloud_hash": int(request.request_cloud_hash),
                    "predicted_pose": pose_from_ros(request.predicted_map_T_lidar.pose),
                    "cloud_width": int(cloud.width),
                    "cloud_height": int(cloud.height),
                    "cloud_point_step": int(cloud.point_step),
                }
            else:
                result = message
                if result.disposition != result.SUCCESS or not result.pose_valid:
                    raise RuntimeError(f"non-success transaction {result.transaction_id}")
                target[item_key] = {
                    "ndt_source_cloud_hash": int(result.ndt_source_cloud_hash),
                    "fitness_saved": float(result.fitness),
                    "iterations_saved": int(result.iterations),
                    "converged_saved": int(result.converged),
                    "step_limited_saved": int(result.step_limited),
                    "saved_raw_pose": pose_from_ros(result.raw_map_T_lidar.pose),
                    "saved_used_pose": pose_from_ros(result.used_map_T_lidar.pose),
                }
    if len(requests) != 4127 or len(results) != 4127 or requests.keys() != results.keys():
        raise RuntimeError(f"unexpected request/result key counts: {len(requests)}/{len(results)}")
    rows = []
    for item_key, request in requests.items():
        rows.append({**request, **results[item_key]})
    return sorted(rows, key=lambda row: (row["scan_end_ns"], row["transaction_id"]))


def select_frames(records):
    selected = {}

    def add(row, cohort, label):
        entry = selected.setdefault(row["key"], {"record": row, "cohorts": set(), "labels": set()})
        entry["cohorts"].add(cohort)
        entry["labels"].add(label)

    def nearest(target_s, cohort, label):
        row = min(records, key=lambda item: (abs(item["time_s"] - target_s), item["scan_end_ns"]))
        add(row, cohort, f"{label}:target={target_s:.3f}")

    for target_s in range(0, 60, 10):
        nearest(float(target_s), "HEALTHY", "HEALTHY_GRID")
    for target_s in range(80, 181, 2):
        nearest(float(target_s), "FAILURE_ONSET", "FAILURE_GRID")
    for target_s in range(250, 311, 3):
        nearest(float(target_s), "WRONG_SHARP", "WRONG_SHARP_GRID")
    end_s = max(row["time_s"] for row in records)
    for target_s in range(350, int(math.floor(end_s)) + 1, 10):
        nearest(float(target_s), "CATASTROPHIC_LATE", "LATE_GRID")

    p5_i1_frames = read_csv(P5_I1 / "frame_manifest.csv")
    p5_i1_by_id = {row["frame_id"]: row for row in p5_i1_frames}
    p5_i1_required = ("F001", "F003", "F026")
    for frame_id in p5_i1_required:
        prior = p5_i1_by_id.get(frame_id)
        if prior is None:
            raise RuntimeError(f"P5-I1 reference frame missing: {frame_id}")
        row = next((item for item in records if item["transaction_id"] == int(prior["transaction_id"])), None)
        if row is None or abs(row["time_s"] - float(prior["time_s"])) > 1e-6:
            raise RuntimeError(f"P5-I1 transaction/time closure failed for {frame_id}")
        cohort = "WRONG_SHARP" if frame_id == "F026" else "HEALTHY"
        purpose = "P5I1_WRONG_SHARP_REFERENCE" if frame_id == "F026" else "P5I1_MULTIMODE_REFERENCE"
        add(row, cohort, f"{purpose}:{frame_id}")

    ordered = sorted(selected.values(), key=lambda item: (item["record"]["scan_end_ns"], item["record"]["transaction_id"]))
    output = []
    for index, entry in enumerate(ordered, 1):
        row = entry["record"]
        output.append({
            "frame_id": f"P2F{index:03d}",
            "transaction_id": row["transaction_id"],
            "scan_end_ns": row["scan_end_ns"],
            "time_s": row["time_s"],
            "cohorts": ";".join(sorted(entry["cohorts"])),
            "selection_labels": ";".join(sorted(entry["labels"])),
            "segment": segment_name(row["time_s"]),
            "request_cloud_hash": row["request_cloud_hash"],
            "ndt_source_cloud_hash": row["ndt_source_cloud_hash"],
            "fitness_saved": row["fitness_saved"],
            "iterations_saved": row["iterations_saved"],
            "converged_saved": row["converged_saved"],
            "step_limited_saved": row["step_limited_saved"],
            "cloud_width": row["cloud_width"],
            "cloud_height": row["cloud_height"],
            "cloud_point_step": row["cloud_point_step"],
            "predicted_pose_xyz_q_xyzw": pose_text(row["predicted_pose"]),
            "saved_raw_pose_xyz_q_xyzw": pose_text(row["saved_raw_pose"]),
            "saved_used_pose_xyz_q_xyzw": pose_text(row["saved_used_pose"]),
        })
    return output


def segment_name(time_s):
    for name, lo, hi in (("0-50", 0, 50), ("50-100", 50, 100), ("100-150", 100, 150),
                         ("150-200", 150, 200), ("200-250", 200, 250), ("250-300", 250, 300),
                         ("300-350", 300, 350)):
        if lo <= time_s < hi:
            return name
    return "350-end" if time_s >= 350 else "outside"


def gt_data(path):
    arr = np.loadtxt(path, comments="#", ndmin=2)
    times = arr[:, 0]
    poses = []
    for row in arr:
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_quat(row[4:8]).as_matrix()
        pose[:3, 3] = row[1:4]
        poses.append(pose)
    if not np.all(np.diff(times) > 0):
        raise RuntimeError("GT timestamps are not strictly increasing")
    return times, poses


def interpolate_gt(times, poses, stamp):
    if stamp < times[0] or stamp > times[-1]:
        return None
    hi = int(np.searchsorted(times, stamp, side="right"))
    if hi == 0:
        return poses[0].copy()
    if hi >= len(times):
        return poses[-1].copy()
    lo = hi - 1
    u = (stamp - times[lo]) / (times[hi] - times[lo])
    out = np.eye(4)
    out[:3, 3] = poses[lo][:3, 3] + u * (poses[hi][:3, 3] - poses[lo][:3, 3])
    q0 = Rotation.from_matrix(poses[lo][:3, :3]).as_quat()
    q1 = Rotation.from_matrix(poses[hi][:3, :3]).as_quat()
    out[:3, :3] = Slerp([0.0, 1.0], Rotation.from_quat([q0, q1]))([u]).as_matrix()[0]
    return out


def pose_error(est, reference):
    residual = np.linalg.inv(reference) @ est
    return float(np.linalg.norm(residual[:3, 3])), math.degrees(
        float(Rotation.from_matrix(residual[:3, :3]).magnitude()))


def apply_frozen_gt_alignment(rows):
    gt_sha, extr_sha, corrected_sha = sha256(GT_PATH), sha256(EXTRINSICS_PATH), sha256(CORRECTED_PATH)
    if (gt_sha != EXPECTED["gt"] or extr_sha != EXPECTED["extrinsics"]
            or corrected_sha != EXPECTED["corrected"]):
        raise RuntimeError(
            "frozen GT/extrinsics/corrected-anchor input SHA mismatch: "
            f"{gt_sha} / {extr_sha} / {corrected_sha}"
        )
    with EXTRINSICS_PATH.open() as stream:
        extrinsics = yaml.safe_load(stream)
    t_imu_lidar = np.asarray(extrinsics["laser_to_imu"]["data"], dtype=float).reshape(4, 4)
    t_lidar_imu = np.linalg.inv(t_imu_lidar)
    times, gt_poses = gt_data(GT_PATH)
    corrected = read_csv(CORRECTED_PATH)
    anchor_row = next((row for row in corrected if Decimal(row["lidar_header_stamp"]) >= EVAL_START), None)
    if anchor_row is None:
        raise RuntimeError("missing P3-R10C first common corrected pose")
    anchor_stamp = float(anchor_row["lidar_header_stamp"])
    gt_anchor = interpolate_gt(times, gt_poses, anchor_stamp)
    if gt_anchor is None:
        raise RuntimeError("official GT does not cover fixed alignment anchor")
    corrected_lidar = pose_from_text(";".join(anchor_row[key] for key in (
        "final_used_tx", "final_used_ty", "final_used_tz", "final_used_qx",
        "final_used_qy", "final_used_qz", "final_used_qw")))
    corrected_imu = corrected_lidar @ t_lidar_imu
    alignment = corrected_imu @ np.linalg.inv(gt_anchor)

    for row in rows:
        gt_imu_raw = interpolate_gt(times, gt_poses, int(row["scan_end_ns"]) / 1e9)
        if gt_imu_raw is None:
            raise RuntimeError(f"GT does not cover selected frame {row['frame_id']}")
        gt_imu = alignment @ gt_imu_raw
        gt_lidar = gt_imu @ t_imu_lidar
        row.update({
            "gt_aligned_map_T_imu_matrix16": matrix_text(gt_imu),
            "gt_map_T_lidar_xyz_q_xyzw": pose_text(gt_lidar),
            "gt_convention": "IMU-origin GT; fixed first-common P3-R10C left alignment; T_map_lidar=T_map_imu*T_imu_lidar",
            "gt_sha256": gt_sha,
            "extrinsics_sha256": extr_sha,
            "corrected_anchor_sha256": corrected_sha,
            "alignment_anchor_time": anchor_row["lidar_header_stamp"],
            "alignment_anchor_map_T_imu_matrix16": matrix_text(alignment),
        })
    return rows, t_imu_lidar, t_lidar_imu, (times, gt_poses, alignment, gt_sha, extr_sha, corrected_sha)


def write_csv(path, rows):
    if not rows:
        raise RuntimeError(f"refusing to write empty CSV {path}")
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def convention_sanity(rows, t_lidar_imu, gt_context, output_path):
    times, poses, alignment, _, _, _ = gt_context
    prior_manifest = read_csv(P5_I1 / "frame_manifest.csv")
    prior_gt = read_csv(P5_I1 / "gt_posthoc_mode_comparison.csv")
    baseline_ids = {row["frame_id"] for row in read_csv(P5_I1 / "baseline_reproduction.csv")}
    p1_manifest = {row["frame_id"]: row for row in prior_manifest}
    p1_gt = {}
    for row in prior_gt:
        if row["frame_id"] in baseline_ids:
            p1_gt.setdefault(row["frame_id"], row)
    results = []
    for frame_id in sorted(baseline_ids):
        old, reference = p1_manifest[frame_id], p1_gt[frame_id]
        stamp = int(old["scan_end_ns"]) / 1e9
        gt_imu = alignment @ interpolate_gt(times, poses, stamp)
        raw_lidar = pose_from_text(old["saved_raw_pose_xyz_q_xyzw"])
        raw_imu = raw_lidar @ t_lidar_imu
        t_error, r_error = pose_error(raw_imu, gt_imu)
        dt = abs(t_error - float(reference["baseline_raw_translation_gt_error_m"]))
        dr = abs(r_error - float(reference["baseline_raw_rotation_gt_error_deg"]))
        results.append({
            "frame_id": frame_id,
            "time_s": old["time_s"],
            "translation_error_recomputed_imu_origin_m": t_error,
            "translation_error_p5_i1_reference_m": reference["baseline_raw_translation_gt_error_m"],
            "translation_abs_delta_m": dt,
            "rotation_error_recomputed_imu_origin_deg": r_error,
            "rotation_error_p5_i1_reference_deg": reference["baseline_raw_rotation_gt_error_deg"],
            "rotation_abs_delta_deg": dr,
        })
    max_t = max(row["translation_abs_delta_m"] for row in results)
    max_r = max(row["rotation_abs_delta_deg"] for row in results)
    if max_t > 1e-6 or max_r > 1e-5:
        raise RuntimeError(f"GT convention sanity mismatch: max_t={max_t} max_r={max_r}")
    write_csv(output_path, results)
    return max_t, max_r


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    bag_sha, map_sha = sha256(args.bag), sha256(args.map)
    if bag_sha != EXPECTED["bag"] or map_sha != EXPECTED["map"]:
        raise RuntimeError(f"fixed input SHA mismatch: bag={bag_sha}, map={map_sha}")

    records = frozen_records(args.bag)
    for row in records:
        row["time_s"] = (row["scan_end_ns"] - EVAL_ORIGIN_NS) / 1e9
    rows = select_frames(records)
    # Freeze cohort membership before loading or interpolating GT.
    print(f"COHORTS_FROZEN={len(rows)}")
    rows, t_imu_lidar, t_lidar_imu, gt_context = apply_frozen_gt_alignment(rows)
    for row in rows:
        row["input_bag_sha256"] = bag_sha
        row["input_map_sha256"] = map_sha
        row["T_imu_lidar_matrix16"] = matrix_text(t_imu_lidar)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "frame_manifest.csv", rows)
    max_t, max_r = convention_sanity(rows, t_lidar_imu, gt_context,
                                     args.output_dir / "gt_convention_sanity.csv")
    cohort_counts = {}
    for row in rows:
        for cohort in row["cohorts"].split(";"):
            cohort_counts[cohort] = cohort_counts.get(cohort, 0) + 1
    print(f"BAG_SHA256={bag_sha}")
    print(f"MAP_SHA256={map_sha}")
    print(f"GT_SHA256={gt_context[3]}")
    print(f"EXTRINSICS_SHA256={gt_context[4]}")
    print(f"SELECTED_UNIQUE_FRAMES={len(rows)}")
    print(f"COHORT_COUNTS={cohort_counts}")
    print(f"GT_CONVENTION_SANITY_FRAMES=5 MAX_T_DELTA={max_t:.3e} MAX_R_DELTA={max_r:.3e}")
    print(f"FRAME_MANIFEST={args.output_dir / 'frame_manifest.csv'}")


if __name__ == "__main__":
    main()
