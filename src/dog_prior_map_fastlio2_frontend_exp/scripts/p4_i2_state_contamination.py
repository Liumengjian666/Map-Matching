#!/usr/bin/env python3
"""Offline NDT-to-IKFoM state-contamination ablation for Floor01.

This analysis reads captured ROS bag messages directly; it never plays the
bag and never starts or changes a localization runtime.
"""

import argparse
import csv
import hashlib
import math
import subprocess
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import yaml
from scipy.spatial.transform import Rotation, Slerp
from scipy.stats import pearsonr, spearmanr

WORKSPACE = Path("/home/jian/livox_ws/dog_loc_paper_ws")
PACKAGE = WORKSPACE / "src/dog_prior_map_fastlio2_frontend_exp"
RESULT_ROOT = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/"
    "p3_r10b_fix1_floor01_full_rerun_20260926"
)
BAG = RESULT_ROOT / "floor01_fix1_runtime_topics.bag"
CONFIG = RESULT_ROOT / "floor01_superloc_smoke.yaml"
GT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/gt/floor01_gt.txt")
EXTRINSICS = Path(
    "/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_extrinsics.yaml"
)
EXPECTED_SHA256 = {
    "bag": "860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db",
    "config": "4e9584a4c1d5c2ada963700892880cdf2a7f4e75e43f0ff258b5fd4272af7d77",
    "gt": "b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f",
    "extrinsics": "fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414",
}
EXPECTED_START_SHA = "b89fe2ee843c664a7a721b510aa7e58044a83500"
RUNTIME_TOPIC_ROWS = 4127
INITIAL_IMU_SAMPLES = 200
TRANSLATION_REPLAY_TOL_M = 0.005
ROTATION_REPLAY_TOL_DEG = 0.05
FRONTEND_ROOT = Path("/media/jian/HIKVISION/comparison algorithm/FAST_LIO2")
EXPECTED_FASTLIO_COMMIT = "7cc4175de6f8ba2edf34bab02a42195b141027e9"
EVAL_START = 1660857393.197807074
OUTPUT_DIR = PACKAGE / "docs/p4_i2_state_contamination"
ERROR_BINS = [0.5, 1.0, 2.0, 5.0, 10.0]
COMPILED_FRONTEND_SOURCES = (
    PACKAGE / "src/fastlio2_frontend_ikfom.cpp",
    PACKAGE / "include/dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp",
    PACKAGE / "include/dog_prior_map_fastlio2_frontend_exp/frontend_types.hpp",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def check_workspace_baseline():
    branch = subprocess.run(
        ["git", "-C", str(WORKSPACE), "branch", "--show-current"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    head = subprocess.run(
        ["git", "-C", str(WORKSPACE), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if branch != "paper" or head != EXPECTED_START_SHA:
        raise RuntimeError(
            f"paper workspace baseline mismatch: branch={branch!r}, HEAD={head}; "
            f"expected branch='paper', HEAD={EXPECTED_START_SHA}"
        )
    source_status = subprocess.run(
        [
            "git",
            "-C",
            str(WORKSPACE),
            "status",
            "--short",
            "--",
            *[str(path.relative_to(WORKSPACE)) for path in COMPILED_FRONTEND_SOURCES],
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if source_status:
        raise RuntimeError(
            "compiled paper frontend sources are dirty; refusing replay: "
            f"{source_status}"
        )
    print(
        f"paper_workspace_branch={branch} head={head} "
        "compiled_frontend_sources_clean=YES"
    )


def pose_message_to_array(message):
    pose = message.pose if hasattr(message, "pose") else message
    if hasattr(pose, "pose"):
        pose = pose.pose
    position = pose.position
    orientation = pose.orientation
    x, y, z = float(position.x), float(position.y), float(position.z)
    qx, qy, qz, qw = (
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    qnorm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not all(math.isfinite(v) for v in (x, y, z, qx, qy, qz, qw)) or qnorm < 1e-12:
        raise RuntimeError("nonfinite or zero quaternion in recorded pose")
    return (x, y, z, qx / qnorm, qy / qnorm, qz / qnorm, qw / qnorm)


def write_pose_row(stream, pose):
    stream.write(",".join(format(value, ".17g") for value in pose))


def extract_runtime_inputs(imu_path: Path, scan_path: Path):
    imu_count = 0
    previous_imu_stamp = 0
    requests = {}
    results = {}
    odometry = {}
    with rosbag.Bag(str(BAG), "r") as bag, imu_path.open("w") as imu_stream:
        imu_writer = csv.writer(imu_stream, lineterminator="\n")
        imu_writer.writerow(["stamp_ns", "ax", "ay", "az", "gx", "gy", "gz"])
        topics = [
            "/input/imu",
            "/dog_livo/ndt/scan_request",
            "/dog_livo/ndt/scan_result",
            "/dog_livo/fastlio2_ndt_odom",
        ]
        for topic, message, _bag_stamp in bag.read_messages(topics=topics):
            if topic == "/input/imu":
                stamp = int(message.header.stamp.to_nsec())
                if stamp <= previous_imu_stamp:
                    raise RuntimeError(
                        "captured IMU header timestamps are not strictly increasing"
                    )
                previous_imu_stamp = stamp
                imu_count += 1
                imu_writer.writerow(
                    [
                        stamp,
                        format(float(message.linear_acceleration.x), ".17g"),
                        format(float(message.linear_acceleration.y), ".17g"),
                        format(float(message.linear_acceleration.z), ".17g"),
                        format(float(message.angular_velocity.x), ".17g"),
                        format(float(message.angular_velocity.y), ".17g"),
                        format(float(message.angular_velocity.z), ".17g"),
                    ]
                )
            elif topic == "/dog_livo/ndt/scan_request":
                requests[int(message.transaction_id)] = (
                    int(message.header.stamp.to_nsec()),
                    int(message.scan_end_ns),
                    pose_message_to_array(message.predicted_map_T_lidar),
                )
            elif topic == "/dog_livo/ndt/scan_result":
                results[int(message.transaction_id)] = (
                    int(message.header.stamp.to_nsec()),
                    int(message.scan_end_ns),
                    int(message.disposition),
                    bool(message.pose_valid),
                    pose_message_to_array(message.used_map_T_lidar),
                )
            else:
                odom_stamp = int(message.header.stamp.to_nsec())
                odometry[odom_stamp] = pose_message_to_array(message)

    if imu_count != 83342:
        raise RuntimeError(f"runtime topic bag IMU count {imu_count}, expected 83342")
    if not (len(requests) == len(results) == len(odometry) == RUNTIME_TOPIC_ROWS):
        raise RuntimeError(
            "runtime topic counts mismatch: "
            f"requests={len(requests)} results={len(results)} odometry={len(odometry)}"
        )
    if sorted(requests) != list(range(1, RUNTIME_TOPIC_ROWS + 1)):
        raise RuntimeError("NDT transaction IDs are not contiguous 1..4127")

    with scan_path.open("w") as scan_stream:
        scan_stream.write(
            "transaction_id,stamp_ns,predictor_tx,predictor_ty,predictor_tz,predictor_qx,predictor_qy,predictor_qz,predictor_qw,"
            "used_tx,used_ty,used_tz,used_qx,used_qy,used_qz,used_qw,"
            "corrected_tx,corrected_ty,corrected_tz,corrected_qx,corrected_qy,corrected_qz,corrected_qw\n"
        )
        previous_stamp = 0
        for transaction_id in range(1, RUNTIME_TOPIC_ROWS + 1):
            request_stamp, request_scan_end, predictor = requests[transaction_id]
            result_stamp, result_scan_end, disposition, pose_valid, used = results[
                transaction_id
            ]
            stamp = request_stamp
            if stamp <= previous_stamp:
                raise RuntimeError("request timestamps are not strictly increasing")
            previous_stamp = stamp
            corrected = odometry.get(stamp)
            if corrected is None:
                raise RuntimeError(
                    f"missing corrected odometry at transaction {transaction_id}"
                )
            if (
                request_scan_end != stamp
                or result_stamp != stamp
                or result_scan_end != stamp
            ):
                raise RuntimeError(
                    f"transaction timestamp disagreement at {transaction_id}"
                )
            if disposition != 0 or not pose_valid:
                raise RuntimeError(f"R10B transaction {transaction_id} is not SUCCESS")
            scan_stream.write(f"{transaction_id},{stamp}")
            for pose in (predictor, used, corrected):
                scan_stream.write(",")
                write_pose_row(scan_stream, pose)
            scan_stream.write("\n")
    return imu_count, len(requests)


def write_parameters(path: Path):
    with CONFIG.open() as stream:
        config = yaml.safe_load(stream)
    frontend = config["dog_prior_map_fastlio2_frontend"]
    imu = frontend["imu"]
    update = frontend["update"]
    pose = frontend["initial_map_T_lidar"]
    extrinsic = frontend["T_imu_lidar"]
    values = [
        int(imu["static_init_samples"]),
        float(imu["gravity_mps2"]),
        *[float(v) for v in imu.get("initial_accel_bias", [0.0, 0.0, 0.0])],
        float(imu["gyro_noise_std_rad_s"]),
        float(imu["accel_noise_std_m_s2"]),
        float(imu["gyro_bias_rw_std_rad_s2"]),
        float(imu["accel_bias_rw_std_m_s3"]),
        float(update["pose_position_sigma_m"]),
        float(update["pose_rotation_sigma_rad"]),
        float(imu["max_static_accel_std_m_s2"]),
        float(imu["max_static_gyro_std_rad_s"]),
    ]
    for source in (pose, extrinsic):
        values.extend(
            float(source[key]) for key in ("x", "y", "z", "qx", "qy", "qz", "qw")
        )
    if len(values) != 27 or int(values[0]) != INITIAL_IMU_SAMPLES:
        raise RuntimeError(
            "R10B runtime parameters do not match expected static initialization"
        )
    path.write_text(" ".join(format(value, ".17g") for value in values) + "\n")


def compile_replay(executable: Path):
    source = PACKAGE / "scripts/p4_i2_state_contamination_replay.cpp"
    command = [
        "g++",
        "-std=c++14",
        "-O2",
        "-Wall",
        "-Wextra",
        "-Wpedantic",
        "-fopenmp",
        f"-I{PACKAGE / 'include'}",
        f"-I{FRONTEND_ROOT / 'include'}",
        "-I/usr/include/eigen3",
        str(source),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True)
    return command


def pose_matrix(record, prefix):
    from scipy.spatial.transform import Rotation

    translation = np.array([float(record[f"{prefix}_t{axis}"]) for axis in "xyz"])
    quaternion = np.array([float(record[f"{prefix}_q{axis}"]) for axis in "xyzw"])
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = translation
    return matrix


def matrix_error(a, b):
    delta = np.linalg.inv(a) @ b
    translation = float(np.linalg.norm(delta[:3, 3]))
    cosine = max(-1.0, min(1.0, (float(np.trace(delta[:3, :3])) - 1.0) * 0.5))
    rotation_deg = math.degrees(math.acos(cosine))
    return translation, rotation_deg


def check_artifact_hashes():
    for name, path in (
        ("bag", BAG),
        ("config", CONFIG),
        ("gt", GT),
        ("extrinsics", EXTRINSICS),
    ):
        actual = sha256(path)
        expected = EXPECTED_SHA256[name]
        if actual != expected:
            raise RuntimeError(f"{name} SHA-256 mismatch: {actual} != {expected}")
        print(f"{name}_sha256={actual}")
    source_head = subprocess.run(
        ["git", "-C", str(FRONTEND_ROOT), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    source_status = subprocess.run(
        ["git", "-C", str(FRONTEND_ROOT), "status", "--short"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if source_head != EXPECTED_FASTLIO_COMMIT or source_status:
        raise RuntimeError(
            f"FAST-LIO2 source is not the clean pinned snapshot: HEAD={source_head}; "
            f"status={source_status!r}"
        )
    print(f"fastlio2_source_head={source_head} worktree_clean=YES")


def run_replay(executable, mode, imu_path, scan_path, params_path, replay_path):
    subprocess.run(
        [
            str(executable),
            mode,
            str(imu_path),
            str(scan_path),
            str(params_path),
            str(replay_path),
        ],
        check=True,
    )
    with replay_path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def run_full_update_gate(replay_rows, scan_path):
    with scan_path.open(newline="") as stream:
        saved_rows = list(csv.DictReader(stream))
    if len(replay_rows) != RUNTIME_TOPIC_ROWS or len(saved_rows) != len(replay_rows):
        raise RuntimeError("FULL_UPDATE replay frame count mismatch")

    corrected_translation = []
    corrected_rotation = []
    predictor_translation = []
    predictor_rotation = []
    for replay, saved in zip(replay_rows, saved_rows):
        if replay["stamp_ns"] != saved["stamp_ns"]:
            raise RuntimeError("FULL_UPDATE timestamp alignment mismatch")
        for estimate_prefix, reference_prefix, t_out, r_out in (
            (
                "corrected_imu",
                "saved_corrected_lidar",
                corrected_translation,
                corrected_rotation,
            ),
            (
                "predictor_imu",
                "saved_predictor_lidar",
                predictor_translation,
                predictor_rotation,
            ),
        ):
            translation, rotation = matrix_error(
                pose_matrix(replay, estimate_prefix),
                pose_matrix(replay, reference_prefix),
            )
            t_out.append(translation)
            r_out.append(rotation)

    result = {
        "corrected_translation_max_m": max(corrected_translation),
        "corrected_rotation_max_deg": max(corrected_rotation),
        "predictor_translation_max_m": max(predictor_translation),
        "predictor_rotation_max_deg": max(predictor_rotation),
    }
    print(f"full_update_frames={len(replay_rows)}")
    for name, value in result.items():
        print(f"{name}={value:.12g}")
    if not (
        result["corrected_translation_max_m"] < TRANSLATION_REPLAY_TOL_M
        and result["corrected_rotation_max_deg"] < ROTATION_REPLAY_TOL_DEG
    ):
        raise RuntimeError("BASELINE_REPLAY_GATE_FAIL; B/C ablations were not started")
    print("FULL_UPDATE_BASELINE_REPLAY_GATE_PASS")
    return result


def check_selective_masks(rows, mode):
    def max_vector_delta(prefix, rotation=False):
        suffix = "_rad" if rotation else ""
        return max(
            math.sqrt(
                sum(float(row[f"delta_{prefix}_{axis}{suffix}"]) ** 2 for axis in "xyz")
            )
            for row in rows
        )

    hidden = {
        "gyro_bias": max_vector_delta("gyro_bias"),
        "accel_bias": max_vector_delta("accel_bias"),
        "gravity_deg": max(abs(float(row["gravity_delta_deg"])) for row in rows),
    }
    if mode == "POSE_ONLY_UPDATE":
        hidden["velocity"] = max_vector_delta("velocity")
    if any(value > 1e-12 for value in hidden.values()):
        raise RuntimeError(f"{mode} hidden-state mask failed: {hidden}")
    allowed = max_vector_delta("position") + max_vector_delta("rotation", rotation=True)
    if mode == "POSE_VEL_UPDATE":
        allowed += max_vector_delta("velocity")
    if allowed <= 1e-10:
        raise RuntimeError(f"{mode} made no allowed state update")
    print(f"{mode}_mask_pass hidden_max={hidden}")
    return hidden


def interp_gt(gt_times, gt_poses, stamp):
    if stamp < gt_times[0] or stamp > gt_times[-1]:
        return None
    upper = int(np.searchsorted(gt_times, stamp, side="right"))
    if upper == 0:
        return gt_poses[0].copy()
    if upper >= len(gt_times):
        return gt_poses[-1].copy()
    lower = upper - 1
    fraction = (stamp - gt_times[lower]) / (gt_times[upper] - gt_times[lower])
    result = np.eye(4)
    result[:3, 3] = gt_poses[lower][:3, 3] + fraction * (
        gt_poses[upper][:3, 3] - gt_poses[lower][:3, 3]
    )
    q0 = Rotation.from_matrix(gt_poses[lower][:3, :3]).as_quat()
    q1 = Rotation.from_matrix(gt_poses[upper][:3, :3]).as_quat()
    result[:3, :3] = Slerp([0.0, 1.0], Rotation.from_quat([q0, q1]))(
        [fraction]
    ).as_matrix()[0]
    return result


def rigid_error(estimate, reference):
    residual = np.linalg.inv(reference) @ estimate
    trans = float(np.linalg.norm(residual[:3, 3]))
    angle = float(np.degrees(Rotation.from_matrix(residual[:3, :3]).magnitude()))
    return trans, angle


def summarize(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not values.size:
        return {
            "count": 0,
            "mean": math.nan,
            "rmse": math.nan,
            "median": math.nan,
            "p95": math.nan,
            "max": math.nan,
        }
    return {
        "count": int(values.size),
        "mean": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(values * values))),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def fmt_stats(stats, unit):
    return (
        f"n={stats['count']}, mean={stats['mean']:.6g} {unit}, "
        f"RMSE={stats['rmse']:.6g} {unit}, median={stats['median']:.6g} {unit}, "
        f"P95={stats['p95']:.6g} {unit}, max={stats['max']:.6g} {unit}"
    )


def read_gt(path):
    values = np.loadtxt(path, comments="#", ndmin=2)
    times = values[:, 0]
    if not np.all(np.diff(times) > 0):
        raise RuntimeError("GT timestamps are not strictly increasing")
    poses = []
    for row in values:
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_quat(row[4:8]).as_matrix()
        pose[:3, 3] = row[1:4]
        poses.append(pose)
    return times, poses


def safe_correlations(xs, ys):
    xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
    valid = np.isfinite(xs) & np.isfinite(ys)
    xs, ys = xs[valid], ys[valid]
    if xs.size < 3 or np.ptp(xs) == 0 or np.ptp(ys) == 0:
        return int(xs.size), math.nan, math.nan
    return (
        int(xs.size),
        float(pearsonr(xs, ys).statistic),
        float(spearmanr(xs, ys).statistic),
    )


def vec(row, prefix, rotation=False):
    suffix = "_rad" if rotation else ""
    return np.array([float(row[f"{prefix}_{axis}{suffix}"]) for axis in "xyz"])


def build_analysis(mode_rows, out_dir, replay_gate, selective_masks):
    gt_times, gt_poses = read_gt(GT)
    full_rows = mode_rows["FULL_UPDATE"]
    first_gt = interp_gt(gt_times, gt_poses, int(full_rows[0]["stamp_ns"]) * 1e-9)
    if first_gt is None:
        raise RuntimeError("first corrected sample has no GT coverage")
    full_first = pose_matrix(full_rows[0], "corrected_imu")
    anchor = full_first @ np.linalg.inv(first_gt)

    per_mode = {}
    for mode, rows in mode_rows.items():
        if len(rows) != RUNTIME_TOPIC_ROWS:
            raise RuntimeError(
                f"{mode} has {len(rows)} rows, expected {RUNTIME_TOPIC_ROWS}"
            )
        for index in range(1, len(rows)):
            previous, current = rows[index - 1], rows[index]
            if previous["stamp_ns"] == current["stamp_ns"]:
                raise RuntimeError(f"duplicate timestamp in {mode}")
        records = []
        gt_by_index = {}
        for index, row in enumerate(rows):
            stamp = int(row["stamp_ns"]) * 1e-9
            gt_pose = interp_gt(gt_times, gt_poses, stamp)
            gt_by_index[index] = gt_pose
            corrected = pose_matrix(row, "corrected_imu")
            corrected_error = None
            eval_time = stamp - EVAL_START
            if stamp >= EVAL_START and gt_pose is not None:
                corrected_error = rigid_error(corrected, anchor @ gt_pose)
            records.append(
                {
                    "index": index,
                    "row": row,
                    "stamp": stamp,
                    "time": eval_time,
                    "gt": gt_pose,
                    "corrected": corrected,
                    "corrected_error": corrected_error,
                    "local_error": None,
                }
            )
        for index in range(1, len(records)):
            previous, current = records[index - 1], records[index]
            dt = current["stamp"] - previous["stamp"]
            if dt <= 0 or dt > 0.25:
                raise RuntimeError(f"non-contiguous NDT time interval {dt:.9f}s")
            if (
                previous["gt"] is None
                or current["gt"] is None
                or current["stamp"] < EVAL_START
            ):
                continue
            gt_delta = np.linalg.inv(previous["gt"]) @ current["gt"]
            predictor = pose_matrix(current["row"], "predictor_imu")
            predicted_delta = np.linalg.inv(previous["corrected"]) @ predictor
            current["local_error"] = rigid_error(predicted_delta, gt_delta)
        per_mode[mode] = records

    out_dir.mkdir(parents=True, exist_ok=True)
    comparison_fields = [
        "frame_index",
        "transaction_id",
        "stamp_ns",
        "eval_time_s",
        "mode",
        "predictor_imu_tx",
        "predictor_imu_ty",
        "predictor_imu_tz",
        "predictor_imu_qx",
        "predictor_imu_qy",
        "predictor_imu_qz",
        "predictor_imu_qw",
        "corrected_imu_tx",
        "corrected_imu_ty",
        "corrected_imu_tz",
        "corrected_imu_qx",
        "corrected_imu_qy",
        "corrected_imu_qz",
        "corrected_imu_qw",
        "velocity_x",
        "velocity_y",
        "velocity_z",
        "gyro_bias_x",
        "gyro_bias_y",
        "gyro_bias_z",
        "accel_bias_x",
        "accel_bias_y",
        "accel_bias_z",
        "gravity_x",
        "gravity_y",
        "gravity_z",
        "local_predictor_increment_translation_error_m",
        "local_predictor_increment_rotation_error_deg",
        "corrected_translation_error_m",
        "corrected_rotation_error_deg",
        "ndt_delta_position_x_m",
        "ndt_delta_position_y_m",
        "ndt_delta_position_z_m",
        "ndt_delta_rotation_x_rad",
        "ndt_delta_rotation_y_rad",
        "ndt_delta_rotation_z_rad",
        "ndt_delta_velocity_x_mps",
        "ndt_delta_velocity_y_mps",
        "ndt_delta_velocity_z_mps",
        "ndt_delta_velocity_norm_mps",
        "ndt_delta_gyro_bias_x_radps",
        "ndt_delta_gyro_bias_y_radps",
        "ndt_delta_gyro_bias_z_radps",
        "ndt_delta_gyro_bias_norm_radps",
        "ndt_delta_accel_bias_x_mps2",
        "ndt_delta_accel_bias_y_mps2",
        "ndt_delta_accel_bias_z_mps2",
        "ndt_delta_accel_bias_norm_mps2",
        "ndt_delta_gravity_x_mps2",
        "ndt_delta_gravity_y_mps2",
        "ndt_delta_gravity_z_mps2",
        "ndt_gravity_direction_delta_deg",
    ]
    comparison_rows = []
    for mode, records in per_mode.items():
        for item in records:
            source = item["row"]
            output = {
                "frame_index": source["frame_index"],
                "transaction_id": source["transaction_id"],
                "stamp_ns": source["stamp_ns"],
                "eval_time_s": f"{item['time']:.9f}",
                "mode": mode,
            }
            for prefix, source_prefix in (
                ("predictor_imu", "predictor_imu"),
                ("corrected_imu", "corrected_imu"),
            ):
                for key in ("tx", "ty", "tz", "qx", "qy", "qz", "qw"):
                    output[f"{prefix}_{key}"] = source[f"{source_prefix}_{key}"]
            for target, source_prefix in (
                ("velocity", "velocity_post"),
                ("gyro_bias", "gyro_bias_post"),
                ("accel_bias", "accel_bias_post"),
                ("gravity", "gravity_post"),
            ):
                for axis in "xyz":
                    output[f"{target}_{axis}"] = source[f"{source_prefix}_{axis}"]
            for axis in "xyz":
                output[f"ndt_delta_position_{axis}_m"] = source[
                    f"delta_position_{axis}"
                ]
                output[f"ndt_delta_rotation_{axis}_rad"] = source[
                    f"delta_rotation_{axis}_rad"
                ]
                output[f"ndt_delta_velocity_{axis}_mps"] = source[
                    f"delta_velocity_{axis}"
                ]
                output[f"ndt_delta_gyro_bias_{axis}_radps"] = source[
                    f"delta_gyro_bias_{axis}"
                ]
                output[f"ndt_delta_accel_bias_{axis}_mps2"] = source[
                    f"delta_accel_bias_{axis}"
                ]
                gravity_direct_delta = float(source[f"gravity_post_{axis}"]) - float(
                    source[f"gravity_pre_{axis}"]
                )
                output[f"ndt_delta_gravity_{axis}_mps2"] = (
                    f"{gravity_direct_delta:.12g}"
                )
            output["ndt_delta_velocity_norm_mps"] = (
                f"{np.linalg.norm(vec(source, 'delta_velocity')):.12g}"
            )
            output["ndt_delta_gyro_bias_norm_radps"] = (
                f"{np.linalg.norm(vec(source, 'delta_gyro_bias')):.12g}"
            )
            output["ndt_delta_accel_bias_norm_mps2"] = (
                f"{np.linalg.norm(vec(source, 'delta_accel_bias')):.12g}"
            )
            output["ndt_gravity_direction_delta_deg"] = source["gravity_delta_deg"]
            local = item["local_error"]
            absolute = item["corrected_error"]
            output["local_predictor_increment_translation_error_m"] = (
                "" if local is None else f"{local[0]:.12g}"
            )
            output["local_predictor_increment_rotation_error_deg"] = (
                "" if local is None else f"{local[1]:.12g}"
            )
            output["corrected_translation_error_m"] = (
                "" if absolute is None else f"{absolute[0]:.12g}"
            )
            output["corrected_rotation_error_deg"] = (
                "" if absolute is None else f"{absolute[1]:.12g}"
            )
            comparison_rows.append(output)
    with (out_dir / "mode_comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=comparison_fields, lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(comparison_rows)

    full_analysis = per_mode["FULL_UPDATE"]
    delta_fields = [
        "frame_index",
        "transaction_id",
        "stamp_ns",
        "eval_time_s",
        "corrected_translation_error_m",
        "corrected_error_bin",
        "delta_position_norm_m",
        "delta_position_x",
        "delta_position_y",
        "delta_position_z",
        "delta_rotation_norm_deg",
        "delta_rotation_x_rad",
        "delta_rotation_y_rad",
        "delta_rotation_z_rad",
        "delta_velocity_norm_mps",
        "delta_velocity_x",
        "delta_velocity_y",
        "delta_velocity_z",
        "delta_gyro_bias_norm",
        "delta_gyro_bias_x",
        "delta_gyro_bias_y",
        "delta_gyro_bias_z",
        "delta_accel_bias_norm",
        "delta_accel_bias_x",
        "delta_accel_bias_y",
        "delta_accel_bias_z",
        "delta_gravity_norm_mps2",
        "delta_gravity_x",
        "delta_gravity_y",
        "delta_gravity_z",
        "gravity_direction_delta_deg",
    ]
    delta_rows = []
    for item in full_analysis:
        row = item["row"]
        position = vec(row, "delta_position")
        rotation = vec(row, "delta_rotation", rotation=True)
        velocity = vec(row, "delta_velocity")
        gyro_bias = vec(row, "delta_gyro_bias")
        accel_bias = vec(row, "delta_accel_bias")
        gravity_delta = vec(row, "gravity_post") - vec(row, "gravity_pre")
        error = (
            item["corrected_error"][0]
            if item["corrected_error"] is not None
            else math.nan
        )
        bin_labels = ["<0.5m", "0.5-1m", "1-2m", "2-5m", "5-10m", ">10m"]
        error_bin = (
            bin_labels[int(np.searchsorted(ERROR_BINS, error, side="right"))]
            if math.isfinite(error)
            else "NO_GT"
        )
        delta_rows.append(
            {
                "frame_index": row["frame_index"],
                "transaction_id": row["transaction_id"],
                "stamp_ns": row["stamp_ns"],
                "eval_time_s": f"{item['time']:.9f}",
                "corrected_translation_error_m": ""
                if not math.isfinite(error)
                else f"{error:.12g}",
                "corrected_error_bin": error_bin,
                "delta_position_norm_m": f"{np.linalg.norm(position):.12g}",
                **{
                    f"delta_position_{axis}": f"{position[i]:.12g}"
                    for i, axis in enumerate("xyz")
                },
                "delta_rotation_norm_deg": f"{np.degrees(np.linalg.norm(rotation)):.12g}",
                **{
                    f"delta_rotation_{axis}_rad": f"{rotation[i]:.12g}"
                    for i, axis in enumerate("xyz")
                },
                "delta_velocity_norm_mps": f"{np.linalg.norm(velocity):.12g}",
                **{
                    f"delta_velocity_{axis}": f"{velocity[i]:.12g}"
                    for i, axis in enumerate("xyz")
                },
                "delta_gyro_bias_norm": f"{np.linalg.norm(gyro_bias):.12g}",
                **{
                    f"delta_gyro_bias_{axis}": f"{gyro_bias[i]:.12g}"
                    for i, axis in enumerate("xyz")
                },
                "delta_accel_bias_norm": f"{np.linalg.norm(accel_bias):.12g}",
                **{
                    f"delta_accel_bias_{axis}": f"{accel_bias[i]:.12g}"
                    for i, axis in enumerate("xyz")
                },
                "delta_gravity_norm_mps2": f"{np.linalg.norm(gravity_delta):.12g}",
                **{
                    f"delta_gravity_{axis}": f"{gravity_delta[i]:.12g}"
                    for i, axis in enumerate("xyz")
                },
                "gravity_direction_delta_deg": row["gravity_delta_deg"],
            }
        )
    with (out_dir / "state_update_deltas.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=delta_fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(delta_rows)

    segment_bounds = [
        (0, 50, "0-50s"),
        (50, 100, "50-100s"),
        (100, 150, "100-150s"),
        (150, 200, "150-200s"),
        (200, 250, "200-250s"),
        (250, 300, "250-300s"),
        (300, 350, "300-350s"),
        (350, math.inf, "350s-end"),
    ]
    segment_rows = []
    segment_cache = {}
    for mode, records in per_mode.items():
        for lower, upper, label in segment_bounds:
            local_values = [
                x
                for x in records
                if x["local_error"] is not None and lower <= x["time"] < upper
            ]
            absolute_values = [
                x
                for x in records
                if x["corrected_error"] is not None and lower <= x["time"] < upper
            ]
            lt = summarize([x["local_error"][0] for x in local_values])
            lr = summarize([x["local_error"][1] for x in local_values])
            at = summarize([x["corrected_error"][0] for x in absolute_values])
            ar = summarize([x["corrected_error"][1] for x in absolute_values])
            segment_cache[(mode, label)] = (lt, lr, at, ar)
            segment_rows.append(
                {
                    "mode": mode,
                    "segment": label,
                    "local_increment_n": lt["count"],
                    "local_translation_rmse_m": lt["rmse"],
                    "local_rotation_rmse_deg": lr["rmse"],
                    "corrected_n": at["count"],
                    "corrected_translation_rmse_m": at["rmse"],
                    "corrected_translation_p95_m": at["p95"],
                    "corrected_rotation_rmse_deg": ar["rmse"],
                }
            )
    with (out_dir / "segment_metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(segment_rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(segment_rows)

    global_stats = {}
    for mode, records in per_mode.items():
        local = [x for x in records if x["local_error"] is not None]
        absolute = [x for x in records if x["corrected_error"] is not None]
        global_stats[mode] = {
            "local_t": summarize([x["local_error"][0] for x in local]),
            "local_r": summarize([x["local_error"][1] for x in local]),
            "absolute_t": summarize([x["corrected_error"][0] for x in absolute]),
            "absolute_r": summarize([x["corrected_error"][1] for x in absolute]),
            "post150_local_t": summarize(
                [x["local_error"][0] for x in local if x["time"] >= 150]
            ),
            "post150_local_r": summarize(
                [x["local_error"][1] for x in local if x["time"] >= 150]
            ),
        }

    # Correlate an update at k against predictor increment error over k->k+1.
    correlations = {}
    for prefix in ("velocity", "gyro_bias", "accel_bias"):
        xs, ys = [], []
        for index in range(len(full_analysis) - 1):
            current, following = full_analysis[index], full_analysis[index + 1]
            if following["local_error"] is None:
                continue
            delta = vec(current["row"], f"delta_{prefix}")
            xs.append(float(np.linalg.norm(delta)))
            ys.append(following["local_error"][0])
        correlations[prefix] = safe_correlations(xs, ys)

    bin_stats = {}
    for label in ("<0.5m", "0.5-1m", "1-2m", "2-5m", "5-10m", ">10m"):
        selected = [row for row in delta_rows if row["corrected_error_bin"] == label]
        bin_stats[label] = {
            key: summarize([float(row[column]) for row in selected])
            for key, column in (
                ("delta_v", "delta_velocity_norm_mps"),
                ("delta_bg", "delta_gyro_bias_norm"),
                ("delta_ba", "delta_accel_bias_norm"),
                ("gravity_deg", "gravity_direction_delta_deg"),
            )
        }
    overall_deltas = {
        key: summarize([float(row[column]) for row in delta_rows])
        for key, column in (
            ("delta_v", "delta_velocity_norm_mps"),
            ("delta_bg", "delta_gyro_bias_norm"),
            ("delta_ba", "delta_accel_bias_norm"),
            ("gravity_deg", "gravity_direction_delta_deg"),
        )
    }

    baseline = global_stats["FULL_UPDATE"]
    protection = {}
    for mode in ("POSE_VEL_UPDATE", "POSE_ONLY_UPDATE"):
        candidate = global_stats[mode]
        local_improvement = (
            100.0
            * (
                baseline["post150_local_t"]["rmse"]
                - candidate["post150_local_t"]["rmse"]
            )
            / baseline["post150_local_t"]["rmse"]
        )
        corrected_improvement = (
            100.0
            * (baseline["absolute_t"]["rmse"] - candidate["absolute_t"]["rmse"])
            / baseline["absolute_t"]["rmse"]
        )
        local_rotation_worsening = (
            100.0
            * (
                candidate["post150_local_r"]["rmse"]
                - baseline["post150_local_r"]["rmse"]
            )
            / baseline["post150_local_r"]["rmse"]
        )
        corrected_rotation_worsening = (
            100.0
            * (candidate["absolute_r"]["rmse"] - baseline["absolute_r"]["rmse"])
            / baseline["absolute_r"]["rmse"]
        )
        protection[mode] = {
            "post150_local_translation_improvement_pct": local_improvement,
            "global_corrected_translation_improvement_pct": corrected_improvement,
            "post150_local_rotation_worsening_pct": local_rotation_worsening,
            "global_corrected_rotation_worsening_pct": corrected_rotation_worsening,
        }

    promising = [
        mode
        for mode, values in protection.items()
        if values["post150_local_translation_improvement_pct"] >= 30.0
        and values["global_corrected_translation_improvement_pct"] >= 20.0
        and values["post150_local_rotation_worsening_pct"] <= 10.0
        and values["global_corrected_rotation_worsening_pct"] <= 10.0
    ]
    mechanism_only = [
        mode
        for mode, values in protection.items()
        if values["post150_local_translation_improvement_pct"] >= 30.0
        and values["global_corrected_translation_improvement_pct"] < 20.0
    ]
    if promising:
        verdict = "STATE_PROTECTION_PROMISING"
    elif mechanism_only:
        verdict = "MECHANISM_CONFIRMED_BUT_INSUFFICIENT"
    else:
        verdict = "NOT_PROMISING"

    # Persist four diagnostic plots, with all mode comparisons in each plot.
    colors = {
        "FULL_UPDATE": "#1f77b4",
        "POSE_VEL_UPDATE": "#ff7f0e",
        "POSE_ONLY_UPDATE": "#2ca02c",
    }
    fig, axis = plt.subplots(figsize=(10, 5))
    for mode, records in per_mode.items():
        samples = [x for x in records if x["local_error"] is not None]
        axis.plot(
            [x["time"] for x in samples],
            [x["local_error"][0] for x in samples],
            label=mode,
            color=colors[mode],
            linewidth=0.8,
        )
    axis.set(
        xlabel="Evaluation time (s)",
        ylabel="Local predictor increment translation error (m)",
        title="NDT-to-IKFoM state update ablation",
    )
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(
        out_dir / "time_vs_local_predictor_translation_increment_error.png", dpi=150
    )
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for mode, records in per_mode.items():
        times = [x["time"] for x in records]
        bg = [np.linalg.norm(vec(x["row"], "gyro_bias_post")) for x in records]
        ba = [np.linalg.norm(vec(x["row"], "accel_bias_post")) for x in records]
        axes[0].plot(times, bg, label=mode, color=colors[mode], linewidth=0.8)
        axes[1].plot(times, ba, label=mode, color=colors[mode], linewidth=0.8)
    axes[0].set_ylabel("Gyro bias norm (rad/s)")
    axes[0].legend()
    axes[1].set_ylabel("Accel bias norm (m/s²)")
    axes[1].set_xlabel("Evaluation time (s)")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "time_vs_bias_norms.png", dpi=150)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(10, 5))
    for mode, records in per_mode.items():
        initial_gravity = vec(records[0]["row"], "gravity_pre")
        initial_gravity /= np.linalg.norm(initial_gravity)
        angles = []
        for item in records:
            gravity = vec(item["row"], "gravity_post")
            gravity /= np.linalg.norm(gravity)
            angles.append(
                np.degrees(np.arccos(np.clip(initial_gravity @ gravity, -1.0, 1.0)))
            )
        axis.plot(
            [x["time"] for x in records],
            angles,
            label=mode,
            color=colors[mode],
            linewidth=0.8,
        )
    axis.set(
        xlabel="Evaluation time (s)",
        ylabel="Gravity direction change from initialized direction (deg)",
        title="Gravity direction evolution",
    )
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "time_vs_gravity_direction_difference.png", dpi=150)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(10, 5))
    for mode, records in per_mode.items():
        samples = [x for x in records if x["corrected_error"] is not None]
        axis.plot(
            [x["time"] for x in samples],
            [x["corrected_error"][0] for x in samples],
            label=mode,
            color=colors[mode],
            linewidth=0.8,
        )
    axis.set(
        xlabel="Evaluation time (s)",
        ylabel="Corrected translation deviation (m)",
        title="Post-hoc corrected trajectory deviation",
    )
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "time_vs_corrected_translation_error.png", dpi=150)
    plt.close(fig)

    summary_lines = [
        "# PAPER-P4-I2 — NDT-to-IKFoM State-Contamination Ablation",
        "",
        "Offline counterfactual only. No runtime localization source, config, map, bag, or NDT output was modified; no rosbag playback or NDT rerun was performed. Modes B/C are research ablations, not runtime algorithms or a novelty claim. Correlation is association, not causal proof.",
        "",
        f"- Start HEAD: `{EXPECTED_START_SHA}`; branch: `paper`.",
        f"- Runtime topic bag SHA-256: `{EXPECTED_SHA256['bag']}`.",
        f"- Runtime config SHA-256: `{EXPECTED_SHA256['config']}`.",
        f"- GT SHA-256: `{EXPECTED_SHA256['gt']}`; extrinsics SHA-256: `{EXPECTED_SHA256['extrinsics']}`.",
        f"- Pinned FAST-LIO2 snapshot: `{EXPECTED_FASTLIO_COMMIT}` (clean worktree verified).",
        f"- Captured NDT transactions reused: {RUNTIME_TOPIC_ROWS}; each mode reuses identical `used_map_T_lidar` measurements.",
        f"- Evaluation origin: `{EVAL_START:.9f}`; no GT extrapolation; common absolute samples: {baseline['absolute_t']['count']}; adjacent increments: {baseline['local_t']['count']}.",
        "",
        "## FULL_UPDATE replay gate",
        "",
        f"- Corrected trajectory max translation replay delta: `{replay_gate['corrected_translation_max_m']:.12g} m` (required < 0.005 m).",
        f"- Corrected trajectory max rotation replay delta: `{replay_gate['corrected_rotation_max_deg']:.12g} deg` (required < 0.05 deg).",
        f"- Predictor max translation/rotation replay deltas: `{replay_gate['predictor_translation_max_m']:.12g} m` / `{replay_gate['predictor_rotation_max_deg']:.12g} deg`.",
        "- Gate: `PASS`; B/C were started only after this gate passed.",
        "",
        "## B/C direct-update mask audit",
        "",
        "These are per-NDT-update deltas (measurement pre/post), not full state changes between scans; IMU propagation still evolves protected states.",
    ]
    for mode, values in selective_masks.items():
        summary_lines.append(
            f"- {mode}: {RUNTIME_TOPIC_ROWS} updates; maximum suppressed direct deltas: "
            + ", ".join(f"{key} `{value:.12g}`" for key, value in values.items())
            + "."
        )
    summary_lines += [
        "",
        "## Global metrics",
        "",
        "GT is used post-hoc only. A single fixed left anchor is defined from the first FULL_UPDATE corrected pose and applied to GT for all three modes, preserving their different first corrected states.",
        "",
        "| Mode | Corrected translation | Corrected rotation | Local predictor increment translation | Local predictor increment rotation |",
        "|---|---|---|---|---|",
    ]
    for mode in ("FULL_UPDATE", "POSE_VEL_UPDATE", "POSE_ONLY_UPDATE"):
        item = global_stats[mode]
        summary_lines.append(
            f"| {mode} | {fmt_stats(item['absolute_t'], 'm')} | {fmt_stats(item['absolute_r'], 'deg')} | {fmt_stats(item['local_t'], 'm')} | {fmt_stats(item['local_r'], 'deg')} |"
        )
    summary_lines += [
        "",
        "## 50-second segments",
        "",
        "| Segment | Mode | N local | Local t RMSE (m) | Local r RMSE (deg) | Corrected t RMSE (m) | Corrected r RMSE (deg) |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for _, _, label in segment_bounds:
        for mode in ("FULL_UPDATE", "POSE_VEL_UPDATE", "POSE_ONLY_UPDATE"):
            lt, lr, at, ar = segment_cache[(mode, label)]
            summary_lines.append(
                f"| {label} | {mode} | {lt['count']} | {lt['rmse']:.6g} | {lr['rmse']:.6g} | {at['rmse']:.6g} | {ar['rmse']:.6g} |"
            )
    summary_lines += [
        "",
        "## FULL_UPDATE direct state changes per NDT update",
        "",
        "| Direct delta | Mean | Median | P95 | Max |",
        "|---|---:|---:|---:|---:|",
    ]
    units = {
        "delta_v": "m/s",
        "delta_bg": "rad/s",
        "delta_ba": "m/s²",
        "gravity_deg": "deg",
    }
    for key, stats in overall_deltas.items():
        summary_lines.append(
            f"| {key} ({units[key]}) | {stats['mean']:.8g} | {stats['median']:.8g} | {stats['p95']:.8g} | {stats['max']:.8g} |"
        )
    summary_lines += [
        "",
        "### FULL_UPDATE direct hidden-state deltas by corrected-error bin",
        "",
        "| Corrected translation deviation bin | N | mean/median/P95/max |Δv| (m/s) | mean/median/P95/max |Δbg| (rad/s) | mean/median/P95/max |Δba| (m/s²) | mean/median/P95/max gravity angle (deg) |",
        "|---|---:|---|---|---|---|",
    ]
    for label, values in bin_stats.items():
        count = values["delta_v"]["count"]
        cells = []
        for key in ("delta_v", "delta_bg", "delta_ba", "gravity_deg"):
            stats = values[key]
            cells.append(
                f"{stats['mean']:.5g}/{stats['median']:.5g}/{stats['p95']:.5g}/{stats['max']:.5g}"
            )
        summary_lines.append(f"| {label} | {count} | " + " | ".join(cells) + " |")
    summary_lines += [
        "",
        "## FULL_UPDATE delta vs next predictor-increment translation error",
        "",
        "| N | State delta | Pearson | Spearman |",
        "|---:|---|---:|---:|",
    ]
    for key, (count, pearson, spearman) in correlations.items():
        summary_lines.append(f"| {count} | |Δ{key}| | {pearson:.6g} | {spearman:.6g} |")
    summary_lines += [
        "",
        "## FULL to protected-mode comparisons",
        "",
        "| Mode | >150s local predictor translation RMSE improvement | Global corrected translation RMSE improvement | >150s local rotation RMSE worsening | Global corrected rotation RMSE worsening |",
        "|---|---:|---:|---:|---:|",
    ]
    for mode, values in protection.items():
        summary_lines.append(
            f"| {mode} | {values['post150_local_translation_improvement_pct']:.3f}% | {values['global_corrected_translation_improvement_pct']:.3f}% | {values['post150_local_rotation_worsening_pct']:.3f}% | {values['global_corrected_rotation_worsening_pct']:.3f}% |"
        )
    summary_lines += [
        "",
        "The no-material-rotation-worsening check is operationalized here as no more than 10% worsening in either post-150s local rotation RMSE or global corrected rotation RMSE. This threshold is an explicit analysis convention, not a general SLAM criterion.",
        "",
        f"## Verdict: `{verdict}`",
        "",
        "The verdict applies only to this captured Floor01 sequence and these offline fixed-measurement counterfactuals. Even a positive result establishes neither generality nor causality; runtime integration is not authorized by this experiment.",
        "",
        "## Artifacts",
        "",
        "- `summary.md`",
        "- `state_update_deltas.csv`",
        "- `mode_comparison.csv`",
        "- `segment_metrics.csv`",
        "- `time_vs_local_predictor_translation_increment_error.png`",
        "- `time_vs_bias_norms.png`",
        "- `time_vs_gravity_direction_difference.png`",
        "- `time_vs_corrected_translation_error.png`",
        "",
    ]
    (out_dir / "summary.md").write_text("\n".join(summary_lines))
    print(f"analysis_output={out_dir}")
    print(f"verdict={verdict}")
    for mode, values in protection.items():
        print(
            f"{mode}_post150_local_t_improvement_pct={values['post150_local_translation_improvement_pct']:.3f}"
        )
        print(
            f"{mode}_global_corrected_t_improvement_pct={values['global_corrected_translation_improvement_pct']:.3f}"
        )
    return verdict


def run_task(smoke_selective=False, out_dir=OUTPUT_DIR):
    check_workspace_baseline()
    check_artifact_hashes()
    with tempfile.TemporaryDirectory(prefix="p4_i2_state_contamination_") as directory:
        temporary = Path(directory)
        imu_path = temporary / "imu.csv"
        scan_path = temporary / "scans.csv"
        params_path = temporary / "runtime_params.txt"
        executable = temporary / "p4_i2_state_contamination_replay"
        counts = extract_runtime_inputs(imu_path, scan_path)
        write_parameters(params_path)
        print(f"extracted_imu={counts[0]} transactions={counts[1]}")
        compile_replay(executable)

        full_path = temporary / "full_update.csv"
        full_rows = run_replay(
            executable, "FULL_UPDATE", imu_path, scan_path, params_path, full_path
        )
        replay_gate = run_full_update_gate(full_rows, scan_path)

        scan_for_selective = scan_path
        if smoke_selective:
            truncated = temporary / "first_20_scans.csv"
            with scan_path.open(newline="") as source, truncated.open(
                "w", newline=""
            ) as target:
                for index, line in enumerate(source):
                    if index > 20:
                        break
                    target.write(line)
            scan_for_selective = truncated
        selective_rows = {}
        selective_masks = {}
        for mode in ("POSE_VEL_UPDATE", "POSE_ONLY_UPDATE"):
            output = temporary / f"{mode}.csv"
            selective_rows[mode] = run_replay(
                executable, mode, imu_path, scan_for_selective, params_path, output
            )
            expected_rows = 20 if smoke_selective else RUNTIME_TOPIC_ROWS
            if len(selective_rows[mode]) != expected_rows:
                raise RuntimeError(
                    f"{mode} row count {len(selective_rows[mode])}, expected {expected_rows}"
                )
            selective_masks[mode] = check_selective_masks(selective_rows[mode], mode)
        if smoke_selective:
            print("P4_I2_SELECTIVE_UPDATE_SMOKE_PASS frames_per_mode=20")
            return
        selective_rows["FULL_UPDATE"] = full_rows
        return build_analysis(
            selective_rows, out_dir, replay_gate, selective_masks
        ), replay_gate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--smoke-selective",
        action="store_true",
        help="run the A gate then a 20-frame B/C implementation smoke",
    )
    parser.add_argument("--out", type=Path, default=OUTPUT_DIR)
    args = parser.parse_args()
    run_task(args.smoke_selective, args.out.resolve())


if __name__ == "__main__":
    main()
