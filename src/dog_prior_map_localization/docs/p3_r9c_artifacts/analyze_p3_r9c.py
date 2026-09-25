#!/usr/bin/env python3
"""Reproducible post-hoc P3-R9C comparison; never launches localization."""

import argparse
from bisect import bisect_left
import csv
import hashlib
import importlib.util
import json
import math
import re
import shutil
import sys
from decimal import Decimal
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


HERE = Path(__file__).resolve().parent
evaluator = None

DEFAULT_OUT = Path("/media/jian/HIKVISION/paper rosbag/imu_deskew_experiments/p3_r9c")
GT_ROLE = "floor01_gt"
SHOCK_CENTER = Decimal("1660857533.043056")
SHOCK_HALF_WIDTH = Decimal("1.0")
BROAD_HIGH_DYNAMIC = (Decimal("138"), Decimal("168"))
BOOTSTRAP_SEED = 20260925
BOOTSTRAP_REPLICATES = 10000
TIE_TRANSLATION_M = 1e-6
TIE_ROTATION_DEG = 1e-6


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_evaluator(path):
    evaluator_path = Path(path).resolve()
    spec = importlib.util.spec_from_file_location("p3_r3_reference_evaluator", evaluator_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load evaluator specified by manifest: {evaluator_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def read_manifest(path):
    with open(path, newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    inputs = {row["name"]: row for row in rows if row["kind"] == "input"}
    if len(inputs) != sum(row["kind"] == "input" for row in rows):
        raise ValueError("input manifest has duplicate input names")
    return rows, inputs


def validate_inputs(inputs, include_gt):
    records = []
    for name, row in inputs.items():
        if name == GT_ROLE and not include_gt:
            continue
        path = Path(row["path"])
        if not path.is_file():
            raise FileNotFoundError(f"manifest input missing: {name}: {path}")
        actual_size = path.stat().st_size
        expected_size = row.get("expected_size_bytes", "").strip()
        if expected_size and actual_size != int(expected_size):
            raise ValueError(f"size mismatch for {name}: {actual_size} != {expected_size}")
        actual_sha = sha256(path)
        expected_sha = row["expected_sha256"].strip().lower()
        if expected_sha and actual_sha != expected_sha:
            raise ValueError(f"SHA256 mismatch for {name}: {actual_sha} != {expected_sha}")
        records.append({
            "name": name,
            "path": str(path),
            "size_bytes": actual_size,
            "expected_sha256": expected_sha,
            "actual_sha256": actual_sha,
            "verification": "PASS",
        })
    return records


def write_csv(path, fields, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore", lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def norm_scalar(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return float(value)
    return str(value).strip().strip("\"'")


def parse_ros_scalar(value):
    value = value.strip().strip("\"'")
    if value.lower() in ("true", "false"):
        return value.lower() == "true"
    try:
        return float(value)
    except ValueError:
        return value


def parse_runtime_dump(text):
    result = {}
    for line in text.splitlines():
        match = re.match(r"\s*\*\s+(/[^:]+):\s*(.*?)\s*$", line)
        if match:
            result[match.group(1)] = parse_ros_scalar(match.group(2))
    return result


def values_equal(a, b, tol=1e-12):
    a, b = norm_scalar(a), norm_scalar(b)
    if isinstance(a, bool) or isinstance(b, bool):
        return isinstance(a, bool) and isinstance(b, bool) and a == b
    if isinstance(a, float) and isinstance(b, float):
        return math.isclose(a, b, rel_tol=tol, abs_tol=tol)
    return a == b


def load_rows(path):
    with open(path, newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def decimal_stamp(value):
    return Decimal(value).normalize()


def pose_from_row(row, prefix):
    return evaluator.pose_from_xyz_q(
        [float(row[f"{prefix}_t{axis}"]) for axis in "xyz"],
        [float(row[f"{prefix}_q{axis}"]) for axis in "xyzw"],
    )


def load_run(path, name):
    rows = load_rows(path)
    records = []
    seen = set()
    for row in rows:
        key = decimal_stamp(row["lidar_header_stamp"])
        if key in seen:
            raise ValueError(f"duplicate exact timestamp in {name}: {key}")
        seen.add(key)
        record = {
            "stamp_key": key,
            "stamp": float(key),
            "row": row,
            "raw_lidar": pose_from_row(row, "raw_ndt"),
            "final_lidar": pose_from_row(row, "final_used"),
            "fitness": float(row["ndt_fitness"]),
            "iterations": int(float(row["ndt_iterations"])),
            "converged": int(float(row["ndt_has_converged"])),
            "translation_limited": int(float(row.get("translation_limited", 0) or 0) != 0),
            "rotation_limited": int(float(row.get("rotation_limited", 0) or 0) != 0),
            "cloud_hash": row.get("cloud_hash", ""),
            "point_count_raw": row.get("cloud_size_raw", row.get("point_count_in", "")),
            "point_count_source": row.get("cloud_size_after_filter", ""),
            "initial_guess_source": row.get("initial_guess_source", ""),
            "initial_guess_reason": row.get("initial_guess_reason", ""),
            "local_imu_prior_enabled": row.get("local_imu_prior_enabled", ""),
            "local_imu_prior_used": row.get("local_imu_prior_used", ""),
        }
        records.append(record)
    records.sort(key=lambda r: r["stamp_key"])
    return records


def parameter_audit(inputs, legacy_rows, mature_rows):
    legacy_cfg = yaml.safe_load(Path(inputs["legacy_effective_params"]["path"]).read_text())
    mature_cfg = yaml.safe_load(Path(inputs["mature_profile"]["path"]).read_text())
    runtime_text = Path(inputs["mature_launch_log_runA"]["path"]).read_text(errors="replace")
    runtime = parse_runtime_dump(runtime_text)
    decoder_audit = Path(inputs["r7e_decoder_audit"]["path"]).read_text()
    legacy_params = legacy_cfg["lidar_update"]
    mature_params = mature_cfg["lidar_update"]
    rows = []

    def add(name, old, new, classification, evidence):
        equal = values_equal(old, new)
        status = classification if classification != "SAME" or equal else "UNEXPECTED_DIFFERENCE"
        rows.append({"parameter": name, "legacy": str(old), "mature": str(new),
                     "status": status, "evidence": evidence})

    mappings = [
        ("ndt_resolution", "NDT resolution (m)", 0.8),
        ("ndt_step_size", "NDT step size", 0.08),
        ("ndt_transformation_epsilon", "transformation epsilon", 0.001),
        ("ndt_max_iterations", "maximum iterations", 40),
        ("ndt_source_voxel_size", "source voxel XY (m)", 0.25),
        ("ndt_source_voxel_z_size", "source voxel Z (m)", 0.25),
        ("ndt_target_voxel_size", "target voxel XY (m)", 0.15),
        ("ndt_target_voxel_z_size", "target voxel Z (m)", 0.15),
        ("ndt_max_source_points", "maximum source points", 1400),
        ("ndt_max_target_points", "maximum target points", 0),
        ("ndt_step_limit_enable", "step limiter enabled", True),
        ("ndt_step_limit_max_translation", "translation step limit (m)", 0.5),
        ("ndt_step_limit_max_rotation_deg", "rotation step limit (deg)", 5.0),
        ("scan_reference_time", "scan reference", "start"),
        ("offset_time_scale", "point-offset scale", 1e-9),
        ("local_imu_rotation_prior_enable", "local IMU rotation prior", True),
    ]
    runtime_key = {
        "ndt_resolution": "/lidar_update/ndt_resolution",
        "ndt_step_size": "/lidar_update/ndt_step_size",
        "ndt_transformation_epsilon": "/lidar_update/ndt_transformation_epsilon",
        "ndt_max_iterations": "/lidar_update/ndt_max_iterations",
        "ndt_source_voxel_size": "/lidar_update/ndt_source_voxel_size",
        "ndt_source_voxel_z_size": "/lidar_update/ndt_source_voxel_z_size",
        "ndt_target_voxel_size": "/lidar_update/ndt_target_voxel_size",
        "ndt_target_voxel_z_size": "/lidar_update/ndt_target_voxel_z_size",
        "ndt_max_source_points": "/lidar_update/ndt_max_source_points",
        "ndt_max_target_points": "/lidar_update/ndt_max_target_points",
        "ndt_step_limit_enable": "/lidar_update/ndt_step_limit_enable",
        "ndt_step_limit_max_translation": "/lidar_update/ndt_step_limit_max_translation",
        "ndt_step_limit_max_rotation_deg": "/lidar_update/ndt_step_limit_max_rotation_deg",
        "scan_reference_time": "/lidar_update/scan_reference_time",
        "offset_time_scale": "/lidar_update/offset_time_scale",
        "local_imu_rotation_prior_enable": "/lidar_update/local_imu_rotation_prior_enable",
    }
    for key, label, fallback in mappings:
        old = legacy_params.get(key, fallback)
        profile = mature_params.get(key, fallback)
        logged = runtime.get(runtime_key[key], profile)
        rows.append({"parameter": label, "legacy": str(old), "mature": str(logged),
                     "status": "SAME" if values_equal(old, profile) and values_equal(profile, logged)
                     else "UNEXPECTED_DIFFERENCE",
                     "evidence": f"R7H effective_params.yaml; R9B profile + runtime dump {runtime_key[key]}"})

    legacy_provenance = Path(inputs["legacy_runtime_provenance"]["path"]).read_text()
    legacy_adapter_log = Path(inputs["legacy_adapter_launch_log"]["path"]).read_text(errors="replace")
    legacy_adapter_launch_path = Path(inputs["legacy_adapter_launch"]["path"])
    legacy_adapter_launch_text = legacy_adapter_launch_path.read_text()
    mature_launch_text = Path(inputs["mature_launch_file"]["path"]).read_text()

    def provenance_sha(pattern, label):
        match = re.search(pattern, legacy_provenance, re.IGNORECASE)
        if not match:
            raise ValueError(f"could not extract {label} SHA from R7H runtime provenance")
        return match.group(1).lower()

    legacy_map_sha = provenance_sha(
        r"Frozen H1 normalized map:.*?SHA-256 `([0-9a-f]{64})`", "legacy H1 map")
    legacy_calibration_sha = provenance_sha(
        r"Calibration:.*?SHA-256 `([0-9a-f]{64})`", "legacy calibration")
    legacy_ndt_sha = provenance_sha(
        r"NDT source SHA-256 in both workspaces: `([0-9a-f]{64})`", "legacy NDT source")
    legacy_launch_sha = provenance_sha(
        r"Adapter launch:.*?SHA-256 `([0-9a-f]{64})`", "legacy adapter launch")
    if sha256(legacy_adapter_launch_path) != legacy_launch_sha:
        raise ValueError("R7H adapter launch file does not match its recorded provenance SHA")
    launch_path_logged = re.search(r"/home/[^\x1b\s]+p3_r7h_floor01_closed_loop\.launch", legacy_adapter_log)
    if not launch_path_logged:
        raise ValueError("R7H adapter launch log does not identify the formal Run A launch")
    if Path(launch_path_logged.group(0)).resolve() != legacy_adapter_launch_path.resolve():
        raise ValueError("R7H Run A launch log path differs from the manifest adapter launch")
    if "/floor01_causal_adapter/laser_to_imu_translation: [0.08, 0.029, 0.03]" not in legacy_adapter_log:
        raise ValueError("R7H Run A runtime dump does not confirm the configured extrinsic translation")

    def launch_rosparam_list(text, name):
        match = re.search(
            rf'<rosparam\s+param="{re.escape(name)}">\s*(\[[^\]]+\])\s*</rosparam>', text)
        if not match:
            raise ValueError(f"missing {name} in R7H adapter launch")
        values = yaml.safe_load(match.group(1))
        if not isinstance(values, list) or not all(isinstance(value, (int, float)) for value in values):
            raise ValueError(f"invalid numeric list for {name} in R7H adapter launch")
        return [float(value) for value in values]

    legacy_extrinsic_rotation = launch_rosparam_list(legacy_adapter_launch_text, "laser_to_imu_rotation")
    legacy_extrinsic_translation = launch_rosparam_list(legacy_adapter_launch_text, "laser_to_imu_translation")
    mature_profile_rotation = [float(x) for x in mature_cfg["deskew"]["T_imu_lidar_rotation_row_major"]]
    mature_profile_translation = [float(x) for x in mature_cfg["deskew"]["T_imu_lidar_translation_m"]]
    def runtime_sha_prefix(parameter):
        value = runtime.get(parameter)
        if not isinstance(value, str):
            raise ValueError(f"R9B runtime dump lacks {parameter}")
        match = re.fullmatch(r"([0-9a-f]{16,64})\.\.\.", value.strip(), re.IGNORECASE)
        if not match:
            raise ValueError(f"invalid abbreviated SHA in R9B runtime dump for {parameter}: {value}")
        return match.group(1).lower()

    if not legacy_adapter_log:
        raise ValueError("R7H runtime log does not provide calibration provenance")
    mature_runtime_calibration_prefix = runtime_sha_prefix("/dataset/calibration_sha256")
    mature_runtime_map_prefix = runtime_sha_prefix("/dataset/map_sha256")
    official_calibration_sha = sha256(inputs["floor01_calibration"]["path"])
    if not official_calibration_sha.startswith(mature_runtime_calibration_prefix):
        raise ValueError("R9B runtime calibration hash prefix does not match official calibration")
    if "superloc_floor01_imu_deskew_r9b.yaml" not in mature_launch_text:
        raise ValueError("R9B launch does not load the manifest-recorded Floor01 profile")
    runtime_translation_match = re.search(
        r"\* /deskew/T_imu_lidar_translation_m:\s*\[([^\]]+)\]", runtime_text)
    if not runtime_translation_match:
        raise ValueError("R9B runtime parameter dump lacks extrinsic translation")
    mature_runtime_translation = [float(x.strip()) for x in runtime_translation_match.group(1).split(",")]
    if not np.allclose(mature_runtime_translation, mature_profile_translation, rtol=0.0, atol=1e-12):
        raise ValueError("R9B runtime translation differs from the manifest profile")

    old_map = legacy_cfg.get("map", {})
    new_map = mature_cfg.get("map", {})
    old_frames = legacy_cfg.get("frames", {})
    new_frames = mature_cfg.get("frames", {})
    verified_map_sha = sha256(inputs["h1_map"]["path"])
    expected_map_sha = inputs["h1_map"]["expected_sha256"].strip().lower()
    if verified_map_sha != expected_map_sha:
        raise ValueError("H1 map SHA changed after the pre-GT manifest verification")
    mature_profile_map_sha = str(mature_cfg["dataset"]["map_sha256"]).strip().lower()
    map_sha_matches = (legacy_map_sha == verified_map_sha == mature_profile_map_sha and
                       verified_map_sha.startswith(mature_runtime_map_prefix))
    add("map SHA-256", legacy_map_sha, mature_profile_map_sha,
        "SAME" if map_sha_matches else "UNEXPECTED_DIFFERENCE",
        f"R7H provenance and R9B profile checked against manifest-verified H1 PCD {verified_map_sha}; "
        f"R9B runtime prefix={mature_runtime_map_prefix}")
    add("map frame", old_frames.get("map_frame"), new_frames.get("map_frame"), "SAME",
        "R7H effective parameters and R9B profile")
    add("LiDAR/base frame", old_frames.get("base_frame"), new_frames.get("base_frame"), "SAME",
        "R7H effective parameters and R9B profile")
    add("map voxel size (m)", old_map.get("voxel_size"), new_map.get("voxel_size"), "SAME",
        "effective map voxelization")
    add("map path", old_map.get("pcd_fallback_path"), new_map.get("pcd_fallback_path"), "SAME",
        "same H1 PCD file")
    mature_profile_calibration_sha = str(mature_cfg["dataset"]["calibration_sha256"]).strip().lower()
    calibration_sha_matches = (legacy_calibration_sha == official_calibration_sha ==
                               mature_profile_calibration_sha and
                               official_calibration_sha.startswith(mature_runtime_calibration_prefix))
    add("Floor01 calibration SHA-256", legacy_calibration_sha,
        mature_profile_calibration_sha,
        "SAME" if calibration_sha_matches else "UNEXPECTED_DIFFERENCE",
        f"R7H provenance and R9B profile/runtime checked against manifest-verified official calibration "
        f"{official_calibration_sha}; R9B runtime prefix={mature_runtime_calibration_prefix}")
    calibration_cfg = yaml.safe_load(Path(inputs["floor01_calibration"]["path"]).read_text())
    official_matrix = [float(x) for x in calibration_cfg["laser_to_imu"]["data"]]
    official_translation = [official_matrix[i] for i in (3, 7, 11)]
    official_rotation = [official_matrix[i] for i in (0, 1, 2, 4, 5, 6, 8, 9, 10)]
    legacy_matches_calibration = (np.allclose(legacy_extrinsic_rotation, official_rotation, rtol=0.0, atol=1e-12) and
                                 np.allclose(legacy_extrinsic_translation, official_translation, rtol=0.0, atol=1e-12))
    mature_matches_calibration = (np.allclose(mature_profile_rotation, official_rotation, rtol=0.0, atol=1e-12) and
                                 np.allclose(mature_profile_translation, official_translation, rtol=0.0, atol=1e-12))
    rotation_old = json.dumps(legacy_extrinsic_rotation, separators=(",", ":"))
    rotation_new = json.dumps(mature_profile_rotation, separators=(",", ":"))
    add("LiDAR-to-IMU rotation (row-major 3x3)", rotation_old, rotation_new,
        "SAME" if legacy_matches_calibration and mature_matches_calibration else "UNEXPECTED_DIFFERENCE",
        "R7H hash-verified adapter launch vs R9B profile; both compared independently to official calibration")
    translation_old = json.dumps(legacy_extrinsic_translation, separators=(",", ":"))
    translation_new = json.dumps(mature_profile_translation, separators=(",", ":"))
    add("LiDAR-to-IMU translation (m)", translation_old, translation_new,
        "SAME" if legacy_matches_calibration and mature_matches_calibration else "UNEXPECTED_DIFFERENCE",
        "R7H launch/runtime dump vs R9B profile/runtime dump; both compared independently to official calibration")

    # Range defaults are supplied by the exact shared NDT source; neither run overrides them.
    range_keys = ("scan_min_range", "scan_max_range")
    range_clear = all(k not in legacy_params and k not in mature_params for k in range_keys)
    add("range filter (m)", "[0.5, 80.0] shared-source defaults",
        "[0.5, 80.0] shared-source defaults", "SAME" if range_clear else "UNEXPECTED_DIFFERENCE",
        "same NDT source SHA; defaults at getParam(scan_min_range/max_range); no profile override")

    old_decoder = "ros-drivers/velodyne 1.7.0, commit 89faa698688a48d4a5080f73c04d7aed8117eec0"
    new_decoder = mature_cfg.get("dataset", {}).get("decoder", "")
    normalize_ws = lambda value: " ".join(value.split())
    decoder_same = ("ros-drivers/velodyne" in decoder_audit and "1.7.0" in decoder_audit and
                    "89faa698688a48d4a5080f73c04d7aed8117eec0" in decoder_audit and
                    new_decoder == old_decoder)
    add("packet decoder", old_decoder, new_decoder, "SAME" if decoder_same else "UNEXPECTED_DIFFERENCE",
        "R7E official decoder audit and R9B dataset overlay")
    point_semantics_same = ("RawData::buildTimings" in decoder_audit and
                            mature_cfg["deskew"].get("point_time_convention") == "seconds_from_scan_start" and
                            mature_cfg["deskew"].get("reference_time") == "start")
    rows.append({"parameter": "point time semantics",
                 "legacy": "official VLP16 firing-time table; first-packet/start reference",
                 "mature": f"field={mature_cfg['deskew'].get('point_time_field')}; convention={mature_cfg['deskew'].get('point_time_convention')}; reference={mature_cfg['deskew'].get('reference_time')}",
                 "status": "SAME" if point_semantics_same else "UNEXPECTED_DIFFERENCE",
                 "evidence": "same raw packet bag, official decoder timing, and first-packet scan-start convention"})

    target_old = "549606" if "target=549606" in Path(inputs["legacy_ndt_node_log"]["path"]).read_text(errors="replace") else "NOT_FOUND"
    target_new = "549606" if "target=549606" in runtime_text else "NOT_FOUND"
    add("loaded target point count", target_old, target_new, "SAME",
        "R7H NDT startup log and R9B NDT startup log")

    mature_ndt_sha = sha256(Path(inputs["mature_ndt_source"]["path"]))
    add("NDT implementation SHA-256", legacy_ndt_sha, mature_ndt_sha, "SAME",
        "R7H formal run provenance SHA versus hash-verified R9B workspace source at frozen start revision")
    add("initial-guess implementation", "previous_pose_delta + local IMU rotation prior; shared code",
        "previous_pose_delta + local IMU rotation prior; shared code", "SAME",
        "same NDT source SHA; per-frame source/use gate below")
    add("initial pose / map normalization", "H1 map frame; initial current_pose is identity",
        "H1 map frame; initial current_pose is identity", "SAME",
        "same map SHA/frame; both first logged output guesses are current_pose/no_previous_pose at identity")

    # These differences define the two closed-loop pipelines under test.
    rows.extend([
        {"parameter": "translation/rotation motion compensation", "legacy": "prior-NDT CV translation + IMU gyro rotation",
         "mature": "EKF/IMU full-SE(3) deskew", "status": "EXPECTED_PIPELINE_DIFFERENCE",
         "evidence": "R7H/R9B formal pipeline definitions"},
        {"parameter": "IMU state propagation and NDT feedback", "legacy": "prior NDT pose CV history",
         "mature": "EKF propagation, NDT correction, later state-driven deskew", "status": "EXPECTED_PIPELINE_DIFFERENCE",
         "evidence": "closed-loop infrastructure contrast explicitly defined by R9C"},
        {"parameter": "gyro-bias static initialization", "legacy": "legacy adapter behavior",
         "mature": "first 200 accepted IMU samples mean gyro", "status": "EXPECTED_PIPELINE_DIFFERENCE",
         "evidence": "R9B recorded experiment profile"},
        {"parameter": "deskew/NDT input topic plumbing", "legacy": "/superloc_adapter/points_deskewed",
         "mature": "/dog_livo/points_deskewed_imu_exp", "status": "EXPECTED_PIPELINE_DIFFERENCE",
         "evidence": "interfaces differ while decoded packet-time semantics and NDT settings are held fixed"},
    ])

    legacy_by = {r["stamp_key"]: r for r in legacy_rows}
    mature_by = {r["stamp_key"]: r for r in mature_rows}
    def first_guess_is_identity(record):
        row = record["row"]
        p = np.asarray([float(row[f"initial_guess_t{axis}"]) for axis in "xyz"])
        q = np.asarray([float(row[f"initial_guess_q{axis}"]) for axis in "xyzw"])
        return (record["initial_guess_source"] == "current_pose" and
                record["initial_guess_reason"] == "no_previous_pose" and
                np.linalg.norm(p) <= 1e-12 and
                min(np.linalg.norm(q - np.asarray([0, 0, 0, 1])),
                    np.linalg.norm(q + np.asarray([0, 0, 0, 1]))) <= 1e-12)

    initial_identity_match = first_guess_is_identity(legacy_rows[0]) and first_guess_is_identity(mature_rows[0])
    rows.append({"parameter": "first emitted initial guess identity", "legacy": str(first_guess_is_identity(legacy_rows[0])),
                 "mature": str(first_guess_is_identity(mature_rows[0])),
                 "status": "SAME" if initial_identity_match else "UNEXPECTED_DIFFERENCE",
                 "evidence": "first rows in each Run-A detailed NDT CSV: current_pose/no_previous_pose, identity pose"})
    common = sorted(set(legacy_by) & set(mature_by))
    guess_mismatch_stamps = []
    prior_mismatch_stamps = []
    for stamp in common:
        a, b = legacy_by[stamp], mature_by[stamp]
        if (a["initial_guess_source"], a["initial_guess_reason"]) != (b["initial_guess_source"], b["initial_guess_reason"]):
            guess_mismatch_stamps.append(stamp)
        if (a["local_imu_prior_enabled"], a["local_imu_prior_used"]) != (b["local_imu_prior_enabled"], b["local_imu_prior_used"]):
            prior_mismatch_stamps.append(stamp)
    protocol_text = Path(inputs["r9b_protocol_report"]["path"]).read_text()
    first_common = common[0]
    startup_expected = (
        guess_mismatch_stamps == [first_common] and prior_mismatch_stamps == [first_common] and
        legacy_by[first_common]["initial_guess_source"] == "local_imu_rotation_prior" and
        mature_by[first_common]["initial_guess_source"] == "current_pose" and
        mature_by[first_common]["initial_guess_reason"] == "no_previous_pose" and
        "PREINIT_REJECTED" in protocol_text and "first published/NDT stamp" in protocol_text
    )
    history_status = "EXPECTED_PIPELINE_DIFFERENCE" if startup_expected else (
        "SAME" if not guess_mismatch_stamps else "UNEXPECTED_DIFFERENCE")
    rows.append({"parameter": "logged initial-guess source/reason", "legacy": f"common={len(common)}",
                 "mature": f"common={len(common)}", "status": history_status,
                 "evidence": f"mismatches={len(guess_mismatch_stamps)}; stamps={guess_mismatch_stamps}; only first common anchor differs because mature Run A rejects pre-initialization scans and has no previous NDT pose; later common source/reason must match"})
    rows.append({"parameter": "logged local IMU prior enable/use", "legacy": f"common={len(common)}",
                 "mature": f"common={len(common)}", "status": history_status if startup_expected else (
                     "SAME" if not prior_mismatch_stamps else "UNEXPECTED_DIFFERENCE"),
                 "evidence": f"mismatches={len(prior_mismatch_stamps)}; stamps={prior_mismatch_stamps}; expected first-output history availability only"})
    return rows


def matrix_error(anchor, pose, gt_anchor, gt_pose):
    estimate_relative = np.linalg.inv(anchor) @ pose
    gt_relative = np.linalg.inv(gt_anchor) @ gt_pose
    err = np.linalg.inv(gt_relative) @ estimate_relative
    return float(np.linalg.norm(err[:3, 3])), evaluator.angle_deg(err[:3, :3])


def interpolate_gt(stamp, times, poses):
    pose, bracket = evaluator.interpolate_pose(times, poses, float(stamp))
    if pose is None:
        return None
    lo, hi, width = bracket
    t = float(stamp)
    left, right = float(times[lo]), float(times[hi])
    nearest = 0.0 if lo == hi else min(t - left, right - t)
    return {"pose": pose, "left": left, "right": right, "width": float(width), "nearest": float(nearest)}


def fnum(value):
    if value is None or not np.isfinite(value):
        return ""
    return f"{float(value):.12g}"


def stamp_text(value):
    """Preserve absolute ROS timestamp decimal precision in CSV/Markdown."""
    if isinstance(value, Decimal):
        stamp = value
    else:
        stamp = Decimal(str(value))
    return str(stamp.normalize())


def sample_stamp_at_relative(records, anchor_key, relative_seconds):
    """Return the exact sampled timestamp corresponding to evaluator crossing time."""
    stamps = [record["stamp"] for record in records]
    target = float(anchor_key) + float(relative_seconds)
    index = bisect_left(stamps, target)
    candidates = [i for i in (index - 1, index) if 0 <= i < len(stamps)]
    if not candidates:
        raise ValueError("cannot map crossing onto an empty timestamp sequence")
    matched_index = min(candidates, key=lambda i: abs(stamps[i] - target))
    matched = stamps[matched_index]
    if abs(matched - target) > 1e-4:
        raise ValueError(f"crossing did not map to a source sample: target={target}, nearest={matched}")
    return records[matched_index]["stamp_key"]


def limiter_pose_delta(raw_pose, final_pose):
    """Separate raw-to-final translation and rotation changes; no causal claim."""
    translation = float(np.linalg.norm(final_pose[:3, 3] - raw_pose[:3, 3]))
    rotation = evaluator.angle_deg(raw_pose[:3, :3].T @ final_pose[:3, :3])
    return translation, rotation


def metric_stats(values):
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return {k: None for k in ("count", "mean", "rmse", "median", "p95", "max", "std")}
    return {"count": int(arr.size), "mean": float(np.mean(arr)),
            "rmse": float(np.sqrt(np.mean(arr * arr))), "median": float(np.median(arr)),
            "p95": float(np.percentile(arr, 95)), "max": float(np.max(arr)),
            "std": float(np.std(arr))}


def add_metric_row(out, population, pipeline, pose_type, t_values, r_values):
    ts, rs = metric_stats(t_values), metric_stats(r_values)
    out.append({"population": population, "pipeline": pipeline, "pose_type": pose_type,
                "count": ts["count"],
                "translation_mean_m": ts["mean"], "translation_rmse_m": ts["rmse"],
                "translation_median_m": ts["median"], "translation_p95_m": ts["p95"],
                "translation_max_m": ts["max"], "translation_std_m": ts["std"],
                "rotation_mean_deg": rs["mean"], "rotation_rmse_deg": rs["rmse"],
                "rotation_median_deg": rs["median"], "rotation_p95_deg": rs["p95"],
                "rotation_max_deg": rs["max"], "rotation_std_deg": rs["std"]})


def bootstrap_delta(values, rng):
    arr = np.asarray(values, dtype=float)
    means = np.empty(BOOTSTRAP_REPLICATES, dtype=float)
    medians = np.empty(BOOTSTRAP_REPLICATES, dtype=float)
    n = len(arr)
    for i in range(BOOTSTRAP_REPLICATES):
        sample = arr[rng.integers(0, n, n)]
        means[i] = np.mean(sample)
        medians[i] = np.median(sample)
    return {
        "mean": float(np.mean(arr)), "mean_ci_low": float(np.percentile(means, 2.5)),
        "mean_ci_high": float(np.percentile(means, 97.5)),
        "median": float(np.median(arr)), "median_ci_low": float(np.percentile(medians, 2.5)),
        "median_ci_high": float(np.percentile(medians, 97.5)),
    }


def build_plots(out, common_rows, window_rows, anchor):
    times = np.asarray([float(row["relative_common_s"]) for row in common_rows])
    fig, ax = plt.subplots(figsize=(14, 5.5))
    for pipeline, color in (("legacy", "#386cb0"), ("mature_imu", "#f0027f")):
        ax.plot(times, [float(row[f"{pipeline}_final_translation_error_m"]) for row in common_rows],
                label=f"{pipeline} final_used", color=color, linewidth=1.15)
        ax.plot(times, [float(row[f"{pipeline}_raw_translation_error_m"]) for row in common_rows],
                label=f"{pipeline} raw NDT", color=color, linewidth=0.75, alpha=0.35, linestyle=":")
    for threshold in (0.5, 1.0, 2.0, 5.0):
        ax.axhline(threshold, color="gray", alpha=0.25, linewidth=0.7)
    ax.set(xlabel=f"Seconds from common anchor ({anchor})", ylabel="Translation error (m)",
           title="Floor01 Run A: first-common-pair relative translation error")
    ax.grid(True, alpha=0.2); ax.legend(ncol=2); fig.tight_layout()
    fig.savefig(out / "translation_error_timeline.png", dpi=160); plt.close(fig)

    fig, ax = plt.subplots(figsize=(14, 5.5))
    for pipeline, color in (("legacy", "#386cb0"), ("mature_imu", "#f0027f")):
        ax.plot(times, [float(row[f"{pipeline}_final_rotation_error_deg"]) for row in common_rows],
                label=f"{pipeline} final_used", color=color, linewidth=1.0)
        ax.plot(times, [float(row[f"{pipeline}_raw_rotation_error_deg"]) for row in common_rows],
                label=f"{pipeline} raw NDT", color=color, linewidth=0.7, alpha=0.35, linestyle=":")
    ax.set(xlabel=f"Seconds from common anchor ({anchor})", ylabel="Rotation error (deg)",
           title="Floor01 Run A: first-common-pair relative rotation error")
    ax.grid(True, alpha=0.2); ax.legend(ncol=2); fig.tight_layout()
    fig.savefig(out / "rotation_error_timeline.png", dpi=160); plt.close(fig)

    fig, ax = plt.subplots(figsize=(14, 4.5))
    delta = [float(row["legacy_final_translation_error_m"]) - float(row["mature_imu_final_translation_error_m"])
             for row in common_rows]
    ax.axhline(0, color="black", linewidth=0.8)
    ax.scatter(times, delta, s=6, alpha=0.5, color="#7fc97f")
    ax.set(xlabel=f"Seconds from common anchor ({anchor})", ylabel="Legacy − mature error (m)",
           title="Paired final_used translation-error difference (positive favors mature IMU)")
    ax.grid(True, alpha=0.2); fig.tight_layout()
    fig.savefig(out / "paired_translation_delta.png", dpi=160); plt.close(fig)

    win_names = [row["window"] for row in window_rows]
    if win_names:
        x = np.arange(len(win_names)); width = 0.36
        fig, ax = plt.subplots(figsize=(11, 5))
        ax.bar(x - width/2, [float(row["legacy_final_t_mean_m"]) for row in window_rows], width,
               label="Legacy", color="#386cb0")
        ax.bar(x + width/2, [float(row["mature_final_t_mean_m"]) for row in window_rows], width,
               label="Mature IMU", color="#f0027f")
        ax.set_xticks(x, win_names, rotation=20, ha="right")
        ax.set_ylabel("Final-used translation error mean (m)")
        ax.set_title("Predeclared window comparison; common GT-supported frames")
        ax.grid(axis="y", alpha=0.2); ax.legend(); fig.tight_layout()
        fig.savefig(out / "window_comparison.png", dpi=160); plt.close(fig)

    shock_abs = float(SHOCK_CENTER)
    zoom = [row for row in common_rows if abs(float(row["stamp"]) - shock_abs) <= 5.0]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    if zoom:
        zx = [float(row["stamp"]) - shock_abs for row in zoom]
        ax.plot(zx, [float(row["legacy_final_translation_error_m"]) for row in zoom],
                label="Legacy final", color="#386cb0")
        ax.plot(zx, [float(row["mature_imu_final_translation_error_m"]) for row in zoom],
                label="Mature IMU final", color="#f0027f")
    ax.axvline(0, color="black", linestyle="--", linewidth=0.8, label="raw-IMU peak stamp")
    ax.set(xlabel="Seconds from preidentified raw-IMU peak", ylabel="Translation error (m)",
           title="High-dynamic neighborhood (descriptive; no impact-causality claim)")
    ax.grid(True, alpha=0.2); ax.legend(); fig.tight_layout()
    fig.savefig(out / "high_dynamic_zoom.png", dpi=160); plt.close(fig)


def extract_shock_from_bag(raw_bag_path):
    try:
        import rosbag
        import rospy
    except Exception as exc:
        raise RuntimeError("ROS Python rosbag/rospy unavailable; source /opt/ros/noetic/setup.bash before running") from exc
    lo = float(SHOCK_CENTER - SHOCK_HALF_WIDTH)
    hi = float(SHOCK_CENTER + SHOCK_HALF_WIDTH)
    records = []
    with rosbag.Bag(str(raw_bag_path), "r") as bag:
        # Record-time seek is widened; selection uses the message header timestamp.
        start = rospy.Time.from_sec(lo - 0.2)
        end = rospy.Time.from_sec(hi + 0.2)
        for _, msg, _ in bag.read_messages(topics=["/input/imu"], start_time=start, end_time=end):
            stamp = float(msg.header.stamp.to_sec())
            if lo <= stamp <= hi:
                a = msg.linear_acceleration
                g = msg.angular_velocity
                avec = np.asarray([a.x, a.y, a.z], dtype=float)
                gvec = np.asarray([g.x, g.y, g.z], dtype=float)
                records.append({"stamp": stamp, "acc": avec, "gyro": gvec,
                                "acc_norm": float(np.linalg.norm(avec)),
                                "gyro_norm": float(np.linalg.norm(gvec))})
    if not records:
        raise ValueError("no /input/imu messages found in predeclared raw shock window")
    return records


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=HERE / "p3_r9c_manifest.csv")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--repo-artifacts", type=Path, default=None)
    args = parser.parse_args()

    # Do not silently overwrite an existing external result directory.
    if args.out.exists() and any(args.out.iterdir()):
        raise FileExistsError(f"output directory is non-empty; refusing overwrite: {args.out}")
    args.out.mkdir(parents=True, exist_ok=True)
    manifest_rows, inputs = read_manifest(args.manifest)
    required = {"raw_input_bag", "legacy_ndt_runA", "legacy_effective_params", "legacy_ndt_node_log",
                "legacy_adapter_launch", "legacy_adapter_launch_log", "legacy_runtime_provenance",
                "mature_ndt_runA", "mature_profile", "mature_launch_log_runA", "h1_map",
                "floor01_calibration", "floor01_gt", "r7i_metadata", "r7i_protocol",
                "r9b_shock_audit", "r9b_shock_raw_samples", "mature_ndt_source", "r3b_evaluator"}
    missing = required - set(inputs)
    if missing:
        raise ValueError(f"missing required manifest inputs: {sorted(missing)}")

    # Gate 1: validate non-GT evidence and actual NDT/runtime parameters.
    verified = validate_inputs(inputs, include_gt=False)
    global evaluator
    evaluator = load_evaluator(inputs["r3b_evaluator"]["path"])
    package_root = Path(inputs["r3b_evaluator"]["path"]).resolve().parent.parent
    if args.repo_artifacts is None:
        args.repo_artifacts = package_root / "docs" / "p3_r9c_artifacts"
    legacy = load_run(inputs["legacy_ndt_runA"]["path"], "legacy")
    mature = load_run(inputs["mature_ndt_runA"]["path"], "mature")
    equivalence = parameter_audit(inputs, legacy, mature)
    write_csv(args.out / "p3_r9c_parameter_equivalence.csv",
              ["parameter", "legacy", "mature", "status", "evidence"], equivalence)
    unexpected = [row for row in equivalence if row["status"] == "UNEXPECTED_DIFFERENCE"]
    if unexpected:
        print(f"PARAMETER_EQUIVALENCE_GATE=FAIL; unexpected={len(unexpected)}; GT_NOT_READ")
        raise SystemExit(2)
    print("PARAMETER_EQUIVALENCE_GATE=PASS; GT access allowed")

    # Gate 2 begins here: verify then load the official GT, for post-hoc evaluation only.
    verified.extend(validate_inputs({GT_ROLE: inputs[GT_ROLE]}, include_gt=True))
    gt_path = Path(inputs[GT_ROLE]["path"])
    gt_times, gt_poses = evaluator.load_gt(gt_path)
    T_imu_lidar, T_lidar_imu = evaluator.load_extrinsics(inputs["floor01_calibration"]["path"])

    r7i_meta = json.loads(Path(inputs["r7i_metadata"]["path"]).read_text())
    old_anchor = Decimal(str(r7i_meta["anchor_stamp"]))
    exact_legacy = {row["stamp_key"]: row for row in legacy}
    exact_mature = {row["stamp_key"]: row for row in mature}
    legacy_keys, mature_keys = set(exact_legacy), set(exact_mature)
    common_keys = sorted(legacy_keys & mature_keys)
    if not common_keys:
        raise ValueError("no exact common NDT timestamps")
    gt_cache = {key: interpolate_gt(key, gt_times, gt_poses) for key in sorted(legacy_keys | mature_keys)}
    anchor_key = next((key for key in common_keys if gt_cache[key] is not None), None)
    if anchor_key is None:
        raise ValueError("no exact common timestamp with GT interpolation support")
    gt_anchor = gt_cache[anchor_key]["pose"]
    legacy_anchor_raw = exact_legacy[anchor_key]["raw_lidar"] @ T_lidar_imu
    legacy_anchor_final = exact_legacy[anchor_key]["final_lidar"] @ T_lidar_imu
    mature_anchor_raw = exact_mature[anchor_key]["raw_lidar"] @ T_lidar_imu
    mature_anchor_final = exact_mature[anchor_key]["final_lidar"] @ T_lidar_imu
    if not np.all(np.isfinite(gt_anchor)):
        raise ValueError("non-finite common-anchor GT pose")

    gt_audit = []
    frame_metrics = []
    metrics_by_pipeline = {"legacy": {}, "mature_imu": {}}
    for pipeline, records, anchor_raw, anchor_final in (
            ("legacy", legacy, legacy_anchor_raw, legacy_anchor_final),
            ("mature_imu", mature, mature_anchor_raw, mature_anchor_final)):
        for rec in records:
            gt = gt_cache[rec["stamp_key"]]
            supported = gt is not None
            gt_audit.append({
                "pipeline": pipeline, "stamp": str(rec["stamp_key"]),
                "gt_supported": int(supported),
                "exclude_reason": "" if supported else ("EARLY_BEFORE_GT" if rec["stamp"] < gt_times[0] else "LATE_AFTER_GT"),
                "gt_left_stamp": stamp_text(gt["left"]) if supported else "",
                "gt_right_stamp": stamp_text(gt["right"]) if supported else "",
                "gt_bracket_width_s": fnum(gt["width"]) if supported else "",
                "nearest_gt_dt_s": fnum(gt["nearest"]) if supported else "",
            })
            raw_pose = rec["raw_lidar"] @ T_lidar_imu
            final_pose = rec["final_lidar"] @ T_lidar_imu
            limiter_translation_delta, limiter_rotation_delta = limiter_pose_delta(
                rec["raw_lidar"], rec["final_lidar"])
            raw_t = raw_r = final_t = final_r = None
            if supported:
                raw_t, raw_r = matrix_error(anchor_raw, raw_pose, gt_anchor, gt["pose"])
                final_t, final_r = matrix_error(anchor_final, final_pose, gt_anchor, gt["pose"])
                metrics_by_pipeline[pipeline][rec["stamp_key"]] = {
                    "raw_t": raw_t, "raw_r": raw_r, "final_t": final_t, "final_r": final_r,
                    "gt_width": gt["width"], "gt_nearest": gt["nearest"],
                }
            frame_metrics.append({
                "pipeline": pipeline, "stamp": str(rec["stamp_key"]), "stamp_float": rec["stamp"],
                "relative_common_s": rec["stamp"] - float(anchor_key),
                "relative_r7i_s": float(rec["stamp_key"] - old_anchor),
                "gt_supported": int(supported), "gt_bracket_width_s": fnum(gt["width"]) if supported else "",
                "raw_translation_error_m": fnum(raw_t), "raw_rotation_error_deg": fnum(raw_r),
                "final_translation_error_m": fnum(final_t), "final_rotation_error_deg": fnum(final_r),
                "raw_to_final_translation_delta_m": fnum(limiter_translation_delta),
                "raw_to_final_rotation_delta_deg": fnum(limiter_rotation_delta),
                "fitness": rec["fitness"], "iterations": rec["iterations"], "converged": rec["converged"],
                "translation_limited": rec["translation_limited"], "rotation_limited": rec["rotation_limited"],
                "cloud_hash": rec["cloud_hash"], "point_count_raw": rec["point_count_raw"],
                "point_count_source": rec["point_count_source"],
                "initial_guess_source": rec["initial_guess_source"],
                "local_imu_prior_used": rec["local_imu_prior_used"],
            })

    common_supported = [key for key in common_keys if gt_cache[key] is not None]
    hq_keys = [key for key in common_supported if gt_cache[key]["width"] <= 0.25]
    common_rows = []
    common_set = set(common_supported)
    for key in common_supported:
        a, b = exact_legacy[key], exact_mature[key]
        ma, mb = metrics_by_pipeline["legacy"][key], metrics_by_pipeline["mature_imu"][key]
        legacy_limit_t, legacy_limit_r = limiter_pose_delta(a["raw_lidar"], a["final_lidar"])
        mature_limit_t, mature_limit_r = limiter_pose_delta(b["raw_lidar"], b["final_lidar"])
        common_rows.append({
            "stamp": str(key), "relative_common_s": fnum(float(key) - float(anchor_key)),
            "relative_r7i_s": fnum(float(key - old_anchor)),
            "gt_bracket_width_s": fnum(gt_cache[key]["width"]),
            "gt_nearest_dt_s": fnum(gt_cache[key]["nearest"]),
            "legacy_raw_translation_error_m": fnum(ma["raw_t"]),
            "legacy_raw_rotation_error_deg": fnum(ma["raw_r"]),
            "legacy_final_translation_error_m": fnum(ma["final_t"]),
            "legacy_final_rotation_error_deg": fnum(ma["final_r"]),
            "mature_imu_raw_translation_error_m": fnum(mb["raw_t"]),
            "mature_imu_raw_rotation_error_deg": fnum(mb["raw_r"]),
            "mature_imu_final_translation_error_m": fnum(mb["final_t"]),
            "mature_imu_final_rotation_error_deg": fnum(mb["final_r"]),
            "legacy_raw_to_final_translation_delta_m": fnum(legacy_limit_t),
            "legacy_raw_to_final_rotation_delta_deg": fnum(legacy_limit_r),
            "mature_imu_raw_to_final_translation_delta_m": fnum(mature_limit_t),
            "mature_imu_raw_to_final_rotation_delta_deg": fnum(mature_limit_r),
            "delta_raw_translation_legacy_minus_mature_m": fnum(ma["raw_t"] - mb["raw_t"]),
            "delta_final_translation_legacy_minus_mature_m": fnum(ma["final_t"] - mb["final_t"]),
            "delta_raw_rotation_legacy_minus_mature_deg": fnum(ma["raw_r"] - mb["raw_r"]),
            "delta_final_rotation_legacy_minus_mature_deg": fnum(ma["final_r"] - mb["final_r"]),
            "legacy_fitness": a["fitness"], "mature_fitness": b["fitness"],
            "legacy_iterations": a["iterations"], "mature_iterations": b["iterations"],
            "legacy_converged": a["converged"], "mature_converged": b["converged"],
            "legacy_limited": int(a["translation_limited"] or a["rotation_limited"]),
            "mature_limited": int(b["translation_limited"] or b["rotation_limited"]),
            "legacy_cloud_hash": a["cloud_hash"], "mature_cloud_hash": b["cloud_hash"],
            "legacy_source_points": a["point_count_source"], "mature_source_points": b["point_count_source"],
            "hq_gt_bracket_le_0p25": int(key in hq_keys),
        })

    anchor_pair = next(row for row in common_rows if decimal_stamp(row["stamp"]) == anchor_key)
    anchor_max = max(abs(float(anchor_pair[k])) for k in (
        "legacy_raw_translation_error_m", "legacy_raw_rotation_error_deg",
        "legacy_final_translation_error_m", "legacy_final_rotation_error_deg",
        "mature_imu_raw_translation_error_m", "mature_imu_raw_rotation_error_deg",
        "mature_imu_final_translation_error_m", "mature_imu_final_rotation_error_deg"))
    if anchor_max > 1e-7:
        raise AssertionError(f"common-anchor error not approximately zero: {anchor_max}")

    # Frozen R7I windows are sourced from its metadata and use the original anchor.
    windows = []
    for name, bounds in r7i_meta["windows_relative_s"].items():
        windows.append((name, Decimal(str(bounds[0])), Decimal(str(bounds[1])), "R7I_FROZEN"))
    windows.append(("RAW_IMU_SHOCK", SHOCK_CENTER - SHOCK_HALF_WIDTH,
                    SHOCK_CENTER + SHOCK_HALF_WIDTH, "ABSOLUTE_RAW_IMU_PEAK_PLUS_MINUS_1S"))
    windows.append(("BROAD_HIGH_DYNAMIC", BROAD_HIGH_DYNAMIC[0], BROAD_HIGH_DYNAMIC[1],
                    "R7I_RELATIVE_BROAD_INTERVAL"))

    window_summary = []
    window_members = {}
    for name, lo, hi, definition in windows:
        selected = [row for row in common_rows if
                    (Decimal(row["stamp"]) >= (lo if definition.startswith("ABSOLUTE") else old_anchor + lo)) and
                    (Decimal(row["stamp"]) <= (hi if definition.startswith("ABSOLUTE") else old_anchor + hi))]
        window_members[name] = selected
        if not selected:
            raise ValueError(f"predeclared window has no common GT-supported frames: {name}")
        entry = {"window": name, "definition": definition, "n_common": len(selected),
                 "absolute_start_stamp": stamp_text(lo if definition.startswith("ABSOLUTE") else old_anchor + lo),
                 "absolute_end_stamp": stamp_text(hi if definition.startswith("ABSOLUTE") else old_anchor + hi),
                 "old_relative_start_s": fnum(float(lo)) if not definition.startswith("ABSOLUTE") else "",
                 "old_relative_end_s": fnum(float(hi)) if not definition.startswith("ABSOLUTE") else ""}
        for pipe, prefix in (("legacy", "legacy"), ("mature_imu", "mature")):
            for pose in ("raw", "final"):
                t = [float(row[f"{pipe}_{pose}_translation_error_m"]) for row in selected]
                r = [float(row[f"{pipe}_{pose}_rotation_error_deg"]) for row in selected]
                ts, rs = metric_stats(t), metric_stats(r)
                for stat in ("mean", "median", "p95"):
                    entry[f"{prefix}_{pose}_t_{stat}_m"] = ts[stat]
                    entry[f"{prefix}_{pose}_r_{stat}_deg"] = rs[stat]
            entry[f"{prefix}_fitness_mean"] = float(np.mean([float(row[f"{prefix}_fitness"]) for row in selected]))
            entry[f"{prefix}_iterations_mean"] = float(np.mean([int(row[f"{prefix}_iterations"]) for row in selected]))
            entry[f"{prefix}_convergence_fraction"] = float(np.mean([int(row[f"{prefix}_converged"]) for row in selected]))
        for pose in ("raw", "final"):
            for axis, field, suffix in (("t", "translation_error_m", "m"), ("r", "rotation_error_deg", "deg")):
                delta = [float(row[f"legacy_{pose}_{field}"]) - float(row[f"mature_imu_{pose}_{field}"])
                         for row in selected]
                entry[f"paired_{pose}_{axis}_delta_mean_{suffix}"] = float(np.mean(delta))
                entry[f"paired_{pose}_{axis}_delta_median_{suffix}"] = float(np.median(delta))
        window_summary.append(entry)

    # Exact timestamp population and supported/HQ reports.
    gt_audit_rows = gt_audit
    legacy_supported = [r for r in legacy if gt_cache[r["stamp_key"]] is not None]
    mature_supported = [r for r in mature if gt_cache[r["stamp_key"]] is not None]
    legacy_hq = [r for r in legacy_supported if gt_cache[r["stamp_key"]]["width"] <= 0.25]
    mature_hq = [r for r in mature_supported if gt_cache[r["stamp_key"]]["width"] <= 0.25]
    metrics_rows = {r["stamp"]: r for r in frame_metrics}
    global_rows = []
    for pop, keys in (("FULL_SUPPORTED", None), ("PAIRED_COMMON", common_supported), ("HQ_COMMON", hq_keys)):
        if pop == "FULL_SUPPORTED":
            for pipe, recs in (("legacy", legacy_supported), ("mature_imu", mature_supported)):
                frame_by = {r["stamp"]: r for r in frame_metrics if r["pipeline"] == pipe}
                for pose, pre in (("raw", "raw"), ("final_used", "final")):
                    vals = [frame_by[str(r["stamp_key"])] for r in recs]
                    add_metric_row(global_rows, pop, pipe, pose,
                                   [float(v[f"{pre}_translation_error_m"]) for v in vals],
                                   [float(v[f"{pre}_rotation_error_deg"]) for v in vals])
        else:
            for pipe in ("legacy", "mature_imu"):
                for pose, pre in (("raw", "raw"), ("final_used", "final")):
                    vals = [metrics_by_pipeline[pipe][key] for key in keys]
                    add_metric_row(global_rows, pop, pipe, pose,
                                   [v[f"{pre}_t"] for v in vals], [v[f"{pre}_r"] for v in vals])
        if pop != "FULL_SUPPORTED":
            for pose, pre in (("raw", "raw"), ("final_used", "final")):
                t_delta = [metrics_by_pipeline["legacy"][key][f"{pre}_t"] -
                           metrics_by_pipeline["mature_imu"][key][f"{pre}_t"] for key in keys]
                r_delta = [metrics_by_pipeline["legacy"][key][f"{pre}_r"] -
                           metrics_by_pipeline["mature_imu"][key][f"{pre}_r"] for key in keys]
                add_metric_row(global_rows, pop, "legacy_minus_mature", pose, t_delta, r_delta)

    rng = np.random.Generator(np.random.PCG64(BOOTSTRAP_SEED))
    bootstrap_rows = []
    better_rows = []
    for pose, pre in (("raw", "raw"), ("final_used", "final")):
        for metric, suffix, tolerance in (("translation", "t", TIE_TRANSLATION_M),
                                          ("rotation", "r", TIE_ROTATION_DEG)):
            deltas = [metrics_by_pipeline["legacy"][key][f"{pre}_{suffix}"] -
                      metrics_by_pipeline["mature_imu"][key][f"{pre}_{suffix}"] for key in common_supported]
            boot = bootstrap_delta(deltas, rng)
            bootstrap_rows.append({"population": "PAIRED_COMMON", "pose": pose, "metric": metric,
                                   "n": len(deltas), "seed": BOOTSTRAP_SEED,
                                   "rng": "NumPy PCG64", "replicates": BOOTSTRAP_REPLICATES,
                                   **boot})
            better_rows.append({"pose": pose, "metric": metric, "n": len(deltas),
                                "mature_better": int(sum(x > tolerance for x in deltas)),
                                "legacy_better": int(sum(x < -tolerance for x in deltas)),
                                "tie": int(sum(abs(x) <= tolerance for x in deltas)),
                                "tie_tolerance": tolerance})

    crossing_rows = []
    crossing_data = {}
    # Both crossing timelines use the same paired, GT-supported exact-stamp population.
    for pipe, recs in (("legacy", [exact_legacy[k] for k in common_supported]),
                       ("mature_imu", [exact_mature[k] for k in common_supported])):
        rel_times = np.asarray([r["stamp"] - float(anchor_key) for r in recs], dtype=float)
        trans_errors = np.asarray([metrics_by_pipeline[pipe][r["stamp_key"]]["final_t"] for r in recs], dtype=float)
        pipe_results = {}
        for threshold in (0.25, 0.5, 1.0, 2.0, 5.0):
            result = evaluator.persistent_crossing(rel_times, trans_errors, threshold,
                                                   min_duration=5.0)
            instant = result["first_crossing_s"]
            persist = result["persistent_crossing_s"]
            instant_stamp = "" if instant is None else stamp_text(sample_stamp_at_relative(recs, anchor_key, instant))
            persistent_stamp = "" if persist is None else stamp_text(sample_stamp_at_relative(recs, anchor_key, persist))
            crossing_rows.append({"pipeline": pipe, "threshold_m": threshold,
                                  "instant_status": "CROSSED" if instant is not None else "NOT_REACHED",
                                  "instant_relative_common_s": "" if instant is None else instant,
                                  "instant_stamp": instant_stamp,
                                  "persistent_status": "PERSISTENT" if persist is not None else "NOT_REACHED",
                                  "persistent_relative_common_s": "" if persist is None else persist,
                                  "persistent_stamp": persistent_stamp,
                                  "persistent_duration_s": result["persistent_duration_s"],
                                  "continuity_gap_max_s": 0.25, "persistence_min_s": 5.0,
                                  "operator": "strict >"})
            pipe_results[threshold] = result
        crossing_data[pipe] = pipe_results

    rapid_rows = []
    for pipe, results in crossing_data.items():
        c1, c5 = results[1.0]["persistent_crossing_s"], results[5.0]["persistent_crossing_s"]
        rapid_rows.append({"pipeline": pipe,
                           "rapid_1m_to_5m_s": fnum(c5 - c1) if c1 is not None and c5 is not None else "NOT_AVAILABLE",
                           "one_meter_persistent": "YES" if c1 is not None else "NO",
                           "five_meter_persistent": "YES" if c5 is not None else "NO"})

    # Pull an independent raw-IMU shock interval from the frozen input bag; no localization is rerun.
    shock_samples = extract_shock_from_bag(Path(inputs["raw_input_bag"]["path"]))
    peak = max(shock_samples, key=lambda x: x["acc_norm"])
    shock_audit = load_rows(inputs["r9b_shock_audit"]["path"])[0]
    if abs(peak["stamp"] - float(shock_audit["peak_timestamp"])) > 1e-6 or \
            abs(peak["acc_norm"] - float(shock_audit["acc_norm_max"])) > 1e-5:
        raise AssertionError("fresh raw-IMU shock extraction does not reproduce frozen R9B audit")
    high_rows = []
    shock_common = window_members["RAW_IMU_SHOCK"]
    for category, selected in (("RAW_IMU_SHOCK_WINDOW", shock_common),
                               ("BROAD_HIGH_DYNAMIC_138_168", window_members["BROAD_HIGH_DYNAMIC"])):
        for pipe, prefix in (("legacy", "legacy"), ("mature_imu", "mature")):
            ndt_prefix = prefix
            for pose, label in (("final", "final_used"), ("raw", "raw_NDT")):
                ts = [float(r[f"{pipe}_{pose}_translation_error_m"]) for r in selected]
                rs = [float(r[f"{pipe}_{pose}_rotation_error_deg"]) for r in selected]
                tstats, rstats = metric_stats(ts), metric_stats(rs)
                high_rows.append({"row_type": "window_summary", "window": category,
                                  "pipeline": pipe, "pose": label, "n_common": len(selected),
                                  "translation_mean_m": tstats["mean"], "translation_median_m": tstats["median"],
                                  "translation_p95_m": tstats["p95"], "rotation_mean_deg": rstats["mean"],
                                  "rotation_median_deg": rstats["median"], "rotation_p95_deg": rstats["p95"],
                                  "fitness_mean": float(np.mean([float(r[f"{ndt_prefix}_fitness"]) for r in selected])),
                                  "iterations_mean": float(np.mean([float(r[f"{ndt_prefix}_iterations"]) for r in selected])),
                                  "convergence_fraction": float(np.mean([float(r[f"{ndt_prefix}_converged"]) for r in selected]))})
    for target_name, target_stamp in (("BEFORE_1S", peak["stamp"] - 1.0),
                                      ("PEAK_NEAREST_FRAME", peak["stamp"]),
                                      ("AFTER_1S", peak["stamp"] + 1.0)):
        selected = min(common_rows, key=lambda r: abs(float(r["stamp"]) - target_stamp))
        for pipe, prefix in (("legacy", "legacy"), ("mature_imu", "mature")):
            source = exact_legacy if pipe == "legacy" else exact_mature
            stamp_key = decimal_stamp(selected["stamp"])
            ndt = source[stamp_key]
            high_rows.append({"row_type": "sample_nearest_target", "window": target_name,
                              "pipeline": pipe, "pose": "final_used_and_raw", "n_common": 1,
                              "target_stamp": stamp_text(target_stamp), "stamp": str(stamp_key),
                              "absolute_dt_s": fnum(float(stamp_key) - target_stamp),
                              "final_translation_error_m": selected[f"{pipe}_final_translation_error_m"],
                              "final_rotation_error_deg": selected[f"{pipe}_final_rotation_error_deg"],
                              "raw_translation_error_m": selected[f"{pipe}_raw_translation_error_m"],
                              "raw_rotation_error_deg": selected[f"{pipe}_raw_rotation_error_deg"],
                              "fitness": ndt["fitness"], "iterations": ndt["iterations"],
                              "converged": ndt["converged"],
                              "limited": int(ndt["translation_limited"] or ndt["rotation_limited"])})
    high_rows.append({"row_type": "raw_imu_window_audit", "window": "RAW_IMU_SHOCK",
                      "pipeline": "raw_imu", "pose": "sensor_measurement", "sample_count": len(shock_samples),
                      "raw_imu_peak_timestamp": stamp_text(peak["stamp"]),
                      "raw_acc_norm_mean_mps2": fnum(np.mean([s["acc_norm"] for s in shock_samples])),
                      "raw_acc_norm_p95_mps2": fnum(np.percentile([s["acc_norm"] for s in shock_samples], 95)),
                      "raw_acc_norm_max_mps2": fnum(peak["acc_norm"]),
                      "raw_gyro_norm_at_peak_radps": fnum(peak["gyro_norm"]),
                      "interpretation_limit": "RAW_IMU_SHOCK only; no physical collision or causal claim"})

    # Exact, machine-readable frame intersection.
    frame_header = list(common_rows[0].keys()) if common_rows else []
    write_csv(args.out / "p3_r9c_common_frames.csv", frame_header, common_rows)
    write_csv(args.out / "p3_r9c_gt_alignment_audit.csv",
              ["pipeline", "stamp", "gt_supported", "exclude_reason", "gt_left_stamp", "gt_right_stamp",
               "gt_bracket_width_s", "nearest_gt_dt_s"], gt_audit_rows)
    frame_fields = list(frame_metrics[0].keys()) if frame_metrics else []
    write_csv(args.out / "p3_r9c_frame_metrics.csv", frame_fields, frame_metrics)
    global_fields = list(global_rows[0].keys()) if global_rows else []
    write_csv(args.out / "p3_r9c_global_summary.csv", global_fields, global_rows)
    win_fields = list(window_summary[0].keys()) if window_summary else []
    write_csv(args.out / "p3_r9c_window_summary.csv", win_fields, window_summary)
    crossing_fields = list(crossing_rows[0].keys()) + ["rapid_1m_to_5m_s"]
    rapid_by = {row["pipeline"]: row["rapid_1m_to_5m_s"] for row in rapid_rows}
    for row in crossing_rows:
        row["rapid_1m_to_5m_s"] = rapid_by[row["pipeline"]]
    write_csv(args.out / "p3_r9c_crossings.csv", crossing_fields, crossing_rows)
    write_csv(args.out / "p3_r9c_bootstrap.csv",
              ["population", "pose", "metric", "n", "seed", "rng", "replicates", "mean", "mean_ci_low",
               "mean_ci_high", "median", "median_ci_low", "median_ci_high"], bootstrap_rows)
    write_csv(args.out / "p3_r9c_better_tie_counts.csv",
              ["pose", "metric", "n", "mature_better", "legacy_better", "tie", "tie_tolerance"], better_rows)
    high_fields = sorted({key for row in high_rows for key in row})
    write_csv(args.out / "p3_r9c_high_dynamic_summary.csv", high_fields, high_rows)

    # Input manifest output includes actual computed hashes; GT is marked POST_GATE.
    input_manifest = []
    verified_by = {row["name"]: row for row in verified}
    for name, src in inputs.items():
        status = verified_by.get(name)
        input_manifest.append({"name": name, "path": src["path"],
                               "expected_sha256": src["expected_sha256"],
                               "actual_sha256": status["actual_sha256"] if status else "",
                               "size_bytes": status["size_bytes"] if status else src.get("expected_size_bytes", ""),
                               "verification_stage": "POST_PARAMETER_GATE" if name == GT_ROLE else "PRE_GT_GATE",
                               "verification": status["verification"] if status else "NOT_READ"})
    write_csv(args.out / "p3_r9c_input_manifest.csv",
              ["name", "path", "expected_sha256", "actual_sha256", "size_bytes", "verification_stage", "verification"],
              input_manifest)

    build_plots(args.out, common_rows, window_summary, str(anchor_key))

    # Determine the descriptive classification using persistent failure, full-population metrics,
    # and the already-frozen stage windows. Deltas are legacy minus mature (positive favors mature).
    legacy_cross = crossing_data["legacy"]
    mature_cross = crossing_data["mature_imu"]
    legacy_severe = legacy_cross[1.0]["persistent_crossing_s"] is not None
    mature_severe = mature_cross[1.0]["persistent_crossing_s"] is not None
    legacy_five = legacy_cross[5.0]["persistent_crossing_s"] is not None
    mature_five = mature_cross[5.0]["persistent_crossing_s"] is not None
    w3 = next(row for row in window_summary if row["window"] == "W3_P050")
    w4 = next(row for row in window_summary if row["window"] == "W4_RAPID")
    w0 = next(row for row in window_summary if row["window"] == "W0_PRE")
    w1 = next(row for row in window_summary if row["window"] == "W1_TRANSIENT")
    d3, d4 = w3["paired_final_t_delta_mean_m"], w4["paired_final_t_delta_mean_m"]
    early_deltas = [w0["paired_final_t_delta_mean_m"], w1["paired_final_t_delta_mean_m"]]
    global_t_delta = next(row["translation_mean_m"] for row in global_rows
                          if row["population"] == "PAIRED_COMMON" and
                          row["pipeline"] == "legacy_minus_mature" and row["pose_type"] == "final_used")
    global_r_delta = next(row["rotation_mean_deg"] for row in global_rows
                          if row["population"] == "PAIRED_COMMON" and
                          row["pipeline"] == "legacy_minus_mature" and row["pose_type"] == "final_used")
    early_to_late_sign_change = (any(delta < -1e-6 for delta in early_deltas) and
                                 any(delta > 1e-6 for delta in (d3, d4))) or (
                                 any(delta > 1e-6 for delta in early_deltas) and
                                 any(delta < -1e-6 for delta in (d3, d4)))
    global_metric_sign_change = global_t_delta * global_r_delta < 0.0
    rapid_by_pipeline = {row["pipeline"]: row["rapid_1m_to_5m_s"] for row in rapid_rows}
    rapid_intervals = {
        pipeline: (None if value == "NOT_AVAILABLE" else float(value))
        for pipeline, value in rapid_by_pipeline.items()
    }
    common_sample_period_s = float(np.median(np.diff([float(key) for key in common_keys])))
    legacy_rapid = rapid_intervals["legacy"]
    mature_rapid = rapid_intervals["mature_imu"]
    both_rapid_available = legacy_rapid is not None and mature_rapid is not None
    one_rapid_available = (legacy_rapid is None) != (mature_rapid is None)
    rapid_delta_s = (mature_rapid - legacy_rapid) if both_rapid_available else None
    rapid_within_one_sample = (both_rapid_available and
                               abs(rapid_delta_s) <= common_sample_period_s)
    if one_rapid_available:
        rapid_relation = "ONE_PIPELINE_ONLY"
    elif not both_rapid_available:
        rapid_relation = "BOTH_NOT_AVAILABLE"
    elif rapid_within_one_sample:
        rapid_relation = "WITHIN_ONE_COMMON_NDT_FRAME"
    else:
        rapid_relation = "DIFFERS_BY_MORE_THAN_ONE_COMMON_NDT_FRAME"
    rapid_disagreement = one_rapid_available or (both_rapid_available and not rapid_within_one_sample)
    if legacy_severe and not mature_severe:
        failure_class = "MATURE_IMU_MATERIALLY_CHANGES_FAILURE"
    elif legacy_five != mature_five or rapid_disagreement or (
            both_rapid_available and (early_to_late_sign_change or global_metric_sign_change)):
        failure_class = "MIXED_STAGE_DEPENDENT_EFFECT"
    else:
        failure_class = "FAILURE_LARGELY_PERSISTS_UNDER_MATURE_IMU"
    transfer = ("REQUIRES_RETEST_ON_MATURE_PIPELINE"
                if failure_class == "FAILURE_LARGELY_PERSISTS_UNDER_MATURE_IMU"
                else "NOT_DIRECTLY_TRANSFERABLE")

    def crossing_text(pipe):
        return "; ".join(f"{threshold:g}m={fnum(crossing_data[pipe][threshold]['persistent_crossing_s']) if crossing_data[pipe][threshold]['persistent_crossing_s'] is not None else 'NOT_REACHED'}s"
                         for threshold in (0.25, 0.5, 1.0, 2.0, 5.0))

    final_global = {(r["population"], r["pipeline"], r["pose_type"]): r for r in global_rows}
    legacy_g = final_global[("FULL_SUPPORTED", "legacy", "final_used")]
    mature_g = final_global[("FULL_SUPPORTED", "mature_imu", "final_used")]
    pair_g = final_global[("PAIRED_COMMON", "legacy_minus_mature", "final_used")]
    boot_lookup = {(r["pose"], r["metric"]): r for r in bootstrap_rows}
    trans_mean_boot = boot_lookup[("final_used", "translation")]
    trans_med_boot = trans_mean_boot
    rot_mean_boot = boot_lookup[("final_used", "rotation")]
    better_lookup = {(r["pose"], r["metric"]): r for r in better_rows}
    final_t_better = better_lookup[("final_used", "translation")]
    final_r_better = better_lookup[("final_used", "rotation")]
    hq_n = len(hq_keys)
    legacy_early = sum(gt_cache[r["stamp_key"]] is None and r["stamp"] < gt_times[0] for r in legacy)
    legacy_late = sum(gt_cache[r["stamp_key"]] is None and r["stamp"] > gt_times[-1] for r in legacy)
    mature_early = sum(gt_cache[r["stamp_key"]] is None and r["stamp"] < gt_times[0] for r in mature)
    mature_late = sum(gt_cache[r["stamp_key"]] is None and r["stamp"] > gt_times[-1] for r in mature)
    startup_audit = next(row for row in equivalence if row["parameter"] == "logged initial-guess source/reason")
    def summary_stats(row, prefix, unit):
        return ", ".join(f"{name}={fnum(row[f'{prefix}_{name}_{unit}'])}{' m' if unit == 'm' else ' deg'}"
                         for name in ("mean", "rmse", "median", "p95", "max"))

    summary = f"""# P3-R9C summary

## Gate and populations

- Parameter-equivalence gate: PASS; unexpected NDT-affecting differences: 0.
- Formal legacy Run A NDT outputs: {len(legacy)}; mature Run A: {len(mature)}.
- Exact common NDT timestamps: {len(common_keys)}; legacy-only: {len(legacy_keys-set(common_keys))}; mature-only: {len(mature_keys-set(common_keys))}.
- GT-supported full populations: legacy {len(legacy_supported)}; mature {len(mature_supported)}.
- Common GT-supported paired population: {len(common_supported)}; HQ bracket <=0.25 s: {hq_n}.
- Common anchor: `{anchor_key}`; all raw/final anchor errors are zero within numerical tolerance (max={anchor_max:.3g}).
- GT support exclusions (no extrapolation): legacy early={legacy_early}, late={legacy_late}; mature early={mature_early}, late={mature_late}.
- GT bracket mean/P95/max on common pairs: {np.mean([gt_cache[k]['width'] for k in common_supported]):.6f}/ {np.percentile([gt_cache[k]['width'] for k in common_supported],95):.6f}/ {max(gt_cache[k]['width'] for k in common_supported):.6f} s.
- Expected startup history difference: `{startup_audit['status']}`; only the first exact common frame has a different logged initial-guess source/prior availability, as recorded in R9B PREINIT_REJECTED. This is disclosed, not treated as numeric initialization equivalence.

## Full GT-supported final_used metrics

- Legacy translation: {summary_stats(legacy_g, 'translation', 'm')}; rotation: {summary_stats(legacy_g, 'rotation', 'deg')}.
- Mature IMU translation: {summary_stats(mature_g, 'translation', 'm')}; rotation: {summary_stats(mature_g, 'rotation', 'deg')}.
- Paired-common final translation legacy-minus-mature: mean={fnum(pair_g['translation_mean_m'])} m, bootstrap 95% CI [{fnum(trans_mean_boot['mean_ci_low'])}, {fnum(trans_mean_boot['mean_ci_high'])}]; median={fnum(pair_g['translation_median_m'])} m, bootstrap 95% CI [{fnum(trans_med_boot['median_ci_low'])}, {fnum(trans_med_boot['median_ci_high'])}].
- Paired-common final rotation legacy-minus-mature: mean={fnum(pair_g['rotation_mean_deg'])} deg, bootstrap 95% CI [{fnum(rot_mean_boot['mean_ci_low'])}, {fnum(rot_mean_boot['mean_ci_high'])}]; median={fnum(pair_g['rotation_median_deg'])} deg, bootstrap 95% CI [{fnum(rot_mean_boot['median_ci_low'])}, {fnum(rot_mean_boot['median_ci_high'])}].
- Paired final translation counts (tie tolerance 1e-6 m): mature better={final_t_better['mature_better']}, legacy better={final_t_better['legacy_better']}, tie={final_t_better['tie']}; rotation counts (1e-6 deg): mature better={final_r_better['mature_better']}, legacy better={final_r_better['legacy_better']}, tie={final_r_better['tie']}.
- Raw-NDT and final-used metrics are separately retained in `p3_r9c_global_summary.csv`; per-frame raw-to-final pose changes are separately retained as translation/rotation deltas and are not interpreted causally.

## Persistent crossings (seconds relative to the common anchor; exact sample stamps in CSV)

- Legacy: {crossing_text('legacy')}.
- Mature IMU: {crossing_text('mature_imu')}.
- Rapid persistent 1 m to 5 m: {rapid_rows[0]['pipeline']}={rapid_rows[0]['rapid_1m_to_5m_s']} s; {rapid_rows[1]['pipeline']}={rapid_rows[1]['rapid_1m_to_5m_s']} s.
- Rapid classifier evidence: relation=`{rapid_relation}`, mature-minus-legacy interval={fnum(rapid_delta_s)} s, one-sample resolution={fnum(common_sample_period_s)} s. Resolution is the median spacing of exact common NDT timestamps; the classifier treats an interval difference within that measured cadence as the same rapid-failure timing pattern.

## Classification

- FAILURE_CLASSIFICATION: `{failure_class}`. The classifier consumes 1 m/5 m persistent status and the 1 m→5 m interval relation as well as paired full-population metric signs and the W0/W1-versus-W3/W4 translation-delta sign pattern. A rapid interval unavailable on only one pipeline or a difference greater than one median common-frame interval is a mixed-effect signal; when paired/window signs conflict, both rapid intervals must be present for the mixed label. The one-frame resolution is data-derived from this run's common NDT sampling cadence, not a claim of a preregistered scientific threshold.
- OLD-MECHANISM TRANSFERABILITY: `{transfer}`.
- W3 paired final translation mean delta (legacy−mature): {fnum(d3)} m; W4: {fnum(d4)} m.
- W0/W1 paired final translation deltas (legacy−mature): {fnum(early_deltas[0])}/{fnum(early_deltas[1])} m; full paired final translation/rotation mean deltas: {fnum(global_t_delta)} m / {fnum(global_r_delta)} deg.
- Persistent severe thresholds remain present in both pipelines: legacy 1 m={legacy_cross[1.0]['persistent_crossing_s'] is not None}, 5 m={legacy_five}; mature IMU 1 m={mature_cross[1.0]['persistent_crossing_s'] is not None}, 5 m={mature_five}. The mixed label reflects different error effects across stages/metrics; it does not mean the sustained failure disappeared.
- The detailed raw-NDT/final-used, window, bootstrap, better/tie-count, and high-dynamic tables are in the accompanying CSVs. NDT fitness/iterations/convergence are descriptive only.

This is a comparison of two closed-loop motion-compensation infrastructures, not an isolated deskew ablation. No physical root cause, collision causality, wrong mode, multimodality, Hessian/geometry degeneracy, or novelty is claimed. `P4_ALLOWED = NO`.
"""
    (args.out / "p3_r9c_summary.md").write_text(summary, encoding="utf-8")
    transfer_text = f"""# P3-R9C old-mechanism evidence transferability

Decision: `{transfer}`.

Failure comparison category: `{failure_class}`.

Evidence to use: common-anchor paired full-population metrics, the five predeclared W0–W4 windows, recomputed persistent crossings, and the rapid 1 m→5 m interval. W3 paired translation mean delta (legacy−mature) is {d3:.6f} m; W4 is {d4:.6f} m. Legacy persistent crossings: {crossing_text('legacy')}. Mature IMU persistent crossings: {crossing_text('mature_imu')}.

R7I/R8 fixed-cloud counterfactuals are conditional on legacy CV-deskew observations. Because R9B changes a closed-loop infrastructure (IMU propagation → deskew → NDT → EKF correction → later state/deskew), those results do not by themselves establish a mature-pipeline mechanism. If the failure largely persists, the old evidence still requires a mature-pipeline retest before being treated as mechanism evidence. If the timeline materially changes or is stage-dependent, do not transfer the old mechanism interpretation directly.

No causal claim is made that point-wise deskew alone caused an outcome. No physical root cause, collision causality, wrong mode, multimodality, or Hessian/geometry degeneracy is claimed. `P4_ALLOWED = NO`.
"""
    (args.out / "p3_r9c_transferability.md").write_text(transfer_text, encoding="utf-8")
    (args.out / "P3_R9C_PROTOCOL.md").write_text((HERE / "P3_R9C_PROTOCOL.md").read_text(encoding="utf-8"), encoding="utf-8")
    shutil.copy2(args.manifest, args.out / "p3_r9c_manifest.csv")
    shutil.copy2(__file__, args.out / "analyze_p3_r9c.py")

    def markdown_table(headers, rows):
        lines = ["| " + " | ".join(headers) + " |",
                 "| " + " | ".join("---" for _ in headers) + " |"]
        lines.extend("| " + " | ".join(str(value) for value in row) + " |" for row in rows)
        return "\n".join(lines)

    def stats_triplet(row, prefix, pose, short, unit):
        return "/".join(fnum(row[f"{prefix}_{pose}_{short}_{stat}_{unit}"])
                         for stat in ("mean", "median", "p95"))

    def window_table(pose, axis):
        rows = []
        field = "translation_error_m" if axis == "translation" else "rotation_error_deg"
        short = "t" if axis == "translation" else "r"
        unit = "m" if axis == "translation" else "deg"
        for win in window_summary:
            rows.append([
                win["window"], win["n_common"],
                stats_triplet(win, "legacy", pose, short, unit),
                stats_triplet(win, "mature", pose, short, unit),
                fnum(win[f"paired_{pose}_{short}_delta_mean_{unit}"]),
                fnum(win[f"paired_{pose}_{short}_delta_median_{unit}"]),
            ])
        return markdown_table(["Window", "n", "Legacy mean/median/P95", "Mature mean/median/P95",
                               "paired Δ mean", "paired Δ median"], rows)

    def global_metric_table(pose_type):
        rows = []
        for population, pipe in (("FULL_SUPPORTED", "legacy"), ("FULL_SUPPORTED", "mature_imu"),
                                 ("PAIRED_COMMON", "legacy"), ("PAIRED_COMMON", "mature_imu"),
                                 ("PAIRED_COMMON", "legacy_minus_mature"), ("HQ_COMMON", "legacy"),
                                 ("HQ_COMMON", "mature_imu")):
            item = final_global.get((population, pipe, pose_type))
            if item is None:
                continue
            rows.append([population, pipe, item["count"],
                         fnum(item["translation_mean_m"]), fnum(item["translation_rmse_m"]),
                         fnum(item["translation_median_m"]), fnum(item["translation_p95_m"]),
                         fnum(item["translation_max_m"]), fnum(item["rotation_mean_deg"]),
                         fnum(item["rotation_rmse_deg"]), fnum(item["rotation_median_deg"]),
                         fnum(item["rotation_p95_deg"]), fnum(item["rotation_max_deg"])])
        return markdown_table(["Population", "pipeline", "n", "t mean", "t RMSE", "t median", "t P95", "t max",
                               "r mean", "r RMSE", "r median", "r P95", "r max"], rows)

    bootstrap_table = markdown_table(
        ["pose", "metric", "n", "Δ mean", "mean 95% CI", "Δ median", "median 95% CI"],
        [[row["pose"], row["metric"], row["n"], fnum(row["mean"]),
          f"[{fnum(row['mean_ci_low'])}, {fnum(row['mean_ci_high'])}]", fnum(row["median"]),
          f"[{fnum(row['median_ci_low'])}, {fnum(row['median_ci_high'])}]"] for row in bootstrap_rows])
    better_table = markdown_table(
        ["pose", "metric", "n", "mature better", "legacy better", "tie", "tie tolerance"],
        [[r["pose"], r["metric"], r["n"], r["mature_better"], r["legacy_better"], r["tie"], r["tie_tolerance"]]
         for r in better_rows])
    crossing_table = markdown_table(
        ["Pipeline", "threshold", "instant status / s / stamp", "persistent status / s / stamp", "duration s"],
        [[r["pipeline"], r["threshold_m"],
          f"{r['instant_status']} / {fnum(r['instant_relative_common_s']) or '—'} / {r['instant_stamp'] or '—'}",
          f"{r['persistent_status']} / {fnum(r['persistent_relative_common_s']) or '—'} / {r['persistent_stamp'] or '—'}",
          fnum(r["persistent_duration_s"])] for r in crossing_rows])
    param_table = markdown_table(
        ["Parameter", "Legacy", "Mature", "Status"],
        [[r["parameter"], r["legacy"], r["mature"], r["status"]] for r in equivalence])
    window_definition_table = markdown_table(
        ["Window", "absolute start stamp", "absolute end stamp", "R7I relative bounds / definition"],
        [[r["window"], r["absolute_start_stamp"], r["absolute_end_stamp"],
          (f"{r['old_relative_start_s']}–{r['old_relative_end_s']} s" if r["old_relative_start_s"] else r["definition"])]
         for r in window_summary])

    def manifest_item(name, field):
        return inputs[name][field]

    high_window_rows = [r for r in high_rows if r["row_type"] == "window_summary"]
    high_sample_rows = [r for r in high_rows if r["row_type"] == "sample_nearest_target"]
    high_audit_rows = [r for r in high_rows if r["row_type"] == "raw_imu_window_audit"]
    high_windows_table = markdown_table(
        ["Window", "pipeline", "pose", "n", "translation mean/median/P95 (m)",
         "rotation mean/median/P95 (deg)", "fitness mean", "iterations mean", "convergence"],
        [[r["window"], r["pipeline"], r["pose"], r["n_common"],
          "/".join(fnum(r[k]) for k in ("translation_mean_m", "translation_median_m", "translation_p95_m")),
          "/".join(fnum(r[k]) for k in ("rotation_mean_deg", "rotation_median_deg", "rotation_p95_deg")),
          fnum(r["fitness_mean"]), fnum(r["iterations_mean"]), fnum(r["convergence_fraction"])]
         for r in high_window_rows])
    high_samples_table = markdown_table(
        ["Target", "pipeline", "target stamp", "nearest NDT stamp", "dt s", "final t/r error", "raw t/r error",
         "fitness", "iterations", "converged", "limited"],
        [[r["window"], r["pipeline"], r["target_stamp"], r["stamp"], r["absolute_dt_s"],
          f"{r['final_translation_error_m']} m / {r['final_rotation_error_deg']} deg",
          f"{r['raw_translation_error_m']} m / {r['raw_rotation_error_deg']} deg",
          r["fitness"], r["iterations"], r["converged"], r["limited"]] for r in high_sample_rows])
    shock_audit = high_audit_rows[0]

    limiter_rows = []
    for pipe, prefix in (("legacy", "legacy"), ("mature_imu", "mature_imu")):
        for suffix, unit in (("translation_delta_m", "m"), ("rotation_delta_deg", "deg")):
            vals = np.asarray([float(r[f"{prefix}_raw_to_final_{suffix}"]) for r in common_rows])
            limiter_rows.append([pipe, unit, fnum(np.mean(vals)), fnum(np.percentile(vals, 95)), fnum(np.max(vals))])
    limiter_table = markdown_table(["Pipeline", "delta", "mean", "P95", "max"], limiter_rows)

    report = f"""# PAPER-P3-R9C — Floor01 motion-compensation pipeline accuracy comparison

## Scope and provenance

This compares two closed-loop motion-compensation infrastructures; it is not an isolated deskew-only ablation. No runtime algorithm, configuration, launch, map, or NDT source was changed, and no localization run was performed. GT was accessed only after the NDT-affecting parameter-equivalence gate passed and was used post-hoc only.

- Paper start commit: `2804be14cb78449d3dd8afefb915719082cb7e03`; source branch `paper`.
- Frozen baseline: `feature/visual-factor-window` at `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`.
- Legacy formal R7H Run A result: `{manifest_item('legacy_ndt_runA', 'path')}` (SHA-256 `{manifest_item('legacy_ndt_runA', 'expected_sha256')}`); effective config SHA-256 `{manifest_item('legacy_effective_params', 'expected_sha256')}`.
- Mature formal R9B full Run A result: `{manifest_item('mature_ndt_runA', 'path')}` (SHA-256 `{manifest_item('mature_ndt_runA', 'expected_sha256')}`); profile SHA-256 `{manifest_item('mature_profile', 'expected_sha256')}`; algorithm commit `2804be14cb78449d3dd8afefb915719082cb7e03`.
- Legacy NDT output span: `{legacy[0]['stamp_key']}`–`{legacy[-1]['stamp_key']}` ({len(legacy)} rows); mature NDT output span: `{mature[0]['stamp_key']}`–`{mature[-1]['stamp_key']}` ({len(mature)} rows).
- Shared raw bag: `{manifest_item('raw_input_bag', 'path')}` (SHA-256 `{manifest_item('raw_input_bag', 'expected_sha256')}`).
- Shared normalized H1 map SHA-256 `{manifest_item('h1_map', 'expected_sha256')}`; Floor01 SP1 calibration SHA-256 `{manifest_item('floor01_calibration', 'expected_sha256')}`; NDT implementation SHA-256 `{manifest_item('mature_ndt_source', 'expected_sha256')}`.
- Complete absolute paths and verified hashes: `p3_r9c_input_manifest.csv`; pre-registered windows and evaluator protocol: `P3_R9C_PROTOCOL.md`.

## Parameter-equivalence gate

Gate: **PASS**; unexpected differences: **{len(unexpected)}**. SAME parameters and the explicitly allowed closed-loop pipeline differences are itemized below. The old extrinsic is extracted from the hash-verified R7H adapter launch referenced by its Run A launch transcript and compared independently against the official calibration; the mature extrinsic is read from the manifest-hashed R9B profile, whose load path, calibration hash prefix, and runtime translation are checked in the formal launch record. The old NDT source SHA is parsed from R7H run provenance and compared to the hash-verified source in the R9B frozen-start workspace. The single first-common-frame initial-guess/prior-history mismatch is recorded as an expected startup-availability difference (mature PREINIT_REJECTED), not hidden or described as identical numeric initialization. From later common frames the logged source/reason and prior-use match.

{param_table}

## Population, time alignment, and evaluator

- NDT outputs: legacy {len(legacy)}, mature {len(mature)}; exact timestamp intersection {len(common_keys)}; legacy-only {len(legacy_keys-set(common_keys))}; mature-only {len(mature_keys-set(common_keys))}.
- GT-supported full samples: legacy {len(legacy_supported)} (early excluded {legacy_early}, late excluded {legacy_late}); mature {len(mature_supported)} (early excluded {mature_early}, late excluded {mature_late}). Paired common GT-supported samples {len(common_supported)}; HQ bracket `<=0.25 s` samples {hq_n}.
- Common anchor `{anchor_key}`; maximum absolute translation/rotation error among all eight raw/final anchor checks `{anchor_max:.3g}`.
- Common paired GT bracket width mean/P95/max: {np.mean([gt_cache[k]['width'] for k in common_supported]):.6f}/{np.percentile([gt_cache[k]['width'] for k in common_supported],95):.6f}/{max(gt_cache[k]['width'] for k in common_supported):.6f} s. No extrapolation.
- Reused unchanged P3-R3B conventions: translation linear interpolation, quaternion SLERP, Floor01 IMU-origin GT transformed with the official Floor01 calibration, and first-common-pair relative evaluation.

## Global metrics (translation m; rotation deg)

Final-used full and paired/HQ populations:

{global_metric_table('final_used')}

Raw-NDT full and paired/HQ populations:

{global_metric_table('raw')}

Paired bootstrap uses PCG64, seed `{BOOTSTRAP_SEED}`, `{BOOTSTRAP_REPLICATES}` resamples, percentile 95% CI. Delta is legacy minus mature; positive favors mature on reference error.

{bootstrap_table}

{better_table}

Raw-to-final NDT LiDAR-pose output change (map-frame translation displacement and relative rotation) is reported descriptively; it is not a causal estimate of limiter benefit:

{limiter_table}

## Frozen windows

For each cell, pipeline statistics are mean/median/P95 on the exact paired common frames. Delta is legacy minus mature.

The absolute boundaries below are taken from the frozen R7I metadata. The broad interval is mapped from the original R7I relative axis; `RAW_IMU_SHOCK` is the separately preidentified absolute IMU-only interval.

{window_definition_table}

### Final-used translation (m)

{window_table('final', 'translation')}

### Final-used rotation (deg)

{window_table('final', 'rotation')}

### Raw-NDT translation (m)

{window_table('raw', 'translation')}

### Raw-NDT rotation (deg)

{window_table('raw', 'rotation')}

## Recomputed persistent crossings

Both pipelines use the same common-anchor-relative paired population and the reused rule: strict `>`, maximum continuity gap `0.25 s`, persistent duration `>=5 s`. Absolute stamps below are exact source-sample stamps.

{crossing_table}

Persistent 1 m→5 m intervals: legacy `{rapid_rows[0]['rapid_1m_to_5m_s']} s`; mature `{rapid_rows[1]['rapid_1m_to_5m_s']} s`. Their relation is `{rapid_relation}` (mature−legacy `{fnum(rapid_delta_s)} s`; median exact-common NDT sample spacing `{fnum(common_sample_period_s)} s`). The classifier uses this rapid-divergence comparison: one-sided availability or a difference exceeding one measured common-frame spacing is a mixed-effect signal; a paired/window mixed effect is accepted only when both rapid intervals exist.

## High-dynamic audit

The raw-IMU preidentified peak is at `{shock_audit['raw_imu_peak_timestamp']}`; the exact ±1 s audit has {shock_audit['sample_count']} IMU samples, acceleration-norm mean/P95/max `{shock_audit['raw_acc_norm_mean_mps2']}/{shock_audit['raw_acc_norm_p95_mps2']}/{shock_audit['raw_acc_norm_max_mps2']} m/s²`, and gyro norm at peak `{shock_audit['raw_gyro_norm_at_peak_radps']} rad/s`.

{high_windows_table}

Nearest NDT samples before/at/after the peak target:

{high_samples_table}

Allowed interpretation: this is a raw-IMU high-dynamic neighborhood and a descriptive pipeline comparison. It does **not** establish a wall collision, physical impact, or causal relation.

## Decision and limits

- FAILURE_CLASSIFICATION: `{failure_class}`. Both pipelines show persistent severe thresholds. The classifier jointly uses persistent 1 m/5 m crossing status, the measured 1 m→5 m interval relation (within or beyond one median common-frame spacing), full-population paired translation/rotation delta signs, and the W0/W1-versus-W3/W4 translation-delta pattern. The mixed label means stage/metric-dependent behavior, not that the large failure regime was removed. Its one-frame timing tolerance is derived from observed common NDT timestamp spacing, not a preregistered threshold.
- OLD-MECHANISM TRANSFERABILITY: `{transfer}`. Prior R7I/R8 fixed-cloud evidence remains conditional on legacy observations and must be retested on mature pipeline; persistent failure alone does not prove identical mechanism.
- R7I/R8 findings are not upgraded to a causal explanation. No deskew-only cause, physical root cause, wrong mode, multimodality, Hessian/geometry degeneracy, or novelty claim.
- `P4_ALLOWED = NO`. No R8A, tuning, runtime edits, visual fusion, or new algorithm work.

## Reproducibility and protection

`analyze_p3_r9c.py` regenerates the CSV/PNG/Markdown result bundle from the manifest. The script verifies hashes, checks the NDT parameter gate before loading GT, extracts the frozen RAW_IMU_SHOCK directly from the shared raw bag without rerunning localization, and records all output populations. Runtime directories were not written by this stage; final Git verification is reported in the handoff.

Artifacts: `p3_r9c_common_frames.csv`, `p3_r9c_frame_metrics.csv`, `p3_r9c_gt_alignment_audit.csv`, `p3_r9c_global_summary.csv`, `p3_r9c_window_summary.csv`, `p3_r9c_crossings.csv`, `p3_r9c_high_dynamic_summary.csv`, `p3_r9c_bootstrap.csv`, `p3_r9c_better_tie_counts.csv`, parameter/input manifests, and five PNG plots.
"""
    formal_report_path = args.out / "P3_R9C_FLOOR01_MOTION_COMPENSATION_ACCURACY_COMPARISON.md"
    formal_report_path.write_text(report, encoding="utf-8")

    # Copy only the compact review bundle into the paper repository.
    analyzer_copy = args.out / "analyze_p3_r9c.py"
    if Path(__file__).resolve() != analyzer_copy.resolve():
        shutil.copy2(Path(__file__).resolve(), analyzer_copy)
    args.repo_artifacts.mkdir(parents=True, exist_ok=True)
    bundle = [p for p in args.out.iterdir() if p.is_file() and p.suffix.lower() in (".csv", ".png", ".md", ".py")]
    for path in bundle:
        shutil.copy2(path, args.repo_artifacts / path.name)
    shutil.copy2(formal_report_path, args.repo_artifacts.parent / formal_report_path.name)

    print(f"P3_R9C_ANALYSIS_PASS common={len(common_supported)} anchor={anchor_key}")
    print(f"failure_class={failure_class} transferability={transfer}")
    print(f"outputs={args.out}")


if __name__ == "__main__":
    main()
