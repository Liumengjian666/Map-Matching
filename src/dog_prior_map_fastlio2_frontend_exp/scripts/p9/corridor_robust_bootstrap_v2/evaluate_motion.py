"""V2 admitted-observation adapter for the frozen moving-state diagnostics.

No accepted full state or covariance is produced by this program.  Even when
all reusable mean/observability gates pass, full coupled covariance and its
propagation to boot remain a mandatory, separate prerequisite for injection.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

OLD_DIRECTORY = Path(__file__).resolve().parent.parent / "corridor_moving_init"
sys.path.insert(0, str(OLD_DIRECTORY))
from diagnose_motion import fit, stage_imu  # Never execute diagnose().
from preintegration import integrate, log

ROOT = Path(__file__).resolve().parents[5]
OLD_CONFIG = ROOT / "docs/p9_r7_cross_dataset_nearoptimal/moving_initialization/bootstrap_config.json"


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def load_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def write_csv(path, rows):
    require(bool(rows), "empty CSV must use explicit NOT_RUN receipt")
    with Path(path).open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def verify_inputs(input_dir, freeze):
    input_dir = Path(input_dir)
    manifest_path = input_dir / "input_manifest.json"
    require(sha(manifest_path) == freeze["input_manifest_sha256"], "input manifest SHA mismatch")
    manifest = load_json(manifest_path)
    required = {"imu.csv", "filter_scans.csv", "raw_timed_scan_index.csv", "raw_timed_points.bin"}
    require(required.issubset(manifest["input_files"]), "consumed input omitted from manifest")
    for name, record in manifest["input_files"].items():
        path = input_dir / name
        require(path.resolve() == Path(record["path"]).resolve(), "manifest input path mismatch")
        require(path.stat().st_size == record["bytes"] and sha(path) == record["sha256"],
                "frozen input SHA mismatch: " + name)
        require(freeze["input_hashes"].get(name) == record, "input receipt changed: " + name)
    return manifest


def numeric_gate(name, value, limit):
    accepted = value is not None and np.isfinite(value) and value <= limit
    return {"gate": name, "value": value, "limit": limit, "pass": bool(accepted)}


def mean_gates(diagnostics, cfg):
    gates = []
    rank = diagnostics["profile_rank"]
    gates.append({"gate": "profile_rank", "value": rank, "limit": 5, "pass": rank == 5})
    lower = diagnostics["sigma_min_lower_bound"]
    gates.append({"gate": "sigma_min_lower_bound_positive", "value": lower,
                  "limit": 0, "pass": bool(np.isfinite(lower) and lower > 0)})
    for value_key, limit_key in [
        ("condition", "profile_condition_max"),
        ("robust_condition", "profile_condition_max"),
        ("gyro_residual_rmse_deg", "gyro_residual_rmse_max_deg"),
        ("position_residual_rmse_m", "position_residual_rmse_max_m"),
        ("velocity_residual_rmse_m_s", "velocity_residual_rmse_max_m_s"),
        ("split_bg_change_rad_s", "split_gyro_bias_change_max_rad_s"),
        ("split_ba_change_m_s2", "split_accel_bias_change_max_m_s2"),
        ("split_gravity_angle_deg", "split_gravity_change_max_deg"),
        ("validation_translation_rmse_m", "validation_position_rmse_max_m"),
        ("validation_rotation_rmse_deg", "validation_rotation_rmse_max_deg"),
    ]:
        gates.append(numeric_gate(value_key, diagnostics[value_key], cfg[limit_key]))
    gates.append({"gate": "gyro_solver_success", "value": diagnostics["solver_success"],
                  "limit": True, "pass": bool(diagnostics["solver_success"])})
    return gates


def observation_uncertainty(row, lever_length, lidar_cfg):
    """Conservative engineering marginal scales, not calibrated pose covariance.

    Do not divide correspondence residual by sqrt(number of points): shared
    scan, submap and gyro errors are correlated.  The reused fitter separately
    retains the original worst-residual-block correlation inflation.
    """
    # The frozen V2 binary computes these from the separately archived actual
    # overlap residual, inverse consistency, gyro mismatch/bias sensitivity and
    # unknown-translation scan sweep. They remain engineering marginal scales.
    terms = [float(row[key]) for key in ("position_sigma_m", "rotation_sigma_deg")]
    require(all(np.isfinite(x) and x >= 0 for x in terms), "invalid observation uncertainty evidence")
    position, angle = terms
    require(position >= lidar_cfg["pose_position_sigma_m"]
            and angle >= lidar_cfg["pose_rotation_sigma_deg"], "V2 uncertainty below frozen floor")
    sr, sp = np.deg2rad(angle), position
    return float(np.hypot(sp, lever_length * sr)), float(sr)


def accepted_rows(rows):
    """Prediction records remain ledger entries, never pose observations."""
    return [r for r in rows if r["role"] == "LIDAR_OBSERVATION"
            and r["true_observation"] == "1" and r["quality_pass"] == "1"]


def validate_pose_rows(rows):
    stamps = np.array([int(r["stamp_ns"]) for r in rows], dtype=np.int64)
    require(len(stamps) >= 3 and np.all(np.diff(stamps) > 0), "insufficient or unordered real observations")
    poses = np.array([[float(r["T" + str(i) + str(j)]) for i in range(4) for j in range(4)]
                      for r in rows]).reshape(-1, 4, 4)
    for T in poses:
        require(np.isfinite(T).all() and np.linalg.norm(T[3] - [0, 0, 0, 1]) < 1e-8,
                "invalid homogeneous pose")
        R = T[:3, :3]
        require(np.linalg.norm(R.T @ R - np.eye(3)) < 1e-5 and abs(np.linalg.det(R) - 1) < 1e-5,
                "invalid pose rotation")
        T[:3, :3] = Rotation.from_matrix(R).as_matrix()  # Only float-carrier SO3 cleanup.
    return poses, stamps


def validate_ledger(ledger, raw_scans, freeze):
    expected = [r for r in raw_scans if int(r["scan_end_ns"]) <= freeze["boot_stamp_ns"]]
    require(len(expected) == 99 and len(ledger) == 99, "must retain all original 99 bootstrap scans")
    for actual, raw in zip(ledger, expected):
        require(int(actual["transaction_id"]) == int(raw["transaction_id"])
                and int(actual["scan_start_ns"]) == int(raw["scan_start_ns"])
                and int(actual["stamp_ns"]) == int(raw["scan_end_ns"]),
                "bootstrap ledger index/time mismatch")


def evaluate(config_path, freeze_path, odometry_path, admission_path, input_dir, output_dir):
    started = time.monotonic()
    config, freeze, admission = map(load_json, (config_path, freeze_path, admission_path))
    require(sha(config_path) == freeze["config_sha256"], "V2 config changed after freeze")
    require(freeze["source_sha256"].get(Path(__file__).name) == sha(__file__),
            "motion evaluator changed after pre-run freeze")
    for name in ("diagnose_motion.py", "preintegration.py", "prepare_bootstrap.py"):
        path = (OLD_DIRECTORY / name).resolve()
        require(freeze["reused_motion_source_sha256"].get(str(path)) == sha(path),
                "reused numerical dependency changed after freeze: " + name)
    old = load_json(OLD_CONFIG)
    require(config["preintegration"] == old["preintegration"], "frozen motion core gates changed")
    for name in ("pose_position_sigma_m", "pose_rotation_sigma_deg"):
        require(config["lidar"][name] == old["lidar"][name], "frozen pose uncertainty floor changed")
    require(admission["status"] == "PASS", "local odometry must PASS before any moving-state fit")
    require(admission["odometry_sha256"] == sha(odometry_path), "local admission/odometry mismatch")
    require(admission["total_pair_count"] == 98 and admission["accepted_pair_count"] >= 90
            and admission["max_consecutive_failures"] <= 2
            and admission["validation_real_observations"] >= 15,
            "local admission failed frozen trajectory counts")
    verify_inputs(input_dir, freeze)
    with Path(odometry_path).open() as stream:
        ledger = list(csv.DictReader(stream))
    with (Path(input_dir) / "raw_timed_scan_index.csv").open() as stream:
        raw_scans = list(csv.DictReader(stream))
    validate_ledger(ledger, raw_scans, freeze)
    require(len(ledger) == 99, "must retain the full 99-scan bootstrap ledger")
    roles = {"COORDINATE_ANCHOR", "LIDAR_OBSERVATION", "CAUSAL_PREDICTION_ONLY", "REJECTED_STOP", "NOT_RUN"}
    require(all(r["role"] in roles for r in ledger), "unknown observation provenance")
    require(len({int(r["transaction_id"]) for r in ledger}) == len(ledger), "duplicate ledger transactions")
    require(all((r["role"] == "LIDAR_OBSERVATION") ==
                (r["true_observation"] == "1" and r["quality_pass"] == "1") for r in ledger),
            "inconsistent true-observation role")
    train_ledger = [r for r in ledger if freeze["estimation_start_ns"] <= int(r["stamp_ns"])
                    < freeze["estimation_end_ns"]]
    require(bool(train_ledger) and len(accepted_rows(train_ledger)) == len(train_ledger),
            "every 5--8s target must be a real accepted observation")
    rows = accepted_rows(ledger)
    require(len(rows) == admission["accepted_pair_count"], "observed-row count differs from admission")
    lidar_poses, stamps = validate_pose_rows(rows)
    require(int(stamps[-1]) <= freeze["boot_stamp_ns"], "future bootstrap observation")
    ext = np.asarray(freeze["T_imu_lidar"], dtype=float)
    require(ext.shape == (4, 4) and np.isfinite(ext).all()
            and np.linalg.norm(ext[3] - [0, 0, 0, 1]) < 1e-8
            and np.linalg.norm(ext[:3, :3].T @ ext[:3, :3] - np.eye(3)) < 1e-8
            and abs(np.linalg.det(ext[:3, :3]) - 1) < 1e-8, "invalid frozen extrinsic")
    imu_poses = lidar_poses @ np.linalg.inv(ext)
    uncertainty = [observation_uncertainty(r, np.linalg.norm(ext[:3, 3]), config["lidar"])
                   for r in rows]
    sp, sr = np.asarray(uncertainty).T
    c = config["preintegration"]
    with (Path(input_dir) / "imu.csv").open() as stream:
        all_imu = list(csv.DictReader(stream))
    train = (stamps >= freeze["estimation_start_ns"]) & (stamps < freeze["estimation_end_ns"])
    valid = (stamps >= freeze["estimation_end_ns"]) & (stamps <= freeze["boot_stamp_ns"])
    require(np.count_nonzero(train) >= 3 and np.count_nonzero(valid) >= 15,
            "not enough real estimation/validation observations")
    output_dir = Path(output_dir)
    output_dir.mkdir()  # A started fit cannot silently be overwritten or repeated.
    write_json(output_dir / "motion_started.json", {
        "odometry_sha256": sha(odometry_path), "admission_sha256": sha(admission_path),
        "config_sha256": sha(config_path), "freeze_sha256": sha(freeze_path),
        "sources": {str(p): sha(p) for p in
                    (Path(__file__), OLD_DIRECTORY / "diagnose_motion.py",
                     OLD_DIRECTORY / "preintegration.py", OLD_DIRECTORY / "prepare_bootstrap.py")},
        "GT_LOADED": False, "NDT_CALLS": 0})
    ts, imu = stage_imu(all_imu, freeze["estimation_end_ns"])
    d, x, pre = fit(imu_poses[train], stamps[train], ts, imu, sp[train], sr[train], c)
    d["rank_interpretation"] = (
        "V2_ACCEPTED_LIDAR_OBSERVATIONS; numerical rank alone is not physical observability; "
        "orientation-uncertainty lower bound, independent validation and full coupled covariance required")
    splits = []
    for lower, upper in ((5., c["state_split_s"]), (c["state_split_s"], 8.)):
        start = freeze["first_sensor_stamp_ns"] + int(lower * 1e9)
        end = freeze["first_sensor_stamp_ns"] + int(upper * 1e9)
        mask = (stamps >= start) & (stamps < end)
        require(np.count_nonzero(mask) >= 3, "insufficient real observations in state split")
        split_stamps, split_imu = stage_imu(all_imu, end)
        split, _, _ = fit(imu_poses[mask], stamps[mask], split_stamps, split_imu, sp[mask], sr[mask], c)
        split["rank_interpretation"] = d["rank_interpretation"]
        split["covariance"] = "NOT_RUN; coupled covariance is not provided by the reused mean fitter"
        splits.append(split)
    d["split_diagnostics"] = splits
    d["split_bg_change_rad_s"] = float(np.linalg.norm(np.asarray(splits[0]["gyro_bias_tentative"])
                                                    - splits[1]["gyro_bias_tentative"]))
    d["split_ba_change_m_s2"] = float(np.linalg.norm(np.asarray(splits[0]["accel_bias_tentative"])
                                                    - splits[1]["accel_bias_tentative"]))
    a, b = (np.asarray(s["gravity_tentative"]) for s in splits)
    d["split_gravity_angle_deg"] = float(np.rad2deg(np.arccos(np.clip(
        a @ b / c["gravity_norm_m_s2"] ** 2, -1., 1.))))
    initial = imu_poses[train][-1]
    fit_end = int(stamps[train][-1])
    vi = x[3 * np.count_nonzero(train) - 3:3 * np.count_nonzero(train)]
    g, ba, bg = (np.asarray(d[key]) for key in
                 ("gravity_tentative", "accel_bias_tentative", "gyro_bias_tentative"))
    val_stamps, val_imu = stage_imu(all_imu, freeze["boot_stamp_ns"])
    validation = []
    for T, t in zip(imu_poses[valid], stamps[valid]):
        DR, dv, dp, Jv, Jp = integrate(val_stamps, val_imu, fit_end, int(t), bg,
                                      c["maximum_imu_gap_s"], c["maximum_endpoint_hold_s"])
        dt = (int(t) - fit_end) * 1e-9
        predicted_p = initial[:3, 3] + vi * dt + .5 * g * dt * dt + initial[:3, :3] @ (dp - Jp @ ba)
        validation.append({"stamp_ns": int(t), "fit_end_ns": fit_end,
                           "translation_error_m": float(np.linalg.norm(T[:3, 3] - predicted_p)),
                           "rotation_error_deg": float(np.rad2deg(np.linalg.norm(
                               log((initial[:3, :3] @ DR).T @ T[:3, :3])))),
                           "observation_role": "ACCEPTED_LIDAR", "state_accepted": False})
    d["validation_translation_rmse_m"] = float(np.sqrt(np.mean([r["translation_error_m"] ** 2 for r in validation])))
    d["validation_rotation_rmse_deg"] = float(np.sqrt(np.mean([r["rotation_error_deg"] ** 2 for r in validation])))
    gates = mean_gates(d, c)
    for i, split in enumerate(splits):
        gates.append({"gate": "split_" + str(i) + "_solver_success", "value": split["solver_success"],
                      "limit": True, "pass": bool(split["solver_success"])})
    all_pass = all(row["pass"] for row in gates)
    d["mean_observability_validation_status"] = "PASS" if all_pass else "FAIL"
    d["initialization_status"] = "FULL_COUPLED_COVARIANCE_REQUIRED" if all_pass else "MOVING_INITIALIZATION_NOT_CERTIFIED"
    d["state_accepted"] = False
    d["covariance"] = "NOT_RUN; no profile pseudoinverse or synthetic covariance may authorize injection"
    d["standard_deviation_gates"] = "NOT_RUN until full coupled covariance including lever arm and boot propagation"
    d["measurement_uncertainty_interpretation"] = (
        "engineering marginal scales from floors, trimmed residual, closure, bias sensitivity and causal CV residual; "
        "not calibrated GICP pose covariance; inherited worst-block correlation inflation retained")
    d["real_estimation_observations"] = int(np.count_nonzero(train))
    d["real_validation_observations"] = int(np.count_nonzero(valid))
    d["predicted_ledger_rows_not_used_as_observations"] = sum(r["role"] == "CAUSAL_PREDICTION_ONLY" for r in ledger)
    d["offline_motion_wall_s"] = time.monotonic() - started
    d["GT_LOADED"] = False
    d["NDT_CALLS"] = 0
    write_json(output_dir / "motion_state_estimate.json", d)
    write_csv(output_dir / "observability_diagnostics.csv", gates)
    write_csv(output_dir / "bootstrap_validation.csv", validation)
    write_csv(output_dir / "imu_preintegration.csv", pre)
    write_csv(output_dir / "pose_uncertainty.csv", [{
        "transaction_id": r["transaction_id"], "stamp_ns": r["stamp_ns"],
        "position_sigma_m": float(p), "rotation_sigma_rad": float(q),
        "provenance": "V2_ACCEPTED_LIDAR_ENGINEERING_MARGINAL_NOT_CALIBRATED_COVARIANCE"}
        for r, p, q in zip(rows, sp, sr)])
    print(json.dumps({"status": d["initialization_status"], "state_accepted": False,
                      "rank": d["profile_rank"], "condition": d["condition"],
                      "validation_position_rmse_m": d["validation_translation_rmse_m"],
                      "validation_rotation_rmse_deg": d["validation_rotation_rmse_deg"]}, allow_nan=False))
    return d


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("config", "freeze", "odometry", "admission-json", "input-dir", "output-dir"):
        parser.add_argument("--" + key, type=Path, required=True)
    args = parser.parse_args()
    evaluate(args.config, args.freeze, args.odometry, args.admission_json, args.input_dir, args.output_dir)
