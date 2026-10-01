#!/usr/bin/env python3
"""Offline-only A3G CSV audit. Never loads points, images, GT, or a runner."""
import argparse
import collections
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
START_SHA = "88831c3906f05589031af8d06c95bc405d9d24d9"
PARAMS_SHA = "7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d"
LEDGER_SHA = "4992c4f56fa49e2b8837f57733b9edc2bde51efb997e6c475a716be35a3cdf05"
LEDGER = PACKAGE / "docs/p6_alg_integration_a3g/RUN_FULL_FIRST_FAILURE/EXTERNAL_RUN_FILES.json"
PARAMS = PACKAGE / "docs/p6_i6d_full_algorithm/corridor01_params_official_calibration.txt"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_ledger(path):
    """Verify every frozen file, not only the eight primary CSVs."""
    entries = json.loads(Path(path).read_text())
    checked = {}
    for entry in entries:
        source = Path(entry["path"])
        if source.name in checked:
            raise ValueError("duplicate ledger basename")
        actual = digest(source)
        if actual != entry["sha256"] or source.stat().st_size != entry["bytes"]:
            raise ValueError("FROZEN_ARTIFACT_SHA_MISMATCH:" + str(source))
        checked[source.name] = {**entry, "verified": True}
    return checked


def read_csv(path):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    if not rows or any(None in row or None in row.values() for row in rows):
        raise ValueError("empty/malformed CSV:" + str(path))
    return rows


def index_unique(rows, key="transaction_id"):
    result = {}
    for row in rows:
        identity = int(row[key])
        if identity in result:
            raise ValueError("duplicate identity:" + str(identity))
        result[identity] = row
    return result


def stamp_sorted(rows, key="stamp_ns"):
    rows = sorted(rows, key=lambda row: int(row[key]))
    stamps = [int(row[key]) for row in rows]
    if any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError("non-unique/non-increasing timestamps")
    return rows


def runs(rows, predicate):
    """Runs are consecutive sensor terminals, not consecutive transaction IDs."""
    result, current = [], []
    for row in rows:
        if predicate(row):
            current.append(row)
        elif current:
            result.append(current)
            current = []
    if current:
        result.append(current)
    return result


def run_summary(run):
    return {"first_tx": int(run[0]["transaction_id"]),
            "last_tx": int(run[-1]["transaction_id"]), "count": len(run),
            "first_stamp_ns": int(run[0]["stamp_ns"]),
            "last_stamp_ns": int(run[-1]["stamp_ns"])}


def values(field, size):
    vector = np.array([float(value) for value in field.split(";")])
    if vector.size != size or not np.isfinite(vector).all():
        raise ValueError("invalid finite vector of size " + str(size))
    return vector


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm == 0:
        raise ValueError("invalid quaternion")
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def matrix_quaternion(matrix):
    """Match the normalized quaternion conversion used on float NDT output."""
    matrix = np.asarray(matrix)
    trace = np.trace(matrix)
    q = np.empty(4)
    if trace > 0:
        scale = math.sqrt(trace + 1)
        q[3] = scale / 2
        q[:3] = np.array([matrix[2, 1]-matrix[1, 2],
                         matrix[0, 2]-matrix[2, 0],
                         matrix[1, 0]-matrix[0, 1]]) / (2*scale)
    else:
        i = int(np.argmax(np.diag(matrix)))
        j, k = (i+1) % 3, (i+2) % 3
        scale = math.sqrt(matrix[i, i]-matrix[j, j]-matrix[k, k]+1)
        q[i] = scale / 2
        q[j] = (matrix[j, i]+matrix[i, j]) / (2*scale)
        q[k] = (matrix[k, i]+matrix[i, k]) / (2*scale)
        q[3] = (matrix[k, j]-matrix[j, k]) / (2*scale)
    return q / np.linalg.norm(q)


def rotation_log(matrix):
    q = matrix_quaternion(matrix)
    if q[3] < 0:
        q = -q
    sine = np.linalg.norm(q[:3])
    return 2*q[:3] if sine < 1e-12 else q[:3]*(2*math.atan2(sine, q[3])/sine)


def innovations(row, extrinsic):
    predicted_p = values(row["predicted_position"], 3)
    predicted_R = quaternion_matrix(values(row["predicted_rotation_xyzw"], 4))
    terminal = values(row["ndt_terminal_pose"], 16).reshape(4, 4)
    if not np.array_equal(terminal[3], [0, 0, 0, 1]):
        raise ValueError("NDT matrix is not a row-major homogeneous transform")
    # Producer poseFromMatrix normalizes the LiDAR quaternion BEFORE applying
    # the inverse extrinsic. Preserve that ordering on float NDT rotations.
    lidar_R = quaternion_matrix(matrix_quaternion(terminal[:3, :3]))
    measured_R = lidar_R @ extrinsic[:3, :3].T
    measured_p = terminal[:3, 3] - measured_R @ extrinsic[:3, 3]
    return {
        "ndt_imu_position": ";".join(format(value, ".17g") for value in measured_p),
        "ndt_imu_rotation_xyzw": ";".join(format(value, ".17g")
                                             for value in matrix_quaternion(measured_R)),
        "translation_innovation_m": float(np.linalg.norm(measured_p-predicted_p)),
        "rotation_innovation_rad": float(np.linalg.norm(rotation_log(predicted_R.T @ measured_R))),
        "ndt_float_rotation_defect": float(np.max(np.abs(terminal[:3, :3].T @ terminal[:3, :3]-np.eye(3)))),
    }


def classification(row, covariance):
    if covariance["valid"] != "1":
        return "P15_UNAVAILABLE"
    if row["lidar_committed"] == "1":
        return "COMMITTED"
    if row["ndt_converged"] != "1":
        return "NDT_NOT_CONVERGED"
    if row["uobs_status"] == "NO_VALID_GEOMETRIC_CORRESPONDENCES":
        return "NO_VALID_GEOMETRIC_CORRESPONDENCES"
    if "map_support_insufficient" in row["event_status"]:
        return "MAP_SUPPORT_INSUFFICIENT"
    if "lidar_reliable_rank_zero_or_invalid" in row["event_status"]:
        return "RELIABLE_RANK_ZERO_OR_INVALID"
    if (row["measurement_preview_valid"] == "1" and row["selected_nis_valid"] == "1"
            and row["nis_accepted"] == "0"):
        return "NIS_REJECTED"
    return "OTHER"


def write_csv(output, name, rows):
    with (output / name).open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def analyze(ledger, params, output):
    if digest(ledger) != LEDGER_SHA:
        raise ValueError("FROZEN_LEDGER_SHA_MISMATCH")
    frozen = verify_ledger(ledger)  # Before parsing any run artifact.
    if digest(params) != PARAMS_SHA:
        raise ValueError("OFFICIAL_CALIBRATION_SHA_MISMATCH")
    parameters = np.array([float(value) for value in params.read_text().split()])
    if parameters.size != 27:
        raise ValueError("expected 27 frozen parameters")
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = quaternion_matrix(parameters[23:27])
    extrinsic[:3, 3] = parameters[20:23]
    def load(name):
        return read_csv(frozen[name]["path"])

    preopt = index_unique(load("trajectory.csv.r1_preopt_capsule.csv"))
    covariance = index_unique(load("trajectory.csv.a3f_r1_covariance.csv"))
    deskew = index_unique(load("trajectory.csv.deskew_evidence.csv"))
    trajectories = index_unique(load("trajectory.csv"))
    terminals = stamp_sorted([row for row in load("trajectory.csv.a3g_health.csv")
                              if row["event"] == "LIDAR_SCAN_END"])
    health = index_unique(terminals)
    events = index_unique([row for row in load("events.csv")
                           if row["event_type"] == "LIDAR_SCAN_END"], "timestamp")
    runtime = index_unique([row for row in load("runtime.csv")
                            if row["event_type"] == "LIDAR_SCAN_END"], "timestamp")
    removal_history = load("trajectory.csv.a3c_r1_marginalization_trace.csv")
    completed = {tx for tx, row in health.items() if row["completed"] == "1"}
    if not (set(preopt) == set(deskew) == set(trajectories) == completed
            and set(health) == set(covariance)
            and set(events) == set(runtime) == {int(health[tx]["stamp_ns"]) for tx in completed}):
        raise ValueError("artifact terminal identity sets disagree")
    for tx, row in health.items():
        stamp = int(row["stamp_ns"])
        if covariance[tx]["stamp_ns"] != row["stamp_ns"]:
            raise ValueError("covariance stamp mismatch")
        if tx in completed:
            if not (preopt[tx]["stamp_ns"] == trajectories[tx]["stamp_ns"] == row["stamp_ns"]
                    and int(deskew[tx]["scan_end_ns"]) == stamp and stamp in events and stamp in runtime):
                raise ValueError("completed terminal stamp mismatch")
            if (preopt[tx]["lidar_committed"] != events[stamp]["lidar_factor_committed"]
                    or row["lidar_factors"] != events[stamp]["lidar_factor_count"]
                    or preopt[tx]["pre_measurement_covariance_valid"] != covariance[tx]["valid"]):
                raise ValueError("event/preopt/covariance cross-check mismatch")

    previous_trajectory = {int(current["transaction_id"]): previous for previous, current
                           in zip(stamp_sorted(list(trajectories.values())),
                                  stamp_sorted(list(trajectories.values()))[1:])}
    support, ndt_audit, geometric, p_audit, deskew_audit = [], [], [], [], []
    for h in terminals:
        tx, stamp = int(h["transaction_id"]), int(h["stamp_ns"])
        c, r = covariance[tx], preopt.get(tx)
        if r is None and (c["valid"] == "1" or h["completed"] == "1"
                          or "producer_square_root_covariance:" not in h["reason"]):
            raise ValueError("unexplained missing terminal preopt capsule")
        category = classification(r, c) if r else "P15_UNAVAILABLE"
        nis_status = "NOT_EXECUTED" if not r else (
            "NIS_NOT_REACHED" if r["measurement_preview_valid"] != "1" else
            "VALID" if r["selected_nis_valid"] == "1" else "INVALID")
        support.append({"transaction_id": tx, "stamp_ns": stamp,
                        "completed": h["completed"], "category": category,
                        "lidar_committed": r["lidar_committed"] if r else "NOT_EXECUTED",
                        "event_status": r["event_status"] if r else h["reason"],
                        "nis_status": nis_status, "selected_nis": r["selected_nis"] if r else "NaN",
                        "nis_threshold": r["nis_threshold"] if r else "NaN",
                        "selected_rank": r["reliable_rank"] if r else "NOT_EXECUTED",
                        "uobs_status": r["uobs_status"] if r else "NOT_EXECUTED",
                        "ndt_converged": r["ndt_converged"] if r else "NOT_EXECUTED",
                        "measurement_preview_valid": r["measurement_preview_valid"] if r else "NOT_EXECUTED",
                        "lidar_source_provenance": events[stamp]["lidar_source_provenance"] if r else "NOT_EXECUTED",
                        "P15_valid": c["valid"], "P15_detail": c["detail"],
                        "active_lidar_factors": h["lidar_factors"],
                        "window_nodes_after": h["window_nodes"], "window_span_after_s": h["window_span"]})
        if not r or not 140 <= tx <= 220:
            continue
        stamp_event, d = events[stamp], deskew[tx]
        innovation = innovations(r, extrinsic)
        predicted_position = values(r["predicted_position"], 3)
        prev = previous_trajectory.get(tx)
        predicted_distance = (float(np.linalg.norm(predicted_position - np.array([float(prev[k]) for k in ["px", "py", "pz"]])))
                              if prev else "NOT_AVAILABLE_FIRST_TERMINAL")
        posterior_distance = float(np.linalg.norm(predicted_position - np.array([float(trajectories[tx][k]) for k in ["px", "py", "pz"]])))
        ndt_keys = ["predicted_position", "predicted_rotation_xyzw", "ndt_terminal_pose",
                    "ndt_converged", "ndt_fitness", "ndt_objective", "ndt_iterations", "ndt_runtime_ms"]
        ndt_audit.append({"transaction_id": tx, "stamp_ns": stamp,
                          **{key: r[key] for key in ndt_keys}, **innovation,
                          "prediction_from_previous_completed_distance_m": predicted_distance,
                          "posterior_minus_predicted_distance_m": posterior_distance,
                          "all_ndt_calls": runtime[stamp]["ndt_calls"],
                          "all_ndt_runtime_ms": runtime[stamp]["ndt_ms"],
                          "zero_objective_zero_iterations": int(float(r["ndt_objective"]) == 0
                                                                  and int(r["ndt_iterations"]) == 0)})
        # Correspondence counts were not serialized. Status exposes 0 / <30,
        # but do not fabricate exact counts or raw neighbor-search statistics.
        map_state = ("INVALID" if category in ["NO_VALID_GEOMETRIC_CORRESPONDENCES", "MAP_SUPPORT_INSUFFICIENT"]
                     else "VALID_INFERRED_FROM_DOWNSTREAM_ADMISSION")
        keys = ["uobs_valid", "uobs_status", "uobs_weak_dimension", "uobs_reliable_dimension",
                "translation_eigenvalues", "rotation_eigenvalues", "translation_weak_ratio",
                "rotation_weak_ratio", "weak_translation_direction_map", "weak_rotation_direction_map",
                "weak_basis", "measurement_preview_valid", "reliable_rank", "selected_nis_valid",
                "selected_nis", "nis_threshold", "nis_accepted"]
        geometric.append({"transaction_id": tx, "stamp_ns": stamp, **{key: r[key] for key in keys},
                          "map_support_state": map_state, "correspondence_count": "NOT_LOGGED",
                          "category": category, "nis_status": nis_status})
        p_audit.append(dict(c))
        raw, count = int(d["raw_point_count"]), int(d["deskew_point_count"])
        timing_valid = (int(d["point_stamp_min_ns"]) == int(d["scan_start_ns"])
                        and int(d["scan_start_ns"]) <= int(d["point_stamp_min_ns"])
                        <= int(d["point_stamp_max_ns"]) <= int(d["scan_end_ns"]))
        deskew_audit.append({**d, "scan_duration_s": (int(d["scan_end_ns"])-int(d["scan_start_ns"]))/1e9,
                             "timestamps_valid": int(timing_valid), "count_preserved": int(raw == count),
                             "nonempty": int(raw > 0), "provenance": stamp_event["lidar_source_provenance"]})

    pose_rows = []
    trajectory = stamp_sorted(list(trajectories.values()))
    for previous, current in zip(trajectory, trajectory[1:]):
        dt = (int(current["stamp_ns"])-int(previous["stamp_ns"]))/1e9
        increment = np.array([float(current[k])-float(previous[k]) for k in ["px", "py", "pz"]])
        R_prev = quaternion_matrix([float(previous[k]) for k in ["qx", "qy", "qz", "qw"]])
        R_cur = quaternion_matrix([float(current[k]) for k in ["qx", "qy", "qz", "qw"]])
        distance, angle = float(np.linalg.norm(increment)), float(np.linalg.norm(rotation_log(R_prev.T @ R_cur)))
        pose_rows.append({"transaction_id": int(current["transaction_id"]), "stamp_ns": int(current["stamp_ns"]),
                          "previous_transaction_id": int(previous["transaction_id"]), "dt_s": dt,
                          "translation_increment_m": distance, "rotation_increment_rad": angle,
                          "implied_speed_m_s": distance/dt, "implied_angular_speed_rad_s": angle/dt})

    rejects = runs(support, lambda row: row["category"] != "COMMITTED")
    completed_rejections = runs(support, lambda row: row["completed"] == "1" and row["category"] != "COMMITTED")
    no_geometry = runs(support, lambda row: row["uobs_status"] == "NO_VALID_GEOMETRIC_CORRESPONDENCES")
    zero_active = runs(support, lambda row: row["active_lidar_factors"] == "0")
    last_commit = next(row for row in reversed(support) if row["category"] == "COMMITTED")
    first_after = next(row for row in support if row["stamp_ns"] > last_commit["stamp_ns"])
    first_exceed = {str(limit): next(row for row in pose_rows if row["translation_increment_m"] > limit)
                    for limit in [0.5, 1.0, 2.0, 5.0]}
    all_ndt = [{"transaction_id": int(r["transaction_id"]), "stamp_ns": int(r["stamp_ns"]),
                **innovations(r, extrinsic)} for r in stamp_sorted(list(preopt.values()))]
    zero_ndt = [r for r in stamp_sorted(list(preopt.values()))
                if int(r["ndt_iterations"]) == 0 and float(r["ndt_objective"]) == 0]
    last_removal = [row for row in removal_history
                    if int(row["oldest_state_stamp_ns"]) == last_commit["stamp_ns"]]
    removal_keys = ["transaction_id", "event_stamp_ns", "enforcement_index", "attempt_index",
                    "oldest_state_stamp_ns", "nodes_before_attempt", "nodes_after_attempt",
                    "span_before_attempt_s", "span_after_attempt_s", "incident_lidar_factor_count",
                    "marginalization_result", "qr_marginalized_rank"]
    removals_201_202 = [{key: row[key] for key in removal_keys} for row in removal_history
                        if int(row["transaction_id"]) in [201, 202]]
    facts = {"stage": "PAPER-P6-ALG-INTEGRATION-A3G-R1", "START_SHA": START_SHA,
             "dataset": "SuperLoc Corridor01", "REAL_REPLAY_COUNT": 0, "GT_USED": False,
             "production_changes": 0, "artifact_sha_gate": "PASS", "verified_artifact_count": len(frozen),
             "script_sha256": digest(__file__), "ledger_sha256": digest(ledger),
             "params_sha256": digest(params), "T_imu_lidar": extrinsic.tolist(),
             "persistence_definition": "at least 20 consecutive stamp-ordered terminal records",
             "completed_terminals": len(completed), "attempted_terminals": len(terminals),
             "last_committed": last_commit, "first_subsequent_noncommit": first_after,
             "last_committed_preopt": preopt[last_commit["transaction_id"]],
             "last_committed_innovation": next(r for r in all_ndt if r["transaction_id"] == last_commit["transaction_id"]),
             "ndt_converged_count": sum(r["ndt_converged"] == "1" for r in preopt.values()),
             "zero_iteration_zero_objective_count": len(zero_ndt),
             "first_zero_iteration_zero_objective_tx": int(zero_ndt[0]["transaction_id"]),
             "first_ndt_innovation_over_1m_descriptive_only": next(r for r in all_ndt if r["translation_innovation_m"] > 1),
             "first_ndt_innovation_over_pi_2_descriptive_only": next(r for r in all_ndt if r["rotation_innovation_rad"] > math.pi/2),
             "last_committed_state_removal": [{key: row[key] for key in removal_keys} for row in last_removal],
             "zero_ndt_and_no_geometry_cooccurrences": sum(r["uobs_status"] == "NO_VALID_GEOMETRIC_CORRESPONDENCES"
                                                          for r in zero_ndt),
             "noncommit_runs": [run_summary(run) for run in rejects],
             "completed_rejection_runs": [run_summary(run) for run in completed_rejections],
             "first_persistent_noncommit": run_summary(next(run for run in rejects if len(run) >= 20)),
             "first_persistent_rejection": run_summary(next(run for run in completed_rejections if len(run) >= 20)),
             "no_geometry_runs": [run_summary(run) for run in no_geometry],
             "first_no_geometry": next(row for row in support if row["uobs_status"] == "NO_VALID_GEOMETRIC_CORRESPONDENCES"),
             "first_persistent_no_geometry": run_summary(next(run for run in no_geometry if len(run) >= 20)),
             "zero_active_runs": [run_summary(run) for run in zero_active],
             "last_active_positive": next(row for row in reversed(support) if int(row["active_lidar_factors"]) > 0),
             "first_increment_exceed": first_exceed,
             "max_increment": max(pose_rows, key=lambda row: row["translation_increment_m"]),
             "category_histogram": dict(collections.Counter(row["category"] for row in support)),
             "window_category_histogram": dict(collections.Counter(row["category"] for row in support
                                                                    if 140 <= row["transaction_id"] <= 220)),
             "last_failure": support[-1],
             "deskew_window_count": len(deskew_audit),
             "deskew_invariant_failures": sum(row["timestamps_valid"] != 1 or row["count_preserved"] != 1
                                               or row["nonempty"] != 1 or row["provenance"] != "WINDOW_OWNED_SE3_DESKEW"
                                               for row in deskew_audit)}
    if output.exists():
        raise ValueError("output already exists; refusing overwrite")
    output.mkdir(parents=True)
    for name, rows in [("LIDAR_SUPPORT_TIMELINE.csv", support), ("NDT_TRANSITION_AUDIT.csv", ndt_audit),
                       ("GEOMETRIC_SUPPORT_TRANSITION.csv", geometric), ("P15_BEFORE_COLLAPSE.csv", p_audit),
                       ("POSE_INCREMENT_TIMELINE.csv", pose_rows), ("DESKEW_WINDOW_AUDIT.csv", deskew_audit),
                       ("LAST_ACTIVE_LIDAR_REMOVAL.csv", removals_201_202)]:
        write_csv(output, name, rows)
    with (output / "ANALYSIS_FACTS.json").open("x") as stream:
        json.dump(facts, stream, indent=2, allow_nan=False)
        stream.write("\n")
    with (output / "ARTIFACT_SHA_GATE.json").open("x") as stream:
        json.dump(list(frozen.values()), stream, indent=2)
        stream.write("\n")
    print(json.dumps({key: facts[key] for key in ["artifact_sha_gate", "last_committed", "first_persistent_rejection",
                                                 "first_persistent_no_geometry", "zero_active_runs", "first_increment_exceed"]}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ledger", type=Path, default=LEDGER)
    parser.add_argument("--params", type=Path, default=PARAMS)
    parser.add_argument("--output", type=Path, required=True, help="new directory only")
    args = parser.parse_args()
    analyze(args.ledger, args.params, args.output)
