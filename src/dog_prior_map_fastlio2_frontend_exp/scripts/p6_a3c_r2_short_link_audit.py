#!/usr/bin/env python3
"""Read-only engineering checks and small report artifacts; no GT or replay."""
import argparse
import csv
import json
import math
from pathlib import Path
import shutil
from collections import Counter, defaultdict


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def write_rows(path, rows):
    if not rows:
        return
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def audit(run, output):
    output.mkdir(parents=True, exist_ok=False)
    events = read(run / "events.csv")
    runtime = read(run / "runtime.csv")
    trajectory = read(run / "trajectory.csv")
    marginal = read(run / "trajectory.csv.a3c_r1_marginalization_trace.csv")
    optimizer = read(run / "trajectory.csv.r1_optimizer_trace.csv")
    deskew = read(run / "trajectory.csv.deskew_evidence.csv")
    terminals = [r for r in events if r["event_type"] == "LIDAR_SCAN_END"]
    console = (run / "console.txt").read_text()
    failure = {}
    failure_path = run / "trajectory.csv.r1_failure_summary.txt"
    if failure_path.exists():
        for line in failure_path.read_text().splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                failure[key] = value
    issues = []

    def require(value, label):
        if not value:
            issues.append(label)

    require("FULL_FIXED_LAG_V3_EXPERIMENTAL_COMPLETE" in console, "run incomplete")
    require(all(int(r["window_nodes"]) <= 48 and float(r["window_span"]) <= 2 + 1e-12
                for r in events), "completed event window bound")
    require(all(events[i]["timestamp"] > events[i-1]["timestamp"]
                for i in range(1, len(events))), "event timestamp monotonicity")
    for r in events:
        pose = [float(r[k]) for k in ("predicted_px", "predicted_py", "predicted_pz",
                                     "predicted_qx", "predicted_qy", "predicted_qz", "predicted_qw")]
        require(all(math.isfinite(v) for v in pose), "nonfinite recorded pose")
        require(abs(sum(v*v for v in pose[3:]) - 1) < 1e-10, "invalid recorded SO3 quaternion")
    require(all(r["post_handoff_ikfom_calls"] == "0" for r in events), "IKFoM leakage")
    require(all(r["visual_factor_count"] == "0" and
                r["event_type"] in ("LIDAR_SCAN_START", "LIDAR_SCAN_END") for r in events),
            "nonzero visual factor/event")
    require(all(r["lidar_source_provenance"] == "WINDOW_OWNED_SE3_DESKEW"
                for r in terminals), "LiDAR provenance violation")
    require(all(r["window_covariance_valid"] == "NOT_REQUESTED_NON_LIDAR_EVENT"
                for r in events if r["event_type"] != "LIDAR_SCAN_END"),
            "non-LiDAR covariance requested")
    require(all(r["optimizer_status"] in ("ACCEPTED_UPDATE", "CONVERGED_WITHOUT_STEP")
                for r in events), "completed event optimizer failure")
    require(all(r["sparse_fallback_count"] == "0" for r in runtime), "sparse fallback")
    require(all(r["candidate_basis_relinearization_calls"] == "0" for r in optimizer),
            "inner candidate basis recomputation")
    require(all(r["marginalization_result"] == "SUCCESS" and r["new_prior_finite"] == "1" and
                float(r["new_prior_symmetry_max_abs"]) == 0 and
                float(r["new_prior_lambda_min"]) >= -1e-6 for r in marginal),
            "invalid or rejected stored prior")
    require(all(r["raw_point_count"] == r["deskew_point_count"] and
                r["point_stamp_min_ns"] == r["scan_start_ns"] and
                int(r["point_stamp_max_ns"]) <= int(r["scan_end_ns"]) for r in deskew),
            "raw deskew count/time contract")

    groups = defaultdict(list)
    for row in optimizer:
        groups[(row["transaction_id"], row["stamp_ns"])].append(row)
    health = []
    for (tx, stamp), rows in groups.items():
        health.append(dict(transaction_id=tx, stamp_ns=stamp,
                           trace_rows=len(rows), accepted_steps=sum(r["accepted"] == "1" for r in rows),
                           candidate_basis_calls=sum(int(r["candidate_basis_relinearization_calls"]) for r in rows),
                           first_cost=rows[0]["surrogate_current_cost"],
                           last_candidate_cost=rows[-1]["surrogate_candidate_cost"],
                           solver_statuses="|".join(sorted({r["solver_status"] for r in rows}))))
    write_rows(output / "optimizer_health.csv", health)
    write_rows(output / "TX90_marginalization.csv", [r for r in marginal if r["transaction_id"] == "90"])
    write_rows(output / "TX90_optimizer.csv", [{k: v for k, v in r.items() if k != "applied_step_components"}
               for r in optimizer if r["transaction_id"] == "90" and r["stamp_ns"] == "1517157228164951397"])
    if failure:
        write_rows(output / "FIRST_FAILURE_optimizer.csv", [r for r in optimizer
                   if r["transaction_id"] == failure.get("transaction_id") and
                   r["stamp_ns"] == failure.get("stamp_ns")])
        for name in ("trajectory.csv.r1_directional_derivative.csv",
                     "trajectory.csv.r1_damping_sweep.csv"):
            if (run / name).exists():
                shutil.copyfile(run / name, output / name)
    # Keep small CSVs, not repeated per-iteration 15N-dimensional steps/matrices.
    for name in ("events.csv", "runtime.csv", "trajectory.csv", "console.txt", "resource.txt",
                 "trajectory.csv.deskew_evidence.csv", "trajectory.csv.r1_preopt_capsule.csv",
                 "trajectory.csv.a3c_r1_marginalization_trace.csv"):
        shutil.copyfile(run / name, output / name)
    for name in ("trajectory.csv.r1_failure_summary.txt", "trajectory.csv.a3c_r1_failure_summary.txt"):
        if (run / name).exists() and (run / name).stat().st_size:
            shutil.copyfile(run / name, output / name)
    counts = Counter((r["transaction_id"], r["enforcement_index"]) for r in marginal)
    result = dict(
        raw_output_path=str(run.resolve()), complete="FULL_FIXED_LAG_V3_EXPERIMENTAL_COMPLETE" in console,
        issue_list=issues, event_count=len(events), processed_lidar_terminals=len(terminals),
        window_deskew_count=len(deskew), ndt_calls=sum(int(r["ndt_calls"]) for r in runtime),
        ndt_converged_count=sum(r["ndt_converged"] == "1" for r in terminals),
        uobs_valid_count=sum(r["uobs_valid"] == "1" for r in terminals),
        unonlocal_probe_count=sum(r["unonlocal_probe_triggered"] == "1" for r in terminals),
        lidar_factor_committed=sum(r["lidar_factor_committed"] == "1" for r in terminals),
        lidar_factor_attempted=sum(r["lidar_factor_attempted"] == "1" for r in terminals),
        covariance_available=sum(r["window_covariance_valid"] == "1" for r in terminals),
        covariance_unavailable=sum(r["window_covariance_valid"] == "0" for r in terminals),
        first_failure=failure,
        optimizer_failure_count=sum(r["optimizer_status"] not in ("ACCEPTED_UPDATE", "CONVERGED_WITHOUT_STEP") for r in events) +
            int(failure.get("optimizer_status") == "FAILED_ALL_CANDIDATES"),
        marginalization_attempts=len(marginal), marginalization_enforcement_episodes=len(counts),
        multi_removal_episodes=sum(n >= 2 for n in counts.values()),
        max_window_nodes=max(int(r["window_nodes"]) for r in events),
        max_window_span_s=max(float(r["window_span"]) for r in events),
        sparse_fallback_count=max(int(r["sparse_fallback_count"]) for r in runtime),
        stored_prior_max_asymmetry=max(float(r["new_prior_symmetry_max_abs"]) for r in marginal),
        min_stored_prior_eigenvalue=min(float(r["new_prior_lambda_min"]) for r in marginal),
        consumed_H_max_asymmetry=max(float(r["consumed_system_symmetry_max_abs"]) for r in marginal),
        Hmm_max_asymmetry=max(float(r["hmm_symmetry_max_abs"]) for r in marginal),
        raw_Schur_max_asymmetry=max(float(r["raw_schur_symmetry_max_abs"]) for r in marginal),
        jitter_used_attempts=sum(float(r["solve_jitter"]) != 0 for r in marginal),
        candidate_basis_calls=sum(int(r["candidate_basis_relinearization_calls"]) for r in optimizer),
        latest_completed_transaction=trajectory[-1]["transaction_id"] if trajectory else None,
        tx90_event=[r for r in terminals if r["timestamp"] == "1517157228164951397"],
        tx90_attempts=[{k: r[k] for k in ("attempt_index", "nodes_before_attempt", "nodes_after_attempt",
                          "span_before_attempt_s", "span_after_attempt_s", "raw_schur_symmetry_max_abs",
                          "new_prior_symmetry_max_abs", "new_prior_lambda_min", "solve_jitter", "marginalization_result")}
                       for r in marginal if r["transaction_id"] == "90"],
        scope="SHORT_REAL_LINK_ENGINEERING_SANITY", GT_USED=False,
        READY_FOR_FORMAL_EXPERIMENT=False)
    (output / "engineering_summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return not issues


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(0 if audit(args.run, args.output) else 1)
