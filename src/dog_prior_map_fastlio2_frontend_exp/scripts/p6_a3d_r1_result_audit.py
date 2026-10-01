#!/usr/bin/env python3
"""Summarize the single A3D-R1 engineering replay, not a precision evaluation."""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import shutil


def read(path):
    with path.open() as source:
        return list(csv.DictReader(source))


def write(path, rows):
    if rows:
        with path.open("w", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=list(rows[0]), lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)


def audit(run, output, frozen):
    events = read(run / "events.csv")
    preopt = read(run / "trajectory.csv.r1_preopt_capsule.csv")
    optimizer = read(run / "trajectory.csv.r1_optimizer_trace.csv")
    marginal = read(run / "trajectory.csv.a3c_r1_marginalization_trace.csv")
    trajectory = read(run / "trajectory.csv")
    trajectory_valid = all(all(math.isfinite(float(r[k])) for k in
        ("px", "py", "pz", "qx", "qy", "qz", "qw")) and
        abs(sum(float(r[k]) ** 2 for k in ("qx", "qy", "qz", "qw")) - 1) <= 1e-10
        for r in trajectory)
    old_preopt = read(frozen / "trajectory.csv.r1_preopt_capsule.csv")
    old_optimizer = read(frozen / "trajectory.csv.r1_optimizer_trace.csv")
    prefix = [r for r in preopt if int(r["transaction_id"]) <= 115]
    differences = []
    for current, old in zip(prefix, old_preopt):
        for key, value in old.items():
            if key != "ndt_runtime_ms" and current[key] != value:
                differences.append(f"preopt:{old['transaction_id']}:{key}")
    # This includes the identical first rejected tx115 candidate, and excludes
    # only newly added termination/rollback diagnostics absent in frozen code.
    prefix_optimizer = [r for r in optimizer if int(r["stamp_ns"]) <= 1517157230686328484]
    for current, old in zip(prefix_optimizer, old_optimizer):
        for key, value in old.items():
            if current[key] != value:
                differences.append(f"optimizer:{old['stamp_ns']}:{old['iteration']}:{key}")
    prefix_identity = (len(prefix) == len(old_preopt) and
                       len(prefix_optimizer) == len(old_optimizer) and not differences)
    tx115_preopt = [r for r in preopt if r["transaction_id"] == "115"]
    tx115_optimizer = [r for r in optimizer if r["transaction_id"] == "115"]
    tx115_marginal = [r for r in marginal if r["transaction_id"] == "115"]
    tx115_event = [r for r in events if r["timestamp"] == "1517157230686328484"]
    successful_marginal = [r for r in marginal if r["marginalization_result"] == "SUCCESS"]
    failed_marginal = [r for r in marginal if r["marginalization_result"] != "SUCCESS"]
    tx115_pass = (prefix_identity and len(tx115_optimizer) == 1 and len(tx115_event) == 1 and
        tx115_event[0]["optimizer_status"] == "CONVERGED_WITHOUT_STEP" and
        tx115_optimizer[0]["accepted"] == "0" and
        tx115_optimizer[0]["termination_reason"] == "NATURAL_SMALL_STEP_NO_ACCEPTED_UPDATE" and
        float(tx115_optimizer[0]["candidate_rollback_state_difference"]) == 0 and
        len(tx115_marginal) == 2 and all(r["marginalization_result"] == "SUCCESS" for r in tx115_marginal) and
        float(tx115_event[0]["window_span"]) <= 2.0)
    for name, rows in (("TX115_preopt.csv", tx115_preopt),
                       ("TX115_optimizer.csv", tx115_optimizer),
                       ("TX115_marginalization.csv", tx115_marginal),
                       ("TX115_event.csv", tx115_event),
                       ("FIRST_FAILURE_marginalization.csv", failed_marginal)):
        write(output / name, rows)
    for name in ("input_identity.json", "identity_gate.json", "command.json"):
        shutil.copyfile(run / name, output / name)
    summary_path = output / "engineering_summary.json"
    summary = json.loads(summary_path.read_text())
    # The reused A3C-R2 analyzer's old label includes all candidates, including
    # a failed (unstored) prior. Explicitly correct it for this failed run.
    if "min_stored_prior_eigenvalue" in summary:
        summary["min_candidate_prior_eigenvalue_including_failure"] = summary.pop("min_stored_prior_eigenvalue")
    summary["issue_list"] = ["marginalization candidate rejected (not stored)"
        if issue == "invalid or rejected stored prior" else issue for issue in summary["issue_list"]]
    summary["min_successfully_stored_prior_eigenvalue"] = min(float(r["new_prior_lambda_min"]) for r in successful_marginal)
    summary["candidate_prior_max_asymmetry_including_failure"] = max(float(r["new_prior_symmetry_max_abs"]) for r in marginal)
    summary["stored_prior_max_asymmetry"] = max(float(r["new_prior_symmetry_max_abs"]) for r in successful_marginal)
    summary["successful_marginalizations"] = len(successful_marginal)
    summary["failed_marginalization_attempts"] = len(failed_marginal)
    summary["optimizer_status_counts"] = dict(Counter(r["optimizer_status"] for r in events))
    summary["natural_small_step_convergence_count"] = sum(r["termination_reason"] == "NATURAL_SMALL_STEP_NO_ACCEPTED_UPDATE" for r in optimizer)
    summary["covariance_first_unavailable_stamp"] = next((r["timestamp"] for r in events
        if r["event_type"] == "LIDAR_SCAN_END" and r["window_covariance_valid"] == "0"), None)
    summary["prefix_identity_exact_excluding_runtime"] = prefix_identity
    summary["prefix_preopt_rows_compared"] = len(prefix)
    summary["prefix_optimizer_rows_compared"] = len(prefix_optimizer)
    summary["prefix_difference_list"] = differences
    summary["tx115_repair_gate"] = "PASS" if tx115_pass else "FAIL"
    summary["failure_stage"] = "MARGINALIZATION_STAGE" if failed_marginal else "OPTIMIZER_STAGE"
    summary["completed_optimized_trajectory_finite_SO3"] = trajectory_valid
    identity = json.loads((run / "identity_gate.json").read_text())
    summary["P3_200_REAL_LINK_GATE"] = "PASS" if (summary["complete"] and not summary["issue_list"] and
        tx115_pass and trajectory_valid and all(identity["identity_checks"].values()) and not identity["discrepancy"] and
        identity["process_exit_code"] == 0) else "FAIL"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return tx115_pass and trajectory_valid


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frozen-run", type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(0 if audit(args.run, args.output, args.frozen_run) else 1)
