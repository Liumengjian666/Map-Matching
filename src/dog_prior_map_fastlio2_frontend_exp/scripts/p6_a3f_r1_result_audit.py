#!/usr/bin/env python3
"""Audit the single short covariance replay; no GT or estimator mutations."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil


def read(path):
    return list(csv.DictReader(path.open())) if path.exists() else []


def write(path, rows):
    if rows:
        with path.open("w", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=list(rows[0]), lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)


def stats(values):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return None
    def quantile(p):
        index = p*(len(values)-1)
        lo = int(index)
        return values[lo] + (index-lo)*(values[min(lo+1,len(values)-1)]-values[lo])
    return dict(mean=sum(values)/len(values), median=quantile(0.5), P95=quantile(0.95), max=values[-1])


def audit(run, output):
    output.mkdir(parents=True, exist_ok=False)
    covariance = read(run/"trajectory.csv.a3f_r1_covariance.csv")
    comparison = read(run/"trajectory.csv.a3f_r1_comparison.csv")
    events = read(run/"events.csv")
    trajectory = read(run/"trajectory.csv")
    runtime = read(run/"runtime.csv")
    marginal = read(run/"trajectory.csv.a3c_r1_marginalization_trace.csv")
    optimizer = read(run/"trajectory.csv.r1_optimizer_trace.csv")
    preopt = read(run/"trajectory.csv.r1_preopt_capsule.csv")
    identity = json.loads((run/"identity_gate.json").read_text())
    terminals = [r for r in events if r["event_type"] == "LIDAR_SCAN_END"]
    available = [r for r in comparison if r["legacy_valid"] == "1"]
    issues = []
    if identity["process_exit_code"] != 0 or identity["discrepancy"]:
        issues.append("first failure / determinism guard; inspect console and identity gate")
    if len(terminals) != 149 or len(covariance) != 149 or len(comparison) != 149:
        issues.append("short link incomplete")
    if any(r["backend"] != "SQUARE_ROOT_QR" or r["valid"] != "1" for r in covariance):
        issues.append("production covariance unavailable/wrong backend")
    if any(r["rank"] != r["columns"] for r in covariance):
        issues.append("production rank failure")
    if any(r["post_handoff_ikfom_calls"] != "0" or r["visual_factor_count"] != "0" or
           "VISUAL" in r["event_type"] for r in events):
        issues.append("legacy state / vision leakage")
    if any(r["lidar_source_provenance"] != "WINDOW_OWNED_SE3_DESKEW" for r in terminals):
        issues.append("deskew provenance")
    if any(float(r["window_span"]) > 2+1e-12 or int(r["window_nodes"]) > 48 for r in events):
        issues.append("window bound")
    if any(r["sparse_fallback_count"] != "0" for r in runtime):
        issues.append("sparse optimizer fallback")
    if any(r["marginalization_result"] != "SUCCESS" for r in marginal):
        issues.append("QR marginalization failure")
    if any(r["candidate_basis_relinearization_calls"] != "0" for r in optimizer):
        issues.append("candidate basis callback")
    if any(r["window_covariance_valid"] != "NOT_REQUESTED_NON_LIDAR_EVENT"
           for r in events if r["event_type"] != "LIDAR_SCAN_END"):
        issues.append("non-LiDAR covariance request")
    finite = all(all(math.isfinite(float(r[k])) for k in ("px","py","pz","qx","qy","qz","qw")) and
        abs(sum(float(r[k])**2 for k in ("qx","qy","qz","qw"))-1)<1e-10 for r in trajectory)
    if not finite:
        issues.append("nonfinite/non-SO3 state")
    summary = dict(complete=identity["process_exit_code"] == 0, issues=issues,
        REAL_REPLAY_COUNT=1, GT_USED=False, READY_FOR_FORMAL_EXPERIMENT="NO",
        LiDAR_terminals=len(terminals), square_root_covariance_requests=len(covariance),
        square_root_available=sum(r["valid"]=="1" for r in covariance),
        square_root_unavailable=sum(r["valid"]!="1" for r in covariance),
        production_rank_failures=sum(r["rank"]!=r["columns"] for r in covariance),
        legacy_shadow_available=sum(r["legacy_valid"]=="1" for r in covariance),
        legacy_shadow_unavailable=sum(r["legacy_valid"]!="1" for r in covariance),
        legacy_failure_histogram=dict(Counter(r["legacy_detail"] for r in covariance if r["legacy_valid"]!="1")),
        P15_eigen_min=min((float(r["P15_min"]) for r in covariance if r["valid"]=="1"),default=None),
        P15_eigen_max=max((float(r["P15_max"]) for r in covariance if r["valid"]=="1"),default=None),
        Pmap_eigen_min=min((float(r["Pmap_min"]) for r in covariance if r["valid"]=="1"),default=None),
        Pmap_eigen_max=max((float(r["Pmap_max"]) for r in covariance if r["valid"]=="1"),default=None),
        triangular_residual_max=max((float(r["triangular_residual"]) for r in covariance),default=None),
        P15_available_comparison_error=stats([float(r["P15_relative_error"]) for r in available]),
        Pmap_available_comparison_error=stats([float(r["Pmap_relative_error"]) for r in available]),
        same_factor_NIS_decision_differences=sum(r["production_NIS_accepted"]!=r["legacy_same_factor_NIS_accepted"] for r in available),
        same_state_probe_trigger_differences=sum(r["production_probe_trigger"]!=r["legacy_probe_trigger"] for r in available),
        same_factor_admission_differences=sum(r["production_LiDAR_committed"]!=r["legacy_same_factor_admission"] for r in available),
        selected_NIS_valid=sum(r["selected_nis_valid"]=="1" for r in preopt),
        lidar_committed=sum(r["lidar_factor_committed"]=="1" for r in terminals),
        lidar_normal_rejection=sum(r["lidar_factor_committed"]!="1" for r in terminals),
        probes=sum(r["unonlocal_probe_triggered"]=="1" for r in terminals),
        NDT_calls=sum(int(r["ndt_calls"]) for r in runtime),
        optimizer_status=dict(Counter(r["optimizer_status"] for r in events)),
        optimizer_failures=sum(r["optimizer_status"] not in ("ACCEPTED_UPDATE","CONVERGED_WITHOUT_STEP") for r in events),
        QR_marginalization_attempts=len(marginal),
        QR_marginalization_failures=sum(r["marginalization_result"]!="SUCCESS" for r in marginal),
        QR_marginalization_ms=stats([float(r["qr_ms"]) for r in marginal]),
        covariance_QR_ms=stats([float(r["qr_ms"]) for r in covariance]),
        stack_max_rows=max((int(r["rows"]) for r in covariance),default=0),
        stack_max_columns=max((int(r["columns"]) for r in covariance),default=0),
        max_covariance_temporary_estimated_bytes=max((int(r["temporary_estimated_bytes"]) for r in covariance),default=0),
        post_handoff_IKFoM_calls=max((int(r["post_handoff_ikfom_calls"]) for r in events),default=0),
        visual_factors=max((int(r["visual_factor_count"]) for r in events),default=0),
        max_completed_window_span=max((float(r["window_span"]) for r in events),default=0),
        max_nodes=max((int(r["window_nodes"]) for r in events),default=0),
        max_sparse_fallback=max((int(r["sparse_fallback_count"]) for r in runtime),default=0),
        finite_SO3=finite, identity_gate=identity,
        engineering_gate="PASS" if not issues else "FAIL")
    write(output/"COVARIANCE_REQUESTS.csv",covariance)
    write(output/"LEGACY_QR_SAME_STATE_COMPARISON.csv",comparison)
    write(output/"OLD_TX136_155_COVARIANCE_REGION.csv",[r for r in covariance if 136<=int(r["transaction_id"])<=155])
    write(output/"TRAJECTORY_SHORT_ENGINEERING.csv",trajectory)
    keys = ["transaction_id","event_stamp_ns","attempt_index","nodes_before_attempt","span_before_attempt_s",
        "nodes_after_attempt","span_after_attempt_s","marginalization_result","qr_marginalized_rank",
        "qr_stack_rows","qr_rows_after_compression","qr_ms","new_prior_symmetry_max_abs","new_prior_lambda_min",
        "legacy_shadow_status","legacy_shadow_lambda_min"]
    write(output/"QR_MARGINALIZATION_HISTORY.csv",[{k:r[k] for k in keys} for r in marginal])
    for tx in (90,115,136,155,200):
        write(output/f"TX{tx}_PREOPT.csv",[r for r in preopt if r["transaction_id"]==str(tx)])
        write(output/f"TX{tx}_MARGINALIZATION.csv",[{k:r[k] for k in keys} for r in marginal if r["transaction_id"]==str(tx)])
    for name in ("input_identity.json","source_identity.json","identity_gate.json","command.json","resource.txt","console.txt"):
        shutil.copyfile(run/name,output/name)
    manifest = [dict(path=str(p),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
        for p in sorted(run.iterdir()) if p.is_file()]
    (output/"EXTERNAL_RUN_FILES.json").write_text(json.dumps(manifest,indent=2)+"\n")
    (output/"engineering_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary,indent=2))


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--run-dir",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    audit(args.run_dir,args.output)
