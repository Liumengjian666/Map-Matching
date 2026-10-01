#!/usr/bin/env python3
"""Audit one QR engineering run. No GT/ATE/RPE, no replay or estimator writes."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil


def read(path):
    return list(csv.DictReader(path.open()))


def write(path, rows):
    if not rows:
        return
    with path.open("w", newline="") as output:
        writer=csv.DictWriter(output,fieldnames=list(rows[0]),lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)


def audit(run, output):
    output.mkdir(parents=True,exist_ok=True)
    events=read(run/"events.csv")
    trajectory=read(run/"trajectory.csv")
    runtime=read(run/"runtime.csv")
    marginal=read(run/"trajectory.csv.a3c_r1_marginalization_trace.csv")
    optimizer=read(run/"trajectory.csv.r1_optimizer_trace.csv")
    preopt=read(run/"trajectory.csv.r1_preopt_capsule.csv")
    identity=json.loads((run/"identity_gate.json").read_text())
    terminals=[r for r in events if r["event_type"]=="LIDAR_SCAN_END"]
    successful=[r for r in marginal if r["marginalization_result"]=="SUCCESS"]
    issues=[]
    if identity["process_exit_code"]!=0: issues.append("process failed; inspect first failure capsule")
    if identity["discrepancy"]: issues.append(identity["discrepancy"])
    if len(terminals)!=149 or len(trajectory)!=149: issues.append("200 raw scans did not produce expected 149 post-handoff terminals")
    if any(r["lidar_source_provenance"]!="WINDOW_OWNED_SE3_DESKEW" for r in terminals): issues.append("source provenance")
    if any(r["post_handoff_ikfom_calls"]!="0" or r["visual_factor_count"]!="0" for r in events): issues.append("legacy/visual path")
    if any("VISUAL" in r["event_type"] for r in events): issues.append("visual event")
    if any(r["marginalization_backend"]!="SQUARE_ROOT_QR" for r in events): issues.append("backend")
    if any(float(r["window_span"])>2.0+1e-12 or int(r["window_nodes"])>48 for r in events): issues.append("window bound")
    if any(r["sparse_fallback_count"]!="0" for r in runtime): issues.append("sparse fallback")
    if any(r["marginalization_result"]!="SUCCESS" for r in marginal): issues.append("marginalization failure")
    if any(r["candidate_basis_relinearization_calls"]!="0" for r in optimizer): issues.append("candidate basis callback")
    if any(r["qr_marginalized_rank"]!="15" or r["qr_status"]!="SUCCESS_SQUARE_ROOT_QR" for r in marginal): issues.append("QR rank/status")
    if any(int(r["qr_rows_after_compression"])>int(r["qr_columns"])-15 for r in marginal): issues.append("prior row bound")
    finite=all(all(math.isfinite(float(r[k])) for k in ["px","py","pz","qx","qy","qz","qw"]) and
        abs(sum(float(r[k])**2 for k in ["qx","qy","qz","qw"])-1)<1e-10 for r in trajectory)
    if not finite: issues.append("nonfinite/non-SO3 completed state")
    timings=[float(r["qr_ms"]) for r in successful]
    summary=dict(complete=identity["process_exit_code"]==0,issues=issues,
        REAL_REPLAY_COUNT=1,GT_USED=False,READY_FOR_FORMAL_EXPERIMENT="NO",
        terminals=len(terminals),raw_before_handoff=51,window_owned_deskews=len(preopt),
        lidar_committed=sum(r["lidar_factor_committed"]=="1" for r in terminals),
        lidar_rejected=sum(r["lidar_factor_committed"]=="0" for r in terminals),
        covariance_available=sum(r["window_covariance_valid"]=="1" for r in terminals),
        covariance_unavailable=sum(r["window_covariance_valid"]=="0" for r in terminals),
        covariance_unavailable_transactions=[r["transaction_id"] for r in preopt if r["pre_measurement_covariance_valid"]=="0"],
        probes=sum(r["unonlocal_probe_triggered"]=="1" for r in terminals),
        ndt_calls=sum(int(r["ndt_calls"]) for r in runtime),
        optimizer_status=dict(Counter(r["optimizer_status"] for r in events)),
        natural_small_step_termination_count=sum(r["termination_reason"]=="NATURAL_SMALL_STEP_NO_ACCEPTED_UPDATE" for r in optimizer),
        candidate_basis_callback_count=sum(int(r["candidate_basis_relinearization_calls"]) for r in optimizer),
        sparse_fallback_count=max(int(r["sparse_fallback_count"]) for r in runtime),
        non_lidar_marginal_covariance_ms=max(float(r["marginal_covariance_ms"]) for r in runtime if r["event_type"]!="LIDAR_SCAN_END"),
        ndt_converged=sum(r["ndt_converged"]=="1" for r in preopt),
        uobs_valid=sum(r["uobs_valid"]=="1" for r in preopt),
        successful_marginalizations=len(successful),finite_SO3=finite,
        max_nodes=max(int(r["window_nodes"]) for r in events),
        max_completed_span=max(float(r["window_span"]) for r in events),
        max_prior_rows=max(int(r["square_root_prior_rows"]) for r in events),
        max_prior_columns_including_pre_elimination=max(int(r["qr_columns"]) for r in successful),
        max_prior_bytes_including_zero_column_extension=8*(max(int(r["square_root_prior_rows"]) for r in events)*
            max(int(r["qr_columns"]) for r in successful)+max(int(r["square_root_prior_rows"]) for r in events)),
        qr_mean_ms=sum(timings)/len(timings),qr_max_ms=max(timings),
        stored_prior_max_asymmetry=max(float(r["new_prior_symmetry_max_abs"]) for r in successful),
        stored_prior_min_eigenvalue=min(float(r["new_prior_lambda_min"]) for r in successful),
        shadow_status=dict(Counter(r["legacy_shadow_status"] for r in marginal)),
        shadow_min_eigenvalue=min(float(r["legacy_shadow_lambda_min"]) for r in marginal),
        identity_gate=identity,
        engineering_gate="PASS" if not issues else "FAIL")
    keys=["transaction_id","event_stamp_ns","enforcement_index","attempt_index","nodes_before_attempt",
        "span_before_attempt_s","nodes_after_attempt","span_after_attempt_s","marginalization_result",
        "marginalization_backend","qr_stack_rows","qr_columns","qr_marginalized_rank","qr_rank_threshold",
        "qr_R_diag_min","qr_R_diag_max","qr_rows_before_compression","qr_rows_after_compression",
        "qr_compression_rank","qr_compression_threshold","qr_discarded_row_jacobian_norm","qr_status","qr_ms",
        "new_prior_symmetry_max_abs","new_prior_lambda_min","new_prior_lambda_max","legacy_shadow_status","legacy_shadow_lambda_min"]
    write(output/"MARGINALIZATION_QR_HISTORY.csv",[{k:r[k] for k in keys} for r in marginal])
    for tx in (90,115,155):
        write(output/f"TX{tx}_marginalization.csv",[{k:r[k] for k in keys} for r in marginal if r["transaction_id"]==str(tx)])
        write(output/f"TX{tx}_optimizer.csv",[r for r in optimizer if r["transaction_id"]==str(tx)])
        selected=[r for r in preopt if r["transaction_id"]==str(tx)]
        write(output/f"TX{tx}_preopt.csv",selected)
        stamps={r["stamp_ns"] for r in selected}
        write(output/f"TX{tx}_event.csv",[r for r in events if r["timestamp"] in stamps])
    write(output/"TRAJECTORY_SHORT_ENGINEERING.csv",trajectory)
    for name in ("input_identity.json","identity_gate.json","command.json","resource.txt","console.txt"):
        shutil.copyfile(run/name,output/name)
    manifest=[]
    for path in sorted(run.iterdir()):
        if path.is_file():
            manifest.append(dict(path=str(path),bytes=path.stat().st_size,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    (output/"EXTERNAL_RUN_FILES.json").write_text(json.dumps(manifest,indent=2)+"\n")
    (output/"engineering_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary,indent=2))


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--run-dir",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    audit(args.run_dir,args.output)
