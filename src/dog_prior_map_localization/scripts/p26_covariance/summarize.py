#!/usr/bin/env python3
"""P26 bounded covariance evidence; never imports GT or a trajectory evaluator."""
import argparse
import csv
import hashlib
import json
import math
import shutil
from collections import Counter
from pathlib import Path

import mpmath as mp
import numpy as np
import yaml

START_SHA = "d373783ead17eff3d3d6e851b404668f306fb01a"


def rows(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def dump_csv(path, data):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(data[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(data)


def numerical_fixture(path, label):
    matrices = {stage: np.zeros((15, 15)) for stage in ("before", "after")}
    for row in rows(path):
        matrices[row["stage"]][int(row["row"]), int(row["column"])] = float(row["value"])
    result = []
    mp.mp.dps = 80
    for stage, covariance in matrices.items():
        precise = mp.matrix([[(mp.mpf(float(covariance[i, j])) +
                              mp.mpf(float(covariance[j, i]))) / 2
                             for j in range(15)] for i in range(15)])
        eig = mp.eigsy(precise, eigvals_only=True)
        try:
            mp.cholesky(precise)
            positive = True
        except ValueError:
            positive = False
        symmetric = (covariance + covariance.T) / 2
        result.append({"fixture": label, "stage": stage,
            "trace": float(np.trace(covariance)),
            "max_asymmetry": float(np.max(np.abs(covariance - covariance.T))),
            "numpy_double_min_eigenvalue": float(np.linalg.eigvalsh(symmetric)[0]),
            "mpmath_80digits_min_eigenvalue": mp.nstr(eig[0], 35),
            "mpmath_80digits_max_eigenvalue": mp.nstr(eig[eig.rows-1], 35),
            "mpmath_cholesky_positive": positive,
            "spectral_condition": mp.nstr(eig[eig.rows-1] / eig[0], 20) if eig[0]>0 else "INDEFINITE"})
    return result


def run_summary(run):
    receipt = json.loads((run / "execution.json").read_text())
    ndt = rows(run / "ndt_diagnostics.csv")
    oosm = rows(run / "oosm.csv")
    lineage = rows(run / "ekf_lineage.csv")
    fixed = rows(run / "oosm_fixed_lag.csv")
    updates = rows(run / "skf_update_diagnostics.csv")
    high = [r for r in lineage if r["event_type"] == "IMU_HIGH_RATE_PUBLISH"]
    pose_fields = ("p_x", "p_y", "p_z", "q_x", "q_y", "q_z", "q_w", "v_x", "v_y", "v_z")
    nonfinite = sum(not all(math.isfinite(float(r[k])) for k in pose_fields)
                    for r in lineage if r["event_type"] in
                    ("IMU_HIGH_RATE_PUBLISH", "CORRECTED_PUBLISH"))
    jumps = [math.sqrt(sum((float(b[k]) - float(a[k])) ** 2 for k in ("p_x", "p_y", "p_z")))
             for a, b in zip(high, high[1:])]
    completed_ids = {r["observation_id"] for r in fixed if not r["status"].startswith("QUEUED")
                     and r["status"] != "FIXED_LAG_DEADLINE_NOT_REACHED_AT_SHUTDOWN"}
    final_stamp = float(high[-1]["state_stamp"])
    unfinished = [r for r in fixed if r["status"].startswith("QUEUED") and
                  r["observation_id"] not in completed_ids]
    bad_release = [r for r in fixed if not r["status"].startswith("QUEUED") and
                   math.isfinite(float(r["release_state_stamp"])) and
                   float(r["release_state_stamp"]) < float(r["deadline_stamp"])]
    summary = {
        "receipt": receipt, "NDT": len(ndt),
        "NDT_converged": sum(r["ndt_has_converged"] == "1" for r in ndt),
        "NDT_OOSM_APPLIED": sum(r["oosm_result"] == "APPLIED" for r in oosm),
        "high_rate": len(high), "corrected": sum(r["event_type"] == "CORRECTED_PUBLISH" for r in lineage),
        "visual_published": len(rows(run / "skf_measurements.csv")),
        "SKF_status": dict(Counter(r["status"] for r in updates)),
        "nonfinite_pose_velocity": nonfinite,
        "late": sum("LATE" in r["status"] for r in fixed),
        "release_before_deadline": len(bad_release),
        "unreleased_at_stop": len(unfinished),
        "unreleased_due_before_watermark": sum(float(r["deadline_stamp"]) <= final_stamp for r in unfinished),
        "max_high_rate_jump_m": max(jumps), "jumps_over_2m": sum(j > 2 for j in jumps),
        "first_scan_stamp": ndt[0]["lidar_header_stamp"], "last_scan_stamp": ndt[-1]["lidar_header_stamp"],
        "final_state_stamp": high[-1]["state_stamp"],
        "span_from_first_scan_sec": float(high[-1]["state_stamp"]) - float(ndt[0]["lidar_header_stamp"]),
    }
    audit = run / "covariance_audit.csv.summary.csv"
    if audit.exists():
        summary["audit"] = rows(audit)
    return summary


def hash_archive(archive):
    data = {str(path.relative_to(archive)): {"size_bytes": path.stat().st_size, "sha256": sha(path)}
            for path in sorted(archive.rglob("*")) if path.is_file() and path.name != "artifact_hashes.json"}
    write_json(archive / "artifact_hashes.json", {"GT_LOADED": False, "files": data,
               "map_and_bag_committed": False})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--refresh-hashes", action="store_true")
    parser.add_argument("--update", action="store_true", help="recompute lightweight summaries in the existing P26 archive")
    args = parser.parse_args()
    repo = args.repo.resolve()
    root = repo / ".p26_r1_skf_covariance_stability"
    archive = repo / "docs/p26_r1_skf_covariance_stability"
    if args.refresh_hashes:
        hash_archive(archive)
        return
    if archive.exists() and not args.update:
        raise ValueError("P26_ARCHIVE_ALREADY_EXISTS")
    names = ("diagnostic_prefix", "diagnostic_psd_prefix", "verification_prefix", "final_verification_prefix")
    summaries = {name: run_summary(root / name) for name in names}
    archive.mkdir(exist_ok=True)
    for name in names:
        source, target = root / name, archive / "runs" / name
        target.mkdir(parents=True, exist_ok=True)
        files = ("execution.json", "skf_update_diagnostics.csv", "oosm.csv", "oosm_fixed_lag.csv",
                 "covariance_audit.csv", "covariance_audit.csv.summary.csv",
                 "covariance_audit.csv.first_matrix.csv", "covariance_audit.csv.reference_matrix.csv",
                 "covariance_audit.csv.reference_event.csv")
        if name == "final_verification_prefix":
            files += ("ndt_diagnostics.csv", "ekf_lineage.csv", "deskew.csv", "resources.csv", "rosparams.yaml")
        for filename in files:
            if (source / filename).exists():
                shutil.copy2(source / filename, target / filename)
    fixture_cases = (
        (root / "diagnostic_psd_prefix/covariance_audit.csv.first_matrix.csv", "eigensolver_false_negative"),
        (root / "verification_prefix/covariance_audit.csv.first_matrix.csv", "double_LLT_false_negative"),
    )
    reference = root / "final_verification_prefix/covariance_audit.csv.reference_matrix.csv"
    if reference.exists():
        fixture_cases += ((reference, "fixed_original_P25_failure_window"),)
    final_fault = root / "final_verification_prefix/covariance_audit.csv.first_matrix.csv"
    if final_fault.exists():
        fixture_cases += ((final_fault, "post_fix_actual_IMU_PSD_loss"),)
    precision = []
    for path, label in fixture_cases:
        shutil.copy2(path, archive / (label + ".csv"))
        precision.extend(numerical_fixture(path, label))
    dump_csv(archive / "precision_reconstruction.csv", precision)
    final = summaries["final_verification_prefix"]
    baseline = summaries["diagnostic_psd_prefix"]
    final["new_jumps_over_2m"] = max(0, final["jumps_over_2m"] - baseline["jumps_over_2m"])
    final["peak_jump_change_from_diagnostic_m"] = final["max_high_rate_jump_m"] - baseline["max_high_rate_jump_m"]
    audit_valid = all(int(row["uncertified_before"]) == 0 and int(row["uncertified_after"]) == 0
                      for row in final["audit"])
    skf_ok = final["SKF_status"].get("APPLIED", 0) > 0 and set(final["SKF_status"]) == {"APPLIED"}
    valid = bool(final["receipt"]["completed"] and audit_valid and skf_ok and
                 not final["nonfinite_pose_velocity"] and not final["late"] and
                 not final["release_before_deadline"] and not final["unreleased_due_before_watermark"] and
                 not final["new_jumps_over_2m"] and final["span_from_first_scan_sec"] > 18.51)
    parameters = yaml.safe_load((root / "final_verification_prefix/rosparams.yaml").read_text())
    valid = valid and parameters["filter"]["experimental_covariance_stability_enable"] is True
    result = {
        "task": "P26-R1-SKF-COVARIANCE-STABILITY-FIX", "START_SHA": START_SHA,
        "final_result": "SKF_COVARIANCE_STABILITY_PREFIX_PASS" if valid else "SKF_COVARIANCE_STABILITY_NOT_ESTABLISHED",
        "short_prefix_acceptance": valid, "GT_LOADED": False, "full_40sec_replay_count": 0,
        "19sec_prefix_count": len(names), "run_summaries": summaries,
        "precision_evidence": precision,
        "first_raw_negative_event": "IMU_PROPAGATE at 1517157237.428112 (raw Eigen sign, actual stored symmetric matrix is positive definite)",
        "first_actual_PSD_loss_after_partial_fix": next((r for r in rows(root / "final_verification_prefix/covariance_audit.csv")
                  if r["before_certified"]=="1" and r["after_certified"]=="0"), None),
        "strict_15d_factor_failures_after_fix": sum(int(r["uncertified_after"]) for r in final["audit"]),
        "limitation": "Repeated fixed-weight conservative CI strongly inflates covariance; 19 s validation does not prove 40 s stability or localisation accuracy.",
    }
    write_json(archive / "results.json", result)
    write_json(archive / "input_freeze.json", {"START_SHA": START_SHA, "GT_LOADED": False,
        "map_sha256": final["receipt"]["map_sha256"], "bag_sha256": final["receipt"]["bag_sha256"],
        "visual_sha256": final["receipt"]["visual_csv_sha256"],
        "dataset_sha256": final["receipt"]["dataset_config_sha256"],
        "P26_overlay_sha256": final["receipt"]["mature_config_sha256"],
        "binary_sha256": final["receipt"]["binary_hashes"]})
    dump_csv(archive / "covariance_operation_summary.csv", final["audit"])
    hash_archive(archive)
    print(json.dumps({"FINAL_RESULT": result["final_result"], "SKF": final["SKF_status"],
                     "NDT": final["NDT"], "NDT_OOSM": final["NDT_OOSM_APPLIED"],
                     "high_rate": final["high_rate"], "certified_audit": audit_valid}, indent=2))


if __name__ == "__main__":
    main()
