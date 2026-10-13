#!/usr/bin/env python3
"""Bounded P27 factor evidence, no GT and no trajectory alignment imports."""
import argparse
import importlib.util
import json
import shutil
from pathlib import Path

import mpmath as mp
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--refresh-hashes", action="store_true")
    parser.add_argument("--update", action="store_true", help="refresh P27 analysis only, after checking frozen raw hashes")
    args = parser.parse_args()
    repo, run = args.repo.resolve(), args.run.resolve()
    spec = importlib.util.spec_from_file_location("p26_evidence", repo / "src/dog_prior_map_localization/scripts/p26_covariance/summarize.py")
    evidence = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(evidence)
    archive = repo / "docs/p27_r1_square_root_covariance"
    if args.refresh_hashes:
        evidence.hash_archive(archive)
        return
    if (archive / "results.json").exists() and not args.update:
        raise ValueError("P27_RESULTS_ALREADY_EXIST")
    if args.update:
        frozen = json.loads((archive / "pre_gt_freeze.json").read_text())
        if any(evidence.sha(run / filename) != digest for filename, digest in frozen["files"].items()):
            raise ValueError("P27_FROZEN_OUTPUT_CHANGED")
    summary = evidence.run_summary(run)
    baseline_path = repo / ".p26_r1_skf_covariance_stability/final_verification_prefix"
    baseline = evidence.run_summary(baseline_path)
    metrics = evidence.rows(run / "sqrt_covariance_summary.csv")
    timed_events = {"IMU_PROPAGATE", "SKF_UPDATE", "NDT_REPLAY_END", "SKF_REPLAY_END"}
    for row in metrics:
        row["timing_measured"] = row["event"] in timed_events
    factors = evidence.rows(run / "sqrt_covariance_factors.csv")
    scales = evidence.rows(run / "sqrt_covariance_scale.csv")
    mp.mp.dps = 80
    precision = []
    for checkpoint in dict.fromkeys(row["checkpoint"] for row in factors):
        S = np.zeros((15, 15))
        for row in factors:
            if row["checkpoint"] == checkpoint:
                S[int(row["row"]), int(row["column"])] = float(row["value"])
        precise_s = mp.matrix([[mp.mpf(float(S[i, j])) for j in range(15)] for i in range(15)])
        precise_p = precise_s * precise_s.T
        eigen = mp.eigsy(precise_p, eigvals_only=True)
        root_positive = all(precise_s[i, i] > 0 for i in range(15))
        mp.cholesky(precise_p)
        # A dense double view may lose tiny positive eigenvalues at high condition.
        # Its sign is NOT used to rewrite the authoritative factor.
        view = S @ S.T
        view_mp = mp.matrix([[mp.mpf(float(view[i, j])) for j in range(15)] for i in range(15)])
        view_eigen = mp.eigsy(view_mp, eigvals_only=True)
        gram_error = mp.norm(view_mp - precise_p) / mp.norm(precise_p)
        precision.append({"checkpoint": checkpoint,
            "factor_positive_diagonal": root_positive,
            "authoritative_80digit_lambda_min": mp.nstr(eigen[0], 35),
            "authoritative_80digit_lambda_max": mp.nstr(eigen[eigen.rows - 1], 35),
            "derived_double_view_80digit_lambda_min": mp.nstr(view_eigen[0], 35),
            "derived_view_relative_gram_error": float(gram_error),
            "authoritative_cholesky_80digit": True})
    params = yaml.safe_load((run / "rosparams.yaml").read_text())
    configs = {}
    for name in ("p26_skf_covariance_stability.yaml", "p27_square_root_covariance.yaml"):
        configs[name] = yaml.safe_load((repo / "src/dog_prior_map_localization/config" / name).read_text())
    p27_config = configs["p27_square_root_covariance.yaml"]
    p27_without_flag = json.loads(json.dumps(p27_config))
    p27_without_flag["filter"].pop("experimental_square_root_covariance_enable")
    config_ok = p27_without_flag == configs["p26_skf_covariance_stability.yaml"]
    invalid = sum(int(r["invalid"]) for r in metrics)
    stage_counts = {r["event"]: int(r["count"]) for r in metrics}
    expected_stages = {"IMU_PROPAGATE", "SKF_UPDATE", "NDT_DAMPING", "OOSM_RESTORE", "HISTORY_SAVE",
                       "NDT_REPLAY_END", "SKF_REPLAY_END", "CAUSAL_PARTIAL_PROPAGATE"}
    new_jumps = max(0, summary["jumps_over_2m"] - baseline["jumps_over_2m"])
    # Read only runtime diagnostics from output bags; never read GT topics.
    import rosbag
    ndt_times = []
    for label, directory in (("P26", baseline_path), ("P27", run)):
        values = {key: [] for key in ("align_time_ms", "preprocess_time_ms", "localization_time_ms")}
        with rosbag.Bag(str(directory / "outputs.bag")) as bag:
            for _, message, _ in bag.read_messages(topics=["/dog_livo/diagnostics"]):
                for status in message.status:
                    diagnostic = {value.key: value.value for value in status.values}
                    for key in values:
                        if key in diagnostic:
                            values[key].append(float(diagnostic[key]))
        for key, samples in values.items():
            ndt_times.append({"run": label, "stage": key, "count": len(samples),
                "mean_ms": float(np.mean(samples)), "p95_ms": float(np.percentile(samples, 95)),
                "max_ms": max(samples)})
    changes = evidence.rows(run / "sqrt_covariance_updates.csv")
    growth = {}
    for stage in ("SKF_UPDATE", "NDT_DAMPING"):
        selected = [r for r in changes if r["event"] == stage]
        ratios = [float(r["trace_after"]) / float(r["trace_before"]) for r in selected]
        growth[stage] = {"count": len(selected), "increasing_trace": sum(v > 1 for v in ratios),
            "median_trace_ratio": float(np.median(ratios)), "max_trace_ratio": max(ratios),
            "final_event": selected[-1]}
    valid = bool(summary["receipt"]["completed"] and config_ok and
        params["filter"]["experimental_square_root_covariance_enable"] and invalid == 0 and
        expected_stages.issubset(stage_counts) and summary["NDT"] == 188 and
        summary["NDT_OOSM_APPLIED"] == 187 and summary["high_rate"] == 3798 and
        summary["visual_published"] == 92 and summary["SKF_status"].get("APPLIED", 0) > 0 and
        not any("NON_PSD" in s or "INVALID_SQUARE_ROOT" in s for s in summary["SKF_status"]) and
        not summary["nonfinite_pose_velocity"] and not summary["late"] and
        not summary["release_before_deadline"] and not summary["unreleased_due_before_watermark"] and
        not new_jumps and all(float(p["authoritative_80digit_lambda_min"]) > 0 for p in precision))
    peak_scale = max(float(r["trace"]) for r in scales)
    scale_unresolved = peak_scale > 1e10
    result = {
        "task": "P27-R1-SQUARE-ROOT-COVARIANCE-INTEGRATION",
        "START_SHA": "c86d38a0d96921e5ff1e835442ac0424a69bf536",
        "GT_LOADED": False, "19sec_prefix_count": 1, "full_40sec_replay_count": 0,
        "NUMERICAL_STATUS": "SQRT_NUMERICAL_STABILITY_PASS" if valid else "SQRT_NUMERICAL_STABILITY_FAILED",
        "COVARIANCE_SCALE_STATUS": "COVARIANCE_SCALE_UNRESOLVED" if scale_unresolved else "NO_EXTREME_SCALE_OBSERVED_IN_PREFIX",
        "FINAL_RESULT": ("SQRT_NUMERICAL_STABILITY_PASS_COVARIANCE_SCALE_UNRESOLVED" if valid and scale_unresolved else
                         "SQRT_NUMERICAL_STABILITY_PASS" if valid else "SQRT_NUMERICAL_STABILITY_FAILED"),
        "prefix": summary, "invalid_factor_writes": invalid, "stage_counts": stage_counts,
        "minimum_factor_diagonal": min(float(r["min_diag"]) for r in metrics),
        "RUNTIME_STATUS": "NO_UNACCEPTABLE_OVERHEAD_OBSERVED_IN_PREFIX",
        "new_jumps_over_2m": new_jumps,
        "max_jump_change_from_P26_m": summary["max_high_rate_jump_m"] - baseline["max_high_rate_jump_m"],
        "config_only_added_square_root_flag": config_ok,
        "factor_extra_payload_bytes_per_snapshot": 15 * 15 * 8 + 1,
        "factor_storage_note": "1800-byte matrix plus tag, before compiler padding; bounded history peak 418 entries",
        "checkpoints": scales, "precision": precision, "timings": metrics,
        "P26_resources": baseline["receipt"]["resources"],
        "P27_resources": summary["receipt"]["resources"],
        "ndt_timings": ndt_times, "covariance_update_scale": growth,
        "limitations": ["19 s only; no GT accuracy or 40 s stability conclusion",
                        "P26 CPU includes per-step dense diagnostics; P27 uses sparse factor diagnostics",
                        "A derived double dense view can lose weak-direction positivity; never refactor it into authority",
                        "Covariance scale and statistical consistency remain distinct from factor PSD",
                        "Offline visual CSV excludes full online camera frontend cost"]}
    target = archive / "runs/square_root_prefix"
    target.mkdir(parents=True, exist_ok=args.update)
    files = ("execution.json", "rosparams.yaml", "ndt_diagnostics.csv", "ekf_lineage.csv", "oosm.csv",
             "oosm_fixed_lag.csv", "deskew.csv", "resources.csv", "skf_measurements.csv",
             "skf_update_diagnostics.csv", "mature_diagnostics.csv", "launch.log", "playback.log",
             "sqrt_covariance_summary.csv", "sqrt_covariance_scale.csv", "sqrt_covariance_factors.csv",
             "sqrt_covariance_updates.csv", "ekf_runtime.csv")
    for filename in files:
        shutil.copy2(run / filename, target / filename)
    evidence.dump_csv(archive / "factor_precision_check.csv", precision)
    evidence.dump_csv(archive / "covariance_scale_summary.csv", scales)
    evidence.dump_csv(archive / "runtime_summary.csv", metrics)
    evidence.dump_csv(archive / "ndt_runtime_summary.csv", ndt_times)
    for source, name in ((Path("/tmp/p25-catkin-AY1eHd/build/dog_prior_map_localization/Testing/Temporary/LastTest.log"), "test_results.txt"),
                         (Path("/tmp/p25-catkin-AY1eHd/logs/dog_prior_map_localization/build.make.014.log"), "build_results.txt")):
        shutil.copy2(source, archive / name)
    # Normalize text copies for Git only. Frozen raw cache and raw SHA records
    # remain untouched; CSV field content is not stripped or reserialized.
    for path in list(target.glob("*")) + [archive / "build_results.txt", archive / "test_results.txt"]:
        if path.suffix == ".csv":
            path.write_bytes(path.read_bytes().replace(b"\r\n", b"\n"))
        elif path.suffix in (".log", ".txt"):
            path.write_text("\n".join(line.rstrip() for line in path.read_text().splitlines()) + "\n")
    resources = []
    for label, data in (("P26", baseline), ("P27", summary)):
        for node, resource in data["receipt"]["resources"].items():
            resources.append({"run": label, "node": node, **resource})
    evidence.dump_csv(archive / "resource_summary.csv", resources)
    evidence.write_json(archive / "input_freeze.json", {
        "task": result["task"], "GT_LOADED": False,
        "receipt_inputs": {k: v for k, v in summary["receipt"].items() if "sha256" in k or k == "binary_hashes"},
        "config_only_added_square_root_flag": config_ok})
    evidence.write_json(archive / "pre_gt_freeze.json", {"task": result["task"], "GT_LOADED": False,
        "archive_text_normalization": "LF line endings; trailing whitespace removed from log/txt copies only; raw cache unchanged",
        "raw_output_bag_local_only": {"path": str(run / "outputs.bag"), "sha256": evidence.sha(run / "outputs.bag")},
        "files": {filename: evidence.sha(run / filename) for filename in files}})
    evidence.write_json(archive / "results.json", result)
    evidence.hash_archive(archive)
    print(json.dumps({k: result[k] for k in ("FINAL_RESULT", "invalid_factor_writes", "minimum_factor_diagonal",
                                          "stage_counts", "checkpoints", "new_jumps_over_2m")}, indent=2))


if __name__ == "__main__":
    main()
