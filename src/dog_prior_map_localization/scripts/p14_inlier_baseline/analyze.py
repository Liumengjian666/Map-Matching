#!/usr/bin/env python3
"""Summarize actual single-pass results, keeping failed default poses invalid."""
import argparse
import collections
import csv
import json
import pathlib
import shutil
import subprocess
import numpy as np
from prepare import sha


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def stats(rows, field):
    values = np.asarray([float(row[field]) for row in rows])
    assert len(values) and np.isfinite(values).all(), field
    return {"mean": float(values.mean()), "p95": float(np.percentile(values, 95)),
            "max": float(values.max()), "min": float(values.min()),
            "median": float(np.median(values))}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--repo", type=pathlib.Path, required=True)
    p.add_argument("--experiment", type=pathlib.Path, required=True)
    p.add_argument("--author", type=pathlib.Path, required=True)
    p.add_argument("--archive", type=pathlib.Path, required=True)
    args = p.parse_args()
    e = args.experiment
    q, r, c, m = [read(e / name) for name in
                  ("queries_result.csv", "reference_encoding.csv", "candidates.csv", "mint_candidates.csv")]
    frozen = json.loads((e / "freeze.json").read_text())
    receipt = json.loads((e / "execution_receipt.json").read_text())
    status = read(e / "run_status.csv")[0]
    assert receipt["exit_code"] == 0 and len(r) == 389 and len(q) == int(status["queries"])
    assert [int(row["id"]) for row in q] == frozen["query_order"][:len(q)]
    assert len(c) == sum(int(row["verify_calls"]) for row in q)
    assert len(m) == sum(int(row["mint_count"]) for row in q)
    assert sum(int(row["verify_success"]) for row in c) == sum(int(row["verify_success"]) for row in q)
    assert not any(int(row["verify_success"]) and not int(row["rigid_success_pose"]) for row in c)
    for row in c:
        pose = np.array([float(row[f"T{i}"]) for i in range(16)]).reshape(4, 4)
        assert np.isfinite(pose).all()
        if row["verify_success"] == "1":
            assert np.isfinite(float(row["inlier_rmse"]))
            assert np.linalg.norm(pose[3] - [0, 0, 0, 1]) < 1e-8
            assert np.linalg.norm(pose[:3,:3].T @ pose[:3,:3] - np.eye(3)) < 1e-8
    resources = {row["stage"]: row for row in read(e / "resources.csv")}
    costs = {field: stats(q, field) for field in
             ["load_ms", "preprocess_ms", "encode_ms", "mint_ms", "beam_ms", "verify_ms",
              "compute_ms", "end_to_end_ms", "cpu_ms"]}
    valid_query_ids = [int(row["id"]) for row in q if int(row["mint_positive"]) > 0 and int(row["beam_positive"]) > 0]
    result = {
        "task": "P14-INLIER-REAL-DATA-BASELINE-R1",
        "final_result": "INLIER_CPP_FUNCTIONAL_SINGLE_SCAN_GEOMETRY_NOT_CERTIFIED",
        "next": "DECIDE_CAUSAL_ACCUMULATION_INPUT_PROTOCOL",
        "repository": "Liumengjian666/Map-Matching", "branch": "research/p9-r4-heldout-visual-evidence",
        "start_sha": "f3a996da317a08bad0886f14dc499c0d24031ce6",
        "code_sha": receipt["code_sha"], "end_sha": "REPORT_OWNING_COMMIT_REPORTED_AT_HANDOFF",
        "push_executed": False,
        "upstream": {"sha": receipt["upstream_sha"], "license": "MIT", "version": "README v1.1.0; CMake project 0.2.0",
                     "backend": "DIRECT_UNMODIFIED_AUTHOR_CPP", "gicp": "NOT_RUN",
                     "author_smoke": "PASS", "transform": "p_reference = T_sensor * p_query"},
        "data": {
            "references": len(r), "reference_nonempty_tokens": sum(int(row["tokens"]) > 0 for row in r),
            "reference_empty_tokens": sum(int(row["tokens"]) == 0 for row in r),
            "reference_tokens_total": sum(int(row["tokens"]) for row in r),
            "queries": len(q), "query_nonempty_tokens": sum(int(row["tokens"]) > 0 for row in q),
            "query_empty_tokens": sum(int(row["tokens"]) == 0 for row in q),
            "query_tokens_total": sum(int(row["tokens"]) for row in q),
            "query_ge32_tokens": sum(int(row["tokens"]) >= 32 for row in q),
            "effective_mint_and_beam_queries": len(valid_query_ids), "effective_query_ids": valid_query_ids,
            "mint_returned_ids": len(m), "mint_positive_candidates": sum(float(row["score"]) > 0 for row in m),
            "beam_returned_ids": len(c), "beam_positive_queries": sum(int(row["beam_positive"]) > 0 for row in q),
            "beam_positive_candidates": sum(float(row["beam_score"]) > 0 for row in c),
            "beam_positive_from_positive_mint": sum(float(row["mint_score"]) > 0 and float(row["beam_score"]) > 0 for row in c),
            "verify_calls": len(c), "verify_successes": sum(int(row["verify_success"]) for row in c),
            "queries_with_verify_success": sum(int(row["verify_success"]) > 0 for row in q),
            "legal_relative_poses": sum(int(row["legal_relative_poses"]) for row in q),
            "gicp_calls": 0, "full_cloud_registration_calls": 0,
            "multiframe_accumulation": False, "gt_loaded": False,
            "correctness_labels": "NONE", "accuracy_gain": "NOT_EVALUATED",
        },
        "failure": {
            "stages": dict(collections.Counter(row["fail_stage"] for row in c)),
            "correspondences_min": min(int(row["correspondences"]) for row in c),
            "correspondences_max": max(int(row["correspondences"]) for row in c),
            "minimum_required_correspondences": 32,
            "ransac_or_geometric_fit_entered_calls": sum(row["fail_stage"] != "1" for row in c),
            "inlier_rmse": "NOT_AVAILABLE; all failed outputs carry author Inf sentinel",
            "nonfinite_successful_poses": 0,
            "failed_identity_outputs_are_not_estimates": True,
        },
        "input_statistics": {"reference": {field: stats(r, field) for field in ["raw_points", "prepared_points", "tokens", "active_height_rows", "ground_Rzz"]},
                             "query": {field: stats(q, field) for field in ["raw_points", "prepared_points", "tokens", "active_height_rows", "ground_Rzz"]}},
        "cost_ms": costs,
        "memory": {
            "peak_rss_kib": max(int(row["peak_rss_kib"]) for row in read(e / "resources.csv")),
            "serialized_database_bytes": (e / "reference_tokens_keypoints.bin").stat().st_size,
            "resident_increment_kib_includes_allocator_and_encoder_workspace": int(resources["QUERY_COMPLETE"]["rss_kib"]) - int(resources["BEFORE_DATABASE"]["rss_kib"]),
            "matcher_index_increment_kib": int(resources["MATCHER_INDEX"]["rss_kib"]) - int(resources["REFERENCE_DB"]["rss_kib"]),
            "matcher_dense_histogram_bytes": len(r) * 10 * 20 * 7 * 4,
            "exclusive_database_allocator_bytes": "NOT_SEPARATELY_MEASURED",
        },
        "offline_reference_db_wall_ms": float(resources["REFERENCE_DB"]["wall_ms"]),
        "process_wall_seconds_including_offline_db": receipt["process_wall_seconds"],
        "comparability": "430-455ms P12/P12B DCReg is reference only: different candidates, point preprocessing and full-map solver; Verify480/480 exits before RANSAC, no demonstrated geometry speedup",
        "no_production_files_changed": True,
        "tests": {"release_build": "PASS", "author_api_smoke": "PASS", "frame_direction": "PASS 1.7431442023888599e-15",
                  "author_preprocessing_exact_parity": "PASS", "seeded_verify_repeat": "PASS", "csv_json_pose_audit": "PASS"},
    }
    args.archive.mkdir(parents=True, exist_ok=True)
    for name in ["freeze.json", "references.csv", "queries.csv", "reference_encoding.csv", "queries_result.csv", "candidates.csv",
                 "mint_candidates.csv", "resources.csv", "run_status.csv", "execution_receipt.json", "run.log", "adapter_test.json", "smoke_release.csv"]:
        shutil.copyfile(e / name, args.archive / name)
    raw_logs = {}
    for name in ["configure.log", "configure_fixed.log", "build_smoke.log", "build_smoke_fixed.log", "smoke_diagnostic.log", "build_final.log"]:
        source = e.parent / name
        # Mechanical Git formatting only; external raw logs remain untouched.
        (args.archive / name).write_text("\n".join(line.rstrip() for line in source.read_text().splitlines()) + "\n")
        raw_logs[name] = {"raw_path": str(source), "raw_sha256": sha(source),
                          "archive_normalization": "trailing whitespace stripped; LF"}
    (args.archive / "raw_log_provenance.json").write_text(json.dumps(raw_logs, indent=2) + "\n")
    author_sources = [f for f in (args.author / "cpp/inlier_core").rglob("*") if f.is_file()]
    author_sources += [args.author / "LICENSE", args.author / "inlier/config/default.yaml"]
    provenance = {
        "upstream_clean": not bool(subprocess.check_output(["git", "-C", str(args.author), "status", "--porcelain"], text=True)),
        "upstream_sha": receipt["upstream_sha"], "binary_sha256": receipt["binary_sha256"],
        "upstream_sources": {str(f.relative_to(args.author)): sha(f) for f in author_sources},
        "caller_sources": receipt["caller_source_sha256"],
        "external_database": {"path": str(e / "reference_tokens_keypoints.bin"), "sha256": sha(e / "reference_tokens_keypoints.bin")},
    }
    (args.archive / "source_provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    (args.archive / "results.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    with (args.archive / "timing_summary.csv").open("w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["stage", "mean_ms", "p95_ms", "max_ms"])
        w.writerows([field, values["mean"], values["p95"], values["max"]] for field, values in costs.items())
    manifest = {file.name: {"bytes": file.stat().st_size, "sha256": sha(file)}
                for file in args.archive.iterdir() if file.is_file() and file.name != "artifact_hashes.json"}
    (args.archive / "artifact_hashes.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
