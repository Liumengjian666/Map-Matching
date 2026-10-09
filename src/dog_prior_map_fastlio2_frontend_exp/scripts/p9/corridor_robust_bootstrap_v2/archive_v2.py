"""Archive the one-shot early-stop outcome; never calls an alignment or fitter.

This post-run utility was added after the quality freeze. Frozen scientific
source files remain unchanged. Artifact hashes exclude only their own manifest.
"""
import csv
import json
import math
import pathlib
import shutil
import subprocess
import sys

import numpy as np

from prepare_v2 import ARCHIVE, HERE, INPUT, OUTPUT, ROOT, sha, write_csv, write_json
from execute_v2_once import verified_command

START_SHA = "368b78f8cd2fa124f03103c3807b702197eac7ee"
RESULT = "BOOTSTRAP_LOCAL_ODOMETRY_NOT_CERTIFIED"
NEXT = "REASSESS_DATASET_BOOTSTRAP_FEASIBILITY"
BUILD = pathlib.Path("/tmp/p9_corridor_robust_bootstrap_v2_build")


def read_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def copy_once(source, target):
    with source.open("rb") as src, target.open("xb") as dst:
        shutil.copyfileobj(src, dst)


def check_outcome():
    freeze, _, _ = verified_command()  # Hash validation only, no subprocess.
    admission = json.loads((OUTPUT / "local_admission.json").read_text())
    summary = json.loads((OUTPUT / "registration/registration_summary.json").read_text())
    completed = json.loads((OUTPUT / "V2_ALIGNMENT_COMPLETED.json").read_text())
    rows = read_rows(OUTPUT / "registration/robust_lidar_odometry.csv")
    if admission["status"] != "FAIL" or not summary["stopped"]:
        raise RuntimeError("this finalizer only accepts the observed early-stop outcome")
    if completed["returncode"] != 0 or admission["attempted_pairs"] != 3:
        raise RuntimeError("unexpected execution receipt")
    if summary["true_observation_pairs"] != 0 or summary["maximum_consecutive_failures"] != 3:
        raise RuntimeError("unexpected observed trajectory counts")
    if len(rows) != 99 or [int(r["transaction_id"]) for r in rows] != list(range(1, 100)):
        raise RuntimeError("missing or reordered 99-scan ledger")
    expected_roles = ["COORDINATE_ANCHOR", "CAUSAL_PREDICTION_ONLY",
                      "CAUSAL_PREDICTION_ONLY", "REJECTED_STOP"] + ["NOT_RUN"] * 95
    if [r["role"] for r in rows] != expected_roles:
        raise RuntimeError("prediction/observation/NOT_RUN ledger changed")
    if any(r["true_observation"] != "0" for r in rows):
        raise RuntimeError("unaccepted pose marked as a real observation")
    if sha(OUTPUT / "registration/robust_lidar_odometry.csv") != admission["odometry_sha256"]:
        raise RuntimeError("admission no longer binds odometry bytes")
    quality = read_rows(OUTPUT / "registration/overlap_quality.csv")[1:]
    if [r["transaction_id"] for r in quality] != ["2", "3", "4"]:
        raise RuntimeError("unexpected attempted-pair quality rows")
    if any(r["overlap_pass"] != "0" or r["trimmed_pass"] != "1" for r in quality):
        raise RuntimeError("reported rejection factor differs from actual results")
    if any(r["reason"] != "OVERLAP|" for r in rows[1:4]):
        raise RuntimeError("additional failure factors need separate reporting")
    if summary["NDT_CALLS"] != 0 or summary["GT_LOADED"]:
        raise RuntimeError("forbidden scope consumed")
    return freeze, admission, summary, completed, quality


def finalize():
    freeze, admission, summary, completed, quality = check_outcome()
    if (ARCHIVE / "results.json").exists():
        raise RuntimeError("refuse to overwrite archived one-shot result")
    registration = OUTPUT / "registration"
    for name in ("imu_rotation_prediction.csv", "robust_lidar_odometry.csv",
                 "overlap_quality.csv", "bidirectional_consistency.csv",
                 "registration_uncertainty.csv", "registration_runtime.csv",
                 "registration_summary.json"):
        copy_once(registration / name, ARCHIVE / name)
    for name in ("V2_ALIGNMENT_STARTED.json", "V2_ALIGNMENT_COMPLETED.json",
                 "runner_config.json", "cpp_input_preflight.csv",
                 "registration_resource.txt", "registration_stdout.log",
                 "registration_stderr.log"):
        copy_once(OUTPUT / name, ARCHIVE / name)

    runtime = read_rows(registration / "registration_runtime.csv")
    cost_rows = []
    phases = ("raw_read_s", "IMU_rotational_deskew_s", "submap_and_prefit_support_s",
              "forward_GICP_s", "independent_reverse_GICP_s", "robust_quality_s", "total_s")
    for phase in phases:
        measured = runtime if phase in ("raw_read_s", "IMU_rotational_deskew_s", "total_s") else runtime[1:]
        values = np.array([float(r[phase]) for r in measured])
        cost_rows.append({"phase": phase, "status": "MEASURED_EARLY_STOP_ONLY",
                          "sample_count": len(values), "total_s": float(values.sum()),
                          "mean_s": float(values.mean()), "median_s": float(np.median(values)),
                          "P95_s": float(np.percentile(values, 95)),
                          "accounting": "CROSS_DATASET_BASELINE_PREPARATION_COST"})
    for phase in ("moving_state_fit", "IKFoM_real_initialization", "NDT_smoke", "baseline_runA", "baseline_runB"):
        cost_rows.append({"phase": phase, "status": "NOT_RUN", "sample_count": 0,
                          "total_s": "", "mean_s": "", "median_s": "", "P95_s": "",
                          "accounting": "CROSS_DATASET_BASELINE_PREPARATION_COST"})
    write_csv(ARCHIVE / "runtime_breakdown.csv", cost_rows)
    write_json(ARCHIVE / "motion_state_estimate.json", {
        "status": "NOT_RUN", "reason": "local odometry admission FAIL before state fitting",
        "state_accepted": False, "velocity": None, "gyro_bias": None,
        "accel_bias": None, "gravity": None, "covariance": None})
    for name, stage in (("observability_diagnostics.csv", "observability"),
                        ("bootstrap_validation.csv", "8-to-10s independent validation"),
                        ("ikfom_real_initialization.csv", "initializeMoving real state injection"),
                        ("baseline_runA.csv", "formal Run A"), ("baseline_runB.csv", "formal Run B"),
                        ("baseline_determinism.csv", "formal A/B determinism"),
                        ("source_t0_uobs_parity.csv", "formal SAME_OBJECTIVE source/T0/U_obs/W2")):
        write_csv(ARCHIVE / name, [{"status": "NOT_RUN", "stage": stage,
                                  "reason": "BOOTSTRAP_LOCAL_ODOMETRY_NOT_CERTIFIED"}])

    verification_logs = {
        "v2_ctest.log": BUILD / "Testing/Temporary/LastTest.log",
        "p9_ctest.log": pathlib.Path("/tmp/p9_r4_release.Eirto1/Testing/Temporary/LastTest.log"),
        "inherited_initialization_ctest.log": pathlib.Path("/tmp/p9_corridor_moving_init_build/Testing/Temporary/LastTest.log")}
    for name, source in verification_logs.items():
        text = source.read_text()
        expected = 5 if name == "v2_ctest.log" else 41 if name == "p9_ctest.log" else 6
        if text.count("Test Passed.") != expected or "Test Failed." in text:
            raise RuntimeError("missing actual passing CTest evidence: " + name)
        copy_once(source, ARCHIVE / name)
    cache = (BUILD / "CMakeCache.txt").read_text()
    if "CMAKE_BUILD_TYPE:STRING=Release" not in cache:
        raise RuntimeError("V2 runner not built as Release")
    for name in ("CMakeCache.txt", "CTestTestfile.cmake"):
        copy_once(BUILD / name, ARCHIVE / ("v2_" + name))
    write_json(ARCHIVE / "verification_receipt.json", {
        "Release_build": "PASS", "frozen_binary_sha256": freeze["binary_sha256"],
        "P9_tests": "41/41 PASS", "V2_CTest_targets": "5/5 PASS",
        "inherited_initialization_P7_tests": "6/6 PASS", "real_input_preflight": "99/99 PASS",
        "compiled_input_preflight": "99/99 PASS", "one_shot_guard_tests": "PASS; mocked child only",
        "wrong_directory_ctest": "No tests found; NOT counted as verification",
        "frozen_scientific_sources_unchanged": True,
        "postrun_additions": ["archive_v2.py", "test_archive.py", "documentation/statistical receipts"],
        "cross_model_review": "SKIPPED per user's current-task reply; no external CLI",
        "independent_review_limit": "implementation reviewer returned actionable findings, then service usage limit; not claimed complete"})

    write_source_diff("x")

    result = {
        "task": "PAPER-P9-R7-R5-ROBUST-CAUSAL-LIDAR-BOOTSTRAP",
        "environment_recovery": "PASS", "protocol": "P9_CORRIDOR01_BOOTSTRAP_V2",
        "git": {"branch": "research/p9-r4-heldout-visual-evidence", "start_sha": START_SHA,
                "worktree": str(ROOT), "end_sha_source": "Git commit containing this archive (self-hash not embedded)",
                "push_executed": False, "remote_start_verification": "research controller provided; not newly network-verified"},
        "persistent_output": str(OUTPUT), "RAW_INPUT": "SHA256 PASS; all 10 manifest files",
        "raw_bag_sha256": "c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811",
        "raw_bag_rehashed_this_turn": False,
        "input_manifest_sha256": freeze["input_manifest_sha256"],
        "raw_input_counts": {"scans": 2777, "IMU_samples": 55957, "timed_points": 79932911},
        "bootstrap": {"start_ns": freeze["first_sensor_stamp_ns"], "end_ns": freeze["boot_stamp_ns"],
                      "ledger_scans": 99, "pair_opportunities": 98, "attempted_pairs": 3,
                      "forward_GICP_calls": 3, "independent_reverse_GICP_calls": 3,
                      "accepted_observations": 0, "prediction_only_frames": 2,
                      "rejected_stop_frames": 1, "NOT_RUN_frames": 95,
                      "max_consecutive_failures": 3, "overlap_pass_count": 0,
                      "trimmed_pass_count": 3, "independent_inverse_consistency_pass_count": 3,
                      "IMU_rotation_consistency_pass_count": 3, "local_admission": "FAIL",
                      "TX1_omitted_prefix_points": 20394, "TX1_raw_points": 28866,
                      "TX1_voxel_anchor_points": 661,
                      "reason": "three consecutive forward-overlap failures; STOP before TX5",
                      "trajectory_admission": admission},
        "TX2": quality[0],
        "MOVING_STATE": "NOT_RUN", "OBSERVABILITY": "NOT_RUN",
        "IKFOM_real_injection": "NOT_RUN", "SCAN_END_formal_runtime": "NOT_RUN",
        "scan_end_deskew_preflight": "99/99 PASS (not formal IKFoM propagation)",
        "BASELINE": {"Run_A": "NOT_RUN", "Run_B": "NOT_RUN", "source_T0_Uobs_W2_parity": "NOT_RUN",
                     "preserved_post_boot_input_scans": 2678},
        "cost": {"registration_child_wall_s": completed["wall_s"], "registration_runner_wall_s": summary["wall_s"],
                 "GICP_alignment_total_s": summary["GICP_s"], "rotational_deskew_total_s": summary["rotational_deskew_s"],
                 "quality_total_s": summary["quality_s"], "peak_RSS_KiB": summary["peak_RSS_KiB"],
                 "NDT_smoke_calls": 0, "Run_A_NDT_calls": 0, "Run_B_NDT_calls": 0,
                 "accounting": "BASELINE_PREPARATION_ONLY; not DUAL-U incremental/online cost",
                 "runtime_summary": cost_rows},
        "ORACLE263_CALLS": 0, "B12_CALLS": 0, "VISUAL_EXTRACTION": 0, "GT_LOADED": False,
        "limitations": ["Only three real pairs attempted; 95 pairs unknown, not 95 measured failures",
                        "TX1 partial cloud after causally unsupported prefix removal limits startup overlap; no ablation proving unique cause",
                        "All three attempts used the single 661-point partial anchor; populated three-frame submap and accepted-history CV were not reached",
                        "Small trimmed residual and inverse consistency do not compensate for low forward overlap",
                        "No mathematical unobservability result: state fitting was not reached",
                        "Marginal prediction uncertainties are engineering assumptions, not certified covariance",
                        "Old R7-R4 result and all R4/R5/R6 scientific results unchanged",
                        "No V3/tuning/repeated V2 permitted; next dataset choice remains research controller's decision"],
        "FINAL_RESULT": RESULT, "NEXT": NEXT}
    write_json(ARCHIVE / "results.json", result)
    write_hash_manifest("x")
    verify()


def write_source_diff(mode):
    patches = []
    for source in sorted(HERE.iterdir()):
        if source.is_file():
            diff = subprocess.run(["git", "diff", "--no-index", "--", "/dev/null", str(source.relative_to(ROOT))],
                                  cwd=ROOT, capture_output=True, text=True, check=False)
            if diff.returncode not in (0, 1):
                raise RuntimeError("source diff generation failed")
            patches.append(diff.stdout)
    with (ARCHIVE / "source_diff.patch").open(mode) as stream:
        stream.write("".join(patches))


def write_hash_manifest(mode):
    files = list(ARCHIVE.iterdir()) + [p for p in HERE.iterdir() if p.is_file()]
    hashes = {str(p.relative_to(ROOT)): sha(p) for p in sorted(files)
              if p.is_file() and p.name != "artifact_hashes.json"}
    with (ARCHIVE / "artifact_hashes.json").open(mode) as stream:
        json.dump(hashes, stream, indent=2, allow_nan=False)
        stream.write("\n")


def normalize_snapshot(text):
    return "\n".join(line.rstrip() for line in text.splitlines()).rstrip() + "\n"


def refresh_postrun_archive():
    """Mechanical pre-commit snapshot hygiene only; never alters frozen files."""
    check_outcome()
    result = json.loads((ARCHIVE / "results.json").read_text())
    actual_tx2 = read_rows(OUTPUT / "registration/overlap_quality.csv")[1]
    if result["TX2"] != actual_tx2 or result["FINAL_RESULT"] != RESULT or result["NEXT"] != NEXT:
        raise RuntimeError("result changed during postrun snapshot hygiene")
    receipts = []
    for name in ("CMakeCache.txt", "CTestTestfile.cmake"):
        source, target = BUILD / name, ARCHIVE / ("v2_" + name)
        original = source.read_text()
        normalized = normalize_snapshot(original)
        if target.read_text() not in (original, normalized):
            raise RuntimeError("archived build snapshot no longer matches actual build")
        target.write_text(normalized)
        receipts.append({"source": str(source), "source_sha256": sha(source),
                         "archive": str(target.relative_to(ROOT)), "archive_sha256": sha(target),
                         "normalization": "rstrip each line; strip final blank lines; final single newline"})
    write_json(ARCHIVE / "build_snapshot_normalization.json", receipts)
    write_source_diff("w")
    write_hash_manifest("w")
    verify()


def check_csv(path):
    with path.open(newline="") as stream:
        rows = list(csv.reader(stream))
    if not rows or not rows[0] or len(rows[0]) != len(set(rows[0])):
        raise RuntimeError("bad CSV header: " + str(path))
    if any(len(r) != len(rows[0]) for r in rows[1:]):
        raise RuntimeError("ragged CSV: " + str(path))
    for row in rows[1:]:
        for cell in row:
            try:
                numeric = float(cell)
            except ValueError:
                continue
            if not math.isfinite(numeric):
                raise RuntimeError("nonfinite CSV metric: " + str(path))


def verify():
    check_outcome()
    manifest = json.loads((ARCHIVE / "artifact_hashes.json").read_text())
    actual = {str(p.relative_to(ROOT)) for p in ARCHIVE.iterdir() if p.is_file() and p.name != "artifact_hashes.json"}
    actual |= {str(p.relative_to(ROOT)) for p in HERE.iterdir() if p.is_file()}
    if set(manifest) != actual:
        raise RuntimeError("artifact hash inventory incomplete or unexpected additions")
    for relative, expected in manifest.items():
        path = ROOT / relative
        if sha(path) != expected:
            raise RuntimeError("artifact changed: " + relative)
        if path.suffix == ".csv":
            check_csv(path)
        elif path.suffix == ".json":
            json.loads(path.read_text(), parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v)))
    ledger = read_rows(ARCHIVE / "formal_frame_ledger.csv")
    if len(ledger) != 2777 or sum(r["role"] == "BOOTSTRAP" for r in ledger) != 99:
        raise RuntimeError("raw frame inventory altered")
    if any(r["formal_NDT_run"] != "NO" for r in ledger):
        raise RuntimeError("formal NDT unexpectedly recorded")
    result = json.loads((ARCHIVE / "results.json").read_text())
    if result["FINAL_RESULT"] != RESULT or result["NEXT"] != NEXT or result["GT_LOADED"]:
        raise RuntimeError("result/scope changed")
    print(json.dumps({"artifact_hashes": "PASS", "files": len(manifest), "CSV_JSON": "PASS",
                      "frozen_source_input_binary_hashes": "PASS", "FINAL_RESULT": RESULT}))


if __name__ == "__main__":
    if sys.argv[1:] == ["finalize"]:
        finalize()
    elif sys.argv[1:] == ["verify"]:
        verify()
    elif sys.argv[1:] == ["refresh-postrun"]:
        refresh_postrun_archive()
    else:
        raise SystemExit("usage: archive_v2.py finalize|verify|refresh-postrun")
