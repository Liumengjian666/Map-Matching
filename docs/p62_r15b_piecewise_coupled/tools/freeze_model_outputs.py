#!/usr/bin/env python3
"""One-shot read-only validation and seal for the pre-reference R15B outputs."""

import csv
import hashlib
import json
import pathlib
import sys


def sha256(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def rows(path):
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main():
    out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "docs/p62_r15b_piecewise_coupled/run_01")
    freeze = out / "MODEL_OUTPUT_FREEZE.json"
    if freeze.exists():
        raise SystemExit("refusing to overwrite an existing model-output freeze")
    if (out / "REFERENCE_ERROR_EXPLORATORY.csv").exists():
        raise SystemExit("reference evaluation already exists; cannot claim pre-reference freeze")

    expected = {
        "ASSOCIATION_BOUNDARY_ANALYSIS.csv": 25,
        "LOCAL_MODEL_DERIVATIVES.csv": 25,
        "SUBSPACE_AND_COUPLING.csv": 50,
        "WEAK_ONLY_VS_COUPLED.csv": 50,
        "CANDIDATE_OBJECTIVES.csv": 150,
        "DYNAMIC_PCL_OBJECTIVE.csv": 150,
        "HELDOUT_COMPARISON.csv": 1200,
        "RUNTIME_AND_MEMORY.csv": 50,
        "NOMINAL_ASSOCIATION_IDENTITIES.csv": 17616,
    }
    row_counts = {}
    for name, expected_count in expected.items():
        path = out / name
        if not path.is_file():
            raise SystemExit("missing required output: " + name)
        actual = len(rows(path))
        if actual != expected_count:
            raise SystemExit(f"{name}: expected {expected_count} data rows, got {actual}")
        row_counts[name] = actual

    expected_keys = {("GEODE_R13", str(i)) for i in range(60, 65)}
    expected_keys |= {("NEWER_COLLEGE_R14B", str(i)) for i in range(768, 788)}
    frames = rows(out / "ASSOCIATION_BOUNDARY_ANALYSIS.csv")
    frame_keys = {(r["dataset"], r["frame_key"]) for r in frames}
    if frame_keys != expected_keys:
        raise SystemExit("frame identity set does not match the frozen 25 frames")
    if any(r["fresh_grid_score_match"] != "1" for r in frames):
        raise SystemExit("at least one reused-grid nominal score did not match R15")

    deriv = rows(out / "LOCAL_MODEL_DERIVATIVES.csv")
    if any(r["status"] != "FIXED_ASSOCIATION_BRANCH_DERIVATIVES_PASS" or r["derivatives_valid"] != "1" for r in deriv):
        raise SystemExit("at least one fixed-branch derivative validation failed")

    candidates = rows(out / "CANDIDATE_OBJECTIVES.csv")
    if {r["method"] for r in candidates} != {"NOMINAL_NDT", "WEAK_ONLY_PROBE", "COUPLED_PROBE"}:
        raise SystemExit("unexpected candidate method set")
    for r in candidates:
        if r["method"] != "NOMINAL_NDT" and (r["candidate_kind"] != "POST_NDT_REFINEMENT_CANDIDATE" or r["converged"] != "0" or r["formal_eligible"] != "0"):
            raise SystemExit("post-NDT refinement candidate inherited convergence/eligibility")

    subspace = rows(out / "SUBSPACE_AND_COUPLING.csv")
    if any(r["new_NDT_aligns"] != "0" or r["formal_candidate_accepted"] != "0" for r in subspace):
        raise SystemExit("align or formal candidate acceptance count violated the contract")
    runtime = rows(out / "RUNTIME_AND_MEMORY.csv")
    if any(r["new_NDT_aligns"] != "0" or r["EKF_runs"] != "0" or r["GT_pose_reads"] != "0" for r in runtime):
        raise SystemExit("runtime evidence reports an align, EKF, or GT read")

    run_log = (out / "formal_run.log").read_text(encoding="utf-8").strip()
    terminal = "R15B_PIECEWISE_COMPLETE frames=25 gaussian_grid_initializations=2 align=0 ekf=0 gt_reads=0"
    if run_log != terminal:
        raise SystemExit("formal run terminal receipt mismatch")
    timing = (out / "process_timing.log").read_text(encoding="utf-8")
    if "Exit status: 0" not in timing:
        raise SystemExit("process timing reports a failed run")

    code_freeze_path = pathlib.Path("docs/p62_r15b_piecewise_coupled/CODE_FREEZE.json")
    code_freeze = json.loads(code_freeze_path.read_text(encoding="utf-8"))
    prereg_path = pathlib.Path("docs/p62_r15b_piecewise_coupled/PRE_REGISTRATION.json")
    if sha256(prereg_path) != code_freeze["pre_registration_sha256"]:
        raise SystemExit("pre-registration identity changed")

    files = sorted(p for p in out.iterdir() if p.is_file() and p != freeze)
    manifest = {
        "status": "MODEL_OUTPUTS_FROZEN_BEFORE_REFERENCE_EVALUATION",
        "frame_count": 25,
        "frozen_frame_keys": sorted([list(k) for k in frame_keys]),
        "row_counts": row_counts,
        "outputs": [{"path": p.name, "bytes": p.stat().st_size, "sha256": sha256(p)} for p in files],
        "formal_run_terminal": terminal,
        "new_ndt_aligns": 0,
        "ekf_runs": 0,
        "gt_pose_reads_before_freeze": 0,
        "reference_error_file_absent_at_freeze": True,
        "code_sha": code_freeze["algorithm_code_sha"],
        "binary_sha256": code_freeze["run_binary_sha256"],
        "pre_registration_sha256": code_freeze["pre_registration_sha256"],
        "note": "This seal covers the model outputs only. Newer College reference poses may be read only after this JSON is written; any resulting error analysis is exploratory MAP_ANCHORED_BENCHMARK_REFERENCE under PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION.",
    }
    with freeze.open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({"status": manifest["status"], "frames": 25, "new_ndt_aligns": 0, "ekf_runs": 0, "gt_reads": 0, "freeze": str(freeze), "freeze_sha256": sha256(freeze)}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
