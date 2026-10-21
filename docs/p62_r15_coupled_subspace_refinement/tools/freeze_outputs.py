#!/usr/bin/env python3
"""One-shot seal of R15 candidate/model outputs before reading R14 references."""
import csv
import hashlib
import json
import pathlib
import sys
from datetime import datetime, timezone

ROOT = pathlib.Path(__file__).resolve().parents[3]
ARCHIVE = ROOT / "docs/p62_r15_coupled_subspace_refinement"
RUN = ARCHIVE / "run_01"
SEAL = ARCHIVE / "CANDIDATE_OUTPUT_FREEZE.json"
REQUIRED = (
    "SUBSPACE_DECOMPOSITION.csv", "COUPLING_BLOCKS.csv",
    "WEAK_ONLY_RESULTS.csv", "COUPLED_RESULTS.csv",
    "TRAINING_OBJECTIVE_COMPARISON.csv", "HELDOUT_COMPARISON.csv",
    "RUNTIME_AND_MEMORY.csv", "formal_run.log",
)


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def rows(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise RuntimeError(f"missing CSV header: {path}")
        result = list(reader)
    return reader.fieldnames, result


def matrix(pipe):
    values = [float(x) for x in pipe.split("|")]
    if len(values) != 16:
        raise RuntimeError("candidate pose does not contain 16 matrix entries")
    return values


def main():
    if SEAL.exists():
        raise RuntimeError("candidate output freeze already exists; refusing overwrite")
    for name in REQUIRED:
        p = RUN / name
        if not p.is_file():
            raise RuntimeError(f"required frozen model output missing: {p}")
    if (RUN / "REFERENCE_ERROR_COMPARISON.csv").exists():
        raise RuntimeError("reference error output must be generated only after this freeze")

    log = (RUN / "formal_run.log").read_text()
    if "R15_LOCAL_MODEL_COMPLETE frames=25 align=0 ekf=0 gt_reads=0" not in log:
        raise RuntimeError("terminal local-model receipt is absent or reports forbidden work")

    sub_header, sub = rows(RUN / "SUBSPACE_DECOMPOSITION.csv")
    expected_sub = {"GEODE_R13": {"60", "61", "62", "63", "64"},
                    "NEWER_COLLEGE_R14B": {str(x) for x in range(768, 788)}}
    actual = {k: set() for k in expected_sub}
    for r in sub:
        if r["dataset"] not in actual or r["frame_key"] in actual[r["dataset"]]:
            raise RuntimeError("unexpected or duplicate source frame in model table")
        actual[r["dataset"]].add(r["frame_key"])
    if actual != expected_sub:
        raise RuntimeError(f"frozen 25-frame input set mismatch: {actual}")

    expected_counts = {
        "WEAK_ONLY_RESULTS.csv": 50,
        "COUPLED_RESULTS.csv": 50,
        "TRAINING_OBJECTIVE_COMPARISON.csv": 50,
        "COUPLING_BLOCKS.csv": 50,
        "HELDOUT_COMPARISON.csv": 1200,
        "RUNTIME_AND_MEMORY.csv": 50,
    }
    schemas = {}
    for name, count in expected_counts.items():
        header, data = rows(RUN / name)
        schemas[name] = {"columns": len(header), "rows": len(data)}
        if len(data) != count:
            raise RuntimeError(f"{name}: expected {count} rows, got {len(data)}")

    # Post-NDT refinements are diagnostic candidates only; they never inherit
    # a native convergence flag or formal observation eligibility.
    for name in ("WEAK_ONLY_RESULTS.csv", "COUPLED_RESULTS.csv"):
        _, data = rows(RUN / name)
        for r in data:
            if r["candidate_kind"] != "POST_NDT_REFINEMENT_CANDIDATE":
                raise RuntimeError(f"unexpected candidate kind in {name}")
            if int(r["converged"]) != 0 or int(r["formal_eligible"]) != 0:
                raise RuntimeError(f"post-NDT refinement incorrectly marked eligible in {name}")
            if r["pose_pipe"] != "NA":
                matrix(r["pose_pipe"])

    _, runtime = rows(RUN / "RUNTIME_AND_MEMORY.csv")
    if any(int(r["new_NDT_aligns"]) != 0 or int(r["EKF_runs"]) != 0 or int(r["GT_reads"]) != 0 for r in runtime):
        raise RuntimeError("local-model accounting reports forbidden operation")

    files = {}
    for name in REQUIRED:
        p = RUN / name
        files[str(p.relative_to(ROOT))] = {"sha256": sha(p), "bytes": p.stat().st_size}
    payload = {
        "task": "P62-R15-COUPLED-SUBSPACE-REFINEMENT-R1",
        "status": "CANDIDATE_AND_DECISION_OUTPUTS_FROZEN_BEFORE_REFERENCE_POSE_READ",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "reference_pose_values_read": False,
        "raw_reference_file_parsed": False,
        "expected_frame_sets": {k: sorted(v) for k, v in expected_sub.items()},
        "csv_schemas": schemas,
        "outputs": files,
        "candidate_semantics": "post-NDT local refinements; converged=false and formal_eligible=false",
        "operation_counts": {"new_ndt_aligns": 0, "ekf_runs": 0, "gt_pose_value_reads": 0},
    }
    with open(SEAL, "x") as f:
        json.dump(payload, f, indent=2, sort_keys=True)
        f.write("\n")
    print(json.dumps({"status": payload["status"], "csv_schemas": schemas}, indent=2))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"CANDIDATE_OUTPUT_FREEZE_FAILED: {exc}", file=sys.stderr)
        raise
