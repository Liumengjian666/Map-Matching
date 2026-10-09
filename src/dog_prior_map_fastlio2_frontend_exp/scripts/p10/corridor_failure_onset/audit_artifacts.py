#!/usr/bin/env python3
"""Audit the paired-run archive and write its portable SHA-256 manifest."""
import csv
import hashlib
import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[5]
ARCHIVE = REPO / "docs/p10_corridor01_failure_onset"
SCRIPT_DIR = REPO / "src/dog_prior_map_fastlio2_frontend_exp/scripts/p10/corridor_failure_onset"
RESULT = ARCHIVE / "results.json"
MANIFEST = ARCHIVE / "artifact_hashes.json"


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def reject_constant(value):
    raise ValueError("nonstandard JSON numeric constant: " + value)


def load_json(path):
    return json.loads(Path(path).read_text(), parse_constant=reject_constant)


def load_csv(path):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or len(reader.fieldnames) != len(set(reader.fieldnames)):
            raise RuntimeError("invalid/duplicate CSV header: " + str(path))
        rows = list(reader)
    if any(None in row or any(value is None for value in row.values()) for row in rows):
        raise RuntimeError("CSV row width mismatch: " + str(path))
    return rows


def main():
    selection = load_json(ARCHIVE / "selection_freeze.json")
    execution = load_json(ARCHIVE / "paired_execution.json")
    process = load_json(ARCHIVE / "process_exit.json")
    evidence = load_json(ARCHIVE / "evidence_freeze.json")
    evaluation = load_json(ARCHIVE / "posthoc_evaluation.json")
    result = load_json(RESULT)
    if process.get("exit_code") != 0 or execution.get("frames") != 346:
        raise RuntimeError("paired execution receipt failed")
    if selection.get("GT_LOADED") is not False or execution.get("GT_LOADED") is not False or \
       evidence.get("GT_LOADED_BY_EXECUTION") is not False or evaluation.get("GT_LOADED") is not True:
        raise RuntimeError("GT phase separation receipt mismatch")
    if evidence.get("source_and_nominal_replay_parity") != "346/346 PASS":
        raise RuntimeError("no-GT evidence freeze parity failed")
    candidates = load_csv(ARCHIVE / "paired_candidates.csv")
    source = load_csv(ARCHIVE / "source_parity.csv")
    drift = load_csv(ARCHIVE / "paired_prefix_aligned_relative_drift.csv")
    expected_ids = list(range(52, 398))
    if [int(row["transaction_id"]) for row in candidates] != expected_ids:
        raise RuntimeError("paired candidate transaction sequence mismatch")
    if len(source) != 346 or any(row["pass"] != "PASS" for row in source):
        raise RuntimeError("source/nominal parity CSV failed")
    if len(drift) != 3 * 346:
        raise RuntimeError("post-hoc relative drift CSV row count mismatch")
    for item in evidence["artifact_hashes"].values():
        if sha(item["path"]) != item["sha256"]:
            raise RuntimeError("no-GT frozen artifact changed: " + item["path"])
    for item in selection["input_hashes"]:
        if sha(item["path"]) != item["sha256"]:
            raise RuntimeError("frozen external input changed: " + item["path"])

    json_files = sorted(p for p in ARCHIVE.rglob("*.json") if p != MANIFEST)
    for path in json_files:
        load_json(path)
    csv_files = sorted(ARCHIVE.rglob("*.csv"))
    for path in csv_files:
        load_csv(path)

    run_freeze = load_json(ARCHIVE / "experiment_freeze.json")
    binary = Path(run_freeze["command"][0])
    code_files = [Path(path) for path in run_freeze["code_sha256"]]
    code_files.extend(p for p in SCRIPT_DIR.glob("*.py") if p not in code_files)
    code_files.extend(p for p in SCRIPT_DIR.glob("*.cpp") if p not in code_files)
    cmake = REPO / "src/dog_prior_map_fastlio2_frontend_exp/scripts/p10/corridor_benchmark/CMakeLists.txt"
    code_files.append(cmake)
    code_files = sorted(set(p.resolve() for p in code_files))
    code_entries = [{"path": str(path), "sha256": sha(path), "bytes": path.stat().st_size}
                    for path in code_files]

    archive_paths = sorted(p for p in ARCHIVE.rglob("*") if p.is_file() and p != MANIFEST)
    archive_entries = [{"path": str(path.relative_to(REPO)), "sha256": sha(path), "bytes": path.stat().st_size}
                       for path in archive_paths]
    gt_path = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/gt/corridor01_gt.txt")
    manifest = {
        "TASK": "P10-CORRIDOR01-FAILURE-ONSET-COUPLED-BENCHMARK",
        "audit": "PASS",
        "GT_access_order": "no-GT paired evidence freeze verified before post-hoc GT hash/evaluation",
        "csv_files_audited": len(csv_files),
        "json_files_audited": len(json_files),
        "paired_rows": len(candidates),
        "source_parity_rows": len(source),
        "posthoc_relative_drift_rows": len(drift),
        "external_inputs": [{"path": item["path"], "sha256": item["sha256"]}
                            for item in selection["input_hashes"]],
        "posthoc_gt": {"path": str(gt_path), "sha256": sha(gt_path)},
        "binary": {"path": str(binary), "sha256": sha(binary), "bytes": binary.stat().st_size},
        "code_files": code_entries,
        "archive_files": archive_entries,
        "self_sha256": "excluded (self-referential)",
    }
    MANIFEST.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"audit": "PASS", "csv": len(csv_files), "json": len(json_files),
                      "paired": len(candidates), "source_parity": len(source),
                      "relative_drift": len(drift), "manifest_sha256": sha(MANIFEST)}, indent=2))


if __name__ == "__main__":
    main()
