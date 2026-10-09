#!/usr/bin/env python3
"""Validate and hash the completed no-GT paired run before any GT access."""
import csv
import hashlib
import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[5]
ARCHIVE = REPO / "docs/p10_corridor01_failure_onset"


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def rows(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def main():
    output = ARCHIVE / "evidence_freeze.json"
    if output.exists():
        raise RuntimeError("evidence freeze already exists; refusing overwrite")
    selection_path = ARCHIVE / "selection_freeze.json"
    run_freeze_path = ARCHIVE / "experiment_freeze.json"
    execution_path = ARCHIVE / "paired_execution.json"
    process_path = ARCHIVE / "process_exit.json"
    candidate_path = ARCHIVE / "paired_candidates.csv"
    parity_path = ARCHIVE / "source_parity.csv"
    selection = json.loads(selection_path.read_text())
    run_freeze = json.loads(run_freeze_path.read_text())
    execution = json.loads(execution_path.read_text())
    process = json.loads(process_path.read_text())
    if selection.get("GT_LOADED") is not False or run_freeze.get("GT_LOADED") is not False:
        raise RuntimeError("a pre-evaluation artifact claims GT access")
    if execution.get("GT_LOADED") is not False or process.get("GT_LOADED") is not False:
        raise RuntimeError("paired run did not attest GT isolation")
    if process.get("exit_code") != 0:
        raise RuntimeError("paired process did not exit successfully")
    if execution.get("frames") != 346 or execution.get("full_align_calls") != 346:
        raise RuntimeError("paired execution frame/alignment count mismatch")
    if execution.get("source_parity") != "PASS" or execution.get("nominal_replay_parity") != "PASS":
        raise RuntimeError("paired execution parity receipt failed")
    if sha(candidate_path) == "" or sha(parity_path) == "":
        raise RuntimeError("empty paired evidence hash")
    candidate_rows = rows(candidate_path)
    parity_rows = rows(parity_path)
    expected_ids = list(range(52, 398))
    if len(candidate_rows) != 346 or len(parity_rows) != 346:
        raise RuntimeError("paired CSV row count mismatch")
    if [int(row["transaction_id"]) for row in candidate_rows] != expected_ids or \
       [int(row["transaction_id"]) for row in parity_rows] != expected_ids:
        raise RuntimeError("paired CSV transaction sequence mismatch")
    if any(row.get("pass") != "PASS" for row in parity_rows):
        raise RuntimeError("source or historical nominal parity failure")
    for row in candidate_rows:
        if row["source_parity"] != "1":
            raise RuntimeError("candidate evidence source parity flag failed")
        if int(row["weak_jets"]) > 2 or int(row["coupled_jets"]) > 2 or \
           int(row["weak_values"]) > 3 or int(row["coupled_values"]) > 3:
            raise RuntimeError("R6 per-frame refinement budget exceeded")
        if row["triggered"] == "0" and any(int(row[key]) for key in
                ("weak_jets", "weak_values", "coupled_jets", "coupled_values")):
            raise RuntimeError("untriggered row performed extra refinement work")
    command = run_freeze["command"]
    binary = Path(command[0])
    if sha(binary) != run_freeze["binary_sha256"]:
        raise RuntimeError("paired binary changed after execution")
    artifact_paths = {
        "selection_freeze": selection_path,
        "experiment_freeze": run_freeze_path,
        "process_exit": process_path,
        "paired_execution": execution_path,
        "common_predictor": ARCHIVE / "common_predictor.csv",
        "paired_candidates": candidate_path,
        "source_parity": parity_path,
        "run_log": ARCHIVE / "run.log",
        "binary": binary,
    }
    artifact_hashes = {name: {"path": str(path), "sha256": sha(path)}
                       for name, path in artifact_paths.items()}
    freeze = {
        "TASK": "P10-CORRIDOR01-FAILURE-ONSET-COUPLED-BENCHMARK",
        "phase": "NO_GT_PAIRED_EVIDENCE_FREEZE",
        "GT_LOADED_BY_EXECUTION": False,
        "oracle_labels_loaded": False,
        "frame_count": 346,
        "transaction_range": [52, 397],
        "source_and_nominal_replay_parity": "346/346 PASS",
        "full_align_calls": 346,
        "causal_feedback": False,
        "paired_per_frame_only": True,
        "artifact_hashes": artifact_hashes,
    }
    output.write_text(json.dumps(freeze, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"evidence_freeze": str(output), "sha256": sha(output),
                      "frames": 346, "GT_LOADED": False}, indent=2))


if __name__ == "__main__":
    main()
