#!/usr/bin/env python3
"""Verify only the sealed R15B model outputs consumed by R15C."""
import hashlib
import json
import pathlib
import sys


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: verify_r15b_outputs.py <R15B archive>")
    root = pathlib.Path(sys.argv[1])
    seal_path = root / "run_01" / "MODEL_OUTPUT_FREEZE.json"
    seal = json.loads(seal_path.read_text(encoding="utf-8"))
    if seal.get("status") != "MODEL_OUTPUTS_FROZEN_BEFORE_REFERENCE_EVALUATION":
        raise RuntimeError("R15B model output seal status is not frozen")
    if seal.get("frame_count") != 25 or seal.get("new_ndt_aligns") != 0 or seal.get("gt_pose_reads_before_freeze") != 0:
        raise RuntimeError("R15B seal violates the fixed model-only contract")
    required = {"CANDIDATE_OBJECTIVES.csv", "SUBSPACE_AND_COUPLING.csv",
                "DYNAMIC_PCL_OBJECTIVE.csv", "HELDOUT_COMPARISON.csv"}
    sealed = {entry["path"]: entry for entry in seal.get("outputs", [])}
    if not required.issubset(sealed):
        raise RuntimeError("required frozen R15B model output missing from seal")
    for name in sorted(required):
        entry = sealed[name]
        path = root / "run_01" / name
        if not path.is_file() or path.stat().st_size != entry["bytes"] or sha(path) != entry["sha256"]:
            raise RuntimeError("R15B frozen output mismatch: " + name)
    print("R15B_FROZEN_MODEL_OUTPUTS_PASS frame_count=25 candidate_outputs=4 gt_values_read=0 align=0")


if __name__ == "__main__":
    main()
