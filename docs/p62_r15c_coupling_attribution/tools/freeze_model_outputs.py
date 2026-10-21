#!/usr/bin/env python3
"""Seal R15C model-side results before any Newer College reference values are read."""
import hashlib
import json
import pathlib
import sys


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    if len(sys.argv) != 5:
        raise SystemExit("usage: freeze_model_outputs.py <archive> <R15C binary> <R15B archive> <R14B archive>")
    archive = pathlib.Path(sys.argv[1])
    binary = pathlib.Path(sys.argv[2])
    r15b = pathlib.Path(sys.argv[3])
    r14b = pathlib.Path(sys.argv[4])
    run = archive / "run_01"
    seal_path = run / "MODEL_OUTPUT_FREEZE.json"
    if seal_path.exists():
        raise SystemExit("refusing to overwrite model-output freeze")
    names = ["GRADIENT_ONLY_RESULTS.csv", "CANDIDATE_METRICS.csv",
             "COUPLED_VS_GRADIENT_ONLY.csv", "RUNTIME_AND_MEMORY.csv",
             "formal_run.log"]
    outputs = []
    for name in names:
        path = run / name
        if not path.is_file():
            raise SystemExit("missing R15C model output: " + name)
        outputs.append({"path": name, "bytes": path.stat().st_size, "sha256": sha(path)})
    if not binary.is_file():
        raise SystemExit("R15C binary missing")
    r15b_seal = r15b / "run_01" / "MODEL_OUTPUT_FREEZE.json"
    r14b_prereg = r14b / "TEST_FRAME_PREREGISTRATION.json"
    for path in (r15b_seal, r14b_prereg):
        if not path.is_file():
            raise SystemExit("required upstream identity missing: " + str(path))
    payload = {
        "task": "P62-R15C-COUPLING-ATTRIBUTION-AND-NOVELTY",
        "status": "R15C_MODEL_OUTPUTS_FROZEN_BEFORE_REFERENCE_EVALUATION",
        "frame_count": 25,
        "candidate_evaluations": 200,
        "new_NDT_aligns": 0,
        "EKF_runs": 0,
        "gt_pose_values_read_before_freeze": 0,
        "R15B_model_output_freeze_sha256": sha(r15b_seal),
        "R14B_test_preregistration_sha256": sha(r14b_prereg),
        "R15C_binary_sha256": sha(binary),
        "outputs": outputs,
        "note": "Model-side candidate and objective outputs frozen before opening Newer College reference pose values. Exploratory spectral-tail probes only; formal weak directions were zero in all 25 frames."
    }
    with seal_path.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print("R15C_MODEL_OUTPUTS_FROZEN frames=25 outputs=%d new_NDT_aligns=0 gt_pose_values_read=0" % len(outputs))


if __name__ == "__main__":
    main()
