"""Frozen trajectory admission. Prediction records never count as observations."""
import csv
import json
from prepare_v2 import ARCHIVE, OUTPUT, sha, write_json

def analyze():
    freeze = json.loads((ARCHIVE / "quality_gate_freeze.json").read_text())
    config = json.loads((ARCHIVE / "bootstrap_v2_config.json").read_text())
    if sha(ARCHIVE / "bootstrap_v2_config.json") != freeze["config_sha256"]:
        raise RuntimeError("trajectory gates changed")
    odometry = OUTPUT / "registration/robust_lidar_odometry.csv"
    rows = list(csv.DictReader(odometry.open()))
    if len(rows) != 99 or [int(r["transaction_id"]) for r in rows] != list(range(1, 100)):
        raise RuntimeError("incomplete bootstrap ledger")
    observed = lambda r: r["role"] == "LIDAR_OBSERVATION" and r["true_observation"] == "1" and r["quality_pass"] == "1"
    if any((r["role"] == "LIDAR_OBSERVATION") != (r["true_observation"] == "1") for r in rows):
        raise RuntimeError("observation provenance contradiction")
    train = [r for r in rows if freeze["estimation_start_ns"] <= int(r["stamp_ns"]) < freeze["estimation_end_ns"]]
    validation = [r for r in rows if freeze["estimation_end_ns"] <= int(r["stamp_ns"]) <= freeze["boot_stamp_ns"]]
    accepted = sum(observed(r) for r in rows)
    maximum = max(int(r["consecutive_failures"]) for r in rows if r["consecutive_failures"])
    gates = {
        "at_least_90_of_98": accepted >= 90,
        "at_most_2_consecutive_failed": maximum <= 2,
        "all_5_to_8s_real_observed": bool(train) and all(observed(r) for r in train),
        "at_least_15_validation_observed": sum(observed(r) for r in validation) >= 15,
        "no_runtime_input_guard_failure": all(r["role"] != "INPUT_RUNTIME_GUARD_FAIL" for r in rows)}
    admission = {"status": "PASS" if all(gates.values()) else "FAIL", "gates": gates,
        "total_pair_count": 98, "attempted_pairs": sum(bool(r["solver_converged"]) for r in rows[1:]),
        "accepted_pair_count": accepted, "max_consecutive_failures": maximum,
        "estimation_real_observations": sum(observed(r) for r in train), "estimation_targets": len(train),
        "validation_real_observations": sum(observed(r) for r in validation), "validation_targets": len(validation),
        "odometry_sha256": sha(odometry), "config_sha256": sha(ARCHIVE / "bootstrap_v2_config.json"),
        "prediction_only_frames": sum(r["role"] == "CAUSAL_PREDICTION_ONLY" for r in rows),
        "not_run_frames": sum(r["role"] == "NOT_RUN" for r in rows), "NDT_CALLS": 0, "GT_LOADED": False}
    write_json(OUTPUT / "local_admission.json", admission)
    write_json(ARCHIVE / "local_admission.json", admission)
    print(json.dumps(admission))

if __name__ == "__main__":
    analyze()
