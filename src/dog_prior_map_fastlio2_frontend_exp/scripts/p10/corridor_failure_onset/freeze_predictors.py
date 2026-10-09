#!/usr/bin/env python3
"""Freeze the fixed 0--35 s common-predictor window before any GT read."""
import csv
import hashlib
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[5]
ARCHIVE = REPO / "docs/p10_corridor01_failure_onset"
DATA = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01")
INPUT = DATA / "results/p9_corridor01_raw_scanend_v1"
MAP = DATA / "map/derived/corridor01_map_normalized.pcd"
GT = DATA / "gt/corridor01_gt.txt"
EXTERNAL = Path("/home/jian/livox_ws/dog_loc_paper_ws/.p9_experiment_cache/p10_corridor01_real_degeneracy_benchmark")
EVAL_ORIGIN_NS = 1517157224188979000
WINDOW_NS = 35_000_000_000
EXPECTED_MANIFEST_SHA = "591bfe3fd619966f40e4e2af6b9151937732483f0123c70eb34aa1031742991a"
EXPECTED_OLD_FRAMES_SHA = "8043ae86bec1d6bd58993b88db99605b58b4f3332d41512b0710e316981d527e"
EXPECTED_OLD_LEDGER_SHA = "50b0dcaf12f44bd009b5e453727f80b714b87ab72e95377603daadb52c0c9dab"
EXPECTED_GT_SHA = "3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03"
EXPECTED_QUALITY_FREEZE_SHA = "496cf243042e2914839f9788fcbed4f045d72a22eb77cdd6c681bdeb8da732be"


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def rigid_from_fields(row, prefix):
    import numpy as np
    matrix = np.array([[float(row[f"{prefix}_r{r}c{c}"]) for c in range(4)]
                       for r in range(3)], dtype=float)
    if not np.isfinite(matrix).all():
        return False
    rotation = matrix[:, :3]
    return (np.linalg.norm(rotation.T @ rotation - np.eye(3)) <= 1e-4 and
            abs(np.linalg.det(rotation) - 1.0) <= 1e-4)


def selected_rows(rows):
    out = []
    for row in rows:
        if row["method"] != "NOMINAL":
            continue
        tx = int(row["transaction_id"])
        start = int(row["scan_start_ns"])
        end = int(row["scan_end_ns"])
        if tx < 52 or start < EVAL_ORIGIN_NS or end > EVAL_ORIGIN_NS + WINDOW_NS:
            continue
        out.append(row)
    return out


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: freeze_predictors.py P10_FRAMES_CSV")
    source = Path(sys.argv[1]).resolve()
    ledger = source.with_name("source_ledger.csv")
    if sha(source) != EXPECTED_OLD_FRAMES_SHA or sha(ledger) != EXPECTED_OLD_LEDGER_SHA:
        raise RuntimeError("historical P10 source artifact hash mismatch")
    hashes_path = REPO / "docs/p10_corridor01_real_degeneracy_benchmark/artifact_hashes.json"
    expected_artifacts = json.loads(hashes_path.read_text())["archive_files"]
    if expected_artifacts["frames.csv"]["sha256"] != EXPECTED_OLD_FRAMES_SHA:
        raise RuntimeError("Git archive does not attest source frames hash")
    if expected_artifacts["source_ledger.csv"]["sha256"] != EXPECTED_OLD_LEDGER_SHA:
        raise RuntimeError("Git archive does not attest source ledger hash")

    old_freeze_path = REPO / "docs/p10_corridor01_real_degeneracy_benchmark/experiment_freeze.json"
    old_freeze = json.loads(old_freeze_path.read_text())
    if sha(old_freeze_path) != "b495bc361a9afee9002f6c603b91420b1dd28d5c3136f8779b208943ebdffb2c":
        raise RuntimeError("historical P10 experiment freeze hash mismatch")
    input_manifest = INPUT / "input_manifest.json"
    if sha(input_manifest) != EXPECTED_MANIFEST_SHA or sha(input_manifest) != old_freeze["input_manifest_sha256"]:
        raise RuntimeError("P9 input manifest mismatch")
    receipts = []
    for item in old_freeze["input_hashes"]:
        path = Path(item["path"])
        actual = sha(path)
        if actual != item["expected"]:
            raise RuntimeError("frozen input hash mismatch: " + str(path))
        receipts.append({"path": str(path), "sha256": actual, "status": "PASS"})

    quality_freeze = REPO / "docs/p9_r7_cross_dataset_nearoptimal/robust_bootstrap_v2/quality_gate_freeze.json"
    if sha(quality_freeze) != EXPECTED_QUALITY_FREEZE_SHA:
        raise RuntimeError("frozen extrinsic receipt mismatch")
    extrinsic = json.loads(quality_freeze.read_text())["T_imu_lidar"]
    p9map = MAP
    if sha(p9map) != "103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f":
        raise RuntimeError("normalized P9 map hash mismatch")

    with source.open(newline="") as stream:
        all_rows = list(csv.DictReader(stream))
    chosen = selected_rows(all_rows)
    if len(chosen) != 346 or [int(r["transaction_id"]) for r in chosen] != list(range(52, 398)):
        raise RuntimeError("fixed whole-scan selection mismatch")
    if any(not rigid_from_fields(row, "prediction") or
           not rigid_from_fields(row, "nominal") or
           not rigid_from_fields(row, "executed") for row in chosen):
        raise RuntimeError("non-rigid historical nominal predictor row")

    scan_index = {}
    with (INPUT / "raw_timed_scan_index.csv").open(newline="") as stream:
        for row in csv.DictReader(stream):
            scan_index[int(row["transaction_id"])] = row
    for row in chosen:
        tx = int(row["transaction_id"])
        current = scan_index.get(tx)
        if not current or int(current["scan_start_ns"]) != int(row["scan_start_ns"]) or int(current["scan_end_ns"]) != int(row["scan_end_ns"]):
            raise RuntimeError("historical scanend row does not match frozen P9 raw index")

    ARCHIVE.mkdir(parents=True, exist_ok=True)
    predictor_path = ARCHIVE / "common_predictor.csv"
    freeze_path = ARCHIVE / "selection_freeze.json"
    if predictor_path.exists() or freeze_path.exists():
        raise RuntimeError("selection artifacts already exist; refusing overwrite")
    matrices = [f"{prefix}_r{r}c{c}" for prefix in ("prediction", "nominal", "executed")
                for r in range(3) for c in range(4)]
    columns = ["transaction_id", "scan_start_ns", "scan_end_ns", "elapsed_s",
               "source_count", "source_hash", "nominal_effective", "nominal_status",
               "nominal_raw_score"] + matrices
    with predictor_path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        for row in chosen:
            out = {key: row[key] for key in columns if key in row}
            out["elapsed_s"] = format((int(row["scan_end_ns"]) - EVAL_ORIGIN_NS) * 1e-9, ".12f")
            out["nominal_effective"] = row["ndt_effective"]
            out["nominal_status"] = row["ndt_status"]
            out["source_hash"] = row["source_hash"]
            out["nominal_raw_score"] = row["raw_score"]
            writer.writerow(out)
    freeze = {
        "TASK": "P10-CORRIDOR01-FAILURE-ONSET-COUPLED-BENCHMARK",
        "phase": "PRE_GT_COMMON_PREDICTOR_AND_WINDOW_FREEZE",
        "GT_LOADED": False,
        "protocol": "COMMON_PREDICTOR_PAIRED_PER_FRAME_DIAGNOSTIC_ONLY",
        "common_predictor_source": "P10 prior nominal branch prediction_r0c*; same P9 scanend transaction/source/map",
        "source_commit": "cd85a78abdeff49abaece571a9ba6ca9bda362d8",
        "source_code_sha": "477465475803f8cab6b518887ee5fce55cc09c38",
        "source_frames_path": str(source),
        "source_frames_sha256": sha(source),
        "source_ledger_sha256": sha(ledger),
        "source_full_run_start_tx": 52,
        "source_full_run_end_tx": 2777,
        "evaluation_origin_ns": EVAL_ORIGIN_NS,
        "window_duration_ns": WINDOW_NS,
        "selection_rule": "whole P9 scan interval; scan_start >= evaluation origin; scan_end <= origin+35s; no GT/score selection",
        "first_transaction": int(chosen[0]["transaction_id"]),
        "last_transaction": int(chosen[-1]["transaction_id"]),
        "frame_count": len(chosen),
        "first_scan_end_elapsed_s": (int(chosen[0]["scan_end_ns"]) - EVAL_ORIGIN_NS) * 1e-9,
        "last_scan_end_elapsed_s": (int(chosen[-1]["scan_end_ns"]) - EVAL_ORIGIN_NS) * 1e-9,
        "excluded_boundary": "TX51 crosses evaluation origin; first full scan is TX52",
        "P9_input_manifest_sha256": sha(input_manifest),
        "P9_map_sha256": sha(p9map),
        "T_imu_lidar": extrinsic,
        "input_hashes": receipts,
        "predictor_csv_sha256": sha(predictor_path),
        "P2B_historical_gt_loaded": False,
        "P2B_derived_bag_available": False,
        "causal_feedback_authorized_by_this_artifact": False,
    }
    freeze_path.write_text(json.dumps(freeze, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"frames": len(chosen), "first_tx": 52, "last_tx": 397,
                      "predictor_sha256": freeze["predictor_csv_sha256"],
                      "GT_LOADED": False}, indent=2))


if __name__ == "__main__":
    main()
