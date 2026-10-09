"""Bind one V2 run to its persistent output, input bytes and pre-run gates."""
import csv
import json
import os
import pathlib
import re
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "corridor_moving_init"))
from prepare_bootstrap import ROOT, INPUT, MAP, CALIB, ANCHOR, sha, write_json, write_csv, verify_input
ARCHIVE = ROOT / "docs/p9_r7_cross_dataset_nearoptimal/robust_bootstrap_v2"
OUTPUT = pathlib.Path("/home/jian/livox_ws/dog_loc_paper_ws/.p9_experiment_cache/p9_corridor01_bootstrap_v2")
OLD = ROOT / "docs/p9_r7_cross_dataset_nearoptimal/moving_initialization"

def prepare(binary):
    start = time.monotonic()
    config_path = ARCHIVE / "bootstrap_v2_config.json"
    config = json.loads(config_path.read_text())
    environment = json.loads((ARCHIVE / "environment_receipt.json").read_text())
    if environment["environment"] != "PASS" or pathlib.Path(config["output_directory"]) != OUTPUT:
        raise RuntimeError("unverified or changed persistent output")
    if not OUTPUT.is_dir() or list(OUTPUT.iterdir()):
        raise RuntimeError("one-shot output must exist and be empty before freeze")
    if os.statvfs(OUTPUT).f_bavail * os.statvfs(OUTPUT).f_frsize < 10 * (1 << 30):
        raise RuntimeError("persistent free capacity below predeclared 10GiB reserve")
    previous = json.loads((OLD / "bootstrap_freeze.json").read_text())
    old_config = json.loads((OLD / "bootstrap_config.json").read_text())
    for key in ("preintegration", "covariance", "frozen_formal_contract"):
        if config[key] != old_config[key]:
            raise RuntimeError("inherited state/formal gate changed: " + key)
    manifest = json.loads((INPUT / "input_manifest.json").read_text())
    verify_input(manifest, previous["input_manifest_sha256"])
    if not manifest["extraction_complete"] or manifest["protocol"] != config["input_protocol"]:
        raise RuntimeError("wrong raw protocol")
    for path in (MAP, CALIB, ANCHOR):
        if sha(path) != previous["input_hashes"][str(path)]["sha256"]:
            raise RuntimeError("map/calibration/anchor changed")
    scans = list(csv.DictReader((INPUT / "raw_timed_scan_index.csv").open()))
    first = int(scans[0]["scan_start_ns"])
    end = max(int(r["scan_end_ns"]) for r in scans if int(r["scan_end_ns"]) <= first + 10_000_000_000)
    count = sum(int(r["scan_end_ns"]) <= end for r in scans)
    if (first, end, count) != (config["bootstrap_start_ns"], config["bootstrap_end_ns"], config["bootstrap_scan_count"]):
        raise RuntimeError("changed first10s scan window")
    text = ANCHOR.read_text()
    translation = [float(v) for v in re.search(r"translation:\s*\[(.*?)\]", text).group(1).split(",")]
    quaternion = [float(v) for v in re.search(r"quaternion_xyzw:\s*\[(.*?)\]", text).group(1).split(",")]
    anchor = np.eye(4)
    anchor[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    anchor[:3, 3] = translation
    runner = dict(config)
    runner["T_imu_lidar"] = previous["T_imu_lidar"]
    runner["initial_map_T_lidar"] = anchor.tolist()
    runner_path = OUTPUT / "runner_config.json"
    freeze = {
        "protocol": config["protocol"], "config_sha256": sha(config_path),
        "environment_sha256": sha(ARCHIVE / "environment_receipt.json"),
        "input_manifest_sha256": sha(INPUT / "input_manifest.json"),
        "input_hashes": previous["input_hashes"], "RAW_SHA": "PASS",
        "first_sensor_stamp_ns": first, "boot_stamp_ns": end,
        "estimation_start_ns": first + 5_000_000_000, "estimation_end_ns": first + 8_000_000_000,
        "T_imu_lidar": previous["T_imu_lidar"], "initial_map_T_lidar": anchor.tolist(),
        "output_directory": str(OUTPUT), "binary": str(pathlib.Path(binary).resolve()),
        "binary_sha256": sha(binary), "source_sha256": {p.name: sha(p) for p in HERE.iterdir() if p.is_file()},
        "reused_motion_source_sha256": {str(p): sha(p) for p in (
            HERE.parent / "corridor_moving_init/diagnose_motion.py",
            HERE.parent / "corridor_moving_init/preintegration.py",
            HERE.parent / "corridor_moving_init/prepare_bootstrap.py")},
        "compiled_project_source_sha256": {str(p): sha(p) for p in (
            list((ROOT / "src/dog_prior_map_fastlio2_frontend_exp/include/dog_prior_map_fastlio2_frontend_exp").glob("*.hpp"))
            + [ROOT / "src/dog_prior_map_fastlio2_frontend_exp/src/p7_replay_io.cpp"])},
        "raw_hash_audit_wall_s": time.monotonic() - start,
        "GT_LOADED": False, "ORACLE263_CALLS": 0, "B12_CALLS": 0, "VISUAL_EXTRACTION": 0,
        "historical_v1_equivalence": "NOT_CLAIMED", "prior_R7_R4_conclusion": "UNCHANGED"
    }
    write_json(runner_path, runner)
    freeze["runner_config_sha256"] = sha(runner_path)
    write_json(OUTPUT / "quality_gate_freeze.json", freeze)
    write_json(ARCHIVE / "quality_gate_freeze.json", freeze)
    write_csv(ARCHIVE / "formal_frame_ledger.csv", [{
        "transaction_id": r["transaction_id"], "scan_start_ns": r["scan_start_ns"], "scan_end_ns": r["scan_end_ns"],
        "raw_points": r["cloud_point_count"], "role": "BOOTSTRAP" if int(r["scan_end_ns"]) <= end else "FORMAL_NOT_RUN",
        "formal_NDT_run": "NO"} for r in scans])
    print(json.dumps({"RAW_SHA": "PASS", "bootstrap_scans": count, "quality_gate_freeze_sha256": sha(ARCHIVE / "quality_gate_freeze.json"), "persistent_output": str(OUTPUT)}))

if __name__ == "__main__":
    prepare(sys.argv[1])
