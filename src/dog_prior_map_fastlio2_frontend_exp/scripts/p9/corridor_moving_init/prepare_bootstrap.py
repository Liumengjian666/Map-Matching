"""One-shot causal bootstrap admission/freeze. Does not estimate or align."""
import csv
import hashlib
import json
import pathlib
import re
import sys
import time
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[5]
ARCHIVE = ROOT / "docs/p9_r7_cross_dataset_nearoptimal/moving_initialization"
INPUT = pathlib.Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p9_corridor01_raw_scanend_v1")
OUTPUT = INPUT.parent / "p9_corridor01_moving_init_v1"
CALIB = INPUT.parents[1] / "calibration/corridor01_extrinsics.yaml"
MAP = INPUT.parents[1] / "map/derived/corridor01_map_normalized.pcd"
ANCHOR = pathlib.Path("/home/jian/livox_ws/superloc_adapter_ws/config/corridor01_init.yaml")

def sha(path):
    h = hashlib.sha256()
    with pathlib.Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()

def write_json(path, value):
    with pathlib.Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")

def write_csv(path, rows, columns=None):
    with pathlib.Path(path).open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns or list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

def verify_input(manifest, expected_manifest_sha=None):
    required={"imu.csv", "filter_scans.csv", "raw_timed_scan_index.csv", "raw_timed_points.bin"}
    if not required.issubset(manifest["input_files"]):
        raise RuntimeError("manifest omitted a consumed input")
    if expected_manifest_sha and sha(INPUT/"input_manifest.json")!=expected_manifest_sha:
        raise RuntimeError("frozen input manifest changed")
    for name, receipt in manifest["input_files"].items():
        path=INPUT/name
        if str(path)!=receipt["path"] or path.stat().st_size!=receipt["bytes"] or sha(path)!=receipt["sha256"]:
            raise RuntimeError("raw input hash mismatch: "+name)

def prepare(binary):
    started = time.monotonic()
    manifest = json.loads((INPUT / "input_manifest.json").read_text())
    if not manifest["extraction_complete"] or manifest["protocol"] != "P9_CORRIDOR01_RAW_SCANEND_V1":
        raise RuntimeError("unaccepted raw input protocol")
    verify_input(manifest)
    hashes = {}
    for name, receipt in manifest["input_files"].items():
        hashes[name] = receipt
    for path, expected in [(CALIB, "59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d"),
                           (MAP, "103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f"),
                           (ANCHOR, "d3e6f560895cb4f6a9efb7058bcff8a13313783e799d82c828e51adcd4bafd1e")]:
        if sha(path) != expected:
            raise RuntimeError("map/anchor/calibration mismatch: " + str(path))
        hashes[str(path)] = {"sha256": expected}
    scans = list(csv.DictReader((INPUT / "raw_timed_scan_index.csv").open()))
    imu = list(csv.DictReader((INPUT / "imu.csv").open()))
    first = int(scans[0]["scan_start_ns"])
    deadline = first + 10_000_000_000
    boot = max(int(s["scan_end_ns"]) for s in scans if int(s["scan_end_ns"]) <= deadline)
    data = re.search(r"laser_to_imu:.*?data:\s*\[(.*?)\]", CALIB.read_text(), re.S).group(1)
    extrinsic = np.array([float(v) for v in data.split(",")]).reshape(4, 4)
    U, _, V = np.linalg.svd(extrinsic[:3, :3])
    projected = U @ np.diag([1, 1, np.linalg.det(U @ V)]) @ V
    rotation_projection_change = float(np.linalg.norm(projected - extrinsic[:3, :3]))
    extrinsic[:3, :3] = projected
    OUTPUT.mkdir()  # Refuse repeat/overwrite of the experiment cache.
    config = ARCHIVE / "bootstrap_config.json"
    freeze = {"protocol": "P9_CORRIDOR01_CAUSAL_MOVING_INIT_V1", "RAW_HASH": "PASS",
              "input_manifest_sha256": sha(INPUT / "input_manifest.json"), "input_hashes": hashes,
              "config_sha256": sha(config), "binary": str(pathlib.Path(binary).resolve()),
              "binary_sha256": sha(binary), "first_sensor_stamp_ns": first, "deadline_ns": deadline,
              "boot_stamp_ns": boot, "bootstrap_scan_count": sum(int(s["scan_end_ns"]) <= boot for s in scans),
              "estimation_start_ns": first + 5_000_000_000, "estimation_end_ns": first + 8_000_000_000,
              "T_imu_lidar": extrinsic.tolist(), "extrinsic_SO3_projection_frobenius": rotation_projection_change,
              "map_anchor": "historical sensor-only first-frame anchor; normalized map, first local LiDAR frame identity",
              "anchor_reference_tx": 1, "anchor_stamp_ns": int(scans[0]["scan_end_ns"]),
              "anchor_epoch_difference_s": abs(int(scans[0]["scan_end_ns"])*1e-9 - 1517157219.18898),
              "anchor_epoch_is_approximate": True, "GT_LOADED": False,
              "ORACLE263_CALLS": 0, "B12_CALLS": 0, "VISUAL_EXTRACTION": 0,
              "formal_state_accepted": False, "input_hash_wall_s": time.monotonic() - started,
              "output_directory": str(OUTPUT)}
    write_json(ARCHIVE / "bootstrap_freeze.json", freeze)
    write_json(OUTPUT / "bootstrap_freeze.json", freeze)
    write_csv(ARCHIVE / "formal_frame_ledger.csv", [
        {"transaction_id": s["transaction_id"], "scan_start_ns": s["scan_start_ns"],
         "scan_end_ns": s["scan_end_ns"], "raw_points": s["cloud_point_count"],
         "role": "BOOTSTRAP" if int(s["scan_end_ns"]) <= boot else "FORMAL_PENDING_INITIALIZATION_GATE",
         "boundary_reason": "missing_leading_IMU_bootstrap_only" if int(s["scan_start_ns"]) < int(imu[0]["stamp_ns"]) else "",
         "formal_NDT_run": "NO"} for s in scans])
    print(json.dumps({"RAW_HASH": "PASS", "boot": boot, "bootstrap_scans": freeze["bootstrap_scan_count"],
                      "freeze_sha": sha(ARCHIVE / "bootstrap_freeze.json"), "output": str(OUTPUT)}))

if __name__ == "__main__":
    prepare(sys.argv[1])
