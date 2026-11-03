#!/usr/bin/env python3
"""Score the sealed B prior/terminal transforms on heldout Source points, without GT or ROS."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

A = Path(__file__).resolve().parents[1]
run_dir = A / "data/full_split_02"
output_dir = run_dir / "heldout_shadow"
identity = json.loads((A / "build/RUN_IDENTITY.json").read_text())
build = json.loads((A / "build/SCORE_BUILD.json").read_text())

def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()

receipt = json.loads((run_dir / "RUN_RECEIPT.json").read_text())
assert receipt["reason"] == "COMPLETED_2375" and receipt["completed"] == 2375
assert receipt["test_GT_access"] is False
command_record = json.loads((run_dir / "COMMAND.json").read_text())
identity_copy = run_dir / "RUN_IDENTITY.json"
assert sha(identity_copy) == command_record["run_identity_sha256"]
assert sha(A / "build/RUN_IDENTITY.json") == command_record["run_identity_sha256"]
identity = json.loads(identity_copy.read_text())
assert sha(A / "data/INPUT_FREEZE_LOCAL.json") == identity["input_freeze_sha256"]
assert sha(A / "data/RAW_IMU_VERIFIED.csv") == identity["raw_imu_sha256"]
assert sha(A / "data/preflight_output/PARTITION_SUPPORT.csv") == identity["partition_support_sha256"]
assert sha(A / "data/preflight_output/SOURCES.csv") == identity["source_files"][str(A / "data/preflight_output/SOURCES.csv")]
assert sha(identity["map_path"]) == identity["map_sha256"]
for path, expected in identity["source_files"].items():
    assert sha(path) == expected, f"frozen code/input SHA mismatch: {path}"
seal = json.loads((run_dir / "ONLINE_SEAL.json").read_text())
for name in ("ONLINE_RESULTS.csv", "NATIVE_NDT.csv", "SPATIAL_SPLIT.csv", "RUN_RECEIPT.json", "RUN_IDENTITY.json"):
    assert sha(run_dir / name) == seal["files"][name]

binary = Path(build["binary"])
assert sha(binary) == build["binary_sha256"]
assert not output_dir.exists()
output_dir.mkdir()
command = [
    "/usr/bin/time", "-v", "env",
    "LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/opt/ros/noetic/lib:/media/jian/HIKVISION/livox_ws/p63_r3_hdl_data/catkin_ws/devel/lib",
    str(binary), str(run_dir), str(A / "data/INPUT_FREEZE_LOCAL.json"),
    str(A / "data/preflight_output/SOURCES.csv"), identity["map_path"],
    str(A / "data/preflight_output/PARTITION_SUPPORT.csv"), str(output_dir),
]
log_path = run_dir / "HELDOUT_SCORE.log"
with log_path.open("x") as log:
    result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
if result.returncode:
    raise SystemExit(f"shadow scorer exited {result.returncode}; see {log_path}")

scores = output_dir / "SCORES.csv"
blocks = output_dir / "BLOCK_SCORES.csv"
assert scores.is_file() and blocks.is_file()
score_seal = {
    "dataset": identity["dataset"],
    "input_run_code_sha": json.loads((run_dir / "COMMAND.json").read_text())["CODE_SHA"],
    "run_identity_sha256": sha(identity_copy),
    "scorer_sha256": sha(A / "src/score_heldout.cpp"),
    "split_header_sha256": sha(A / "src/spatial_split.hpp"),
    "score_binary_sha256": sha(binary),
    "map_sha256": identity["map_sha256"],
    "input_freeze_sha256": identity["input_freeze_sha256"],
    "partition_support_sha256": identity["partition_support_sha256"],
    "sealed_B_inputs": {
        name: seal["files"][name]
        for name in ("ONLINE_RESULTS.csv", "NATIVE_NDT.csv", "SPATIAL_SPLIT.csv", "RUN_RECEIPT.json")
    },
    "score_outputs": {"SCORES.csv": sha(scores), "BLOCK_SCORES.csv": sha(blocks)},
    "score_wall_log_sha256": sha(log_path),
    "align_calls": 0,
    "ros_connection": False,
    "reference_or_GT_access": False,
    "validation_policy": "offline shadow only; cannot affect NDT, UKF, pose, or admission",
    "created_unix_time": time.time(),
}
seal_path = output_dir / "SHADOW_SCORE_SEAL.json"
with seal_path.open("x") as stream:
    json.dump(score_seal, stream, indent=2)
    stream.write("\n")
print(json.dumps({"status": "SHADOW_SCORE_SEALED", "output_dir": str(output_dir),
                  "scores_sha256": score_seal["score_outputs"]["SCORES.csv"],
                  "blocks_sha256": score_seal["score_outputs"]["BLOCK_SCORES.csv"],
                  "GT_access": False, "align_calls": 0}, indent=2))
