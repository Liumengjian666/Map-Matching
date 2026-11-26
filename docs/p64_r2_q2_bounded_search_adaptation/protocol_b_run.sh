#!/usr/bin/env bash
set -eo pipefail
workspace=$(git rev-parse --show-toplevel)
source_dir="$workspace/docs/p64_r2_q2_bounded_search_adaptation"
data_dir=/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1
out=$1
[[ "$out" = /* && ! -e "$out" ]]
mkdir "$out"
source /opt/ros/noetic/setup.bash
/usr/bin/python3 -B "$source_dir/protocol_b_prepare.py" "$out/frozen" > "$out/prepare.log"
env -u LD_LIBRARY_PATH LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 \
  bash "$source_dir/protocol_b_build.sh" "$out/window" > "$out/build.log" 2>&1
env -u LD_LIBRARY_PATH LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 \
  LD_LIBRARY_PATH=/opt/ros/noetic/lib /usr/bin/time -v timeout 90 "$out/window" "$out/frozen" \
  "$data_dir/gate2_048_diff_map_formal_01/nav_a_diff_prior_voxel0p15.pcd" "$out/trajectory.csv" > "$out/run.log" 2>&1
/usr/bin/python3 -B - "$out" "$source_dir" <<'PY'
import csv
import hashlib
import json
from pathlib import Path
import sys

out, source = map(Path, sys.argv[1:])
inputs = {
    "output": out / "trajectory.csv",
    "anchor": out / "frozen/anchor.tsv",
    "imu": out / "frozen/imu.tsv",
    "scans": out / "frozen/scans.tsv",
    "input_receipt": out / "frozen/BLIND_INIT_RECEIPT.json",
    "exe": out / "window",
    "preregistration": source / "protocol_b_preregistration.md",
    "build_source": source / "protocol_b_window.cpp",
    "prep_source": source / "protocol_b_prepare.py",
}
with inputs["output"].open(newline="") as f:
    assert len(list(csv.DictReader(f))) == 30
seal = {name: {"path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        for name, path in inputs.items()}
with (out / "seal.json").open("x") as f:
    json.dump(seal, f, indent=2)
    f.write("\n")
PY
