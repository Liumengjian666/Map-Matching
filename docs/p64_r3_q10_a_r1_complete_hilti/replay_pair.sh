#!/usr/bin/env bash
set -euo pipefail
d=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
build=$(realpath "$1")
input=$(realpath "$2")
out=$(realpath -m "$3")
port="$4"
scope="$5"
map=$(realpath "$input/map.pcd")
mkdir "$out"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export LD_LIBRARY_PATH=/opt/ros/noetic/lib
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
P64_R1_INPUT_RECEIPT="$out/b_points.csv" "$build/research" "$input" "$map" "$out/b" B -1 20 S3   "$d/../p64_r3_q6_fine_grained_geometry_assisted_ndt/geometry_entry.py" >"$out/b.log" 2>&1
P64_R1_INPUT_RECEIPT="$out/a_points.csv" bash "$build/replay_hdl.sh" "$build" "$input" "$out/a" "$port" >"$out/a.log" 2>&1
/usr/bin/python3 "$d/compare_baselines.py" "$out" "$out/comparison.csv" --scope "$scope"
