#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
run=$(realpath "$1")
binary=$(realpath "$2")
map=/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_048_diff_map_formal_01/nav_a_diff_prior_voxel0p15.pcd
mkdir -p "$run/native"
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
for case in G1 G2 G3 W1 W2 W3; do
  if [[ "$case" == G* ]]; then
    folder="$run/inputs/$case"
    target="$folder/map.pcd"
  else
    folder="/home/jian/p64_r3_q3_ws/run/$case"
    target="$map"
  fi
  /usr/bin/time -f '%e,%M' -o "$run/native/$case.time" "$binary" "$folder" "$target" "$run/native/$case" B -1 20 S3 "$here/unused" >"$run/native/$case.log" 2>&1
done
/usr/bin/time -f '%e,%M' -o "$run/native/W2_A.time" "$binary" /home/jian/p64_r3_q3_ws/run/W2 "$map" "$run/native/W2_A" A -1 20 S3 "$here/unused" >"$run/native/W2_A.log" 2>&1
