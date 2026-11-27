#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
binary=$1
map="${P64_MAP:-/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_048_diff_map_formal_01/nav_a_diff_prior_voxel0p15.pcd}"
env -u LD_LIBRARY_PATH LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_LIBRARY_PATH=/opt/ros/noetic/lib \
  /usr/bin/time -f '%e %U %S %M' -o "$3.wall" "$binary" "$2" "$map" "$3" "$4" "$5" "$6" "$7" "$here/geometry_entry.py"
