#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
map=/home/jian/rosbag/dog/rect_record_20261029_fastlivo2/rerun_20261102_182309/pcd/rerun_20261102_182309_all_downsampled_points.pcd
env -u LD_LIBRARY_PATH LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_LIBRARY_PATH=/opt/ros/noetic/lib \
  /usr/bin/time -f '%e %U %S %M' -o "$3.wall" "$1" "$2" "$map" "$3" DIRECTION_DC -1 25 S3 \
  "$here/../p64_r3_q6_fine_grained_geometry_assisted_ndt/geometry_entry.py"
