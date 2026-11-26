#!/usr/bin/env bash
set -euo pipefail
stage=/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_051_runtime
src=$(dirname "$(realpath "$0")")
out=$1
if [[ -e "$out" ]]; then
  echo "refusing to overwrite $out" >&2
  exit 2
fi
g++ -std=c++17 -O3 -fopenmp -I"$stage/src/hdl_localization/include" \
  -I"$stage/src/ndt_omp/include" -I/usr/include/pcl-1.10 -I/usr/include/eigen3 \
  -I/opt/ros/noetic/include "$src/protocol_b_window.cpp" \
  -L"$stage/devel/lib" -Wl,-rpath,"$stage/devel/lib" "$stage/pose_estimator.o" -lndt_omp \
  -L/opt/ros/noetic/lib -Wl,-rpath,/opt/ros/noetic/lib -lrostime -lroscpp \
  -lpcl_filters -lpcl_registration -lpcl_io -lpcl_common -lpcl_kdtree -lpcl_search -o "$out"
