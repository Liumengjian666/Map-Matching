#!/usr/bin/env bash
set -euo pipefail
q=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
stable=/home/jian/dog_visual_loc_ws/src/dog_prior_map_localization
env -u LD_LIBRARY_PATH g++ -std=c++17 -O2 -I"$stable/include" -I/home/jian/fast_lio2_ws/devel/include \
 -I/opt/ros/noetic/include -I/usr/include/pcl-1.10 -I/usr/include/eigen3 \
 "$q/dog_deskew.cpp" "$stable/src/livox_imu_deskewer.cpp" -L/opt/ros/noetic/lib \
 -Wl,-rpath,/opt/ros/noetic/lib -lrosbag -lrosbag_storage -lroscpp -lroscpp_serialization -lrostime -lcpp_common \
 -lpcl_io -lpcl_common -pthread -o "$1"
