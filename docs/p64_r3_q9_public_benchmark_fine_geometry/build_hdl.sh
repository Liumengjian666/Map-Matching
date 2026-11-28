#!/usr/bin/env bash
set -euo pipefail
source_root=$(realpath "$1")
stage=/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_051_runtime
test "$(git -C "$source_root" rev-parse HEAD)" = de2dc7769aa2412876d05ae6161408d3295b6a3c
mkdir -p "$source_root/lib"
include=(-I"$source_root/include" -I"$stage/devel/include" -I"$stage/src/ndt_omp/include" -I"$stage/src/fast_gicp/include" -I/opt/ros/noetic/include -I/usr/include/eigen3 -I/usr/include/pcl-1.10)
for stem in pose_estimator hdl_localization_nodelet; do
    case "$stem" in
      pose_estimator) path="$source_root/src/hdl_localization/pose_estimator.cpp";;
      *) path="$source_root/apps/hdl_localization_nodelet.cpp";;
    esac
    env -u LD_LIBRARY_PATH g++ -std=c++17 -DNDEBUG -O2 -fPIC -fopenmp "${include[@]}" -c "$path" -o "$source_root/lib/$stem.o"
done
env -u LD_LIBRARY_PATH g++ -shared -fopenmp "$source_root/lib/pose_estimator.o" "$source_root/lib/hdl_localization_nodelet.o" -o "$source_root/lib/libhdl_localization_nodelet.so" \
  -L"$stage/devel/lib" -Wl,-rpath,"$stage/devel/lib" -lndt_omp -lfast_gicp \
  -L/opt/ros/noetic/lib -Wl,-rpath,/opt/ros/noetic/lib -lnodeletlib -lroscpp -lrostime -ltf2 -ltf2_ros -lpcl_ros_tf -leigen_conversions \
  -lpcl_filters -lpcl_registration -lpcl_io -lpcl_common -lpcl_kdtree -lpcl_search
