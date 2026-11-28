#!/usr/bin/env bash
set -euo pipefail
d=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
upstream=$(realpath "$1")
out=$(realpath -m "$2")
stage=/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_051_runtime
test "$(git -C "$upstream" rev-parse HEAD)" = de2dc7769aa2412876d05ae6161408d3295b6a3c
python3 "$d/../p64_r3_q5_integrated_baseline_v1/prepare_build.py" "$out"
python3 "$d/materialize_equivalence.py" "$d/.." "$upstream" "$out"
mkdir "$out/lib"
ln -s "$upstream/package.xml" "$out/package.xml"
ln -s "$upstream/nodelet_plugins.xml" "$out/nodelet_plugins.xml"
base=(-std=c++17 -O2 -DNDEBUG -fopenmp -I"$d" -I"$stage/src/ndt_omp/include" -I/usr/include/pcl-1.10 -I/usr/include/eigen3 -I/opt/ros/noetic/include)
lib=(-L"$stage/devel/lib" "-Wl,-rpath,$stage/devel/lib" -lndt_omp -L/opt/ros/noetic/lib -Wl,-rpath,/opt/ros/noetic/lib -lrostime -lroscpp -lpcl_filters -lpcl_registration -lpcl_io -lpcl_common -lpcl_kdtree -lpcl_search -lcrypto)
study=("${base[@]}" -I"$out" -I"$d/../p64_r2_q1_candidate_search" -I"$stage/src/hdl_localization/include")
official=("${base[@]}" -I"$upstream/include" -I"$stage/devel/include" -I"$stage/src/fast_gicp/include")
env -u LD_LIBRARY_PATH g++ "${study[@]}" -c "$out/pose_estimator.cpp" -o "$out/tracker.o"
env -u LD_LIBRARY_PATH g++ "${study[@]}" "$out/research_pipeline.cpp" "$out/tracker.o" "${lib[@]}" -o "$out/research"
env -u LD_LIBRARY_PATH g++ "${official[@]}" -fPIC -c "$upstream/src/hdl_localization/pose_estimator.cpp" -o "$out/original_pose.o"
env -u LD_LIBRARY_PATH g++ "${official[@]}" -shared -fPIC "$out/official_nodelet.cpp" "$out/original_pose.o" "${lib[@]}" -lfast_gicp -lnodeletlib -ltf2 -ltf2_ros -lpcl_ros_tf -leigen_conversions -o "$out/lib/libhdl_localization_nodelet.so"
env -u LD_LIBRARY_PATH g++ "${study[@]}" "$d/fixed_baseline.cpp" "${lib[@]}" -o "$out/fixed"
