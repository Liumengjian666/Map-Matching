#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
b=$(realpath -m "$1")
stage=/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_051_runtime
python3 "$here/../p64_r3_q5_integrated_baseline_v1/prepare_build.py" "$b"
q1="$here/../p64_r2_q1_candidate_search"
(cd "$stage/src/ndt_omp/include/pclomp" && sha256sum -c "$q1/upstream_sha256.txt")
opts=(-std=c++17 -O2 -DNDEBUG -fopenmp -I"$b" -I"$q1" -I"$stage/src/hdl_localization/include" -I"$stage/src/ndt_omp/include" -I/usr/include/pcl-1.10 -I/usr/include/eigen3 -I/opt/ros/noetic/include)
env -u LD_LIBRARY_PATH g++ "${opts[@]}" -c "$b/pose_estimator.cpp" -o "$b/tracker.o"
env -u LD_LIBRARY_PATH g++ "${opts[@]}" "$here/pipeline.cpp" "$b/tracker.o" \
  -L"$stage/devel/lib" -Wl,-rpath,"$stage/devel/lib" -lndt_omp \
  -L/opt/ros/noetic/lib -Wl,-rpath,/opt/ros/noetic/lib -lrostime -lroscpp \
  -lpcl_filters -lpcl_registration -lpcl_io -lpcl_common -lpcl_kdtree -lpcl_search -o "$b/v1"
