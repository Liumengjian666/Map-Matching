#!/usr/bin/env bash
set -euo pipefail
root=$(git rev-parse --show-toplevel)
source_dir="$root/docs/p64_r2_q1_candidate_search"
ndt_include="${P64_Q1_NDT_INCLUDE:-/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_051_runtime/src/ndt_omp/include}"
(cd "$ndt_include/pclomp" && sha256sum -c "$source_dir/upstream_sha256.txt")
mkdir -p "$root/run/build"
env -u LD_LIBRARY_PATH g++ -std=c++17 -O3 -DNDEBUG -fopenmp -I"$ndt_include" \
  -I/usr/include/pcl-1.10 -I/usr/include/eigen3 "$source_dir/qualification.cpp" \
  -lpcl_io -lpcl_filters -lpcl_registration -lpcl_common -lpcl_kdtree -lpcl_search \
  -o "$root/run/build/qualification"
