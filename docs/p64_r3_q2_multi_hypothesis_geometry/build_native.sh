#!/usr/bin/env bash
set -eu
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
upstream=/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_051_runtime/src/ndt_omp/include
q1="$here/../p64_r2_q1_candidate_search"
exe="${1:?provide a new executable path}"
test ! -e "$exe"
(cd "$upstream/pclomp" && sha256sum -c "$q1/upstream_sha256.txt")
env -u LD_LIBRARY_PATH g++ -std=c++17 -O3 -DNDEBUG -fopenmp -I"$upstream" -I"$q1" \
  -I/usr/include/pcl-1.10 -I/usr/include/eigen3 "$here/native_export.cpp" \
  -lpcl_io -lpcl_filters -lpcl_registration -lpcl_common -lpcl_kdtree -lpcl_search -o "$exe"
