#!/usr/bin/env bash
set -euo pipefail
root=$(git rev-parse --show-toplevel)
docs="$root/docs/p64_r2_q2_bounded_search_adaptation"
q1="$root/docs/p64_r2_q1_candidate_search"
inc="${P64_Q2_NDT_INCLUDE:-/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_051_runtime/src/ndt_omp/include}"
dest="${P64_Q2_BUILD:-$root/run/build}"
mkdir -p "$dest"
test ! -e "$dest/search"
(cd "$inc/pclomp" && sha256sum -c "$q1/upstream_sha256.txt")
python3 -B "$docs/expose_profile_access.py" "$q1/scoring.hpp" "$dest/scoring_profile.hpp"
env -u LD_LIBRARY_PATH g++ -std=c++17 -O3 -DNDEBUG -fopenmp -I"$inc" \
  -I"$dest" -I/usr/include/pcl-1.10 -I/usr/include/eigen3 "$docs/search.cpp" \
  -lpcl_io -lpcl_filters -lpcl_registration -lpcl_common -lpcl_kdtree -lpcl_search \
  -o "$dest/search"
