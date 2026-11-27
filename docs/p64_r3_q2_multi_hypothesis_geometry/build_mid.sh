#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
q1="$here/../p64_r2_q1_candidate_search"
run="${1:?new output directory for source and binary}"
upstream=/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_051_runtime/src/ndt_omp/include
(cd "$upstream/pclomp" && sha256sum -c "$q1/upstream_sha256.txt")
mkdir "$run"
# The sole change is the frozen number of input frames.
sed 's/frames.size() == 12/frames.size() == 3/; s/input must contain 12 frames/input must contain 3 frames/' "$q1/qualification.cpp" >"$run/qualification.cpp"
env -u LD_LIBRARY_PATH g++ -std=c++17 -O3 -DNDEBUG -fopenmp -I"$upstream" -I"$q1" \
  -I/usr/include/pcl-1.10 -I/usr/include/eigen3 "$run/qualification.cpp" -lpcl_io -lpcl_filters \
  -lpcl_registration -lpcl_common -lpcl_kdtree -lpcl_search -o "$run/qualification"
