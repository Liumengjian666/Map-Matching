#!/usr/bin/env bash
set -euo pipefail
d=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
s=/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_051_runtime
env -u LD_LIBRARY_PATH g++ -std=c++17 -O2 -DNDEBUG -fopenmp -I"$1" -I"$s/src/ndt_omp/include" -I/usr/include/eigen3 -I/usr/include/pcl-1.10 "$d/native_trials.cpp" -L"$s/devel/lib" "-Wl,-rpath,$s/devel/lib" -lndt_omp -lpcl_filters -lpcl_registration -lpcl_io -lpcl_common -lpcl_kdtree -lpcl_search -o "$2"
