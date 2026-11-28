#!/usr/bin/env bash
set -euo pipefail
d=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=$(realpath "$d/../..")
cd "$root"
asset=run/assets/sheldonian_full.e57
test "$(stat -c %s "$asset")" = 5447128064
printf '%s\n' '46575d3eaf3169a24e24ba0013cc48610125ac842d6745e5e8156c6aecc59756  run/assets/sheldonian_full.e57' | sha256sum --check
mkdir run/r1/full_map
g++ -std=c++17 -O3 -Irun/libe57/include -Irun/libe57/build -I/usr/include/eigen3 "$d/e57_full.cpp" run/libe57/build/libE57Format.a -lxerces-c -o run/r1/e57_full
run/r1/e57_full "$asset" run/r1/full_map/world.xyzf run/r1/full_map/scan_audit.tsv
source /opt/ros/noetic/setup.bash
dense=run/assets/exp18_corridor_lower_gallery_2_imu.txt
export OPENBLAS_NUM_THREADS=1
/usr/bin/python3 "$d/prepare_full_map.py" run/r1/full_map/world.xyzf run/r1/full_map/scan_audit.tsv "$dense" run/r1/full_map/products
/usr/bin/python3 "$d/qualify_full_overlap.py" run/r1/full_map/products run/input_prefix10 run/r1/raw_scan_0000_prefix.pcd "$dense" run/r1/full_map/qualification
if [ -d run/r1/pye57_source/pye57 ]; then
  PYTHONPATH="$root/run/r1/pye57_source:${PYTHONPATH:-}" /usr/bin/python3 "$d/validate_map_source.py" "$asset" run/r1/full_map/scan_audit.tsv run/r1/full_map/secondary_reader.json
fi
