#!/usr/bin/env bash
set -eo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
bag=$(realpath "$1")
assets=$(realpath "$2")
calibration=$(realpath "$3")
out=$(realpath -m "$4")
docs=$(realpath -m "$5")
original_package=$(realpath "$6")
source /opt/ros/noetic/setup.bash
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=4
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
mkdir "$out"
/usr/bin/time -f '%e,%M' -o "$out/prepare.time" /usr/bin/python3 "$here/prepare_corridor.py" \
    "$bag" "$assets" "$calibration" "$out/input" --frames 270
/usr/bin/time -f '%e,%M' -o "$out/research.time" /home/jian/p64_r3_q6_ws/run/local_build/v1 \
    "$out/input" "$out/input/map.pcd" "$out/research" B -1 20 S3 "$here/unused" >"$out/research.log" 2>&1
/usr/bin/python3 "$here/corridor_results.py" "$out/input" "$out/research" "$docs"
if /usr/bin/time -f '%e,%M' -o "$out/official.time" bash "$here/replay_hdl.sh" "$original_package" "$out/input" "$out/official" >"$out/official.log" 2>&1; then
  /usr/bin/python3 "$here/official_results.py" "$out/input" "$out/official" "$docs"
else
  printf 'Official HDL replay ended with a transport/runtime error. See %s\n' "$out/official.log" >&2
  exit 1
fi
