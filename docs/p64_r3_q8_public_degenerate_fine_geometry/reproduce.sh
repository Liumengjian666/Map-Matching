#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
run=$(realpath -m "${1:?pass a new run directory}")
ref=$(realpath "${2:?pass official Shield_tunnel1.txt}")
mkdir "$run"
export OPENBLAS_NUM_THREADS=1
p=/home/jian/geode/Shield_tunnel1_gamma
bash "$here/build_curve.sh" "$run/build"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
/usr/bin/python3 "$here/prepare.py" "$p/Shield_tunnel1_gamma.bag" "$p/inspection/scan_ledger.csv" "$p/inspection/imu_records.csv" "$run/inputs"
bash "$here/baseline.sh" "$run" /home/jian/p64_r3_q6_ws/run/local_build/v1
/usr/bin/python3 "$here/match.py" "$run" "$run/results" "$run/build/native_curve"
/usr/bin/python3 "$here/assess.py" "$run" "$run/results" "$ref"
/usr/bin/python3 "$here/followup.py" "$run" "$run/results"
