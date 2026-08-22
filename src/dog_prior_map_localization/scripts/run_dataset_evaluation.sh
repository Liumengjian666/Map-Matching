#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 4 ]; then
  echo "用法: run_dataset_evaluation.sh <name> <bag> <duration_sec> <output_dir>" >&2
  exit 2
fi

NAME="$1"
BAG="$2"
DURATION="$3"
OUT="$4"

mkdir -p "$OUT"

python3 - "$OUT" <<'PY'
import sys
from pathlib import Path
out = Path(sys.argv[1])
for pattern in ["dog_odom.bag", "dog_odom.bag.active", "latest_runtime.csv", "cpu_samples.txt", "node.log", "play.log", "record.log", "metrics.json"]:
    p = out / pattern
    if p.exists() and p.is_file():
        p.unlink()
PY

source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
source /home/jian/dog_prior_map_livo_ws/devel/setup.bash

rosparam set /use_sim_time true

roslaunch dog_prior_map_localization dog_prior_map_localization.launch \
  config:="${DOG_PRIOR_CONFIG:-/home/jian/dog_prior_map_livo_ws/src/dog_prior_map_localization/config/dog_prior_map_localization.yaml}" \
  use_cpp:=true rviz:=false \
  runtime_csv_path:="$OUT/latest_runtime.csv" \
  > "$OUT/node.log" 2>&1 &
NODE_PID=$!
sleep 3

rosbag record -O "$OUT/dog_odom.bag" /dog_livo/odom_high_rate /dog_livo/odom_corrected > "$OUT/record.log" 2>&1 &
REC_PID=$!
sleep 2

(
  for _ in $(seq 1 $((DURATION + 15))); do
    pgrep -f dog_prior_map_ekf_node_cpp | while read -r pid; do
      ps -p "$pid" -o pid,etimes,pcpu,pmem,rss,cmd --no-headers || true
    done
    sleep 1
  done
) > "$OUT/cpu_samples.txt" &
MON_PID=$!

rosbag play --clock -r "${DOG_PRIOR_PLAY_RATE:-1}" --duration="$DURATION" "$BAG" --topics \
  /livox/imu /livox/lidar /image_left/image_rect \
  > "$OUT/play.log" 2>&1 || true
sleep 2

GLOBAL_RUNTIME_CSV="/home/jian/dog_prior_map_livo_ws/validation/latest_runtime.csv"
if [ ! -f "$OUT/latest_runtime.csv" ] && [ -f "$GLOBAL_RUNTIME_CSV" ]; then
  cp "$GLOBAL_RUNTIME_CSV" "$OUT/latest_runtime.csv"
fi

for node in $(rosnode list 2>/dev/null | grep '^/record' || true); do
  rosnode kill "$node" >/dev/null 2>&1 || true
done
rosnode kill /dog_prior_map_ekf >/dev/null 2>&1 || true
sleep 2

wait "$REC_PID" 2>/dev/null || true
wait "$NODE_PID" 2>/dev/null || true
wait "$MON_PID" 2>/dev/null || true

python3 /home/jian/dog_prior_map_livo_ws/src/dog_prior_map_localization/scripts/evaluate_runtime_and_accuracy.py \
  --name "$NAME" \
  --odom-bag "$OUT/dog_odom.bag" \
  --runtime-csv "$OUT/latest_runtime.csv" \
  --cpu-samples "$OUT/cpu_samples.txt" \
  --window-sec 10 \
  --output "$OUT/metrics.json"
