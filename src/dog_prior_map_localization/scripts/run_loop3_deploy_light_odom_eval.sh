#!/usr/bin/env bash
set -euo pipefail

BAG="${1:-/home/jian/rosbag/loop3/bag/loop3_2026-08-11-16-09-07.bag}"
DURATION="${2:-0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${3:-/home/jian/dog_prior_map_livo_ws/validation/loop3/deploy_light_odom_full_${STAMP}}"
CONFIG="/home/jian/dog_prior_map_livo_ws/src/dog_prior_map_localization/config/dog_prior_map_localization_deploy_light_odom.yaml"

mkdir -p "$OUT"
source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
source /home/jian/dog_prior_map_livo_ws/devel/setup.bash
rosparam set /use_sim_time true

roslaunch dog_prior_map_localization dog_prior_map_localization.launch \
  config:="$CONFIG" use_cpp:=true rviz:=false \
  runtime_csv_path:="$OUT/latest_runtime.csv" > "$OUT/node.log" 2>&1 &
NODE_PID=$!
sleep 3

rosbag record -O "$OUT/dog_odom.bag" \
  /dog_livo/odom_high_rate /dog_livo/odom_corrected \
  /dog_livo/path_high_rate /dog_livo/path_corrected > "$OUT/record.log" 2>&1 &
REC_PID=$!
sleep 2

(
  while kill -0 "$NODE_PID" >/dev/null 2>&1; do
    pgrep -f dog_prior_map_ekf_node_cpp | while read -r pid; do
      ps -p "$pid" -o pid,etimes,pcpu,pmem,rss,cmd --no-headers || true
    done
    sleep 1
  done
) > "$OUT/cpu_samples.txt" &
MON_PID=$!

PLAY_ARGS=(--clock -r "${DOG_PRIOR_PLAY_RATE:-1}" "$BAG" --topics /livox/imu /livox/lidar /image_left/image_rect)
if [ "$DURATION" != "0" ]; then
  PLAY_ARGS=(--clock -r "${DOG_PRIOR_PLAY_RATE:-1}" --duration="$DURATION" "$BAG" --topics /livox/imu /livox/lidar /image_left/image_rect)
fi
rosbag play "${PLAY_ARGS[@]}" > "$OUT/play.log" 2>&1 || true
sleep 2

for node in $(rosnode list 2>/dev/null | grep '^/record' || true); do rosnode kill "$node" >/dev/null 2>&1 || true; done
rosnode kill /dog_prior_map_ekf >/dev/null 2>&1 || true
sleep 2
wait "$REC_PID" 2>/dev/null || true
wait "$NODE_PID" 2>/dev/null || true
kill "$MON_PID" >/dev/null 2>&1 || true
wait "$MON_PID" 2>/dev/null || true

python3 /home/jian/dog_prior_map_livo_ws/src/dog_prior_map_localization/scripts/evaluate_runtime_and_accuracy.py \
  --name deploy_light_odom_loop3 \
  --odom-bag "$OUT/dog_odom.bag" \
  --runtime-csv "$OUT/latest_runtime.csv" \
  --cpu-samples "$OUT/cpu_samples.txt" \
  --window-sec 10 \
  --output "$OUT/metrics.json"

echo "$OUT"
