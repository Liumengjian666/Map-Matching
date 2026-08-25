#!/usr/bin/env bash
set -euo pipefail

OUT_ROOT=${OUT_ROOT:-/home/jian/rosbag/loop3}
RUN_NAME=${RUN_NAME:-absolute_ndt_like_shixiong_loop3_$(date +%Y%m%d_%H%M%S)}
OUT="$OUT_ROOT/$RUN_NAME"
BAG=${BAG:-/home/jian/rosbag/loop3/bag/loop3_2026-08-11-16-09-07.bag}
CONFIG=${CONFIG:-/home/jian/livox_ws/dog_light_loc_ws/src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml}
REF_BAG=${REF_BAG:-/home/jian/rosbag/loop3/fastlivo2location_backend_only_loop3_20260817_141325/fastlivo2location_backend_only_result.bag}
REF_TOPIC=${REF_TOPIC:-/localization}

mkdir -p "$OUT/ros_home/log" "$OUT/eval"
export ROS_HOME="$OUT/ros_home"
export ROS_LOG_DIR="$OUT/ros_home/log"
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11311}

source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/dog_light_loc_ws/devel/setup.bash

cleanup() {
  set +e
  [[ -n "${CPU_PID:-}" ]] && kill -INT "$CPU_PID" 2>/dev/null
  [[ -n "${REC_PID:-}" ]] && kill -INT "$REC_PID" 2>/dev/null
  [[ -n "${NODE_PID:-}" ]] && kill -INT "$NODE_PID" 2>/dev/null
  [[ -n "${CORE_PID:-}" ]] && kill -INT "$CORE_PID" 2>/dev/null
}
trap cleanup EXIT

wait_for_bag() {
  local bag_path="$1"
  local active_path="${bag_path}.active"
  for _ in $(seq 1 60); do
    [[ -f "$bag_path" && ! -f "$active_path" ]] && return 0
    sleep 1
  done
  echo "等待rosbag落盘超时：$bag_path" >&2
  return 1
}

roscore > "$OUT/roscore.log" 2>&1 &
CORE_PID=$!
for _ in $(seq 1 30); do
  rostopic list >/dev/null 2>&1 && break
  sleep 1
done
rostopic list >/dev/null

rosparam set use_sim_time true
roslaunch dog_prior_map_localization dog_prior_map_localization_split.launch \
  config:="$CONFIG" rviz:=false runtime_csv_path:="$OUT/runtime.csv" \
  ndt_diagnostics_csv_path:="$OUT/ndt_diagnostics.csv" \
  > "$OUT/node.log" 2>&1 &
NODE_PID=$!
sleep 8

while true; do
  ps -C dog_prior_map_ekf_node_cpp -o pid=,%cpu=,%mem=,rss= >> "$OUT/cpu_samples.txt" 2>/dev/null || true
  sleep 1
done &
CPU_PID=$!

rosbag record -O "$OUT/abs_ndt_result.bag" \
  /dog_livo/odom_corrected /dog_livo/odom_high_rate /dog_livo/path_corrected \
  /dog_livo/ndt_odom /dog_livo/ndt_pose /dog_livo/ndt_path /dog_livo/diagnostics /tf /clock \
  > "$OUT/record.log" 2>&1 &
REC_PID=$!
sleep 3

rosbag play --clock "$BAG" --topics /livox/lidar /livox/imu /clock > "$OUT/play.log" 2>&1
sleep 3
cleanup
trap - EXIT
wait_for_bag "$OUT/abs_ndt_result.bag"

python3 /home/jian/livox_ws/dog_light_loc_ws/src/dog_prior_map_localization/scripts/evaluate_runtime_and_accuracy.py \
  --name "$RUN_NAME" \
  --odom-bag "$OUT/abs_ndt_result.bag" \
  --runtime-csv "$OUT/runtime.csv" \
  --cpu-samples "$OUT/cpu_samples.txt" \
  --window-sec 10 \
  --output "$OUT/eval/self_metrics.json" | tee "$OUT/eval/self_metrics.txt"

if [[ -f "$REF_BAG" ]]; then
  python3 /home/jian/livox_ws/dog_light_loc_ws/src/dog_prior_map_localization/scripts/compare_odom_to_reference.py \
    --ref-bag "$REF_BAG" --ref-topic "$REF_TOPIC" \
    --ours-bag "$OUT/abs_ndt_result.bag" --ours-topic /dog_livo/odom_corrected \
    --sample-hz 10 --align-start \
    --output "$OUT/eval/vs_fastlivo2location_backend_align_start.json" \
    | tee "$OUT/eval/vs_fastlivo2location_backend_align_start.txt"
fi

echo "完成：$OUT"
