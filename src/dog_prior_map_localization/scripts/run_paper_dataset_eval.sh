#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "用法: $0 DATASET_NAME INPUT_BAG [CONFIG] [DURATION_SEC] [PLAY_RATE]" >&2
  exit 2
fi

DATASET_NAME=$1
INPUT_BAG=$2
WORKSPACE=/home/jian/livox_ws/dog_light_loc_paper_ws
CONFIG=${3:-$WORKSPACE/src/dog_prior_map_localization/config/dog_prior_map_localization_ndt.yaml}
DURATION_SEC=${4:-0}
PLAY_RATE=${5:-1.0}
RUN_NAME=${RUN_NAME:-${DATASET_NAME}_$(date +%Y%m%d_%H%M%S)}
OUTPUT_ROOT=${OUTPUT_ROOT:-/home/jian/rosbag/paper_localization}
OUTPUT_DIR=$OUTPUT_ROOT/$DATASET_NAME/$RUN_NAME
ROS_PORT=${ROS_PORT:-11321}

[[ -f "$INPUT_BAG" ]] || { echo "输入bag不存在: $INPUT_BAG" >&2; exit 2; }
[[ -f "$CONFIG" ]] || { echo "配置不存在: $CONFIG" >&2; exit 2; }

mkdir -p "$OUTPUT_DIR/ros_home/log" "$OUTPUT_DIR/analysis"
export ROS_HOME=$OUTPUT_DIR/ros_home
export ROS_LOG_DIR=$OUTPUT_DIR/ros_home/log
export ROS_MASTER_URI=http://127.0.0.1:$ROS_PORT

source /home/jian/livox_ws/devel/setup.bash
source "$WORKSPACE/devel/setup.bash"

cp "$CONFIG" "$OUTPUT_DIR/config_used.yaml"
git -C "$WORKSPACE" rev-parse HEAD > "$OUTPUT_DIR/git_head.txt"
git -C "$WORKSPACE" status --short --branch > "$OUTPUT_DIR/git_status.txt"
md5sum "$INPUT_BAG" > "$OUTPUT_DIR/input_md5.txt"
MAP_PATH=$(sed -n 's/^[[:space:]]*pcd_fallback_path:[[:space:]]*"\([^"]*\)".*/\1/p' "$CONFIG" | head -1)
if [[ -n "$MAP_PATH" && -f "$MAP_PATH" ]]; then
  md5sum "$MAP_PATH" > "$OUTPUT_DIR/map_md5.txt"
fi

cleanup() {
  set +e
  [[ -n ${PLAY_PID:-} ]] && kill -INT "$PLAY_PID" 2>/dev/null
  [[ -n ${MONITOR_PID:-} ]] && kill -INT "$MONITOR_PID" 2>/dev/null
  [[ -n ${RECORD_PID:-} ]] && kill -INT "$RECORD_PID" 2>/dev/null
  [[ -n ${LAUNCH_PID:-} ]] && kill -INT "$LAUNCH_PID" 2>/dev/null
  [[ -n ${CORE_PID:-} ]] && kill -INT "$CORE_PID" 2>/dev/null
  wait ${PLAY_PID:-} ${MONITOR_PID:-} ${RECORD_PID:-} ${LAUNCH_PID:-} ${CORE_PID:-} 2>/dev/null
}
trap cleanup EXIT

roscore -p "$ROS_PORT" > "$OUTPUT_DIR/roscore.log" 2>&1 &
CORE_PID=$!
for _ in $(seq 1 30); do
  rostopic list >/dev/null 2>&1 && break
  sleep 1
done
rostopic list >/dev/null
rosparam set /use_sim_time true

roslaunch dog_prior_map_localization dog_prior_map_localization_split.launch \
  config:="$CONFIG" rviz:=false runtime_csv_path:="$OUTPUT_DIR/runtime.csv" \
  ndt_diagnostics_csv_path:="$OUTPUT_DIR/ndt_diagnostics.csv" \
  > "$OUTPUT_DIR/node.log" 2>&1 &
LAUNCH_PID=$!
sleep 5

printf 'wall_time,node,pid,cpu_percent,rss_kb\n' > "$OUTPUT_DIR/resources.csv"
(
  while kill -0 "$LAUNCH_PID" 2>/dev/null; do
    stamp=$(date +%s.%N)
    for node in dog_prior_map_ndt_node_cpp dog_prior_map_ekf_node_cpp; do
      pids=$(pgrep -f "$WORKSPACE/devel/lib/dog_prior_map_localization/$node" || true)
      for pid in $pids; do
        if read -r cpu rss < <(ps -p "$pid" -o %cpu=,rss=); then
          printf '%s,%s,%s,%s,%s\n' "$stamp" "$node" "$pid" "$cpu" "$rss"
        fi
      done
    done
    sleep 1
  done
) >> "$OUTPUT_DIR/resources.csv" &
MONITOR_PID=$!

rosbag record -O "$OUTPUT_DIR/result.bag" \
  /dog_livo/ndt_odom /dog_livo/odom_corrected /dog_livo/odom_high_rate \
  /dog_livo/diagnostics /tf > "$OUTPUT_DIR/record.log" 2>&1 &
RECORD_PID=$!
sleep 2

PLAY_ARGS=(--clock -r "$PLAY_RATE" "$INPUT_BAG" --topics /livox/lidar /livox/imu /image_left/image_rect)
if [[ "$DURATION_SEC" != "0" ]]; then
  PLAY_ARGS=(--clock -r "$PLAY_RATE" --duration="$DURATION_SEC" "$INPUT_BAG" --topics /livox/lidar /livox/imu /image_left/image_rect)
fi
rosbag play "${PLAY_ARGS[@]}" > "$OUTPUT_DIR/play.log" 2>&1 &
PLAY_PID=$!
wait "$PLAY_PID"
unset PLAY_PID
sleep 2
cleanup
trap - EXIT

for _ in $(seq 1 30); do
  [[ -f "$OUTPUT_DIR/result.bag" && ! -f "$OUTPUT_DIR/result.bag.active" ]] && break
  sleep 1
done
[[ -f "$OUTPUT_DIR/result.bag" && ! -f "$OUTPUT_DIR/result.bag.active" ]]

python3 "$WORKSPACE/src/dog_prior_map_localization/scripts/analyze_paper_run.py" \
  --name "$RUN_NAME" --bag "$OUTPUT_DIR/result.bag" \
  --resources "$OUTPUT_DIR/resources.csv" --output "$OUTPUT_DIR/analysis/metrics.json" \
  | tee "$OUTPUT_DIR/analysis/metrics.txt"

printf '%s\n' "$OUTPUT_DIR"
