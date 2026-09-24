#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 3 ]]; then
  echo "usage: bash run_p3_r9b_floor01.sh RUN_DIR DURATION_SEC|full ROS_MASTER_PORT" >&2
  exit 2
fi

RUN_DIR=$1
DURATION=$2
MASTER_PORT=$3
if [[ -e "$RUN_DIR" ]]; then
  echo "refusing to reuse existing run directory: $RUN_DIR" >&2
  exit 2
fi
mkdir -p "$RUN_DIR"

source /opt/ros/noetic/setup.bash
source /home/jian/livox_ws/devel/setup.bash
source /home/jian/livox_ws/dog_loc_paper_ws/devel/setup.bash
export ROS_PACKAGE_PATH="/home/jian/livox_ws/superloc_adapter_ws/src:$ROS_PACKAGE_PATH"
export CMAKE_PREFIX_PATH="/home/jian/livox_ws/superloc_adapter_ws/devel:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="/home/jian/livox_ws/superloc_adapter_ws/devel/lib:$LD_LIBRARY_PATH"
if ! rospack find velodyne_pointcloud >/dev/null 2>&1 || \
   ! rospack plugins --attrib=plugin nodelet | grep -q 'velodyne_pointcloud'; then
  echo "velodyne_pointcloud nodelet plugin is unavailable in this ROS environment" >&2
  exit 1
fi
export ROS_MASTER_URI="http://127.0.0.1:${MASTER_PORT}"
export ROS_HOSTNAME=127.0.0.1
export ROS_IP=127.0.0.1

RAW_BAG="/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_full_baseline/floor01_canonical_raw_inputs.bag"
PACKAGE="dog_prior_map_localization"
LAUNCH="p3_r9b_floor01_imu_deskew_experimental.launch"
RESOURCE_MONITOR="/home/jian/livox_ws/dog_loc_paper_ws/src/dog_prior_map_localization/docs/p3_r9b_artifacts/resource_monitor.py"

roscore -p "$MASTER_PORT" >"$RUN_DIR/roscore.log" 2>&1 &
ROSCORE_PID=$!
LAUNCH_PID=""
MONITOR_PID=""

cleanup() {
  local status=$?
  if [[ -n "$MONITOR_PID" ]] && kill -0 "$MONITOR_PID" 2>/dev/null; then
    kill -INT "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true
  fi
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  if kill -0 "$ROSCORE_PID" 2>/dev/null; then
    kill -INT "$ROSCORE_PID" 2>/dev/null || true
    wait "$ROSCORE_PID" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

MASTER_READY=0
for _ in $(seq 1 100); do
  if rosparam list >/dev/null 2>&1; then
    MASTER_READY=1
    break
  fi
  if ! kill -0 "$ROSCORE_PID" 2>/dev/null; then
    echo "roscore exited before becoming ready" >&2
    exit 1
  fi
  sleep 0.2
done
if [[ $MASTER_READY -ne 1 ]]; then
  echo "timed out waiting for isolated ROS master on port $MASTER_PORT" >&2
  exit 1
fi
rosparam set /use_sim_time true

roslaunch "$PACKAGE" "$LAUNCH" \
  rviz:=false reference_time:=start \
  deskew_csv_path:="$RUN_DIR/deskew.csv" \
  runtime_csv_path:="$RUN_DIR/runtime.csv" \
  ndt_diagnostics_csv_path:="$RUN_DIR/ndt.csv" \
  oosm_csv_path:="$RUN_DIR/oosm.csv" \
  >"$RUN_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!

NODES_READY=0
for _ in $(seq 1 600); do
  NODES=$(rosnode list 2>/dev/null || true)
  if grep -qx '/dog_prior_map_ekf' <<<"$NODES" && \
     grep -qx '/dog_prior_map_ndt' <<<"$NODES" && \
     grep -qx '/p3_r9b_velodyne_transform' <<<"$NODES"; then
    NODES_READY=1
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "roslaunch exited before nodes became ready" >&2
    exit 1
  fi
  sleep 0.2
done
if [[ $NODES_READY -ne 1 ]]; then
  echo "timed out waiting for R9B nodes" >&2
  exit 1
fi

python3 -u "$RESOURCE_MONITOR" --output "$RUN_DIR/resource_samples.csv" &
MONITOR_PID=$!

if [[ "$DURATION" == "full" ]]; then
  rosbag play --clock -r 1.0 "$RAW_BAG" >"$RUN_DIR/play.log" 2>&1
else
  rosbag play --clock -r 1.0 --duration "$DURATION" "$RAW_BAG" >"$RUN_DIR/play.log" 2>&1
fi

kill -INT "$MONITOR_PID" 2>/dev/null || true
wait "$MONITOR_PID" 2>/dev/null || true
MONITOR_PID=""
echo "R9B replay completed: $RUN_DIR"
