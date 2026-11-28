#!/usr/bin/env bash
set -eo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
package=$(realpath "$1")
inputs=$(realpath "$2")
result=$(realpath -m "$3")
port="${4:-11729}"
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="http://127.0.0.1:$port"
export ROS_IP=127.0.0.1
export ROS_PACKAGE_PATH="$package:${ROS_PACKAGE_PATH:-}"
export LD_LIBRARY_PATH="$package/lib:${LD_LIBRARY_PATH:-}"
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
export OMP_NUM_THREADS=4
export OPENBLAS_NUM_THREADS=1
mkdir "$result"
export ROS_LOG_DIR="$result"
roscore -p "$port" >"$result/core.log" 2>&1 &
master_pid=$!
pnode=
trap 'if [[ -n "$pnode" ]]; then kill "$pnode" 2>/dev/null || true; fi; kill "$master_pid" 2>/dev/null || true' EXIT
for _ in {1..30}; do
    rosparam list >/dev/null 2>&1 && break
    sleep .3
done
rosparam set /hdl_q9/use_global_localization false
rosparam set /hdl_q9/use_imu true
rosparam set /hdl_q9/cool_time_duration 2.0
rosparam set /hdl_q9/specify_init_pose false
rosparam set /hdl_q9/enable_robot_odometry_prediction false
rosrun nodelet nodelet standalone hdl_localization/HdlLocalizationNodelet __name:=hdl_q9 >"$result/nodelet.log" 2>&1 &
pnode=$!
/usr/bin/python3 "$here/stream_hdl.py" "$inputs" "$result/trajectory.csv"
