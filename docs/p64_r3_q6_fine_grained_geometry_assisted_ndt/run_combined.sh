#!/usr/bin/env bash
set -euo pipefail
q=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
out=${1:?new output directory}
bin=${2:?offline binary}
dog=${3:?rotation-deskewed 50-frame directory}
mkdir "$out"
omni=/home/jian/p64_r2_q2_ws/run/protocol_b_frozen
w2=/home/jian/p64_r3_q3_ws/run/W2
map=/home/jian/rosbag/dog/rect_record_20261029_fastlivo2/rerun_20261102_182309/pcd/rerun_20261102_182309_all_downsampled_points.pcd
one() {
  local window=$1 input=$2 mode=$3 index=$4 style=$5
  "$q/run_v1.sh" "$bin" "$input" "$out/${window}_$mode" "$mode" "$index" 25 "$style" >"$out/${window}_$mode.log" 2>&1
}
{
  one omni "$omni" B -1 S0
  one omni "$omni" C 25 S0
  one omni "$omni" E2_SHADOW 25 S0
  one omni "$omni" E2_ACTIVE 25 S0
} &
first=$!
{
  one w2 "$w2" E2_SHADOW 32 S3
  one w2 "$w2" E3_SHADOW 32 S3
} &
second=$!
wait "$first"
wait "$second"
P64_MAP="$map" one dog "$dog" A -1 S3
P64_MAP="$map" one dog "$dog" E2_SHADOW 30 S3
