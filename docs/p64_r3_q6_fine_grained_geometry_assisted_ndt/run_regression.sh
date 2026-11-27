#!/usr/bin/env bash
set -euo pipefail
q=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
run=${1:?new absolute output directory}
bin=${2:?compiled v1 path}
mkdir "$run"
omni=/home/jian/p64_r2_q2_ws/run/protocol_b_frozen
w2=/home/jian/p64_r3_q3_ws/run/W2
dog=/home/jian/p64_r3_q5_ws/run/dog_input
dogmap=/home/jian/rosbag/dog/rect_record_20261029_fastlivo2/rerun_20261102_182309/pcd/rerun_20261102_182309_all_downsampled_points.pcd
for spec in "omni $omni 25 S0" "w2 $w2 32 S3" "dog $dog 30 S3"; do
    read -r name data force style <<< "$spec"
    if [[ "$name" == dog ]]; then export P64_MAP="$dogmap"; fi
    for mode in A FIXED; do
        "$q/run_v1.sh" "$bin" "$data" "$run/${name}_$mode" "$mode" "$force" 25 "$style"
    done
done
