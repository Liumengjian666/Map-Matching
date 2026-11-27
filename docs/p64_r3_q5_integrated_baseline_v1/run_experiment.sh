#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
run="${1:?new absolute run directory}"
[[ "$run" = /* ]]
mkdir "$run" "$run/results" "$run/final" "$run/cc" "$run/fixed"
export OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
"$here/build.sh" "$run/build"
"$here/build_reference.sh" "$run/ref"
python3 "$here/reference_fixtures.py" point-plane "$run/point_planes.tsv"
"$run/ref/reference" "$run/point_planes.tsv" "$run/results/DEGENERACY_DETECTION_RESULTS.csv"
python3 "$here/reference_fixtures.py" m3c2 "$run/cc" "$run/results/M3C2_RESULTS.csv"
python3 "$here/native_geometry_qualification.py" "$run" "$run/results/M3C2_ORIGINAL_QUALIFICATION.csv"
python3 "$here/fine_geometry.py" "$run" "$run/results/FINE_GEOMETRY_RESULTS.csv"
python3 "$here/fixed_regression.py" "$run/fixed" "$run/results/fixed_regression.csv"
# ROS Python only for bounded input extraction.
set +u
source /opt/ros/noetic/setup.bash
set -u
python3 "$here/dog_prepare.py" "$run/dog_input"
omni=/home/jian/p64_r2_q2_ws/run/protocol_b_frozen
w2=/home/jian/p64_r3_q3_ws/run/W2
dogmap=/home/jian/rosbag/dog/rect_record_20261029_fastlivo2/rerun_20261102_182309/pcd/rerun_20261102_182309_all_downsampled_points.pcd
one() {
    local name=$1
    shift
    "$here/run_v1.sh" "$run/build/v1" "$1" "$run/final/$name" "$2" "$3" "$4" "$5" >"$run/final/$name.log" 2>&1
}
one omni_A "$omni" A -1 25 S0
one omni_OFF "$omni" OFF -1 25 S0
one omni_B "$omni" B -1 25 S0
one omni_C "$omni" C 25 25 S0
one omni_SHADOW "$omni" SHADOW 25 25 S0
one omni_TIMEOUT "$omni" SHADOW 25 .00001 S0
one w2_A "$w2" A -1 25 S3
one w2_SHADOW "$w2" SHADOW 32 25 S3
P64_MAP="$dogmap" one dog_A "$run/dog_input" A -1 25 S3
P64_MAP="$dogmap" one dog_SHADOW "$run/dog_input" SHADOW 30 25 S3
python3 "$here/check_gate.py" "$run/final/omni_A" "$run/final/omni_OFF"
python3 "$here/check_gate.py" "$run/final/omni_A" "$run/final/omni_SHADOW"
python3 "$here/check_gate.py" "$run/final/w2_A" "$run/final/w2_SHADOW"
python3 "$here/check_gate.py" "$run/final/dog_A" "$run/final/dog_SHADOW"
one omni_ACTIVE "$omni" ACTIVE 25 25 S0
one w2_ACTIVE "$w2" ACTIVE 32 25 S3
P64_MAP="$dogmap" one dog_ACTIVE "$run/dog_input" ACTIVE 30 25 S3
python3 "$here/evaluate.py" "$run/final" "$run/results"
