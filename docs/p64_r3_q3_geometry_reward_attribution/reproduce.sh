#!/usr/bin/env bash
set -eo pipefail
d=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
run="${1:?supply a new absolute run directory}"
mkdir "$run"
out="$run/report"
mkdir "$out"
export OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
env -u LD_LIBRARY_PATH "$d/build_native.sh" "$run/native_export"
python3 "$d/attribution.py" control "$run" "$out"
python3 "$d/attribution.py" plan "$run" "$out"
for p in hard omni; do env -u LD_LIBRARY_PATH "$run/native_export" "$run/${p}_extra.tsv" "$run/${p}_extra"; done
python3 "$d/attribution.py" blind "$run" "$out"
python3 "$d/evaluate.py" "$run" "$out"
python3 "$d/scale_controls.py" "$run" "$out"
env -u LD_LIBRARY_PATH "$d/build_grid.sh" "$run/grid"
env -u LD_LIBRARY_PATH "$d/build_window.sh" "$run/protocol_b_window"
source /opt/ros/noetic/setup.bash
fmt='%e %U %S %M'
/usr/bin/time -o "$run/window_prepare.wall" -f "$fmt" /usr/bin/python3 "$d/mid_prepare.py" prepare "$run"
map=/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_048_diff_map_formal_01/nav_a_diff_prior_voxel0p15.pcd
for p in "$run"/W?; do
  if test -e "$p/anchor.tsv"; then
    /usr/bin/time -o "$p/window.wall" -f "$fmt" env -u LD_LIBRARY_PATH LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_LIBRARY_PATH=/opt/ros/noetic/lib timeout 90 "$run/protocol_b_window" "$p" "$map" "$p/trajectory.csv" >"$p/engine.log" 2>&1
  fi
done
python3 "$d/mid_prepare.py" trigger "$run"
if test -s "$run/mid_plan.tsv"; then
  /usr/bin/time -o "$run/mid_grid.wall" -f "$fmt" env -u LD_LIBRARY_PATH "$run/grid" grid "$run/mid_plan.tsv" "$run/mid_grid"
  python3 "$d/mid_score.py" seeds "$run"
  /usr/bin/time -o "$run/mid_fit.wall" -f "$fmt" env -u LD_LIBRARY_PATH "$run/grid" fit "$run/mid_plan.tsv" "$run/mid_fit" "$run/mid_seeds.tsv"
fi
python3 "$d/mid_score.py" assemble "$run"
/usr/bin/time -o "$run/mid_fixed.wall" -f "$fmt" env -u LD_LIBRARY_PATH "$run/native_export" "$run/mid_export.tsv" "$run/mid_fixed"
/usr/bin/time -o "$run/mid_rescore.wall" -f "$fmt" python3 "$d/mid_score.py" score "$run"
python3 "$d/mid_score.py" evaluate "$run" "$out"
python3 "$d/archive.py" "$run" "$out"
