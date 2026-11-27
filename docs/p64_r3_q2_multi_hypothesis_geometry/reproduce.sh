#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
run="${1:?provide a fresh output directory}"
mkdir "$run"
run=$(realpath "$run")
out="$run/results"
mkdir "$out"
export OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
"$here/build_native.sh" "$run/native_export"
python3 "$here/native_analysis.py" plan "$run"
env -u LD_LIBRARY_PATH "$run/native_export" "$run/native_plan.tsv" "$run/native"
python3 "$here/native_analysis.py" analyze "$run" "$out"
python3 "$here/geometry_controls.py" "$run" "$out"
python3 "$here/shared.py" "$run" "$out"
python3 "$here/evaluate.py" "$run" "$out"
"$here/build_mid.sh" "$run/mid_build"
python3 "$here/mid_acquire.py" plan "$run"
/usr/bin/time -o "$run/mid_grid.wall" -f '%e %U %S %M' env -u LD_LIBRARY_PATH "$run/mid_build/qualification" grid "$run/mid_plan.tsv" "$run/mid_grid"
python3 "$here/mid_acquire.py" seeds "$run"
/usr/bin/time -o "$run/mid_fit.wall" -f '%e %U %S %M' env -u LD_LIBRARY_PATH "$run/mid_build/qualification" fit "$run/mid_plan.tsv" "$run/mid_fit" "$run/mid_seeds.tsv"
python3 "$here/mid_acquire.py" qualify "$run"
python3 "$here/mid_review.py" "$run" "$out"
python3 "$here/check_archive.py" "$run" "$out"
