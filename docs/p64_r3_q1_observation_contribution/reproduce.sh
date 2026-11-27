#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
run=${1:?provide a new directory}
mkdir -- "$run"
run=$(realpath "$run")
out="$run/report"
mkdir -- "$out"
export OPENBLAS_NUM_THREADS=1
export MPLBACKEND=Agg
export PYTHONDONTWRITEBYTECODE=1
fmt='wall_s=%e user_s=%U system_s=%S max_rss_kb=%M'
python3 "$here/freeze.py" "$run" "$out"
python3 "$here/neutral.py" "$out"
"$here/build.sh" "$run/score_fixed"
for frame in H307 H515 H309 O164; do
  env -u LD_LIBRARY_PATH "$run/score_fixed" "$run/${frame}_nominal.tsv" "$run/${frame}_nominal"
done
python3 "$here/observations.py" "$run" "$out"
for frame in H307 H515 H309 O164; do
  /usr/bin/time -o "$run/${frame}_perturb_time.txt" -f "$fmt" env -u LD_LIBRARY_PATH \
    "$run/score_fixed" "$run/${frame}_perturb.tsv" "$run/${frame}_perturb"
done
python3 "$here/neighborhood.py" "$run" "$out"
python3 "$here/geometry_selfcheck.py" >"$run/geometry_selfcheck.log"
/usr/bin/time -o "$run/geometry_time.txt" -f "$fmt" python3 "$here/geometry.py" "$run/geometry" \
  "$run/H307.npz" "$run/H515.npz" "$run/H309.npz" "$run/O164.npz"
/usr/bin/time -o "$run/selection_time.txt" -f "$fmt" python3 "$here/contribution_selection.py" --run "$run" \
  --hypotheses "$out/fixed_hypothesis_manifest.csv" --out "$run/blind"
python3 "$here/patch_diagnostics.py" "$run" "$out"
python3 "$here/evaluate.py" "$run" "$out"
python3 "$here/prepare_omni_observations.py" "$run/omni"
mkdir -- "$run/omni_repeat"
python3 "$here/evaluate_omni_repeatability.py" "$run/omni" "$run/omni_repeat"
python3 "$here/archive.py" "$run" "$out"
python3 "$here/validate.py" "$run" "$out"
