#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 1 ]]; then
  echo "Usage: bash run_qualification.sh ABSOLUTE_EMPTY_RUN_DIRECTORY" >&2
  exit 2
fi
root=$(git rev-parse --show-toplevel)
docs="$root/docs/p64_r2_q1_candidate_search"
run="$1"
[[ "$run" = /* && ! -e "$run" ]]
mkdir -p "$run"
python3 -B "$docs/prepare_inputs.py" prepare --output "$run/input"
exe="$run/build/qualification"
P64_Q1_BINARY="$exe" bash "$docs/build.sh" > "$run/build.log" 2>&1
execute() {
  local mode="$1" output="$2" seconds="$3" threads="$4"
  shift 4
  env -u LD_LIBRARY_PATH OMP_NUM_THREADS="$threads" /usr/bin/time -v \
    timeout "$seconds" prlimit --as=3221225472 -- \
    "$exe" "$mode" "$run/input/online.tsv" "$output" "$@" \
    > "$run/$mode.stdout.log" 2> "$run/$mode.stderr.log"
}
execute gate "$run/gate" 180 4
python3 -B "$docs/seal_experiment.py" gate "$run"
execute smoke "$run/smoke" 120 1
execute grid "$run/rest" 720 1
python3 -B "$docs/seal_experiment.py" grid "$run"
python3 -B "$docs/peaks.py" "$run/official/grid.csv" "$run/official/modes.csv" "$run/official/fit_seeds.tsv"
execute fit "$run/fit" 120 1 "$run/official/fit_seeds.tsv"
cp "$run/fit/fit.csv" "$run/official/fit.csv"
python3 -B "$docs/seal_experiment.py" seal "$run"
/usr/bin/python3 -B "$docs/prepare_inputs.py" references --online-seal "$run/online_seal.json" --output "$run/reference.tsv"
python3 -B "$docs/evaluate.py" --online "$run/input/online.tsv" \
  --manifest "$docs/qualification_sample_manifest.csv" --grid "$run/official/grid.csv" \
  --modes "$run/official/modes.csv" --fits "$run/official/fit.csv" --curvature "$run/fit/curvature.csv" \
  --reference "$run/reference.tsv" --seal "$run/online_seal.json" --out "$run/evaluation"
