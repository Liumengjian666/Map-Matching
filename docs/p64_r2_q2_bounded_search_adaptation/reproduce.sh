#!/usr/bin/env bash
set -euo pipefail
[[ $# == 1 && "$1" = /* && ! -e "$1" ]]
root=$(git rev-parse --show-toplevel)
q1="${P64_Q1_WORKSPACE:-/home/jian/p64_r2_q1_ws}"
docs="$root/docs/p64_r2_q2_bounded_search_adaptation"
q1_docs="$q1/docs/p64_r2_q1_candidate_search"
out="$1"
mkdir "$out"
cp "$q1/run/input/online.tsv" "$out/online.tsv"
P64_Q2_BUILD="$out/build" bash "$docs/build.sh" > "$out/build.log" 2>&1
binary="$out/build/search"
export OMP_NUM_THREADS=1
env -u LD_LIBRARY_PATH "$binary" validate "$out/online.tsv" "$q1/run/official/grid.csv" unused
env -u LD_LIBRARY_PATH /usr/bin/time -v timeout 120 prlimit --as=3221225472 -- \
  "$binary" gate "$out/online.tsv" "$q1/run/gate/native_score_consistency.csv" "$out/official_score_parity.csv"
env -u LD_LIBRARY_PATH /usr/bin/time -v timeout 1200 prlimit --as=3221225472 -- \
  "$binary" search "$out/online.tsv" "$q1/run/official/grid.csv" "$out/official" "$q1/run/official/fit.csv"
python3 -B "$docs/seal_q2.py" "$q1" "$out"
python3 -B "$docs/evaluate_q2.py" --online "$out/online.tsv" \
  --manifest "$q1_docs/qualification_sample_manifest.csv" --grid "$out/official/grid.csv" \
  --retained "$out/official/retained.csv" --fits "$out/official/fit.csv" \
  --runtime "$out/official/runtime_breakdown.csv" --reference "$q1/run/reference.tsv" \
  --seal "$out/online_seal.json" --q1-run "$q1/run" --out "$out/evaluation"
