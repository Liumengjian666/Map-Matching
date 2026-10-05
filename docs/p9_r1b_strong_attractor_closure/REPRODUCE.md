# Reproduce R1B

Run from the paper workspace. Stable robot workspace is never used.

```bash
P9_SCRIPTS=src/dog_prior_map_fastlio2_frontend_exp/scripts/p9
P9_ARCHIVE='/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/dual_u_r1_closure_20261003/same_objective'
P9_MAP='/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/floor01_h1_map_p5_frozen.pcd'
P9_CANONICAL=docs/p9_r1a_true_profile_closure/canonical_oracle.csv
P9_OUTPUT=docs/p9_r1b_strong_attractor_closure

cmake -S "$P9_SCRIPTS" -B /tmp/p9_r1b_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/p9_r1b_build -j2
(cd /tmp/p9_r1b_build && ctest --output-on-failure)

for P9_STAGE in initial bisect refine; do
  python3 "$P9_SCRIPTS/run_strong_attractor_experiment.py" \
    --stage "$P9_STAGE" --runner /tmp/p9_r1b_build/p9_strong_attractor_closure \
    --archive "$P9_ARCHIVE" --map "$P9_MAP" --canonical "$P9_CANONICAL" \
    --output "$P9_OUTPUT"
done

python3 "$P9_SCRIPTS/summarize_strong_attractor_closure.py" \
  --output "$P9_OUTPUT" --canonical "$P9_CANONICAL" --uobs "$P9_ARCHIVE/dual_u.csv"
git diff --check
```

GNU9.4.0, PCL1.10.0+dfsg-5ubuntu1, Release; Python3/NumPy/SciPy.
These commands overwrite generated diagnostic sidecars; use a separate output
directory to preserve an existing run. Runtime fields vary.

The formal archive has350 initial samples,32 bisections,707 path evaluations,
and38 full NDT calls, one per distinct PART A endpoint group. No canonical
oracle refinement is rerun and no weak grid is used. Source preprocessing and
target grid are the unchanged R1A contract. Input SHA256, source prepared
counts/FNV, product reconstruction and request coverage are validated.

Development first ran the15 MAIN representative refinements. The23 supplemental
local-cross groups were then added to complete PART A coverage without rerunning
the MAIN refinements. `full_refine_main_branch_map.csv` and
`full_refine_supplemental_branch_map.csv` are the two disjoint execution chunks;
`full_refine_branch_map.csv` is their union. Fresh reproduction calls all38 once.
The initial350/bisection32/path707 stages were repeated once with the same
settings after input-gate fixes; initial non-timing fields agree350/350. No
additional full NDT calls resulted from those development repeats.

CTest must run in the build directory: the installed older CTest does not
support the newer `--test-dir` invocation. Five P9 tests pass; this is not a
claim about the full production suite.
