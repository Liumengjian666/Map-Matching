# Reproduce the bounded offline experiment

From the paper workspace, with the original mounted Floor01 archive:

```bash
P9_SCRIPTS=src/dog_prior_map_fastlio2_frontend_exp/scripts/p9
P9_ARCHIVE='/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/dual_u_r1_closure_20261003/same_objective'
P9_MAP='/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/map/frozen/floor01_h1_map_p5_frozen.pcd'
P9_OUTPUT=docs/p9_r1a_true_profile_closure
P9_OLD_PROFILE=docs/p9_dual_u_nonlocal_r1/sidecars/profile_cohort_final

python3 "$P9_SCRIPTS/prepare_true_profile_oracle.py" \
  --archive "$P9_ARCHIVE" \
  --r1-recovery "$P9_OLD_PROFILE/oracle_recovery.csv" --output "$P9_OUTPUT"

cmake -S "$P9_SCRIPTS" -B /tmp/p9_r1a_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/p9_r1a_build -j2
(cd /tmp/p9_r1a_build && ctest --output-on-failure)

LD_LIBRARY_PATH=/lib/x86_64-linux-gnu /tmp/p9_r1a_build/p9_true_profile_closure \
  --canonical "$P9_MAP" "$P9_ARCHIVE/frozen/cohort_frozen.csv" \
  "$P9_ARCHIVE/candidates.csv" "$P9_ARCHIVE/dual_u.csv" \
  "$P9_OUTPUT/oracle_terminal_requests.csv" "$P9_OUTPUT"

LD_LIBRARY_PATH=/lib/x86_64-linux-gnu /tmp/p9_r1a_build/p9_true_profile_closure \
  --solvers "$P9_MAP" "$P9_ARCHIVE/frozen/cohort_frozen.csv" \
  "$P9_ARCHIVE/candidates.csv" "$P9_ARCHIVE/dual_u.csv" \
  "$P9_OUTPUT/canonical_oracle.csv" "$P9_OUTPUT"

python3 "$P9_SCRIPTS/summarize_true_profile_closure.py" \
  --output "$P9_OUTPUT" --uobs "$P9_ARCHIVE/dual_u.csv" --r1-profile "$P9_OLD_PROFILE"
git diff --check
```

These commands regenerate CSV/JSON/report outputs. Runtime fields vary between runs. They never read GT, replay localization, alter maps, or use production EKF. They make22 canonical refinements plus35 endpoint refinements, not a new exhaustive search. The adaptive-grid gate fails; `adaptive_grid.csv` is intentionally absent.

Build: GNU9.4.0, PCL `1.10.0+dfsg-5ubuntu1`, Release. Map SHA256: `2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570`. Source clouds are checked against their frozen prepared point counts and FNV hashes by the C++ runner. External CSV and historical clusterer digests are in `oracle_preparation.json`; generated CSV and source digests are in `results.json`.
