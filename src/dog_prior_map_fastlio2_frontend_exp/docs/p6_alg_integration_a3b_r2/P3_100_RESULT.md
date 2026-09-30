# P3 ADAPTIVE_SELECTED_NIS, frame_limit=100

## Result

`SHORT_REAL_LINK_ENGINEERING_FAIL_STOP_AT_FIRST_FAILURE`.

The one authorized short replay stopped at transaction 90, timestamp
`1517157228164951397`, with
`producer_optimizer:marginalized_prior_not_finite_psd`. The optimizer itself
reported `ACCEPTED_UPDATE` after 8 iterations and reduced its captured
iteration-local surrogate from `17.972697968746459` to
`13.976734288274939`. The failure occurred in the subsequent fixed-lag
marginalization/prior PSD validation, not in candidate rejection. Transaction
83 was passed in this run; the first failure moved to transaction 90.

No retry, parameter adjustment, alternate policy, or further real-data run was
performed. P3-200 was not started.

## Terminal capsule at tx90

- NDT: converged; fitness `31.17423679322231`; objective `587.75769881464896`;
  32 iterations; reported NDT runtime `58.742549 ms`.
- U_obs: valid; reliable rank 5; weak dimension 1; pre-measurement covariance
  valid.
- U_nonlocal: triggered.
- Selected NIS: `8.4101449565544115`, threshold `15.086`, accepted.
- LiDAR factor: attempted and committed; factor counts before optimize were
  IMU `40`, LiDAR `17`, visual `0`.
- Optimizer: 8 trace rows; all candidate basis callback trace values were zero;
  no same-state retry projector discrepancy was observed.
- Failure capsule transaction-safe check: maximum state difference after
  diagnostic capture `0`.

The complete pre-optimizer capsule, optimizer trace, derivative and damping
diagnostics, deskew evidence, events/runtime CSVs, failure summary, and
`/usr/bin/time -v` resource output are preserved in `RUN_P3_100/`.

## Frozen input identity used

The V3 raw manifest passed its runtime identity gate. Its source bag SHA matches
the existing IMU input manifest (`c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`).
The checked input hashes were:

| Input | SHA256 |
|---|---|
| `raw_timed_points.bin` | `4ba09d8ae7004056dcc4e9d63d2ab09915748c7e68bc29d0dc8bd44a24fc95ff` |
| `raw_timed_catalog.csv` | `fdaf9607bc1933269f5f999ee044d37aa1eefabedc054f522da5078ce708123f` |
| `filter_scans.csv` | `41d0b2040a5de8a8bd428c382a7b6dc18fabcaa8d3e7d2cc331018e0585edf1d` |
| `RAW_TIMED_INPUT_MANIFEST.txt` | `fb20125c63aa4c7110851b8bb08ec33e8938d94be18e28e9737d207001377575` |
| normalized map | `103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f` |
| official calibration parameters | `7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d` |
| `imu.csv` | `7dc881d4ebfeacea9354be569a5952e5e466ccdd366637e52a7797a6f37457aa` |

Initialization stamp was `1517157224188979000`; profile was `corridor01`,
visual input/provenance were both `NONE`, and the run command supplied
`frame_limit=100` and `ADAPTIVE_SELECTED_NIS`.

## Structure observed before the failing terminal

- 100-frame input limit was passed as requested; it includes scans before the
  handoff. Output contains 39 scan-start rows, 38 completed scan-end rows, and
  a pre-optimizer capsule for the failing tx90 terminal.
- All 38 completed LiDAR terminals converged in NDT, requested covariance,
  passed through valid U_obs, and attempted a LiDAR factor. 34 factors were
  committed; the remaining attempts were rejected by their normal selected
  NIS gate.
- U_nonlocal was triggered 15 times among completed terminals (plus tx90).
- All 39 available deskew evidence rows report
  `WINDOW_OWNED_SE3_DESKEW`; raw and deskew point counts match; point-time
  minima equal catalog scan starts.
- Completed-event maximum window size was 40 nodes and maximum span
  `1.958914906 s`. At the failing tx90 pre-optimizer capsule the window had 41
  nodes and span `2.017077923 s`; because the terminal failed before successful
  marginalization, the final duration bound cannot be claimed for that event.
- Visual event/factor count remained zero; post-handoff IKFoM calls remained
  zero; the recorded sparse solver fallback count remained zero.
- Recorded completed event poses and tx90 pre-optimizer pose fields were
  finite. The failure was the explicit Schur prior PSD validation; this report
  does not relabel it as a state-finiteness pass.

## Runtime resources

The single run took `10.52 s` wall time (`9.37 s` user, `1.12 s` system), with
maximum RSS `72,824 KiB`. Among completed terminal rows, 68 NDT align calls were
recorded across the base registrations and triggered probes; tx90 executed its
NDT/probe path but failed before its runtime row was committed, so no inferred
grand total is reported. The largest recorded completed-terminal timings were
`8.172 ms` linearization, `14.285 ms` solve, and `4.299 ms` marginal covariance.

These are short-link engineering diagnostics, not localization accuracy or a
formal benchmark. No GT/ATE/RPE was read or computed.
