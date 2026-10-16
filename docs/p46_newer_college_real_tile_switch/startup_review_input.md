# P46 startup-attribution second-opinion input

Scope: adversarially assess the P45 first-frame 0.202 m EKF high-rate position jump using only these frozen P44/P45 facts and the quoted current source ordering. No new localization run is part of this review.

## Frozen-run identity and matched first-frame inputs

- Dataset: Newer College 2020, `01_short_experiment`.
- P44 Global run: `/media/jian/HIKVISION/paper rosbag/NewerCollege/.p44_300frame/headless_300`.
- P45 Local run: `/media/jian/HIKVISION/paper rosbag/NewerCollege/.p45_local_map/formal300`.
- Both have 300 scan receipts and 300 OOSM APPLIED results. P45 has one initial tile load and no subsequent target update in its 300 scans.
- The first scan header is `1583836591182590976 ns` in both runs.
- Both first scans have 1400 filtered source points and the same logged cloud hash `b8191bc3a3063105`.
- Both first NDT initial guesses are identical: translation `(-3.3771578625504559, 8.3231144341813756, 0.13788850483323301)` m; quaternion `(0.013515811774093205, -0.011521024499653339, 0.50112447042693675, 0.86519295764926796)` in x,y,z,w order.
- P44 used the global preprocessed target with 3,142,751 points. P45's first local map diagnostic records `initial_tile_load`, 21 tile IDs, 552,716 target points, and the same first scan timestamp.

## First NDT and OOSM facts

| Quantity | P44 Global | P45 Local |
|---|---:|---:|
| first raw NDT translation (m) | `(-3.3940305710, 8.3361530304, 0.2319031209)` | `(-3.4209148884, 8.3788366318, 0.3273395896)` |
| correction from common initial guess (m) | 0.096402496 | 0.202265553 |
| correction rotation | 1.3182493 deg | 0.8425431 deg |
| fitness / iterations | 0.0206946 / 7 | 0.0309529 / 4 |
| first OOSM replay rewrite translation | 0.096614185 m | 0.202457299 m |
| first OOSM replay rewrite rotation | 1.3182491 deg | 0.8425428 deg |
| replay IMU samples | 11 | 11 |

For both first callbacks, `state_now_before_ndt=state_now_after_replay=1583836591.2885962`, NDT measurement stamp / rollback stamp `1583836591.182591`, and lag is about 106.005 ms. OOSM reports APPLIED. No different first-frame timestamp, seed, source hash, or replay count is present in the frozen artifacts.

The EKF high-rate state immediately before the first OOSM is identical in both logs at `1583836591288596186 ns`: `(-3.3773033681608027, 8.3239403715916662, 0.13812230665032266)` m. At the next published high-rate state `1583836591298597245 ns` (10.001059 ms later), P44 is `(-3.3936636344249611, 8.3385547056108607, 0.2321631291005582)` m and P45 is `(-3.4205290186715991, 8.3805852186276404, 0.32761994474421519)` m. The adjacent position jumps are 0.096565591 m (P44) and 0.202451106 m (P45). Over the first 300 adjacent high-rate intervals, mean step is 0.003175 m for P44 and 0.004784 m for P45; the respective 95th-percentile steps are 0.019065 m and 0.021999 m.

Across the first 30 NDT/OOSM updates, P45's replay translation rewrites for scans 1–5 are `0.202457, 0.189163, 0.107161, 0.004611, 0.036653 m`; therefore the correction is not monotonically decaying or zero after frame 4. P44's corresponding values are `0.096614, 0.036505, 0.017093, 0.027345, 0.023364 m`.

## Source-order evidence for the first Local target

Current committed source is `0464ff01fae842a137cba6c39d3ec091b10ec7df`, same as the checked remote branch head at review time. In `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp`, `handleCloudLocked()`:

1. Calls `local_map_tiles_.ensureCoverage(source, initial_guess, ...)` at lines 1865–1866.
2. On update, assigns `target_cloud_ = local_map_update.target` and calls `ndt_.setInputTarget(target_cloud_)` at lines 1885–1891.
3. Writes the per-scan map diagnostic at lines 1907–1910.
4. Calls `ndt_.align(aligned, initial_guess.cast<float>())` at line 1937.

These operations are in the same serialized cloud callback. This establishes that the committed code supplies the updated target before that callback's NDT alignment. However, the frozen NDT row does not itself store a target-generation ID/hash or target point count, and P45's `resource_summary.json` has zero observed node PIDs because the old monitor missed ROS descendants. Do not overstate what the output CSV alone joins to the NDT call or treat its aggregate CPU fields as valid.

P45's report says its NDT HWM was recorded from the NDT-local map diagnostics; the separate process monitor's CPU / role rows are invalid. The P45 artifact set does not carry an explicit per-run binary hash beside the first NDT row, so an exact executable fingerprint is not directly joined to that row.

## Question for the independent reviewer

Based only on this evidence, assess:

1. Is the first 0.202 m jump best described as a target-associated NDT correction propagated by a correctly applied OOSM replay, or is there evidence for a timestamp/initial-state/replay contract bug?
2. Which causal claims are supported, and which remain unproven because the CSV lacks an align-to-target generation fingerprint or run-time executable fingerprint?
3. Does this startup evidence show a blocking causal contract error that should prevent the prescribed P46 local 1800-scan run, or is it a bounded startup-risk observation to retain and monitor?

Do not recommend tuning or changing the NDT/EKF algorithm, and do not infer accuracy from the jump alone.
