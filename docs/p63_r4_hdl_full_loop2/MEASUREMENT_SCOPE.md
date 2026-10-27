# Measurement scope and unchanged state update

This engineering trial reuses the exact R3 native libraries. The R4 Python scheduler only publishes the next original scan after matching native/callback/adapter/status/odom receipts. It neither writes an estimator state nor selects a candidate. No IMU, external odometry, global relocation, recovery, new optimizer or new quality gate is active.

## Native state lineage

Pinned HDL `de2dc7769aa2412876d05ae6161408d3295b6a3c`:

- `apps/hdl_localization_nodelet.cpp:227–277`: `predict(stamp)` without IMU, then `correct(stamp,filtered)`, then publish the filtered map-frame pose at the original scan stamp.
- `src/hdl_localization/pose_estimator.cpp:143–215`: one `registration->align()`; build the raw NDT observation; update `last_observation`; call `ukf->correct(observation)`; expose `imu_guess.inverse()*registration->getFinalTransformation()` as the prediction residual. With IMU disabled its published label is `motion_model`.
- `apps/hdl_localization_nodelet.cpp:290`: the loaded global map becomes the registration target. No scan insertion is introduced.

The native UKF update is unconditional on the external R3 screen. A complete native-align/callback/odom receipt, together with this frozen control flow, establishes that the native update path ran. No independent internal UKF success counter or posterior accuracy certificate is claimed. External-invalid outputs remain in the raw trajectory and still entered the author's UKF before the controller could stop the experiment.

## Geometric quantities have different scopes

`apps/hdl_localization_nodelet.cpp:449–474` transforms the source to the map frame first. Its status metric discards aligned points whose distance **from the map origin** exceeds 25 m. Within that subset, a nearest-neighbour distance below 0.5 m is an inlier. `inlier_fraction` is the ratio for that subset; `matching_error` is mean squared distance of its inliers, in m². Zero inliers can produce a non-finite matching error. This is the unmodified author's code, not a newly corrected sensor-centred overlap.

The separate read-only trace records `getFitnessScore()` over all registered source points as `fitness_all_nn_m2`. That quantity and the cropped inlier error need not agree. Neither is an independent position reference. Original non-finite measurements are kept; summaries state total, finite and non-finite sample counts.

The external valid-update screen is exactly R3: finite raw and filtered pose, filtered quaternion norm within (0.99,1.01), native and status convergence, finite all-point fitness and status matching error, and native inlier fraction at least 0.50. The screen is an experimental statistic/stop rule; it is not an author-native UKF admission policy.

## Timing and memory

- `ndt_compute_ms`: the unchanged R3 interposition around native NDT computation, excluding subsequent diagnostic queries.
- `diagnostic_ms`: read-only fitness/target/trace work measured by the same R3 probe.
- `callback_ms`: full original callback including native prediction, matching, filter update, publication and attached diagnostics. This is not an uninstrumented author benchmark.
- `bridge_ms`: CustomMsg decoding and PointCloud2 publication/trace in the exact R3 bridge.
- Handshake wall time: input validation/publication through the arrival of all same-stamp receipts. Includes scheduling and message transport; not interchangeable with callback time.
- Run wall clock: launch through shutdown; startup, screenshot pauses and shutdown are included. Full native timing distributions contain every actual processed frame.
- Manager RSS includes the localization nodelet, global-map nodelet and trace in one process. Native `getrusage` and 50 ms sampling are both retained.
- Owned-tree RSS includes ROS helpers and RViz; the whole-tree column additionally includes the input/recorder process. It is a simultaneous sum of RSS, so shared mappings can be counted more than once. It is not unique physical RAM or whole-machine use. Short unsampled transients outside native `getrusage` are not excluded by a sampled maximum.

The offline handshake preserves original scan timestamps and their inter-scan differences for native prediction while deliberately decoupling delivery from wall-clock 10 Hz. Neither a short native mean nor a complete trial establishes 10 Hz real-time operation.

## Accuracy boundary

No certified independent GT is available. No old FAST-LIVO2 trajectory is treated as GT; no ATE/RPE is computed; start/end physical coincidence is not assumed. Finite output, convergence, high inlier fraction, consistent RViz display and a frozen map cannot by themselves certify localization correctness or a repeated-structure failure.
