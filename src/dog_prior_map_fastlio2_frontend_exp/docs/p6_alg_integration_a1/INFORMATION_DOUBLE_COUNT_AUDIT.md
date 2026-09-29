# Information double-count audit

The fixed-lag window owns one observation-ID set shared by IMU, LiDAR, and
visual records.  A duplicate ID is rejected and counted in
`WindowSummary::duplicate_measurement_count`; the test reports one rejected
duplicate.  The retained Schur prior is the sole summary of marginalized
factors.  No original marginalized factor is assembled again.

The existing terminal transaction cache and IKFoM committed state are not
automatically copied into this experimental window.  A future adapter must
choose one source of each measurement and pass it once.  The current
`setWindowPredictionSeed` API is feedback only; it does not inject an EKF
posterior into the window or vice versa implicitly.
