# Baseline results

No localization baseline was run. The raw sensor archive was unavailable as a complete file, and external storage became unresponsive during a read-only copy. Input gates for ROS bag integrity, raw VLP-16 packet availability, IMU coverage, firing-time decoding, deskew, initialization, and frame consistency therefore remained unmet.

| Metric | P57 result |
|---|---|
| Smoke scans | 0 |
| Full replay scans | 0 |
| CONTROL NDT calls | 0 |
| NDT convergence / fitness / score | Not measured |
| OOSM | Not measured |
| Hessian / degeneracy | Not measured |
| Localization trajectory / ATE | Not produced; map-to-GT contract remains unresolved |
| Runtime / RSS / VmHWM | Not measured |

This is a source/storage and coordinate-contract limitation, not an NDT failure, not evidence that the route is degenerate, and not evidence that any proposed algorithm fails.
