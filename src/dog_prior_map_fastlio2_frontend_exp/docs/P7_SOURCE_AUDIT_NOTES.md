# P7-B source-grounded baseline

The pre-P7 formal experiment path is `p6_i6d_run.py` →
`p6_i6b_closed_loop` → `p6_i1_branched_recovery.cpp`.
Those legacy sources remain unchanged as historical references and posthoc
parity oracles. The new formal entry is `p7_single_state_runner`.

The extracted current-frame NDT keeps the formal parameters: resolution 0.8 m,
step 0.08, transformation epsilon 1e-5, maximum iterations 80. Map preprocessing
is two successive 0.15 m isotropic voxel passes. Source preprocessing uses
finite XYZ, range 0.5–80 m, a 0.25 m isotropic voxel pass, and deterministic
evenly spaced capping at 1400 points. Float coordinates and the legacy FNV
offset, metadata ordering and point-bit hashing are preserved.

Legacy P6 applies `limitStep` (0.5 m / 5 degrees). The new P7 formal path does
not clip, blend or replace the raw NDT terminal. Its only seed is current IKFoM
prediction. Valid effective registrations receive the existing full-pose IKFoM
measurement update with unchanged empirical noise. Ineffective registrations
leave the propagated state unchanged. This stage does not use projected
updates, NIS gates, recovery, observability admission, visual measurements,
nonlocal probes or DCReg, and does not change U_obs normalization.

`GT_USED=false`: neither runner nor wrapper opens ground-truth data. Frozen
packed clouds retain their existing input preprocessing; P7-B does not add a
second deskew operation or claim new SE(3) deskew functionality.

## GEOMETRIC_RADIUS_SEARCH_OUTPUT_CLEAR_REQUIRED

In legacy `GeometricNdt::geometricObservations()`, the `leaves` and
`squared_distances` vectors are declared outside the source-point loop. During
P7-C migration, every source-point query must explicitly execute
`leaves.clear(); squared_distances.clear();` before `radiusSearch`.
P7-B does not modify the legacy implementation or implement this next stage.

## Validation environment and parity boundary

The local camera SDK exports an older USB library through `LD_LIBRARY_PATH`.
PCL IO requires a symbol missing from that library. Standalone builds and
validation subprocesses omit that environment variable to use the installed
system libraries; no SDK, global configuration or direct P7 USB dependency is
modified or added.

Parity is checked only before the first legacy clipping or new ineffective
registration. Raw terminals are compared, not legacy post-limiter measurements.
Frozen Corridor metadata marks source hashes unavailable. A separate posthoc
oracle may recompute hashes with unchanged legacy preprocessing; such results
are explicitly identified as recomputed, not historical logged hashes, and
never feed the new estimator.

## P7-B validation (100-frame prefix only)

Standalone NDT/input tests passed 2/2, P7-A catkin regression passed 7/7,
and preserved P6 reference regression passed 8/8. The unchanged legacy
STRICT_BASELINE completed 100 frames at `/tmp/p7b_legacy_reference_100`.
P7 completed one frame and then 100 frames at `/tmp/p7b_single_state_1` and
`/tmp/p7b_single_state_100`, with 100 registration calls, 93 effective full-pose
updates, seven iteration-limit prediction-only frames, and finite states.
The first ineffective P7 result is tx47. No other terminal failure category
occurred in this prefix.

Legacy first clipping and first corrected-trajectory divergence are both
tx23. The parity prefix tx1–22 has identical predictions, zero iteration or
convergence mismatches, maximum raw terminal difference 6.41412e-10 m /
2.77775e-9 degrees, and all 100 independently recomputed legacy source hashes
match P7. These are extraction/parity checks, not accuracy evaluation.

The 100-frame run measured mean prediction 0.1864 ms, cloud IO 0.5335 ms,
NDT total 48.0893 ms, alignment 41.5930 ms (P95 134.4901 ms), IKFoM update
0.02885 ms, frame total 48.8456 ms and peak RSS 52.1641 MiB. Timing is
environment-specific; no parameters were changed to obtain these numbers.
No longer prefix or full Corridor run was performed in P7-B.
