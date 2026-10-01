# Reference-guided tracking infrastructure

No third-party code was copied. The bounded seed/state infrastructure is a
clean-room implementation; sensor weights and innovation definitions are not
borrowed or changed.

- [hdl_localization pose estimator](https://github.com/koide3/hdl_localization/blob/master/src/hdl_localization/pose_estimator.cpp): preserves a last observation separately from the IMU prediction, considers different prediction sources, and records registration/prediction discrepancy. R4 keeps a committed-registration anchor separate from Window prediction; it never writes that anchor back as a Window state or prior.
- [Autoware NDT scan matcher](https://github.com/autowarefoundation/autoware_core/blob/main/localization/autoware_ndt_scan_matcher/src/ndt_scan_matcher_core.cpp): checks registration quality and iteration/seed-result diagnostics rather than relying on a convergence flag alone. R4 retains the existing geometric/reliability/NIS safety chain and adds an explicit ineffective-registration classification.
- [PCL NDT implementation](https://github.com/PointCloudLibrary/pcl/blob/master/registration/include/pcl/registration/impl/ndt.hpp): both a zero Newton update and reaching the iteration budget can set `converged_`. The installed PCL 1.10 implementation was also checked. R4 therefore requires a finite registration with an iteration count strictly between zero and the unchanged configured maximum. This is a tracking-wrapper classification, not a change to the NDT optimizer. It is not proof that every zero step is a localization failure.

The two-failure trigger, two-seed budget, 0.5 s extrapolation horizon and 2 s
anchor lifetime are explicitly declared engineering configuration choices for
the roughly 10 Hz input, not values claimed to be copied from those projects.
No GT was used to choose them.

## Scope and review

Normal admissible frames use the original nominal seed and common admission
chain. Abnormal frames may evaluate at most two additional registration seeds.
Each hypothesis still passes current support, reliable-rank, covariance,
nonlocal risk and selected NIS. Only the first admissible hypothesis is submitted
once to the adapter. Rejected previews allocate no observation IDs.

Recovery hypotheses use their own nominal seed as the center of the existing
positive/negative registration probes. Mixing an alternative nominal seed with
probes centered on an unrelated, drifting Window prediction was inconsistent
candidate wiring. Probe terminals now receive the same ineffective-registration
classification as nominal terminals. Current Window prediction and covariance
still determine innovation, probe perturbation size, factor preview and NIS.
The existing U_nonlocal function, covariance formula and rejection rules are
unchanged. A displaced-Window fixture verifies that a successful recovery
registration does not authorize overwriting the Window or bypassing NIS.

Core estimator source files are not modified. The reviewer checks are encoded
in the source-boundary, healthy-path parity and recovery-admission regressions.
An external cross-model review remains a manual decision-AI handoff, as requested
by the user; no external review CLI was launched.
