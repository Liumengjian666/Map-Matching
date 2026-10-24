# Cover letter — draft for author approval only

[Submission date]

Editor-in-Chief
IEEE Sensors Journal

Dear Editor-in-Chief,

Please consider our manuscript, **“Gate-Aware Budgeted NDT Recovery for Prior-Map LiDAR–Inertial Localization,”** for publication as a Regular Paper in IEEE Sensors Journal.

The paper examines an observation-path failure in lidar–inertial prior-map localization: a geometry predicate evaluated at an inaccurate prediction can prohibit registration and make recovery unreachable. We separate this attempt-level screening from post-registration quality admission and retain a fixed-budget pair of native normal distributions transform restarts. The native optimizer, inertial propagation, delayed filter-update mechanism, and post-registration quality thresholds remain unchanged.

The central evidence is a real inertial closed-loop causal ablation. On a complete 60-scan perturbed-initialization Newer College window, diagnostic-only geometry screening produces 60 converged registrations but no accepted map observation. Combining the same access change with budgeted restarts yields 57 accepted map observations using eight additional registrations, reducing reference-relative position/rotation RMSE from 7.986422 m/13.052649° to 0.209943 m/2.984149°. A separate historical matched-segment experiment reports an 82.2% reduction in NDT-process high-water memory, accompanied by latency and tail-error trade-offs.

The manuscript is positioned as a focused sensor-data-processing and system-integration study. It does not claim a new NDT objective, multistart principle, or EKF model. It compares its technical scope with 2Fast-2Lamaa, SuperLoc, tightly coupled prior-map localization, and Autoware NDT. Published full-sequence results are separated from the controlled ablation rather than used for an incompatible state-of-the-art ranking. The map-anchored reference, conditional pose-frame interpretation, exposed development window, initial rotation transient, and incomplete robot-deployment validation are disclosed.

We believe the measurement-admission and inertial-continuity question is relevant to the journal's sensor data processing and integrated sensing systems readership. Our claim is the experimentally isolated interaction of optimizer access and bounded recovery in this implementation; sufficient novelty and evidential scope remain matters for peer review.

[Before submission, the corresponding author must confirm: all authors approve the manuscript and submission; the work is not simultaneously under consideration elsewhere; any related prior publication/preprint is disclosed; copyright, data/software release permissions, funding, conflicts, and AI-assistance statements are accurate. No such confirmation is asserted by this draft.]

The proposed submission includes the manuscript, graphical abstract, editable source, and supplementary protocol/negative-evidence material. [Confirm the final upload set and article classification in the current author portal.]

Sincerely,

[Corresponding author name]
[Affiliation]
[Postal address]
[E-mail]
[ORCID, if supplied by the author]
