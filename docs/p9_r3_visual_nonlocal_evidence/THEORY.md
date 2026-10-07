# P9-R3: map-independent visual-motion evidence

## Frozen scope and information boundaries

DUAL-U remains within-basin observability plus inter-basin/competing-terminal ambiguity. R2B's formal failure remains unchanged, with its basic discrimination signal retained as LiDAR-side auxiliary candidate evidence. This experiment tests viability of a **map-independent visual-motion evidence** channel, not sensor-independent vision or pure monocular evidence: it uses P4's calibrated reference LiDAR depth.

Start commit4957015ce21c5c9071f17746c172f6fe13a9bde6, branch research/p9-r3-visual-nonlocal-evidence. No stable/core modification, new NDT, pose switching, estimator updates, probability, measurement covariance or fusion. All32 targets are defined by the committed R2A nominal table, not labels. No canonical pose/ID or GT enters construction.

The standalone blind process projects the legacy adjacent CSV immediately to a measurement-only field allowlist, discarding GT/error columns. First gate is exact transaction_cur matching and12/32 VALID aggregate parity. To honor evidence-before-label isolation, expected2/9 major and10/23 NO_MAJOR parity is verified only after all evidence files freeze, before classification. A discrepancy stops interpretation as VISUAL_ARCHIVE_ALIGNMENT_FAIL, not a scientific negative.

## Visual frontend and availability

Reuse P4-I3 frontend and data-reader code without modifying parameters: official Floor01 MEI rectification; Shi-Tomasi500/.01/10; KLT21x21, level3, termination30/.01; FB<=1px; reference LiDAR depth projection association<=2px; EPNP RANSAC100/2px/.99; >=30 correspondences, >=20 inliers, LM refinement. OpenCV single thread. Seed is current transaction ID, identical to P4 adjacent convention, independently of lag/labels/candidates. All four lags1/2/4/8 are attempted and archived; no tuning or further lag is allowed.

Pairing uses the committed P4 sync table: nearest camera header stamp within20ms; validate actual image headers and request scan_end/cloud-end frame, preserving P4 temporal approximation. No GT interpolation, new propagation or time correction. Reference clouds are the same captured P4 request clouds, not map points. Images/clouds may be cached for diagnostic I/O, not for different frontend rules.

Use smallest VALID lag, not best residual/inlier/GT lag. Missing visual stays missing, not U=0. Availability never enters the classifier. Gate requires at least6/9 available major and14/23 available NO_MAJOR. All32 rows, including missing measurements, remain archived.

## Pose/transform convention

P4 PnP maps reference camera points into current camera: T_Ccur_Cref. Visual IMU increment is

`D_vis = T_IC inverse(T_Ccur_Cref) inverse(T_IC) = inverse(T_map_imu_ref) T_map_imu_cur`.

Synthetic tests use a nonidentity reference, camera lever arm and a known current IMU pose, including LiDAR-to-IMU conversion. Existing P4 PnP/MEI sanity is called directly.

All NDT T0/representatives are map-to-LiDAR, not map-to-IMU. Their validated float pose matrices are lifted to normalized rigid rotations using the same geometry carrier as R2B; Floor01 official T_IL/T_IC raw calibration semantics stay exactly P4. Convert `T_map_imu = T_map_lidar inverse(T_IL)`. Reference T_ref is the existing R1 same-objective registration raw nominal at transaction t-L, in the same map frame. The historical R2A manifest has no whole-registration hash, so all used raw reference/target xyz-quaternions and stamps are checked against R1 dual_u.csv whose SHA256 was previously frozen in R2A; do not silently treat a new registration hash as historical proof. Then freeze the whole registration file prospectively. Validate all32 R1 targets against R2A T0 within1e-5m/1e-4deg and record reference states/source hashes/timestamps. Camera identity is also checked against P4's archived frame `d`, sensor_msgs/Image/bgr8/640x480. No GT reference pose or objective-best replacement is used.

## Candidate and primary evidence contract

Load only R2B B12 competitive representatives, whose hashes were frozen before labels in R2B; retain T0. Exact score competitiveness is inherited, not recomputed/tuned. Additional strict center eligibility is `translation(Tk,T0)>.2m OR rotation(Tk,T0)>2deg`, the existing neighborhood contract. Inside-center representatives are archived but excluded from visual competing set. No oracle basin admission is evaluated.

For an available measurement, `D_k=inverse(T_ref)T_k`, `E_k=inverse(D_vis)D_k`, `r_t(k)=||translation(E_k)||` meters. Rotation geodesic degrees is secondary, without a mixed score. `r_nom=r_t(T0)`; if eligible alternatives exist, `r_alt=min r_t(Tk)`, `G=r_nom-r_alt`, `U_visual=max(0,G)`. With available visual and no alternative, U=0 and G=0 by empty-set convention, while r_alt remains blank. With unavailable visual, all residual/evidence scalars remain blank. Exact residual ties use lowest local non-oracle cluster ID. No quality-derived classifier or weighting.

## Predeclared evaluation

Freeze SHA256 of adjacent audit, four-lag measurements, candidate set/reference states, selected measurements, candidate residuals and all32 non-oracle evidence rows before opening the H1 frame-label table. Then verify adjacent label parity and multi-lag coverage. If coverage fails, stop primary classification; no misleading small-subset ROC, GT or complementarity rescue.

On sufficient visual-available coverage: U_visual alone, frame-level ROC with averaged exact ties; one-sided AUC label permutation10000 replicates, PCG64 seed20261012, preserving observed class counts; p=(1+#null>=observed)/10001. LOFO threshold maximizes training-only balanced accuracy, higher threshold breaks exact ties, prediction U>=threshold. No missing-as-zero and no availability feature.

Primary success requires AUC>=.80, p<.05, pooled LOFO balanced accuracy>=.75, and every delete-one-major AUC>=.70. This is a new visual preregistration, not a change to R2B's earlier nested BA certificate. If basic discrimination passes but major-deletion AUC fails, classify COHORT_DEPENDENT. Otherwise adequate coverage/basic failure is NOT_DISCRIMINATIVE.

Only after gate snapshot freezes may post-hoc GT compare the translation-residual-minimizing candidate against T0 on U>0 frames. Use prior frozen map/GT closure, no fitting or correction; report improved/same/worse only. No pose replacement is authorized. Only primary PASS permits Spearman and complementary quadrants with R2B U_comp. For this secondary descriptor, “high” means strictly positive evidence and “low” means zero, fixed here without label-derived thresholds. No fused scalar.

## Cost and verification

Report P4 component timings per lag and4-lag sum. Pair total includes both rectification costs conservatively, matching P4; extraction/hashing/bag I/O wall time is reported separately. Serial smallest-valid early-stop cost estimate sums attempted prefix pair costs through chosen lag (all four on unavailable frames); no actual early stopping in this diagnostic run.

Release build/P9 tests, original P4 MEI/PnP sanity, new IMU/LiDAR direction/availability/missing tests, CSV/JSON/hash audit and explicit-path diff/commit gate. Unrelated untracked files stay outside staging, tests run only in the build directory, no push.
