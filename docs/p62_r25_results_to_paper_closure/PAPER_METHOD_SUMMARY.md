# Frozen paper method and claim boundaries

## What is retained

The research method is **C+G**: an unchanged native PCL Full NDT solver, causal real IMU/EKF prediction, existing Deskew/OOSM, the P45 local target manager, a fixed post-registration quality gate, and event-only map-yaw±5deg restarts with a strict12-call per60-scan run budget. G changes responsibility: poor pre-alignment nearest-neighbor geometry is logged but does not veto optimization; corrupted/missing map data still fail closed. All post-quality checks remain active. No candidate means no map observation; real IMU continues.

G-only is a mechanism ablation, not the recommended method. F, relative D/E, Coupled and new offline seed-search designs are excluded. The retained idea is a measured system combination, not an asserted new optimizer or unprecedented multistart method.

## Implementation-accurate equations

For preprocessed scan S and prediction TP, pregeometry computes median/p90 nearest-map distances and overlap@.5. Original hard predicate is median<=1m AND p90<=3m AND overlap>=.50. G records this predicate and continues to optimization if it fails.

Divide scan azimuth into16 scanner-frame sectors. Odd sectors form8 possible quality blocks; select blocks from input counts only (>=10 points), require >=4 active blocks and >=300 even-sector training points. For an active block b:
L_b(T)=mean_p min(d(Tp,M)^2/(.5m)^2,1).
Q(T)=mean_active_blocks L_b(T).
Overlap(T)=fraction of active-block points with nearest-map distance<=.5m.

Frozen r17::quality requires finite valid native candidate, convergence, available Coverage and overlap>=.50. Frozen r20::accepted additionally requires Q<=.30. Full NDT uses the quality points too: this is Coverage Consistency Mean, NOT independent holdout.

When nominal is rejected, C optimizes seeds with R_seed=Rz(+/-5deg) R_TP and p_seed=p_TP. This rotates map-axis attitude without rotating translation. Each seed gets one Full align, maximum two extra/event and12/run. Choose accepted maximum native getTransformationProbability; ties use hypot(translation/.8m, rotation/5deg), then lower candidateID.

Important source distinction: r20::accepted uses r17::quality, not r17::base or Candidate::eligible. The .8m/5deg values in r20::distanceToPrediction are tie-break normalization scales, not hard TP-correction bounds in this retained policy. Existing separate adjacent-NDT publication limiter is .5m/5deg; EKF update limits are2m/20deg. Do not write a paper claiming a nonexistent .8m/5deg C hard gate, or claim the discontinued robust paired D>.02 selector is active. No frozen code was changed to repair narrative inconsistencies.

Pose convention T_A_B maps B to A, T_map_lidar=T_map_imu T_imu_lidar. Measurement time is scan start; apply the original OOSM historical update and IMU replay, evaluate actual output at its own time. Initialization reference is used once only. Historical predicted samples can lag the latest OOSM revision; R24 F did not solve query freshness.

## What the evidence supports

- P45: per-process memory reduction on one300-scan region, with latency/tail tradeoffs and no live tile crossing.
- R19: stable different native terminal neighborhoods and initialization sensitivity, not strict local-minimum proof or general degeneracy.
- R20/R22: simple quality/multistart can help but absolute gates and relative-only admission have concrete failures.
- R23/R24/R25: real fusion lineage and one decisive C+G versus G-only causal recovery instance. G-only gives no recovery in this instance; both optimizer access and an accepted restart matter.
- DOG R25: coordinate prerequisite BLOCKED; no new deployment accuracy or runtime claim.

The .30 threshold was developed on R19 and later frozen; the present W1 development case does not establish its universality. No independent mocap accuracy, new mathematical novelty, global relocalization, deployment readiness, or10Hz guarantee is claimed.
