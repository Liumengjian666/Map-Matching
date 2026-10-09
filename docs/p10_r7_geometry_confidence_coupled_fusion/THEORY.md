# P10-R7 frozen development contract

START ea12a9edd757bc08bebd83f75405c28982704b13. Floor01 is development data.
Only covariance on a recommended local R6 measurement changes. All R6 model,
trigger .12m/3deg, two-second retained anchor, physical caps, score/regularized
quality, map-product second derivatives, fixed filter/noise/deskew remain.
Invalid covariance retains nominal through the old fixed-noise update.

## Coordinates and frozen heuristic

R6 Q=[W,S], sorted positive eigenvalues lambda, k=1/2, are at nominal T0.
eta=[(p-p0)/.8, Log(R R0^T)]; the SAME original nominal chart is retained at
the candidate, NOT silently recentered. Extrinsic is T_imu_lidar, and the actual
measurement is T_map_imu=T_map_lidar*inverse(T_imu_lidar).
Let theta=Log(Rcandidate Rnominal^T), phi=Log(Rpred_imu^T Rmeasured_imu).
The actual residual r=[p_measured-p_pred, Log(Rpred^T Rmeasured)] has Jacobian:

    J0 = [.8 I, -skew(Rcandidate*t_lidar_imu);
            0, J_left_inverse(phi)*Rpred_imu^T]
    J = J0 * diag(I,J_left(theta))

Leverage block equivalently +skew(Rmeasured_imu*t_imu_lidar). Reuse the existing
normalizedRegistrationToPoseResidualJacobian and permute columns. At zero
innovation and theta, this reduces to the task's J with Rmeasured_imu^T.
Test by perturbing ORIGINAL eta and invoking real lidarMeasurementToImu,
including nonidentity rotations/nonzero lever/nonzero innovation.

R0 is the actual old pose noise diag(sigma_position^2 I,sigma_rotation^2 I),
Floor01 .04m^2/.01rad^2. No new process/measurement scale or filter model.

    epsilon = 1e-6*lambda[k]        # fixed strong-relative positive floor
    m_i = clip(lambda[k]/max(lambda[i],epsilon),1,20)
    Rchart = solve(J,R0)*J^(-T)     # two QR solves, no explicit inverse
    q_i = w_i^T Rchart w_i
    Rnew = R0 + sum_i (m_i-1)*q_i*(J*w_i)*(J*w_i)^T

This adds PSD uncertainty only; the strong chart covariance block and weak/strong
cross block retain baseline values. It does NOT claim unchanged marginal strong
INFORMATION under arbitrary correlations, calibrated likelihood, or independent
anchor information. Relative heuristic weak variance exactly multiplies by m_i.
At repeated weak eigenvalues this diagonal recipe can depend on the arbitrary
eigenbasis when baseline chart noise is anisotropic. Counterexample: span(tx,rx),
lambda0=lambda1=.1,lambda2=1,zero lever/innovation and R0=.04/.01 yields output
tx/rx variance .4/.1; a45deg weak-basis rotation yields .2488/.33625. Preserve
the requested per-eigenvector rule; report degeneracy and do NOT claim a unique
basis-independent weak-subspace covariance under degeneracy.
Require sorted positive finite lambda, orthonormal Q, finite unit poses, R0 SPD,
full-rank J, relative reconstruction residual<=1e-10, finite symmetric Rnew SPD.
SO(3) near-pi failures follow existing helper guards; fail to nominal.

Rnew is frozen in the initial predicted residual tangent for ONE existing
iterated IKFoM update, exactly as the API accepts a fixed noise matrix. Internal
iterations change the residual reference without retransporting R: this is a
local first-order heuristic, NOT an exact per-iteration covariance identity.
Report actual update rotation as the tangent-change diagnostic, no IKFoM edits.

## Ordered causal experiment and bounds

directional_coupled_shadow, directional_weak_only_feedback,
directional_coupled_feedback: each all4127 scans, same six hash-verified inputs.
Control/R6V1 fixed-noise outcomes and input contracts reused and verified.
Only recommended refinements build covariance; shadow fixed-noise nominal
update must match Control source/nominal/state exactly, excluding elapsed time.
Shadow records already evaluated weak and chosen coupled candidates plus their
small covariance matrices for paired comparisons, no second optimization.
Weak-only differs only by disabling R6 strong conditional step.
Feedback through real lidarMeasurementToImu + existing covariance overload;
next scan/source/prediction derives from actual corrected state, not splicing.
Prediction-only ineffective nominal frames retained, no invalid injection.

Ordinary extra jet/value/align=0. Triggered max2/3/0; total nominal align1.
One shared NDT map/source, no second filter. Full time includes I/O, deskew,
nominal, refinement/covariance/update/logging, not just matrix construction.
Performance mean<=100ms/P95<=150ms; not worst-case guarantee.

All runtime rules/sources/binary/inputs frozen BEFORE runs, all outputs and
engineering checks frozen BEFORE FIRST R7 GT load. Same historical fixed map/GT
alignment; no refit or extrapolation, expected4126 valid GT timestamps.
Primary corrected IMU trajectory; raw measurement and paired candidate separate.
Large jump >.5m OR >10deg; goal Coupled tRMSE>=1% better than Control and stable
P95/rotation. Compare Weak-only drift relief vs R6 and Coupled vs Weak-only but
do not attribute all closed-loop differences to one strong step.
At most one source/numeric targeted repair, no multiplier tuning, no new Anchor
guards, optimizer changes, Oracle/B12/visual/Corridor, production feedback or push.
Stop after this task; no automatic R8.
