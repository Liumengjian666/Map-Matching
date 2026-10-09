# P10 Corridor01 failure-onset paired benchmark protocol

## Scope and claim boundary

This run performs only Experiment A, a fixed common-predictor, per-frame paired
scan-to-map comparison. It is not a pair of independent closed-loop runs, not
an IKFoM trajectory, and not an absolute map-to-GT accuracy result. Experiment
B (causal feedback) was not run because the historical P2B derived bag and
frozen Run C outputs required to reuse its trusted localization chain are not
present in this execution environment. The task forbids substituting the
previous P10 gyro/constant-velocity diagnostic as a long-term inertial chain.

The fixed window is the first 35 seconds after the frozen P2B evaluation origin
`1517157224.188979 s`. Selection is label/score blind and uses complete P9
scan intervals only: TX52 through TX397, 346 scans. TX51 crosses the origin and
is excluded. The first included scan ends at +0.143537266 s; the last ends at
+34.938102467 s. The common predictor is the historical P10 NOMINAL branch's
per-frame non-GT `prediction` record. Its sequence is not asserted to be a
trusted P2B inertial trajectory.

## Paired execution

For every selected transaction, the runner streams one frozen raw timed scan,
uses the frozen causal rotational deskew and frozen normalized Corridor01 map,
and invokes one ordinary PCL NDT alignment from the same predictor pose. It
requires exact prepared-source count/hash parity and strict historical nominal
pose/status/score replay parity before accepting the row. The same resulting
nominal object, map, source, timestamp, predictor and reconstructed nominal
control Anchor snapshot are passed to the unchanged R6 Weak-only and Coupled
refiners. No additional complete NDT alignment is run for either refinement.
The Anchor is reconstructed causally from prior historical nominal
`executed`/`prediction` rows; it is not represented as an archived R6-arm
Anchor state.

The R6 trigger is recomputed from the frozen common predictor and nominal
measurement: translation innovation `>0.12 m OR` rotation innovation `>3 deg`.
Untriggered rows perform zero extra jets/values. The frozen R6 search,
quality, weak/strong step bounds, objective and PCL implementation are not
modified. A non-GT `recommended` candidate is the method's selected measurement;
otherwise the method falls back to nominal. The raw proposed alternative is
retained in the candidate CSV for post-hoc diagnostics but is not counted as a
selected output when the frozen non-GT policy rejected it.

## Input and source identity

The source is `P9_CORRIDOR01_RAW_SCANEND_V1` from the existing read-only
external dataset directory. The experiment freeze rehashes its complete
historical receipt set, the scan-end map, calibration, old P10 frame and source
ledger records. Each paired frame checks the actual prepared count/hash against
the old P10 same-objective frame record. The complete per-frame checks are in
`source_parity.csv`; all 346 pass. Historical nominal status, pose and raw
score are also replayed against the archived common predictor record.

## GT evaluation

Candidate/source/nominal evidence was SHA-frozen before the evaluator opened
GT (`evidence_freeze.json`, `GT_LOADED_BY_EXECUTION=false`). The evaluator then
verified the frozen GT SHA, interpolated at the actual scan-end sensor
timestamp without extrapolation, and converted each `map_T_lidar` to
`map_T_imu` using the inverse of the frozen laser-to-IMU extrinsic. It fits one
proper rigid position Kabsch transform from Control nominal samples in the
first 10 seconds, with unit scale, then applies that exact transform to all
three methods and all times. The metric name is
**PREFIX-ALIGNED RELATIVE DRIFT**. It does not assert absolute map/GT frame
closure. The first-10-second fit residual is reported as a diagnostic and is
large, so its onset values cannot be interpreted as the historical P2B failure
onset.

## Historic result and causal blocker

The archived P2B failure-onset result remains unchanged: persistent `0.5 m`,
`1 m`, and `2 m` crossing at +11.2375 s, +13.1537 s, and +16.4819 s. It came
from the historical P2B localization/evaluation chain. The present paired run
has first strict ordinary-NDT non-success at +10.531506283 s (TX155), but only
two consecutive non-success rows and no five-second persistent strict-NDT
failure. Its prefix-aligned relative position error is already above 0.5 m at
the first included sample and its Control prefix-fit RMSE is 5.8036 m. These
are material indications that the common P10 diagnostic predictor is not the
historical P2B baseline; comparing the numeric onset values across these chains
would be invalid.

The existing evaluation audit states that the P2B estimate is converted with
the fixed laser-to-IMU extrinsic and GT is interpolated on sensor time. In the
current environment the corresponding historical derived bag and Run C
`result.bag` / `ndt_determinism.csv` are absent. No causal feedback result or
causal localization accuracy claim is made.
