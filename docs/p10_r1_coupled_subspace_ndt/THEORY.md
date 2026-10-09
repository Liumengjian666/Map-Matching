# P10-R1 coupled subspace NDT prototype

This is algorithm development on the existing P9 GROUP A cohort, not an
independent held-out test or actual localization success evaluation. Historical
P9 outcomes remain unchanged. No production EKF/IKFoM path is modified.

## Shared objective and chart

Reuse `ExactPclNdt`, `FrameContext`, `poseAtEta`, `strongNativePullback` and
`iterativeNewton` from the existing P9 code. Energy is the exact dynamic PCL
score with its original sign and source normalization: `E=-S/N`. It is not a
calibrated likelihood. The nominal LiDAR terminal T0 anchors the same map
product chart: translation is `t0+0.8*eta_t`, rotation is
`Exp(eta_rotation)*R0`. Archived W/S are fixed; `eta=W*u+S*v`.

At the preceding unrefined fixed-u state, use a joint native pullback in basis
`[W,S]`, including the gradient-times-second-chart-derivative terms. Partition
the resulting Hessian into Hvv and Hvu, solve

`Hvv*delta_v = -Hvu*delta_u`, `v_pred=v_previous+delta_v`.

A finite positive definite block with condition at most 1e8 is required for
the raw LDLT solve. Report its original residual before clipping. The strong
predictor move is capped at norm 0.10, inherited from the existing Newton trust
scale. Invalid/singular/indefinite or nonfinite solves explicitly fall back to
the warm-start coordinates. Predictions are local first-order models, not
branch certificates. Conditional correction holds u fixed and reuses the
unchanged Newton20 solver and its actual dynamic-energy line search.
In particular, predictor-only C need not have a preceding conditional
stationary state. Record the preceding strong-gradient norm, do not claim an
implicit-minimizer tracking theorem, and do not silently add a gradient
correction term while calling C prediction-only.

## Non-oracle weak search and four controls

The historical R1A experiments used oracle u_b, even in ZERO_V rows. The R1B
beta path used oracle v_b. Neither is a permissible proposal source here.
The new search process does not accept canonical poses, target coordinates,
oracle IDs or GT files. It only receives frozen source/cohort, map and U_obs.

Use the original 17-point grid for k=1 or 9x9 grid for k=2, the original
coordinate box and the combined weak displacement limits of 2 m/15 degrees.
Start with u=0, then visit the nearest remaining weak coordinate; ties use the
original grid index. The same ordered pool is used for every method.
Each method has an independent state initialized at nominal u=v=0.

- A: weak-only, v=0 at every node, no conditional correction.
- B: previous fixed-u strong endpoint as warm start, then Newton20.
- C: coupled predictor from its own preceding fixed-u state, no correction.
- D: coupled predictor followed by the same Newton20 correction as B.

Every endpoint receives one standard full NDT refinement. The refined terminal
is an output only: it never feeds the fixed-u path state or changes proposals.
One nominal T0 refinement per frame is an explicit ordinary-NDT reference.
The original NDT parameters are 0.8 m/0.08/1e-5/80, outlier ratio 0.55, with
unchanged source/target preprocessing and map. Source count/hash is checked.

B/D have equal weak-node and full-NDT-call budgets and equal 20-iteration
conditional correction caps. Newton20 is not 20 objective evaluations: its
line search can use up to 181 evaluator calls. D additionally spends a joint
predictor jet. Record actual evaluator counts and timings; do not describe the
methods as equal CPU cost or hide the predictor overhead. Pattern100 is read
as a historical control implementation, not added as another experiment arm.

## Evaluation and development boundary

Freeze candidate outputs before a separate evaluator reads the seven closed
canonical targets: 616/P02,P03,P10,P12,P13,P17 and 2226/P09. Recovery uses the
existing <=0.2 m AND <=2 degree proximity convention and successful full
refinement. Also preserve pre-refine recovery, distances, branch/pose changes,
strong coordinate history, raw score/energy, failures, actual budgets and
stage timings. Overlapping canonical neighborhoods may admit the same
terminal to multiple IDs; ID recall is not a count of independent discovered
attractors. No GT is loaded.

Development target is D recovery >=5/7. Beating warm-start at comparable budget
is desirable but must be established from actual results, not assumed.
At most two subsequent targeted revisions are authorized by the task; each
must have a concrete reason, preserve the original outputs and record its
changed setting and all four comparisons. Stop immediately when the target is
met or the bounded revisions are exhausted. No Corridor01 bootstrap, visual
module, probability modeling, production state correction or new global
optimization is introduced.
