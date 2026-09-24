# P3-R8 candidate mechanism hypotheses (not conclusions)

At most these three are retained. None is a root-cause finding or an authorization for implementation.

## H1 — Initialization-sensitive registration

**Support:** Changing only per-frame NDT initialization changes raw endpoint errors on fixed source/target inputs in both tests. Corridor W1+W2 has paired baseline-minus-oracle mean improvement about 0.333 m; Floor W3+W4 has 0.891 m mean improvement. All tested runs converged.

**Contradiction / boundary:** Corridor often retains a near-reference oracle endpoint in the early window, while Floor persistent W3+W4 retains none of 119 inside 0.15 m/2 deg. Windows and failure phases differ; the same endpoint behavior is not replicated.

**Missing test:** Apply the frozen R6 three-initialization probe to predeclared Corridor persistent-phase frames. Keep identical baseline-conditioned clouds and report raw endpoints, errors, objectives, and local perturbations; no GT may be used online.

## H2 — Observation/objective-limited registration

**Support:** In Floor W3+W4, even the oracle-initialized raw endpoints remain 0.645 m mean from the reference and 0/119 satisfy the near-reference box. Fitness/probability rank is mixed per frame, so the endpoint/objective relation is not uniform.

**Contradiction / boundary:** In Corridor early-failure windows the oracle endpoint is often retained near the reference. Existing evidence does not isolate observation geometry, objective shape, or sensor quality; the clouds are conditioned on prior NDT history.

**Missing test:** Use a small frozen-cloud, phase-matched registration comparison that separates initialization effect from source-cloud/time changes and records existing runtime-observable diagnostics. This stage does not authorize Hessian, mode, or geometry-cause claims.

## H3 — Prediction-correction coupling

**Support:** Corridor R4/R5 quantify prediction error and small raw correction through parts of the early onset/maintenance period; the previous accepted NDT translation increment is reused. Floor perfect-current-increment initialization produces only a modest persistent-union gain (+0.04198 m mean) and does not recover localization.

**Contradiction / boundary:** Floor has no full predictor/carry audit. Its oracle gain is large in W4, indicating that initialization changes matter but not whether the predictor caused the accumulated error. Corridor R5 leaves initial onset unresolved and explicitly does not prove unstable feedback.

**Missing test:** An offline Floor predictor/carry/error decomposition on the already frozen R7H/R7I records, or a targeted test proving which prediction component changes the endpoint. Do not modify predictor parameters or runtime code before this.
