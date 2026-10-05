# R1C support-aware local branch closure

FINAL_RESULT = SUPPORT_FIXED_POINT_CONTINUATION_UNSTABLE

NEXT = DISCRETE_SUPPORT_TRANSITION_EVIDENCE

## Outcome

All14 prescribed root correctors were executed. None certified; no continuation
step followed an uncertified root. No new support-consistent local branch,
multibranch, capture interval or energy barrier was established. The new tool
implements the fixed-support FD/Newton corrector, exact support history and
IFT predictor/step policy, but the real-data predictor path was never reached.

No GT, transported W, weak grid, bound expansion, posterior probability,
production EKF, or full NDT was used. Production/stable workspace untouched.

## Numerical contract

Chart is the unchanged map product chart eta=[delta_t/.8,delta_theta], fixed
T0 W/S, k2/strong4. Energy is exact negative PCL optimizer score/source count,
not a posterior NLL. Derivatives only central FD of frozenScore, h=.001.

Predictor is dv/dalpha=-Hvv^-1 Hvu u_b. Corrector permits8 support rounds,
Newton20/round, trust .10 to .50, frozen-value acceptance only. Certification
requires exact pointwise support equality, norm(g_v)<=1e-5, raw Hvv SPD and
valid derivative blocks. No damped Hessian certifies SPD. Step starts/maxes.05,
halves failure, terminates below.005, and cannot start from a failed root.

Synthetic FD/predictor/support-flow tests PASS. Real independent directional FD
checks41/42 PASS: tx2226/P09 T0 direction0 fails gradient agreement:
absolute1.055002291e-4, relative2.6876431%. Curvature agreement passes there
(relative.1988312%). Overall real derivative FD contract: FAIL, not PASS.
Maximum gradient disagreement over42 checks1.802245010e-4; maximum curvature
absolute disagreement.090907158. Symmetry/Huv-Hvu asymmetry0 by construction,
which is not independent proof of FD precision. No h/tolerance tuning followed.

## Per-basin root attempts

Every basin: forward certified0/attempted1, reverse certified0/attempted1,
accepted0, no matched certified alpha, no recapture or barrier test. Shared
tx616 forward root: norm(g)=5.720283099e-5, self-consistent support, Hvv SPD,
trust exhausted. Closure1.96605e-6m/2.18683e-5deg. These are repeated targets,
not independent scan-frame evidence.

|basin|reverse norm(g)|reverse support equal|outer/inner|support updates F/R|reverse endpoint closure m/deg|reverse status|
|---|---:|---|---|---|---|---|
|616/P02|3.475527262e-5|YES|1/20|0/0|.000003844/.000125718|MAX_INNER_ITERATIONS|
|616/P03|2.939832900e-5|YES|2/39|0/1|.000761389/.087050805|MAX_INNER_ITERATIONS|
|616/P10|1.334005465e-4|NO|8/157|0/8|.002141421/.925967589|SUPPORT_FIXED_POINT_NOT_CLOSED|
|616/P12|2.236574987e-5|YES|1/20|0/0|0/.000002672|FROZEN_TRUST_EXHAUSTED|
|616/P13|7.797819851e-5|YES|4/78|0/3|.001763341/.104720777|FROZEN_TRUST_EXHAUSTED|
|616/P17|5.303556471e-5|YES|1/20|0/0|.000037437/.003552961|MAX_INNER_ITERATIONS|
|2226/P09|1.846106365e-4|YES|3/60|3/2|.000912667/.073866187|MAX_INNER_ITERATIONS|

P09 forward: norm(g)=1.522114589e-4, support equal, outer4/inner80,
MAX_INNER_ITERATIONS, closure.001237584m/.079134569deg.

All14 raw Hvv are SPD: smallest eigenvalues4.707029331..5.496492869,
condition numbers13.233975802..22.416951615. Support equal13/14, closure
within.02m/.2deg13/14. P10 reverse violates angular root closure too.
All14 fail stationarity. Thus no broad support-cycle causation is supported.

## Requested gates

Attempted certificate rate: forward0/7, reverse0/7. Accepted-node certificate
rate:N/A (0accepted); >=80% stability prerequisite cannot be met.

Matched-alpha separation:N/A, no certified pair. Distinct certified branch
intervals:empty. Local recapture/barrier:NOT_TRIGGERED. This is not evidence
of a single branch, nor absence of alternative basins.

Support events17 outer support updates; continued0, certified branch switch0,
unresolved17, exact support cycles0. Events are within failed root correctors,
not certified-branch causal transitions. Full signatures are archived and
hash/content equality independently checked.

EVENT_SPAWN_ATTEMPTED=NO. Spawned/merged0, max active0. Canonical discovery
recall:N/A (not attempted), not0/7 measured recall. Full refine:0 final certified
branches,0calls,0observed escapes. P09 historical escape is not rerun or treated
as an R1C escape measurement.

## Cost and provenance

Formal run:27756 frozen value evaluations,81 dynamic value evaluations,
0 full NDT calls. Dynamic count includes7 frameContext setup calls. Wall
time1.318889864s including map setup/output; per-node mean69.2349895ms,
max226.211318ms including corrector and final joint FD, not just corrector.

Before formal run,14 seed probe correctors ran once with the same numerical
parameters,0 full NDT. These development calls are disclosed separately from
formal-run evaluation totals. Exact inputs use the R1B6-asset hash manifest;
prepared source counts/FNV and chart reconstruction are checked by reused
FrameContext and independent Python audit. Two source frames only.

Initial configure/build ordering and inherited libusb library-path failures
were environment/command sequencing, not branch evidence. Corrected before
the probes. Formal algorithms exited normally. Summary initially used an
incompatible imported CSV writer signature for empty conditional tables;
fixed with an explicit header writer and regression test, then summarized
the unchanged formal evidence without rerunning optimization.

## Verification and scope

Release build PASS; P9 CTest7/7 PASS; CSV/JSON audit PASS; input/support/hash
verification PASS; product-chart reconstruction PASS; diff-check PASS.
This does not relabel real FD41/42 as a scientific PASS. Optional external
Codex CLI was not invoked; bounded independent read-only review used.

The scientific limit is strict stationarity/FD/support fixed-point closure
under the prescribed evaluator and budget. Float point-transform/FD numerical
error has not been separated from true branch geometry or solver insufficiency.
No relaxed threshold, double evaluator, alternate start, random jump or NDT
rescue was used. The support-branch model is not mathematically disproved.
