# R1C2 numerical stationarity diagnostic — predeclared rules and audit amendments

The initial FD steps and numerical-envelope rules were written before new
measurements. Subsequent stricter reference and independent-FD audit gates
are disclosed below; this is not an assertion that the entire final document
was immutable before the first measurement. No envelope/tolerance was enlarged.
Production R1C FD, corrector and certificates are unchanged. No align(), support optimization,
continuation, GT, probability or estimator is invoked.

## Expression and carriers

Use the exact archived prepared sources, target grid and **last inner-round
frozen support** for each of the 14 R1C endpoints. Recover that support by
replaying recorded round poses (value/support capture only); verify full
pointwise membership against the R1C sidecar, not only its hash. Six TX616
forward entries share one root; report 14 logical / 9 independent roots.

E = -(1/N) sum_i sum_{leaf in s_i} [-d1 exp(-d2 r^T C^-1 r / 2)],
with exactly the original guard: 0 <= d2 exp(...) <= 1 and finite.
E is the mean negative PCL optimization score, **not posterior NLL**.

FLOAT uses unmodified poseAtEta(), Matrix4f, PointXYZ transform and double sum.
DOUBLE takes the same float source/base values exactly into double, constructs
R=Exp(eta_rot) R0, t=t0+0.8 eta_translation in double, and transforms points in
double. Leaves, their ordering, covariance, constants, guard and summation
formula remain identical. DOUBLE is a continuous numerical control only.

## FD and convergence

h = [0.004,0.002,0.001,0.0005,0.00025,0.000125]. Ordinary central gradient,
diagonal second differences and four-corner mixed differences are used.
d=-H^-1 g, lambda²=g^T H^-1 g, D=0.5 lambda² require unmodified H SPD.

A stable h region has >=3 consecutive samples; adjacent displacement relative
change <=0.20 and max eigenvalue relative change <=0.10. Report FLOAT and DOUBLE
separately; absence is not silently converted into a pass. Direction at a norm
within the measured gradient envelope is labelled unresolved near zero.
Displacement relative change divides by max(||d_h||,||d_h/2||,1e-300).
Eigenvalue relative change is the maximum over all four sorted eigenvalues,
dividing by max(|lambda_h|,|lambda_h/2|). "Finest" in original-root gates means
the smallest prescribed h (0.000125), not an adaptively selected step. A
non-SPD FLOAT H anywhere in the selected DOUBLE region makes its displacement
envelope indeterminate; never use a regularized inverse to fill that gap.

At original roots, choose the longest DOUBLE-stable contiguous region (ties:
finest h). If none exists, numerical stationarity is indeterminate. On this
region define b_g=g_float-g_double, b_d=d_float-d_double. Uniform rules:

EPS_G_NUM = max_h ||b_g(h)|| + max_adjacent ||b_g(h)-b_g(h/2)||.
EPS_DV_NUM = max_h ||b_d(h)|| + max_adjacent ||b_d(h)-b_d(h/2)||.
EPS_E_NUM = 2 max_local_probe |E_float-E_double| + rho_E.

Subtracting the DOUBLE series before measuring spread isolates float noise
from shared O(h²) truncation. The probe set is root and +/-0.001 along each
strong axis. rho_E = 64 epsilon_double max(1,|E|) (M+32), M = frozen leaf-term
count: conservative accumulation/arithmetic resolution, not a fit to pass.
These envelopes are deterministic sensitivity bounds, not confidence intervals.

## Fixed-support reference minimization

Only v varies, u and support stay fixed. Max 100 iterations, trust initially
0.10 / maximum 0.50; deterministic halving line search, strict actual DOUBLE
energy decrease. Use Richardson-extrapolated central FD at h=0.000125 and h/2
to remove leading O(h²) bias; compare against the extrapolate at h/2 and h/4.
No dynamic derivatives. Do not regularize H to claim SPD.

Reference termination requires both extrapolated raw H SPD, eigenspectrum
agreement <=10%, and for both extrapolates D <= rho_E and
||d|| <= sqrt(2 rho_E / lambda_min(H)); their displacement disagreement must
also be below that resolution. The latter follows
from a quadratic energy change below double arithmetic resolution. Independent
FD uncertainty is also reported from the two extrapolates. This is a
resolution-limited reference, not an exact stationary-point proof.

## Numerically resolved original-root certificate

Requires exact support equality; all DOUBLE-region H SPD; stable DOUBLE FD;
successful reference termination; original-root finest DOUBLE displacement and
actual root-to-reference displacement <= EPS_DV_NUM; finest DOUBLE predicted
decrement and measured root-to-reference energy decrease <= EPS_E_NUM;
finest FLOAT ||g|| <= EPS_G_NUM. No recentering is fed to production.
Original FLOAT h=.001 SPD/finite derivatives remain independently required.
For a complete certificate, independently directed FD must additionally pass
the **unchanged R1C audit criteria** (gradient abs<=1e-4 OR relative<=.02,
curvature abs<=.02 OR relative<=.05): FLOAT at original h=.001 and DOUBLE at
finest h=.000125, three original deterministic six-dimensional directions.
Their numeric-floor classification and complete certification are separate.
P10 reverse cannot pass while its original support equality fails, irrespective
of gradient calibration. Floating residuals at the projected reference are
reported separately from original-root certification.

Recompute the independent directional FD test at the original TX2226 T0 as
well as endpoint roots, so a root recentering does not disguise that old failure.
Report ordinary multi-h stability at reference endpoints too; when relative
changes near zero are unresolved, do not claim raw relative stability. The
reference termination instead uses the explicitly stated resolution test.

## Reference precision audit amendment (not a certificate-tolerance change)

The conservative roundoff bound can stop a reference while an actual DOUBLE
Newton trial still measurably lowers E. Before accepting the reference
termination, also require E(v+2^-l d) >= E(v) for every l=0,...,15;
otherwise continue the same Newton
iteration. EPS definitions, FD steps, trust policy and all original-root
classification limits remain unchanged. This additional *stricter* gate is
uniform over every root, and prevents a resolution-limited original point
from masquerading as a tightly minimized double control. Failed/budget-limited
references remain indeterminate. The initial resolution-only diagnostic is not
the formal archived reference run.
This tests the stated Newton proposals only, not all possible descent
directions. Under the first conservative reference, four independent roots
(TX616 forward, P03/P12/P17 reverse) accepted zero steps; their double gradient
norms were respectively 9.813e-6, 2.039e-5, 2.393e-5, 2.591e-5. This is the
reason for the stricter control, not for enlarging a certificate threshold.
