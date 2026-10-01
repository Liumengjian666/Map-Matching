# TX155 numerical root-cause attribution

PRIMARY_ROOT_CAUSE_CLASS = A3E-R1-A

ROUNDED_NORMAL_EQUATION_COMPONENT_INDEFINITENESS_AMPLIFIED_BY_SCHUR

More precisely, the dominant identified error is **aggregate rounded
normal-information assembly**, not the IMU factor's theoretical PSD definition.
MINIMAL_REPAIR_FAMILY = FAMILY-3: SQUARE-ROOT / QR MARGINALIZATION.
This is an evidence-based repair-family recommendation, not an implemented repair.

Convention: Schur minima below refer to the active retained 15D block.
The full retained 600D matrix has another 585 exact structural-zero modes;
when the active minimum is positive, the full matrix minimum is therefore 0.

## 1. The negative mode is already in the rounded consumed matrix

For each normalized retained vector v, compute at 100 digits:

```text
y = H_mr v
z = solve(H_mm, y)
q = v^T H_rr v - y^T z
x = [-z; v]
q_full = x^T H_c x
```

No 600-by-600 high-precision inverse/Schur is formed. The six directional
identities and component accounting hold to better than 1e-65 absolute.

| Direction | Production double q | 100-digit q | Double minus HP |
|---|---:|---:|---:|
| negative 0 | -97.63543658 | -97.63861718 | +0.00318061 |
| negative 1 | -32.17425139 | -32.17652250 | +0.00227110 |
| negative 2 | -27.94810239 | -27.95210946 | +0.00400707 |
| smallest positive 0 | 71.91286977 | 71.91430345 | -0.00143368 |
| smallest positive 1 | 183.23952195 | 183.24106053 | -0.00153858 |
| smallest positive 2 | 214.47872962 | 214.48086323 | -0.00213360 |

These are Rayleigh quotients on production eigenvectors; they are not new
high-precision eigenvalues. The complete active high-precision Schur minimum
is -97.63861721468071708644, identical at 100 and 120 digits to the displayed
85 significant digits. Thus class B (a healthy rounded H_c damaged mainly by
the production double solve/subtraction) is contradicted.

## 2. Component accounting requires an explicit assembly residual

Define in exact-entry high precision:

```text
A = H_c - (H_prior_local + H_IMU + H_LiDAR + H_visual)
```

The capsule's separately computed diagnostic components do not add exactly
to the accumulated, symmetrized production matrix. A includes accumulation
rounding, final symmetrization, and differences from independently evaluated
diagnostic expressions. The capsule cannot attribute A to one specific
addition instruction. Its skew part has zero quadratic contribution;
the directional quantities below are unaffected by skew.

| Direction | Prior | IMU | LiDAR | Visual | A | Total H_c |
|---|---:|---:|---:|---:|---:|---:|
| negative 0 | 16.093865 | -0.002098 | 0.723723 | 0 | -114.454107 | -97.638617 |
| negative 1 | 16.740575 | -0.000198 | 1.146231 | 0 | -50.063130 | -32.176522 |
| negative 2 | 27.727947 | -0.000019 | 14.037575 | 0 | -69.717613 | -27.952109 |

||A||_F = 207.94234729, ||A||_F / ||H_c||_F = 2.54488400e-17.
Its symmetric/skew Frobenius norms are 182.70199713 / 99.29753291.
In the first lifted direction, independently recorded factors contribute
+16.815490 while A contributes -114.454107. The negative information is not
a true IMU measurement property. The Gaussian factor is J^T R^-1 J;
rounded normal-matrix entries and their accumulation can lose PSD.

The 100-digit symmetric IMU component minimum is only -0.00297409,
not the float64 diagnostic eigensolver's -640.55888. At this scale, the
absolute tiny eigenvalues returned by a double eigensolver can have errors
of hundreds. This distinction is important for attributing the large Schur mode.

Directly summing the frozen components at 100 digits, without A, yields
a Schur minimum +3.29438085912. Adding A recovers -97.63861721468.
This is the decisive aggregate-assembly counterfactual.

## 3. Conditioning and cancellation

Exact-binary H_mm, 100-digit symmetric eigensolver:

- minimum: 7,559,408.681902902;
- maximum: 2.358732100456731e18;
- 2-norm condition: 3.120260062276427e11;
- condition times double epsilon: 6.928369127915227e-5;
- frozen LDLT pivot ratio: 3.2048706862810438e-12;
- 15 positive, 0 negative, 0 near-zero pivots; jitter 0.

The original trace condition proxy is 3.1202876856952295e11; its tiny
absolute eigenvalue is less accurate than the 100-digit result.

| Spectral quantity | Value |
|---|---:|
| ||H_rr||_2 | 2.358732100456733e18 |
| ||correction||_2 | 2.358732100456731e18 |
| ||S_prod||_2 | 349971.311599817 |
| spectral cancellation ratio | 6.739787011896e12 |
| Frobenius cancellation ratio | 7.952192757883e12 |

First negative direction: a=2.182451035114599758e18,
b=2.182451035114599856e18, a-b=-97.63861718;
scalar cancellation ratio=2.235233453775e16. Other negative-direction ratios
are 6.20635748e16 and 5.22968782e16. Relative machine-scale information error
in the full system is therefore large relative to the retained weak curvature.

Small backward error alone does not bound forward error independently of
conditioning. The old float64 H residual is 2.53e-21; evaluating the stored
solution residual with exact-entry 100-digit arithmetic gives 3.10777e-17.
The double residual itself underwent rounding/cancellation. HP relative X
forward error is 1.87287e-15; y forward error is 2.96169e-12. These measured
errors and the small difference in directional q do not support the production
linear solve as the dominant source of the -97.6 mode. The condition bound is
a warning, not proof that its worst-case forward error was realized.
The whole production Schur's matrix relative difference from HP is 1.02807e-6;
the scaled-double difference is 0.00144173.

## 4. Invertible scaling is mathematically equivalent but insufficient

Use D=diag(sqrt(H_ii)), B=D^-1 H D^-1, solve in B,
then S_recovered=D_r S_B D_r. Active diagonal entries are strictly positive;
no floors/jitter/rank changes. Exact disconnected zero coordinates use
identity scaling, not a fabricated positive information diagonal.

H_mm condition falls to 135.30538433. High-precision scaling congruence
matches unscaled high-precision Schur with relative error 4.84e-89.
Nevertheless, recovered scaled-double minimum is -610.92432415,
versus production -97.63543658 and HP -97.63861721. It adds numerical
error and cannot repair information already lost in rounded H_c.
FAMILY-1 therefore does not meet its evidence criterion.

## 5. Rank sensitivity does not justify a generalized-inverse repair

All pseudoinverse calculations use 100-digit eigenspaces, not unstable
double small-eigenvalue eigenvectors.

| Relative threshold | Rank/nullity | H_mr range residual | g_m range residual | Schur min |
|---|---:|---:|---:|---:|
| eps * 15 | 15 / 0 | 0 | 0 | -97.63861721 |
| 1e-14 | 15 / 0 | 0 | 0 | -97.63861721 |
| 1e-12 | 15 / 0 | 0 | 0 | -97.63861721 |
| 1e-10 | 12 / 3 | 3.20475787e-12 | 1.92547651e-10 | -85.14054737 |

The first three oracles preserve all positive modes and reproduce the same
negative Schur. Relative 1e-10 deliberately discards three genuinely positive
eigenmodes and still does not restore PSD. This is sensitivity, not evidence
of a true nullspace. FAMILY-2 is not selected.

## 6. FORENSIC-ONLY PSD-roundoff cleanup

At 100 digits, symmetrize each raw component (quadratic-form equivalent),
then use its high-precision eigenbasis. Only negative eigenvalues within
eps_double * 30 * spectral_scale are set to zero. Positive modes are preserved
at 100 digits. Reassemble in high precision, not the old double accumulator.

- Schur of original independent component sum: +3.29438085912.
- Schur of PSD-cleaned independent component sum: +3.29793595644.
- PSD-cleaned sum **plus original aggregate assembly residual A**:
  -97.63475048898.

Thus isolated component tiny-negative cleanup changes very little; retaining
A preserves nearly the entire failure. A claim that component eigen-clamping
alone repairs production would be false. THIS IS NOT A PRODUCTION REPAIR.
PRODUCTION_CLAMP_USED=NO; no stored prior was changed.

## Decision and limits

Prioritize FAMILY-3: square-root/QR marginalization that preserves factor
information without forming/subtracting two 1e18-scale normal matrices.
This family should also avoid loss of moderate prior information during
normal-information assembly; merely changing the final LDLT precision is not
supported as a sufficient fix. No factor square-root implementation was made.

The capsule has rounded H/J-independent component products, not the original
factor Jacobians/covariances. It cannot reconstruct the exact pre-rounding
factor objective or identify a single rounding instruction. Classification A
is supported at the normal-information assembly level, with this limitation.
Class C is not selected because additional production double Schur error is
small relative to the existing negative mode, not materially causal here.
No mathematical factor-level negative information, nullspace-policy failure,
alias recurrence, or incoming negative-prior accumulation was demonstrated.

A3E_R1_TX155_NUMERICAL_ROOT_CAUSE_ISOLATED=PASS
REAL_REPLAY_COUNT=0
READY_FOR_FORMAL_EXPERIMENT=NO
