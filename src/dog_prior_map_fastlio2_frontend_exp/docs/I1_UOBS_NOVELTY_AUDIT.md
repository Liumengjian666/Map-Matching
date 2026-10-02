# I1 U_obs novelty audit and research gate

Base: `739079facf5a298d160b7efc643d17b3839ef975`. Date: 2026-10-02.
Evidence and exact code pins are in [architecture audit](I1_MATURE_ARCHITECTURE_AUDIT.md).
The object assessed is the **present U_obs mathematical detector**, not a
hypothetical future visual fusion method. U_obs and all P7 history are retained.

## Decision

`NOVELTY_RED_FLAG=YES`

`UOBS_NOVELTY_AT_RISK=YES`

`UOBS_NOVELTY_CANDIDATE=NO` **for advancing the present detector as a distinct
primary contribution now**; this is not a proof that every possible U_obs
research direction is exhausted.

`RESULT=NOVELTY_AT_RISK`. Stop before new production integration and A/B under
the task's explicit novelty gate. The lack of a 2026 SKF PDF is **not** the
reason. Nor is lack of already-demonstrated performance a novelty disproof.
The reason is that the existing detector's identifiable differences remain
implementation/metric choices inside established information/subspace methods;
no defensible additional property has been established for this implementation.
No single paper is claimed to contain an identical P7 implementation.

## What is actually being compared

Current code computes a weight-averaged geometric proxy, not the exact PCL
score Hessian and not posterior covariance:

`H_phys = sum_i w_i J_i^T Sigma_i^-1 J_i / sum_i w_i`,
`J_i = [-skew(R p_i), I]`, `D=diag(I,L I)`, `H_bar=D^T H_phys D`.

`L=0.8 m` equals the configured NDT resolution parameter. The actual target-grid
leaf size is 1.0 m in the checked PCL lifecycle; see the reproduced result in
[code map](I1_CURRENT_CODE_MAP.md). Covariances already include PCL's 0.01
relative eigenvalue floor, then P7 applies its safeguards. Pair multiplicity
and regularization mean this proxy must not be called calibrated Fisher
information or sensor-independent physical observability without validation.

## Closest-method comparison

Labels concern the specified component, not entire-system equivalence:
**SAME** = same mathematical operation; **SIMILAR** = related construction with
different model/coordinates; **DIFFERENT** = an evidenced difference;
**UNKNOWN** = inaccessible/unverified, never silently counted as DIFFERENT.

| U_obs component | SKF 2024 preprint | DCReg code | X-ICP / released comparator | FMCW T&R 2026 | SA-LIVO 2026 | Frame-equivariance 2026 | LF-GICP 2026 |
|---|---|---|---|---|---|---|---|
| Weighted geometric `J^T W J` | SIMILAR, plane noise | SIMILAR, registration normal equations | SIMILAR, feature constraints | SIMILAR, modeled point/normal noise | SIMILAR, plane uncertainty | SIMILAR, point-plane form | SIMILAR, GICP information |
| Target NDT voxel covariance inverse | DIFFERENT, not this proxy | DIFFERENT from audited detector | DIFFERENT | DIFFERENT | SIMILAR Gaussian plane geometry, not same residual model | DIFFERENT | SIMILAR covariance-weighted registration, different detector |
| Radial weight and weight average | DIFFERENT implementation | DIFFERENT | DIFFERENT | DIFFERENT weights | DIFFERENT weights | DIFFERENT | DIFFERENT |
| Fixed physical L | DIFFERENT marginal-unit thresholds | DIFFERENT separate block characterization | DIFFERENT checked comparator | DIFFERENT adaptive Schur-derived scale | UNKNOWN equivalent fixed length not established | SAME characteristic-length metric family | DIFFERENT normal-field criterion |
| `D^T H D` | UNKNOWN as a fixed-L operation | DIFFERENT selected characterization | UNKNOWN fixed physical form | SAME congruence operation; DIFFERENT scale rule | UNKNOWN fixed physical form | SAME equivalent generalized metric operation | UNKNOWN fixed physical form |
| Joint mixed 6D eigenspace | DIFFERENT: block marginal bases | DIFFERENT Schur 3+3 bases | SAME in solution-remapping comparator, not all original X-ICP | SAME full scaled 6D spectrum | SAME full joint information spectrum | SAME subspace-level analysis | SIMILAR 6D direction in weighting, 3D detector |
| Retaining cross rotation/translation effects | SIMILAR inverse includes coupling | SIMILAR Schur includes coupling | SAME in full-6D comparator | SAME full-6D form | SAME full-6D form | SAME full-subspace representation | SIMILAR full Hessian weighting |
| Weak/reliable bases | SIMILAR marginal selection | SIMILAR per-block characterization | SAME eigenspace partition idea | SAME eigenspace partition idea | SIMILAR soft directional gates | SAME subspace object | SIMILAR directional weights |
| Current-frame geometric support | SIMILAR current scan/map | SIMILAR current registration | SIMILAR current registration | SIMILAR live-to-taught submap | SIMILAR current geometry plus visual history | SIMILAR current correspondences | SIMILAR current field plus temporal detector history |
| Prior-map localization context | DIFFERENT LIVO SLAM | SIMILAR registration; full carrier differs | SIMILAR registration | SAME prior taught-map localization setting | DIFFERENT odometry/SLAM | DIFFERENT pairwise analysis scope | DIFFERENT local-map odometry |
| Low-compute same-grid implementation | UNKNOWN direct P7 comparison | DIFFERENT backend | DIFFERENT backend | DIFFERENT backend/sensor | DIFFERENT low-cost filter carrier | DIFFERENT analytical study | DIFFERENT voxel implementation |

The 2026 SKF publisher preview supports covariance-based coupling-aware
detection and directional selective fusion. Its exact formulas, fixed-length
handling and changes from 2024 remain UNKNOWN. The SKF formula cells above are
explicitly **preprint evidence**, not journal-equation verification.

### SKF: real difference, limited inference

The preprint obtains rotation/translation marginal covariances by inverting
coupled information, then uses separate 3D eigenbases. U_obs retains mixed 6D
vectors directly. Thus “SKF ignores coupling” is false, and “SKF already has
the identical U_obs eigenbasis” is also unsupported. This difference does not
establish novelty against other joint-spectrum methods.

### Fixed physical scale: algebraic overlap

For nonsingular D, let `x=D v`. Then

```text
D^T H_phys D v = lambda v
             <=> H_phys x = lambda (D^-T D^-1) x
M = D^-T D^-1 = diag(I, L^-2 I).
```

Multiplying M by `L^2` gives `diag(L^2 I,I)`, the characteristic-length family
explicitly considered in [Degenerate in Whose Frame?](https://arxiv.org/abs/2608.15532v1).
That global positive factor rescales eigenvalues, not relative partitions
outside numerical-floor effects. This is our algebraic comparison, not a claim
that the paper used NDT or proved P7 identical. Fixed physical scaling is not
min-max normalization, but that distinction itself is not a new spectral method.

Similarly, division by positive `sum(w)` does not change eigenvectors or
relative eigenvalue ratios. Averaging alone cannot explain different partitions
from an otherwise identical weighted sum, except near absolute numerical floors.

The frame paper concerns body-frame/reference-point changes. P7 uses rotation
about the LiDAR origin plus additive map translation: a pure shift of the **map
origin** does not change `Rp` or this Jacobian. Do not import an adjoint-origin
counterexample without the correct chart transport. Changing the **physical
reference point of the pose increment** is the relevant unresolved issue.
Unit-conversion tests must also convert absolute covariance floors, not only L.

### Most relevant additional overlaps and limits

[FMCW T&R, Section III-E2](https://arxiv.org/html/2603.10248v1) already combines
weighted prior-map information, scaled full-6D eigenanalysis and reliable-space
remapping. Its Schur-derived scale is frame-adaptive, unlike P7's fixed L. Its
Doppler input is absent here; it is neither an identical detector nor a drop-in
carrier. Equation 45's printed covariance unscaling is not blindly adopted:
covariance congruence requires the corresponding transpose on both sides.

[SA-LIVO, Section VII-B–VII-E, equations 33–37](https://arxiv.org/html/2606.25699v1) gates joint information
and its vector in a 6D basis before inertial estimation. Thus “joint 6D + filter
integration” is not by itself a defensible new contribution. It includes visual
information and is not evidence for this P7 noise/chart adaptation.

[LF-GICP](https://arxiv.org/abs/2608.19522) identifies how covariance-weighted
registration can retain in-plane information even in geometrically weak scenes.
This motivates a test of U_obs; it does **not** prove failure of P7's different
6D classifier or transfer LF-GICP's numerical ratios to it. Changing PCL's
eigenvalue floor alone is not established as a remedy.

DCReg's Schur characterization and X-ICP/LOAM-style constrained/remapped updates
are mature prior art. SuperLoc already targets prior-map alignment risk;
FAST-LIVO2/COIN-LIO already demonstrate resource-conscious current-state
geometry-driven fusion. Their detectors/carriers differ, but these application
labels cannot supply the missing mathematical contribution.

## Smallest surviving research question (not implemented)

**Does a run-fixed physical metric preserve useful changes of coupled weak
directions that frame-adaptive block balancing hides, when correspondences,
covariance regularization and downstream use are held identical?**

This is a falsifiable question, not a claim of novelty or effectiveness. It
would require controlled geometry and independent weak-direction reference,
reference-point/unit tests, and a real degenerate sequence with independent
map/test provenance. “Choose L equal to a software parameter” alone provides
no demonstrated invariance, statistical calibration or detection benefit.
Do not replace U_obs by the frame paper's metric and call it our invention.

U_obs is worth retaining as a research asset. Advancing the current formulation
as the primary contribution is **not approved by this audit**. No new detector,
visual frontend, filter gain, NIS gate, recovery or threshold was invented.

## Review and verification

A fresh-context adversarial reviewer checked the novelty inference and chart
assumptions. Required corrections incorporated: scope the stop to the current
claim; distinguish map origin from pose reference point; account for averaging
cancellation; describe LF-GICP as a risk, not proof; do not treat a carrier as
a specified downstream mechanism. Final factual review also corrected the
SA-LIVO section reference and distinguished five frames from five observations.
Cross-model review was offered separately; no selection/authorization had been
received at this checkpoint, so no external CLI was invoked.

Existing binaries/tests rerun with `LD_LIBRARY_PATH` unset:

| Check | Result | Meaning |
|---|---|---|
| P7 standalone build | PASS | Existing targets build; no new production target |
| `build/p7_b` CTest | 8/8 PASS | Includes U_obs, NDT, replay, remapping, admission/reporting |
| Main `build` CTest | 7/7 PASS | Includes registration FD, propagation, deskew, runtime |
| `build/p7_a_research_all` CTest | 8/8 PASS | Existing P6 research regressions |
| Installed PCL lifecycle probe | 1.0 m grid / 0.8 configured | Reproduced baseline contract issue, not fixed |
| `git diff --check` / staged equivalent | PASS | Documentation-only change |
| Protected packages versus start SHA | EMPTY | No dog localization/interface source edits |

Stable dog workspace HEAD remains `41999ea700c66c4cadf0eca9e0c5d73caa2783fd`;
it and the rescue directory were not edited. Only the three I1 Markdown audit
documents are staged for the single research commit. No production dependency,
input asset, calibration, numerical parameter or test expectation changed.

I1 implementation, new candidate unit tests, staged replays, GT evaluation and
AB performance tables are **NOT RUN** under the novelty stop gate. Historical
P7-D/E runs are not relabeled I1 results. No RMSE, resource benefit, false-alarm
rate or scientific performance conclusion is manufactured from this audit.
