# I2 U_obs falsification

## Preregistered protocol (before detector results)

Start: ca8d1fc267a4a1e5d7d5f42aa9980b0886423f70. Only runtime change:
PCL resolution-before-target order; commit 664a28a. No parameter changes.
Grid contract regression was RED with the old order and GREEN with the fix.
Standalone 8/8; 1/20/100 FULL_POSE replays finite. The 100-frame result is
98 SUCCESS, 2 iteration limits, 98 valid Uobs, weak dimensions 3:49 / 4:49.
Outputs: /tmp/i2_grid_fix_{1,20,100}. These are behavior checks, not accuracy claims.

### Analytic references

Coordinates are [map rotation, map translation/L], L=0.8 m. Reference is
the continuous surface symmetry, NOT an eigenvalue label from any detector.
Deterministic samples represent local surfels, not finite patch boundaries:

* Plane z=2: weak span {rotation z, translation x, translation y}, dimension 3.
* Parallel walls y=+/-2: weak span {rotation y, translation x, translation z}, dimension 3.
* Three orthogonal planes (corner): no continuous rigid symmetry, dimension 0.
* Circular tunnel along x without end caps: weak {rotation x, translation x}, dimension 2.
* Rectangular corridor with floor/ceiling: weak {translation x}, dimension 1.

Samples cover +/-5 m axially, tangential variance 0.05 m^2, normal variance
0.0005 m^2 (1% ratio, consistent with PCL's covariance inflation scale).
Positive tangential covariance creates information absent from ideal sliding
surfaces; this is an explicit falsification target, not hidden ground truth.
All methods receive exactly the same observations/H_phys. Also rotate the
whole scene deterministically and move the increment reference point.
Transition: rectangular corridor plus end-wall observations with weights
1, .1, .01, .001, 0. Exact symmetry is absent for every positive weight;
near-degeneracy is reported as a spectrum trend, not scored as exact nullity.

### Comparators and scaling

1. DCReg characterization adapter: call upstream DetectDegeneracy and
CharacterizeDegeneracy directly, ce7db8220f549a4a4391729e3bf4de4d4ab74635,
default condition threshold 10. H_phys replaces its ICP objective only for
the explicitly labeled shared-information characterization comparison.
No solver or preconditioner is applied. Failed factorization is unavailable.
The two marginal Schur bases are zero-padded into 6D, exactly as its block
characterization, NOT conditionally lifted. Its native-threshold results
are descriptive and cannot by themselves establish a representation advantage.
2. FMCW teach-and-repeat full-spectrum block scaling, arXiv:2603.10248v1,
equations 32–38. No matching implementation pin was located in I1; this is
a small equation reproduction, NOT an official-code reproduction. Schur
dominant-eigenvalue length, full 6D spectrum, official ratio gamma=80.
Its ICP objective is NOT claimed equivalent to NDT: compare only the
representation operation on shared information. A separate paired scaling
ablation uses the SAME 0.05 ratio for fixed and this adaptive metric, so a
threshold difference cannot be credited as a scaling advantage.
X-ICP original localizability/absolute-threshold branches remain
OBJECTIVE_DEPENDENT references, not a third quantitative comparator.

Weak vectors are transported back into the common fixed-L chart and
orthonormalized for comparison only. Metrics: all principal angles where
defined, Frobenius projector distance, dimension excess/deficit; report
dimension mismatch alongside angles (containment is not correctness).

### Robustness

Metres to centimetres converts points x100, covariances/floor x10000,
and L x100. Compare weak projectors in the same dimensionless chart.
Reference-point shifts c=(0.1,0.2,0.3), (1,2,3) m use p_new=p-c and
T_old_from_new=[[I,0],[skew(c)/L,I]]. This is not a map-origin translation.
Compare transported subspaces without modifying the detector's metric.

### Real-frame reference and limits

Freeze tx1..100 from /tmp/i2_grid_fix_100, including rejected frames in
availability counts. Use successful raw terminals only for curvature.
No frame selection by Uobs output or GT error. Same prepared cloud, map,
target grid and terminal for every detector. GT is not opened by this audit.
An offline-only translation unit reuses current_frame_ndt.cpp internals;
it is NOT linked into production and avoids copying its preprocessing or
observation extraction. It records exact source hash and geometry parity.

Independent reference: central finite differences of the actual scalar PCL
NDT score, with map-spatial rotation Exp(delta)*R and t+L*delta_t. Use h=.01
and h/2=.005, no search for a favorable step. Negative score is the cost.
Require finite curvature, relative Hessian discrepancy <=0.1, PSD within
1e-6*max(1, spectral magnitude), and agreeing reference weak dimensions at
the two steps, plus weak-projector Frobenius difference <=0.1. Otherwise
reference UNAVAILABLE, never repaired to PSD. Dimension agreement alone is
insufficient: diag(.049,.051,1,1,1,1) versus swapped first entries is a
required negative test for this reference gate.
Reference weak ratio .05 is shared only as an engineering cutoff; this is
objective sensitivity agreement, not calibrated ground truth observability.
PCL float transformation and changing radius-search support are limitations.
Report coverage, reference failures and all successful-frame results.
This is the standard finite-difference/local-curvature diagnostic already
used by P6 score-gradient audits, not a new detector or online mechanism.

Before execution, freeze engineering decision gates (not universal constants):
all five base analytic cases must match weak dimension with max principal
angle <=5 degrees; unit and transported projector errors <=1e-8 for an
invariance PASS. Real reference coverage must reach 50% of successful frames,
dimension agreement >=80% and median max angle <=15 degrees. Distinct
advantage requires >=20% and >=0.1 absolute reduction in median projector
distance against the paired SAME-threshold adaptive scaling, without lower
dimension accuracy; it must also not be bought by higher unavailability.
These cutoffs determine survival, not detector tuning. Even if local-curvature
reference is unavailable, controlled counterexamples remain valid evidence
against a claimed robust representation. A failed perturbation protocol alone
does not prove Uobs worse or authorize tuning/replacing the reference.

No downstream integration, trajectory A/B, threshold sweep or Uobs repair.
Survival requires all user gates, including a distinct advantage; analytic
or transport failure is recorded rather than fixed. No claim of equivalence
between continuous-surface nullspace and discrete NDT objective nullspace.

## Results

### Source and data evidence

* [DCReg pinned source](https://github.com/JokerJohn/DCReg/blob/ce7db8220f549a4a4391729e3bf4de4d4ab74635/DCReg/include/dcreg.hpp):
  DetectDegeneracy / CharacterizeDegeneracy; actual header called read-only.
* [FMCW paper](https://arxiv.org/html/2603.10248v1), equations 32–40:
  adaptive Schur length and joint spectrum. Official gamma=80 includes the
  equality boundary; paired scaling alone uses Uobs's strict .05 cutoff.
  Coordinate order is explicitly permuted and basis unscaled before scoring.
* [Frame-equivariance audit](https://arxiv.org/html/2608.15532v1), V-C–F:
  reference-point and subspace transport are existing evaluation concerns.
  This I2 audit does not implement its proposed new metric.
* [Official dataset listing](https://superodometry.com/datasets) identifies
  Corridor01 as SubT-MRS/RC2 and links separate map/bag/trajectory assets.
  [SubT-MRS CVPR paper, section 3.2](https://openaccess.thecvf.com/content/CVPR2024/papers/Zhao_SubT-MRS_Dataset_Pushing_SLAM_Towards_All-weather_Environments_CVPR_2024_paper.pdf)
  describes survey-scanner FARO maps. Its
  [supplement, Figure 10A](https://openaccess.thecvf.com/content/CVPR2024/supplemental/Zhao_SubT-MRS_Dataset_Pushing_CVPR_2024_supplemental.pdf)
  identifies the Long Corridor map. Together with the local official-release
  lineage record, this supports independent survey map vs robot test scan
  acquisition. It is publisher-level provenance, not a point-by-point
  forensic certificate. GT trajectories themselves use scan-map and other
  constraints; they are not independent degeneracy labels and are not read.
  The direct PDF fetch returned 403; indexed primary-source section text was
  available. No new claim of reading the entire PDFs is made.

Runtime map SHA256 is
103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f.
The frozen packed cloud/IMU manifest is unchanged. This is one 100-frame
prefix, not an evaluation of every corridor segment or the full sequence.

### Controlled cases

Full per-case angles, projector distances, spectra, unit/reference-point
tests and real-frame diagnostics are in
[I2_UOBS_FALSIFICATION_RESULTS.json](I2_UOBS_FALSIFICATION_RESULTS.json).

| Analytic scene | True weak dim | Uobs | DCReg native | FMCW native | Adaptive .05 paired |
|---|---:|---:|---:|---:|---:|
| Plane | 3 | 3 | 3 | 2 | 3 |
| Parallel planes | 3 | 3 | 3 | 2 | 3 |
| Corner | 0 | 3 | 0 | 0 | 0 |
| Circular tunnel | 2 | 2 | 2 | 1 | 2 |
| Rectangular corridor | 1 | 3 | 1 | 0 | 1 |

Uobs dimension accuracy: 3/5. Dimension excess: 5 total (corner +3,
corridor +2); deficit: 0. These are counts relative to continuous-surface
symmetry, not calibrated false-positive rates on a real dataset. Plane max
angle is 0.099642 degrees; parallel/tunnel are numerically zero. Corridor
has a zero containment angle but two extra weak dimensions: its projector
distance is sqrt(2), NOT a correct subspace. Corner empty-reference angles
are undefined, distance sqrt(3). Mean projector distance is 0.629949.

DCReg: 5/5 dimensions, numerical-zero projector error, under its distinct
native block threshold. FMCW native: 1/5, four missing dimensions total.
This does NOT establish Uobs superior to FMCW representation: at the paired
.05 threshold the SAME adaptive scaling gets 5/5 and mean projector error
0.0007021, with no extra or missing dimensions. Conversely DCReg's native
advantage is not attributed solely to its representation.

As end-wall weight falls 1 -> .1 -> .01 -> .001 -> 0, Uobs's smallest
relative eigenvalue falls monotonically .0179843 -> .00243181 -> .000868892
-> .000712524 -> .000695148. Thus its continuous spectrum does respond to
the weakening constraint. However its weak-dimension label stays 3 throughout,
including the closed geometry; paired adaptive labels are 0,0,1,1,1.
Positive-weight cases are near-degeneracy trends, NOT exact-nullity accuracy
scores. Smooth eigenvalues alone do not establish useful classification.

### Units and reference point

Metres/centimetres Uobs max projector discrepancy 3.43133e-15: PASS.
Whole-scene rotation max transported discrepancy 5.71764e-15: PASS.
Physical reference-point transport: FAIL, max 1.0007281; even the smaller
c=(.1,.2,.3)m shift gives discrepancies up to 0.0714945. The larger shift
changes weak dimension for some scenes. Exact coordinate transport was used;
the detector reuses the fixed diagonal metric rather than transporting that
metric. No patch is made. All alternatives also fail strict 1e-8
reference-point invariance; paired adaptive max is 0.0236700. This is NOT a
claim that a different comparator solves every coordinate issue.

### Real frames and unavailable reference

100 frames considered; 98 NDT-effective and Uobs-valid. The two iteration-limit
frames are detector unavailable, not discarded successful observations.
All 98 source hashes match; all 98 correspondence counts and weak dimensions
match production. H_phys matches production **exactly** (relative error 0).
The offline NDT object receives one discarded initialization align solely to
initialize PCL score constants/derivative storage; the frozen terminal is
never replaced, and no extra align enters production.

Independent curvature reference: **0/98 available**: 92 STEP_UNSTABLE and
6 INDEFINITE_COST_CURVATURE. Median two-step relative discrepancy 0.420526,
range [0.0735186, 1.24997]. No reference step, tolerance, PSD rule or weak
threshold was changed after inspecting this failure. Therefore real
Uobs/reference and comparator/reference angular agreement are **NOT
AVAILABLE**, not zero error. No real-data superiority claim is supported.

Real weak histogram: Uobs {3:49,4:49}; DCReg {0:5,1:93}; FMCW native
{0:57,1:41}; paired adaptive {0:32,1:66}. On 95 adjacent successful-frame
pairs, Uobs projector change median 0.0110874, P95 0.0905723; DCReg median
0.0065840; FMCW median 0; paired adaptive median 0.0315178. Lower change
alone is not correctness (an always-empty or always-full subspace is stable).
Those numbers are coordinate-chart diagnostics. Transporting each next-frame
basis to the preceding scan's physical increment origin gives Uobs median
0.0114114, P95 0.0959957, max 0.1267054. Both versions are retained in JSON;
neither substitutes for the unavailable independent direction reference.

### Resources and verification

Offline Uobs extraction+analysis+classification: mean 3.059064 ms,
P95 3.897321 ms. DCReg characterization only: mean 0.010771 ms.
FMCW Python characterization only: mean 0.110673 ms, P95 0.137813 ms;
timings are stored in the JSON and include
interpreter/NumPy overhead. Neither comparator number includes constructing
the shared observations/H_phys: do NOT claim a speedup from these unequal
timing scopes. Fixed production 100-frame Uobs mean 2.875671 ms.

Offline audit peak RSS: 58.121094 MiB; production check: 52.320313 MiB.
Difference 5.800781 MiB is a **cross-process accounting difference**, not an
isolated Uobs memory overhead: offline process retains raw/intermediate map
and runs curvature/reference tools. No reliable isolated incremental RSS was
measured, so that metric is UNAVAILABLE. No new production allocation added.

Build PASS; offline Python mathematical tests PASS; P7 standalone 8/8,
P7-A 7/7, P6 research 8/8 PASS. The inline upstream DCReg header emits existing
unused-parameter warnings; offline inclusion of the production TU emits an
anonymous-namespace linkage warning. These are offline-only and no upstream
or production refactor was made to silence them.

### Scientific decision

**UOBS_CORE_STOP. UOBS_SURVIVES=NO. DISTINCT_ADVANTAGE=NO.
FIXED_SCALE_SUPPORTED=NO (under the frozen I2 contract).**

This is a stop of the current primary-contribution route, not a proof that
all NDT information proxies are useless. The frozen fixed-L detector fails
two independently declared analytic cases and reference-point equivalence;
same-threshold adaptive scaling has fewer false weak dimensions. It fails
the first required survival gate irrespective of the unavailable real
reference. The real reference failure prohibits an effectiveness win claim
but does not undo those controlled counterexamples. Labels:
FIXED_SCALE_NOT_SUPPORTED / no demonstrated distinct advantage.

Do not repair Uobs, change L/ratio, add another scaling, or add downstream
fusion. If a mature detector is required as a non-novel baseline, recommend
the existing **DCReg Schur/block characterization**, with its explicitly
limited block-separable semantics. This recommendation is based on controlled
cases and reusable code, NOT proven real-sequence superiority. No replacement
has been implemented; all P7 history and current Uobs source remain intact.

### Reproduction and artifact fingerprints

```bash
env -u LD_LIBRARY_PATH cmake -S src/dog_prior_map_fastlio2_frontend_exp/scripts/i2 -B build/i2 -DCMAKE_BUILD_TYPE=Release -DDCREG_ROOT=/home/jian/livox_ws/DCReg
env -u LD_LIBRARY_PATH cmake --build build/i2 --parallel 2
```

The executable takes OUTPUT_PREFIX then MAP_PCD FILTER_SCANS_CSV SCANS_CSV
PACKED_XYZ REGISTRATION_CSV. These are the frozen Corridor01 inputs and
/tmp/i2_grid_fix_100/registration.csv; it has no GT argument. Run analysis:

```bash
OPENBLAS_NUM_THREADS=1 python3 src/dog_prior_map_fastlio2_frontend_exp/scripts/i2/analyze_falsification.py /tmp/i2_audit --production-uobs /tmp/i2_grid_fix_100/uobs.csv --registration /tmp/i2_grid_fix_100/registration.csv --output /tmp/i2_results.json
```

SHA256:

* /tmp/i2_audit_geometry.csv: da10ecf4d62cec04c79cfcd3c3f886fbc5405a62aa318ab0beee0d0b6b8077fb
* /tmp/i2_audit_curvature.csv: 5c13512103de3ddb6293189b7ee6912d58cd50eaaf733c9ba1acd6ec400fb0b7
* /tmp/i2_grid_fix_100/registration.csv: 0aedd32d20af0f43a9b56aa7c6fad93b9bf29d7d4805be380a13c42b43f7312b
* /tmp/i2_grid_fix_100/uobs.csv: 761270c3017d692a1a106cac7b8e49ffb581d7912e18e1354eae4ed49cc66c06

### Adversarial review ledger

The doubt-driven-development/code-review skills required explicit review.
Fresh reviewer identified weak-direction step instability despite dimension
agreement, missing DCReg block-lifting semantics, threshold attribution and
unspecified decision cutoffs. All were accepted and the preregistered protocol
was clarified before detector results were inspected.

User authorized a separate Codex CLI read-only review, executable 0.159.2,
default gpt-6-astra/high. This is independent CLI context, not a guaranteed
different model architecture. The initial model-list refresh timed out, but
the actual review completed successfully. Its four findings were actionable:
require full production parity, use common populations for paired statistics,
retain ineffective/invalid availability, and remove exact-nullity scoring
from transition rows. Reporting code was corrected, including missing/truncated
population negative tests; no detector parameters or reference gates changed.
The two rejected transactions remain explicitly listed in the durable JSON.
The real paired-reference population is empty and is reported as such.
The official FMCW <= equality boundary was also matched explicitly with a
boundary unit test; none of these reporting corrections changes the measured
Uobs classifications or the scientific stop decision.
