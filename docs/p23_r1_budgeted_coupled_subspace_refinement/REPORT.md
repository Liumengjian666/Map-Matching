# P23 small-budget Coupled prototype: cost passes, extra accuracy not established

TASK: P23-R1-BUDGETED-COUPLED-SUBSPACE-REFINEMENT

FINAL_RESULT: **COUPLED_LOCAL_GAIN_NOT_ESTABLISHED**

Repository: Liumengjian666/Map-Matching. Branch: research/p9-r4-heldout-visual-evidence.
START_SHA: 805515a59df230f0742c91543c42dd3600b16bd3.
Capture code: 74d293de13811e26a2e78bc019aa0c1878ece59f. Tested PROTOTYPE_SHA:
ce1298a35fa5cef0af946a14ca44c994f33fb012.
CODE_SHA (including posthoc/archive tools): bc39306ec34c31176dff468dfb08a1af8a5f795a.
END_SHA/push status belong to the final
delivery receipt; this file does not assert that its containing commit is uploaded.
Only isolated research workspace changed; stable dog and dirty paper workspaces untouched.

## Scope and real execution

One full original Visual replay captured actual NDT Source at setInputSource.
396 NDT/converged/OOSM/corrected, 8004 high-rate states, 169 Visual, zero Late,
nonfinite or pending overflow. Exactly eight selected Sources captured, 1400
points each, 16800 XYZ bytes each; uint64 ns IDs, original ordered float32 XYZ,
width/height/density and FNV/SHA metadata. Original five pose matrices saved
directly as LE float64 row-major; no quaternion-based reconstruction.

P19 full physical parity passed. Max translation difference=0m for NDT and
EKF; normalized quaternion comparison residuals <=3.183e-15deg (frozen allowed
1e-8deg). Timestamps/discrete/velocity fields satisfy original exact contract.
All eight source FNVs also equal P19. No P22 reconstruction was retried.

Exactly 24 offline alignments (8xA/B/C), 24 STARTED/24 COMPLETED; A reproduced
online Raw/Used/fitness/iterations/limit flags **exactly**, zero pose difference.
No additional replay, data/parameter search, optimizer/math change, or feedback.
Candidate/result SHA freeze preceded GT. The prototype uses one shared target
cloud/grid; PCL align resets each optimization from its explicit pose. C is
constructed before B and never uses B's terminal. Original map preprocessing,
stableCloudHash and Step Limit function bodies are reused verbatim and tested.

## Inputs and implementation

Map SHA: 103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f.
Bag SHA: 7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e.
Visual SHA: 10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79.
Target=226164 points, FNV3151c1dbad4c27a9. Same two-stage0.15m map/target voxel.
NDT .8/.08/.001/40; Source .25m/1400; Step Limit .5m/5deg; velocity_blend .6.
EKF binary unchanged: fbc1750dc605a7fd3b9549b310394fbff3a5781cad2272ef55b9f49df8c3435a.
Capture NDT SHA: 83959ca4dfd23360eee3ddcaf122d310a504be4ddb90155a6d6eb161e8367364.
Actual ROS parameters are archived. Only three opt-in capture output keys added.
P21 fallback disabled, CONTROL/Visual-on/camera-off/fixed-lag .15/rate .25 unchanged.

New production change is only default-false output/experimental_ndt_source_capture_enable
plus exact frame:ns requests/directory; save immediately after setInputSource.
Exclusive files refuse overwrite; IO failure stops, partial captures are invalid.
Synchronous IO is not assumed scheduling-neutral: actual one-run parity established it.
No change to existing Coupled source, native objective, EKF, OOSM, deskew or Visual math.

Offline APIs reused: ObservablePclNdt::nativeJet/dynamicScore, coupledNominalJet,
coupledJointJet (jointPullback J/K/g*K), coupledStrongStep (strongStep guards).
The new code is a bounded driver, not a newly invented Hessian/descriptor.
Old 16-candidate/2m/15deg grid is not called. Existing R2 math is not claimed new.

## Geometry and candidate budget

All Hessians finite, symmetric, positive curvature. Translation scale .8.
Original weak criterion yields k=2 only for frame2, k=1 for seven main frames.
Only weakest eigenvector explored; second weak coordinate remains zero when k=2.
All nonzero proposals obey <=.20m/2deg; measured translations about.1998m,
largest rotation.442242deg. Alpha frozen .999*minimum geometric bound.

|frame|H eigenvalues, ascending|weak vector (translation;rotation)|selected u|
|---|---|---|---:|
|2|2.596053,2.876119,4.373671,20.530923,135.966197,333.571544|-.992526,.048451,.107661;-.025189,.007027,-.016443|0|
|284|2.102763,5.312572,5.692011,26.877295,1257.082169,2114.036542|-.998171,.053166,.028754;.001171,.000553,-.000420|.2497502314|
|293|2.223535,5.400305,5.811465,28.064665,1447.549251,2095.653771|.997853,-.046616,-.045572;.006028,-.001560,-.000240|0|
|296|2.259980,5.467649,6.310940,27.903803,1413.203823,2061.190027|.997491,-.060434,-.036215;-.006902,-.000934,.000615|.2497561059|
|308|2.265404,5.300966,6.125360,27.470713,1226.310454,2173.900395|.998560,-.044804,-.027773;.009681,-.002184,-.000361|0|
|347|2.172667,5.831267,6.450510,26.966018,1747.905212,1987.395824|.998214,-.049743,-.032619;.005462,-.000672,.000434|0|
|368|2.125447,4.996246,7.187624,28.986786,1575.999412,2037.734074|.997809,-.057030,-.033500;.000924,-.001420,.000148|0|
|396|2.071028,5.103278,6.438313,28.549498,1081.843828,1610.739279|.998228,-.046991,-.036196;.004202,-.002199,-.000794|0|

Each frame evaluated precisely two signs plus nominal score reuse. Raw score
maximization, strict improvement, zero wins ties; never GT selection. All 24
candidate rows and full precision are in weak_candidates.csv. Only284/296
selected PLUS; six retained ZERO (not six nonzero Alternative candidates).

|frame|zero / plus / minus raw score|
|---|---|
|2|1688.083997 / 1587.808170 / 1666.882086|
|284|2356.500189 / 2358.816205 / 2350.495078|
|293|2418.864512 / 2411.649636 / 2415.255497|
|296|2459.757319 / 2468.584400 / 2465.662616|
|308|2469.067175 / 2460.544052 / 2464.580317|
|347|2334.983925 / 2322.603281 / 2332.619859|
|368|2293.874380 / 2292.601420 / 2291.161888|
|396|2223.411014 / 2220.886066 / 2220.546061|

Score means sum of original PCL objective; higher is better, E=-score/N.
It is not fitness or a calibrated correctness probability.

## Strong correction actual behavior

B/C use identical weak u. Predictor uses Hvu at zero and no gradient term.
One residual at actual predicted nonzero eta uses full chart and du=0; total
strong cap .10. No repeated predictor contribution or Newton loop.

|frame|predictor norm|conditional correction norm|Hvv condition|damping|weak/predicted/corrected score|
|---|---:|---:|---:|---:|---|
|284|1.694849204e-10|.003973722879|402.070091|0|2358.816205/2358.816205/2358.649654|
|296|4.927427340e-11|.003669818666|370.765503|0|2468.584400/2468.584400/2468.088729|

Both uncapped, finite, successful guard checks; solve residuals1.571e-17/4.847e-17.
Other six: no strong solve, not an estimated zero condition number.
Full strong vectors/gradients are in coupled_correction.csv.
In the nominal eigensystem Hvu is theoretically diagonal-block decoupled;
observed predictors are almost zero. Do not credit the small gains as substantial
Hvu transport. Both conditional candidates have LOWER real raw score than
Weak-only (.166551875/.495671366 decline); no monotonic safeguard was added
after seeing results. Reasons beyond observed behavior remain unproven.

## GT contract and all actual errors

Only after candidate freeze: same ControlV2 0–3s fixed complete SE(3), original
seconds/interpolation and LiDAR-to-IMU reference conversion. No refit.
Frame2 is .504301s before the first official GT; error unavailable, no extrapolation,
no replacement or deletion. Seven main samples are fixed challenges, not random
holdout or full trajectory RMSE. Per-frame rotation and all pose stages retained.

All 24 Step Limit flags false; Raw=Used for every method. Table shows translation
errors in m. A=Nominal, B=Weak-only, C=Coupled.

|frame|Initial A/B/C|Raw (=Used) A/B/C|Raw C-B|
|---|---|---|---:|
|2|N/A/N/A/N/A|N/A/N/A/N/A|N/A|
|284|.882284175/.765392272/.764026629|.875184376/.760176065/.759951309|-.000224757|
|293|.849231026/.846525060/.846525060|.846525060/.841775394/.841775394|0|
|296|.830844593/.919296150/.917006482|.818345136/.919635513/.918572458|-.001063055|
|308|.816350905/.836476512/.836476512|.836476512/.841176066/.841176066|0|
|347|1.357337246/1.355995916/1.355995916|1.355995916/1.354656894/1.354656894|0|
|368|.831312152/.809587010/.809587010|.809587010/.808861755/.808861755|0|
|396|5.763615210/5.784378607/5.784378607|5.784378607/5.799498743/5.799498743|0|

|frame|iterations A/B/C|fitness A/B/C|Raw rotation error A/B/C deg|
|---|---|---|---|
|2|2/1/1|.602399828/.604473644/.604473644|N/A/N/A/N/A|
|284|1/1/1|.065021971/.064362534/.064400239|2.891514/2.991561/3.032867|
|293|1/1/1|.028798804/.028793367/.028793367|3.506634/3.615221/3.615221|
|296|1/1/1|.022338437/.022155241/.022133869|3.325689/3.363838/3.318308|
|308|1/1/1|.012938901/.012970063/.012970063|2.924250/2.830210/2.830210|
|347|1/1/1|.024358365/.024380560/.024380560|1.275073/1.276062/1.276062|
|368|3/1/1|.075995512/.076010011/.076010011|1.778059/1.846013/1.846013|
|396|1/1/1|.047033076/.046901429/.046901429|1.406638/1.399390/1.399390|

All24 reported convergence. B/C ZERO rows still perform the permitted full
refinement FROM A raw, not original A initial and not a copy of A output.
Their changes must not be attributed to weak perturbation. A-to-B raw pose
separation exceeds.20m only284(.208180095)/296(.204290163); this is changed
terminal position, not proof of independent matching basins. C-to-B .000904189m/
.001110768m for these two, otherwise identical.

Primary counts: B raw strictly better than A4/7, but only1>=.10m; C strictly
better than B2/7, both millimetric and0>=.10m. No>=.20m risk (C-B or C-A),
but296 worsens B-A .101290377m / C-A .100227322m. Score improves respectively
7.866556925/8.227530367. Frame308 B/C raw error increases .004699553m while
score improves1.546428260. Thus objective-better/GT-worse=2/7 for each B-A/C-A,
0/7 C-B. Preserve small changes as well as engineering thresholds.

Frame284 B candidate gain retained (candidate.765392272 ->raw.760176065).
Frame296 wrong candidate not corrected (B.919296150 ->.919635513;
C.917006482 ->.918572458). Frame396 zero preferred; extra full refinement
retains one-iteration/low-fitness but5.799498743m error. Coupled does not resolve
this tail case; no new coupled trajectory was run.

## Complete costs and memory

Explicit calls:44 values,12 jets,24 full aligns. PCL internal kernel calls
are not these explicit counts. Jet includes whole native derivative evaluation;
weak/coupled outer timings contain chart/eigen/solve. Never add nested timers again.
Incremental method cost includes proposal setup plus candidate full align,
fitness, Step Limit and terminal objective; not just small matrix solve.

|frame|complete A/B/C refine ms|C incremental ms|
|---|---|---:|
|2|36.651679/7.122599/7.029709|11.926452|
|284|7.666666/7.376612/7.357687|20.580611|
|293|7.473972/7.490664/7.489827|12.881049|
|296|7.477674/7.557992/7.571891|20.695291|
|308|7.574824/7.553661/7.562672|13.040708|
|347|7.276739/7.290848/7.263190|12.552932|
|368|11.671809/7.125113/7.102974|12.223350|
|396|7.146136/7.168484/7.144110|12.292353|

Nominal align mean/P95/max=8.854333/25.041998/33.690181ms (first-call cold cost retained).
B align=4.595157/4.818461/4.819707ms; C align=4.586297/4.824146/4.825784ms.
B incremental=12.596077/12.986328/13.031697ms.
C incremental=14.524093/20.655153/20.695291ms: **30/50ms engineering gate PASS**.
Weak setup mean5.260330ms; eigen.004042ms. Active Coupled setup7.808724/7.779321ms,
two strong solves total.020464/.016623ms. Native derivatives/values dominate.
Map/setup54.161ms separately; whole offline process.356603s; peak59328KiB=57.9375MiB,
including map loading/transient preprocessing and shared NDT grid.
One capture ROS replay wall169.352910s; NDT peak77.894531MiB/CPU7.95s,
EKF peak17.968750MiB/CPU9.45s. Do not sum peaks as de-duplicated system memory.
Source IO-specific time was not separately instrumented. No online camera IO/
MEI/KLT/PnP here, hence no complete 10Hz/100MiB deployment claim.

## Verification, limits and final decision

Capture contract C++ test passed. Bounded policy + existing R2 synthetic math
test passed (mock backend, no real alignment). Python4/4 passed; helper text
equals production, exact IDs/default-off/single align/capture hashes checked.
Only NDT target rebuilt; EKF hash remains unchanged. Independent capture,
local-math and posthoc read-only reviews completed. GT conversion cross-check
<=6.67e-16; error re-evaluation<=2.29e-12; counts/deltas/timers/hashes agree.
No external cross-model CLI. Linker initially needed the same explicit system
libusb dependency as original NDT; fixed build dependency, not objective/math.
Original runner resource CSV uses CRLF; launch logs contain trailing blanks for
empty ROS parameters. Scoped attributes preserve these two frozen files exactly;
live source and other evidence retain normal whitespace checks.

Q1 yes, valid relatively weak mixed directions in8/8; Q2 nonzero selected2/8;
Q3 Weak-only gives one>=.10m benefit and an objective/GT counterexample;
Q4 no meaningful additional Coupled benefit; Q5 no>=.20m new risk but smaller
risks and candidate-score declines preserved; Q6 not uniformly;
Q7 no Step Limit effect in these24 trials; Q8 measured incremental budget passes.

SOURCE_CAPTURE=PASS; BASELINE_PARITY=PASS; OPTIMIZER_PARITY=PASS;
COUPLED_MATH=EXISTING_R2_LOCAL_MATH_REUSED;
ACCURACY=COUPLED_LOCAL_GAIN_NOT_ESTABLISHED; RUNTIME=COMPUTATIONAL_BUDGET_PASS.

Unique NEXT: keep Coupled offline, no continuous feedback. Ask decision AI to
define one fixed-budget, non-GT candidate-rejection experiment within the
Coupled research line, centered on296's objective/GT counterexample with284
as its frozen beneficial control. Test candidate trust rather than enlarge
weak range/grid or retune fusion. This is a proposed contract, not executed
or a claim of originality, universal rejection capability or trajectory gain.
