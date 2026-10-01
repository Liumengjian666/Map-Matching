# Attribution and its limits

PRIMARY_ROOT_CAUSE_CLASS = **A3G-R1-H / ROOT_CAUSE_NOT_ISOLATED**.

## What is established

The **direct sustained non-admission mechanism** is map-support rejection,
not missing P15, not continuing NIS rejection, and not the NDT convergence flag.
After tx182, completed tx183–365 have zero commits: 181 no-valid-correspondence
statuses plus two MAP_SUPPORT_INSUFFICIENT statuses. Preview is invalid and NIS
is NOT_REACHED in all 183. tx188–365 is the 178-record exact-status streak.

The first earlier loss is tx166–172. Every no-correspondence terminal in the
whole completed prefix (188) also has NDT iterations0/objective0/converged1.
The terminal equals its prediction-derived seed within float/log precision in
the audited window. A success flag or tiny seed innovation therefore does not
establish that a registration constraint was obtained.

Source reading solely interprets these logged fields: the producer calls NDT
on prediction*T_imu_lidar, then geometricObservations at its terminal; that
helper searches target voxel leaves within0.8 m and rejects invalid observations.
analyzeGeometricObservability sets NO_VALID when accepted correspondence count
is0, MAP_SUPPORT_INSUFFICIENT when positive but <30. The CSV does **not** contain
raw neighbor counts, rejected covariance counts, weights or exact positive
accepted counts. It cannot distinguish empty geometric searches from all
returned observations failing validity tests. No search or NDT was rerun.

P15 valid/full rank through tx365 and all81 tx140–220 terminals is established.
Window active LiDAR counts drain naturally as the last committed state reaches
the2 s limit: last positive tx201, removal of last tx182 factor at tx202. Historical
LiDAR information remains in the prior; starvation here means no **new** external
LiDAR constraint, not loss of all previously accumulated information.

TX366_COVARIANCE_FAILURE = **DOWNSTREAM_OF_LIDAR_MEASUREMENT_STARVATION** in
time. Causal necessity/sufficiency and correctness of the numerical rank gate
remain unproven. We did not inspect new matrices, modify rank rules or suggest
a rank fix as the next authorized action.

## Competing hypotheses

| Hypothesis | Verdict | Evidence / limitation |
|---|---|---|
| H1 incorrect NDT terminal first | NOT ESTABLISHED | Large innovations and saturated iteration counts precede support loss. tx160 rotation innovation1.624 rad, tx165 translation3.881 m. Which pose is correct is unknown. No abnormal independent NDT pose at tx166: it effectively returns seed. |
| H2 prediction divergence first, then lost basin | NOT ESTABLISHED | Motion growth >0.5 m at tx160 precedes first loss tx166; >1 m at tx176 precedes permanent loss tx183. Prediction motion grows across these intervals, but there is no state velocity/bias history or independent pose evidence to locate a physically erroneous prediction. Current prior/IMU dynamics and old LiDAR corrections both affect prediction. |
| H3 correct NDT continuously rejected by faulty support/U_obs | NOT ESTABLISHED | Direct support rejection is proven; “NDT correct” and “support implementation wrong” are not. Converged1 plus zero iterations/score is not healthy registration evidence. Missing correspondence rejection details prevent narrower attribution. |
| H4 sustained NIS starvation | CONTRADICTED as primary sustained mechanism |16 transient NIS rejections overall,13 in tx140–220. Subsequent commits recover. All183 final completed rejections do not reach NIS. Earlier NIS episodes may contribute to trajectory evolution but their causal role is unresolved. |
| H5 covariance failure earlier than starvation | CONTRADICTED |314/314 completed terminals have valid P15; first unavailable is tx366, after183 completed noncommits. |
| H6 deskew/timestamp failure | NOT ESTABLISHED; structural anomaly not supported |81/81 timestamp interval, nonempty count preservation, and Window-owned provenance checks pass. Displacement grows and has maxima spikes; CSV alone cannot prove physical deskew correctness from a potentially wrong state. |
| H7 other unique cause | NOT ESTABLISHED |No strictly identified upstream mechanism beyond the measured support/admission chain. |

The broad A/B/C/D classifications would overclaim: A requires wrong NDT first;
B requires wrong prediction first; C cannot be justified just by first support
loss preceding the >1 m milestone, since >0.5 m already precedes it and permanent
loss starts **after** the first >1 m increment. D confuses transient NIS rejection
with permanent support failure. Class H preserves these distinctions.

## Last actual measurement and quantitative transition

tx182 stamp1517157237443522287: selected rank5; NIS14.25169804323739,
threshold15.086, accepted1; U_obs VALID_GEOMETRIC_GAUSS_NEWTON_PROXY,
map-support valid (inferred from successful admission), NDT converged1,
fitness16.096816828119337, objective170.95783834476933, iterations40.
Predicted map_T_imu position [18.84510535,-18.71374009,2.843835062] m;
quaternion xyzw [0.14227717183209052,-0.082627436335635782,
0.30156634753227285,0.93914197604840743].
NDT raw map_T_lidar and converted map_T_imu are retained in the CSV/JSON.
Converted position [18.617047220864194,-18.311478189630257,2.3462194538698635]
m; innovation0.679298491 m /0.402429971 rad. None of these are GT errors.

tx183: support absent, iterations0/objective0; seed innovation7.515e-7 m /
2.006e-8 rad; completed increment0.995601004 m. tx185 and187 briefly produce
nonzero NDT objectives15.898806 and24.856071, but support remains insufficient.
At tx188 exact no-correspondence persistence resumes; fitness rises from14.039987
to66.044057 at201,71.394847 at202 and384.592079 at215. Scores stay zero, not a
successful low-cost localization optimum. Fitness and objective are different
PCL metrics and must not be combined or treated as accuracy.

## Covariance and deskew limits

tx140–220 P15_min ranges1.186966e-6…2.632153e-6;
P15_max0.047883…2.416670; Pmap_min7.973432e-5…9.775374e-5;
Pmap_max0.032021…2.064723. Rank equals columns throughout; triangular residual
2.49e-17…8.23e-17. “Healthy” here means reported finite/full-rank/available,
not calibrated uncertainty or physically correct states. At365 P15_max1264.452418,
Pmap_max1241.952121; these have grown substantially. At366 rank498/600,
global pivot6.809639e15 and threshold907.226224 appear only much later. Its
logged residual0 is initialization: triangular solve was NOT_EXECUTED.

Deskew window counts27614…29107; durations0.100775553…0.100839450 s; allmin
timestamps equal catalog-start, allmax lie within end, output count unchanged,
provenance WINDOW_OWNED_SE3_DESKEW. Mean displacement grows0.096630 at140
to1.079561 m at220; P95 grows0.178287→2.093892 m. Max displacement spikes:
tx1651.870857→tx1664.123055 m; tx1680.743838→1693.353809 m; overall window
max5.983417 m at211. These are disclosed internal kinematic observations;
no per-point geometry or true motion was inspected, so they neither prove nor
exclude state-induced deskew distortion. DESKEW_ANOMALY_NOT_SUPPORTED applies
only to timestamp/count/provenance failure as an identified cause.

## Decision boundary

Forensic task acceptance can be PASS with class H, as explicitly permitted by
the task's criterion9. This is **not** successful upstream mathematical/root-cause
isolation or an engineering pass. Next investigation target, if separately
authorized, is the pre-collapse prediction / NDT seed-to-terminal / geometric
support boundary around tx160–166 and recovery/loss tx174–188. Existing CSVs
lack rejection counts and independent physical pose evidence. There is no
evidence-based unique production repair target in this round; no repair chosen.
