# P29-R1 — fixed Source / initialization feedback isolation

START_SHA: `9b6f32616fae7f408f1950ebba094f1ce72cadc1`.
Existing research branch; no protected workspace mutation, historical cache
cleanup, parameter search, runtime math change or source reconstruction.

Frozen sensor requests (frame:uint64 scan_ns):
7:1517157219794140100, 193:1517157238552942991,
270:1517157246318675041, 320:1517157251361368895,
340:1517157253378448009, 347:1517157254084407091,
365:1517157255899825096, 396:1517157259026242018.

One full A capture and one full B capture using the SAME P28 binaries/input
hashes, runtime/config/initialization, rate .25, CONTROL, camera node OFF,
MSCKF feature file and common time/noise adapter. All old PnP paths are OFF.
Only the existing bounded Source capture settings and diagnostic paths are
added. A MSCKF OFF; B ON. Existing ExperimentalSourceCapture saves actual
post-preprocess NDT Source and five poses. A separate passive ROS observer
records only the eight full pre-filter deskew messages for pointwise comparison;
it does not reconstruct, transform or republish a cloud. No additional align
or filter update is added to these replays.

Capture validity: all selected identity, count, ordered XYZ SHA256 and stable
hashes must agree with their own online rows. Full P28 replay parity is measured
separately: timestamps/discrete states/hashes exact; physical poses <=1e-9 m,
<=1e-8 deg using normalized quaternion differences. If a capture differs, it
must NOT be described as exact P28 replay. Preserve and report the first actual
difference; do not rerun until an agreeable result appears.

Offline: eight independent 2x2 tests, SA/IA, SA/IB, SB/IA, SB/IB. One native PCL
NDT align per cell; maximum 32, no candidate search and no state writeback.
Each optimization is explicitly initialized, sharing the frozen map/grid.
Reuse P23 production-equivalent map voxel, ordered hash and Step Limit helpers.
Resolution .8, step .08, epsilon .001, iterations 40, target voxel .15;
Source is already the exact production input, NOT preprocessed a second time.
For derived Used, fix the same previous pose PA for BOTH IA/IB on SA, and PB
for BOTH IA/IB on SB. Raw optimizer comparisons are independent of Step Limit.
Self-cell raw parity uses pre-existing P23 float/PCL tolerance <=1e-5 m,
<=1e-4 deg, fitness <=1e-6*max(1,abs(reference)); iterations/convergence exact
when nominal was selected online. Existing multi-NDT replacement is explicitly
distinguished from native nominal; no additional candidate aligns are budgeted.

Freeze all captures, input matrices and four-cell outputs before GT. Preserve
formal Control V2 0–3 s SE(3), original interpolation and IMU origin. Selected
frame errors do NOT establish full-trajectory improvement. Report map-frame
AB displacement, shared-transform errors, diagnostic independently fitted A/B
prefix transforms, and fixed 1-second RPE separately. Independently fitted
transforms are diagnostic only and never replace the formal metric.

Attribution: source and initialization effects are measured on the same fixed
cells. Their interaction and full closed-loop history remain possible; temporal
precedence alone is not a proof of a unique root cause. No fix is implemented
in this task. Stop after finite analysis and report one next decision.
