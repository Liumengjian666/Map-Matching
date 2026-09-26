# PAPER-P4-I4: sparse visual translation fusion

Verdict: `LOCAL_ONLY_IMPROVEMENT`.
Start SHA: `a328d3e3adb9109035a7d6f965d2db2c271cee17`. Offline fixed-measurement replay only.

## Reproduction gates

```json
{
  "corrected_translation_max_m": 5.4117422466702744e-14,
  "corrected_rotation_max_deg": 2.091309789151873e-06,
  "predictor_translation_max_m": 4.6279657102031596e-14,
  "predictor_rotation_max_deg": 2.4148365394514667e-06
}
```
VISUAL_SKIP and BASELINE: all original replay CSV fields exactly equal. Visual poses are never recomputed.
Position-update contracts PASS: rotated reference-body Z is in the XY nullspace; XYZ seed update equals the analytical linear Kalman solution. Python compilation/lint and independent timestamp/count/finite-output/metric checks PASS. Read-only code review found no blocking defect.
Offline compilation retains two anonymous-namespace subobject-linkage warnings from the included I2/runtime implementation. No production build target is changed.

## Method and limitations

Reuse the unchanged P4-I2 input parser, initializer, full-update baseline, state writer and runtime IKFoM implementation. An offline-only header access seam accesses that same filter; no ROS target, header, runtime source, launch or configuration is modified.
Events use integer nanosecond timestamps. NDT uses saved scan-end stamps; reference snapshots and visual updates use actual image stamps. IMU samples are consumed causally up to each event, with the existing held-last-two-input tail when no new IMU sample exists. Exact ties order NDT, visual update, then reference snapshot; non-ties always follow timestamp order.
Reference snapshots and skipped visual events use disposable candidates without committing IMU subdivision. Accepted visual candidates are committed at image time. This makes skipped visual updates recover the original numerical baseline exactly.
z_ref_cur = translation(T_IC inverse(PnP) inverse(T_IC)); rotation is necessary only for this verified direction/lever-arm conversion. The C++ visual input has no orientation fields. target = p_ref + R_ref z_ref_cur. XYZ H observes map position; XY H[:,position] = first two rows of R_ref^T. XY does not directly observe reference-body Z.
Use the same IKFoM covariance, standard Kalman gain, Joseph covariance and SO3/S2 tangent reset. No state gain row is frozen; position residuals may indirectly change attitude/velocity/bias via cross covariance. Fixed calibrated extrinsics retain the existing constraint. No visual rotation measurement is supplied.
Historical anchors are deterministic snapshots: their uncertainty and correlation with current state are ignored as requested by this pseudo-measurement prototype. This is not a statistically exact relative-state update or full VIO.
All saved NDT poses and visual translations remain frozen from baseline despite changed filter states. This evaluates a fixed-measurement counterfactual, not an actual closed-loop rerun with recomputed deskew/NDT/visual depth.
GT is loaded only after all replay modes finish. Use one common first-baseline-pose alignment for all modes, the existing IMU-origin GT convention, no per-mode fit. Score corrected states at the original NDT timestamps. Between-scan predictor increments include any intervening visual corrections, so they are not pure IMU-only increments.
All 4127 saved NDT events are replayed; 4126 states have GT coverage (the last is excluded, never extrapolated). All 1802 frozen visual pairs update at their true image stamps: 961 before and 841 after the matched current NDT stamp. No artificial ordering by modality.
Persistent crossing requires >threshold continuously for >=5s, retaining the prior evaluator. Predefined 'clearly delayed' means both 2m and 5m crossing delayed >=5s or absent. LOCAL_ONLY requires >=10% late scan-interval local RMSE improvement in a primary mode if the global gate fails; sensitivity modes cannot win the primary gate.

## Global trajectory metrics

```json
[
  {
    "mode": "BASELINE",
    "segment": "all",
    "count": 4126,
    "t_mean": 14.660182042205129,
    "t_rmse": 21.936132729375835,
    "t_median": 6.034344271830073,
    "t_p95": 46.757084095160685,
    "t_max": 54.298934504711355,
    "r_mean": 7.6021256296461,
    "r_rmse": 9.906078073266109,
    "r_median": 5.256541785546884,
    "r_p95": 19.08964428500653,
    "r_max": 22.682863869442393
  },
  {
    "mode": "BASELINE",
    "segment": "150-end",
    "count": 2644,
    "t_mean": 22.443868635217253,
    "t_rmse": 27.392906834641643,
    "t_median": 20.19096570197663,
    "t_p95": 50.42933903691806,
    "t_max": 54.298934504711355,
    "r_mean": 10.643158040173068,
    "r_rmse": 12.175511106160776,
    "r_median": 11.478670562399532,
    "r_p95": 19.74766190953088,
    "r_max": 22.682863869442393
  },
  {
    "mode": "VISUAL_XYZ_005",
    "segment": "all",
    "count": 4126,
    "t_mean": 14.632980559951735,
    "t_rmse": 21.931998954950288,
    "t_median": 6.036275770137087,
    "t_p95": 47.45858397883955,
    "t_max": 54.30566400308956,
    "r_mean": 7.742615905543392,
    "r_rmse": 10.088683193587558,
    "r_median": 5.200575476120131,
    "r_p95": 19.563273846804176,
    "r_max": 23.3933356294356
  },
  {
    "mode": "VISUAL_XYZ_005",
    "segment": "150-end",
    "count": 2644,
    "t_mean": 22.400527682796135,
    "t_rmse": 27.387794497286837,
    "t_median": 20.160806646207803,
    "t_p95": 50.89996585088759,
    "t_max": 54.30566400308956,
    "r_mean": 10.851492603945747,
    "r_rmse": 12.41043749490537,
    "r_median": 11.531433141203923,
    "r_p95": 20.277632491819364,
    "r_max": 23.3933356294356
  },
  {
    "mode": "VISUAL_XY_005",
    "segment": "all",
    "count": 4126,
    "t_mean": 14.634948542937327,
    "t_rmse": 21.92997975039049,
    "t_median": 6.042502729700313,
    "t_p95": 47.46019300084501,
    "t_max": 54.307211522721275,
    "r_mean": 7.744003699144594,
    "r_rmse": 10.088728418109035,
    "r_median": 5.234882947499393,
    "r_p95": 19.557650515132828,
    "r_max": 23.411572605075083
  },
  {
    "mode": "VISUAL_XY_005",
    "segment": "150-end",
    "count": 2644,
    "t_mean": 22.399534598328913,
    "t_rmse": 27.385062478510566,
    "t_median": 20.176997106347528,
    "t_p95": 50.91394891664449,
    "t_max": 54.307211522721275,
    "r_mean": 10.853691924665918,
    "r_rmse": 12.410503665872678,
    "r_median": 11.526244979411436,
    "r_p95": 20.267725552230303,
    "r_max": 23.411572605075083
  },
  {
    "mode": "VISUAL_XYZ_003",
    "segment": "all",
    "count": 4126,
    "t_mean": 14.502024295966827,
    "t_rmse": 21.767614612560138,
    "t_median": 6.031999004801685,
    "t_p95": 48.19444790261272,
    "t_max": 54.317184057352605,
    "r_mean": 7.816469819610136,
    "r_rmse": 10.254142227044323,
    "r_median": 5.08158999641826,
    "r_p95": 20.03978155915722,
    "r_max": 30.203120328292332
  },
  {
    "mode": "VISUAL_XYZ_003",
    "segment": "150-end",
    "count": 2644,
    "t_mean": 22.19883146529976,
    "t_rmse": 27.1824974039832,
    "t_median": 20.538241765516645,
    "t_p95": 51.16316902237295,
    "t_max": 54.317184057352605,
    "r_mean": 10.97154039821801,
    "r_rmse": 12.622408536605896,
    "r_median": 11.546296397297855,
    "r_p95": 21.100946097226593,
    "r_max": 30.203120328292332
  },
  {
    "mode": "VISUAL_XYZ_010",
    "segment": "all",
    "count": 4126,
    "t_mean": 14.664036160217814,
    "t_rmse": 21.95429560407752,
    "t_median": 6.043812928405857,
    "t_p95": 46.86607542199154,
    "t_max": 54.300229674747996,
    "r_mean": 7.652745629660466,
    "r_rmse": 9.952849831641164,
    "r_median": 5.29097975594974,
    "r_p95": 19.045487665301117,
    "r_max": 22.792785076842545
  },
  {
    "mode": "VISUAL_XYZ_010",
    "segment": "150-end",
    "count": 2644,
    "t_mean": 22.448909445371463,
    "t_rmse": 27.41560173629502,
    "t_median": 20.172588492800823,
    "t_p95": 50.61298730675612,
    "t_max": 54.300229674747996,
    "r_mean": 10.711318136974121,
    "r_rmse": 12.235686974002345,
    "r_median": 11.547568886632924,
    "r_p95": 19.770428653882615,
    "r_max": 22.792785076842545
  }
]
```

## Crossings

```json
[
  {
    "mode": "BASELINE",
    "threshold_m": 0.25,
    "crossing_s": 28.03748917579651
  },
  {
    "mode": "BASELINE",
    "threshold_m": 0.5,
    "crossing_s": 84.91932845115662
  },
  {
    "mode": "BASELINE",
    "threshold_m": 1.0,
    "crossing_s": 93.5928385257721
  },
  {
    "mode": "BASELINE",
    "threshold_m": 2.0,
    "crossing_s": 151.48321318626404
  },
  {
    "mode": "BASELINE",
    "threshold_m": 5.0,
    "crossing_s": 157.43361401557922
  },
  {
    "mode": "VISUAL_XYZ_005",
    "threshold_m": 0.25,
    "crossing_s": 28.03748917579651
  },
  {
    "mode": "VISUAL_XYZ_005",
    "threshold_m": 0.5,
    "crossing_s": 84.91932845115662
  },
  {
    "mode": "VISUAL_XYZ_005",
    "threshold_m": 1.0,
    "crossing_s": 93.5928385257721
  },
  {
    "mode": "VISUAL_XYZ_005",
    "threshold_m": 2.0,
    "crossing_s": 152.79433941841125
  },
  {
    "mode": "VISUAL_XYZ_005",
    "threshold_m": 5.0,
    "crossing_s": 158.74473810195923
  },
  {
    "mode": "VISUAL_XY_005",
    "threshold_m": 0.25,
    "crossing_s": 28.03748917579651
  },
  {
    "mode": "VISUAL_XY_005",
    "threshold_m": 0.5,
    "crossing_s": 84.91932845115662
  },
  {
    "mode": "VISUAL_XY_005",
    "threshold_m": 1.0,
    "crossing_s": 93.5928385257721
  },
  {
    "mode": "VISUAL_XY_005",
    "threshold_m": 2.0,
    "crossing_s": 152.5926194190979
  },
  {
    "mode": "VISUAL_XY_005",
    "threshold_m": 5.0,
    "crossing_s": 158.74473810195923
  },
  {
    "mode": "VISUAL_XYZ_003",
    "threshold_m": 0.25,
    "crossing_s": 28.03748917579651
  },
  {
    "mode": "VISUAL_XYZ_003",
    "threshold_m": 0.5,
    "crossing_s": 84.91932845115662
  },
  {
    "mode": "VISUAL_XYZ_003",
    "threshold_m": 1.0,
    "crossing_s": 93.5928385257721
  },
  {
    "mode": "VISUAL_XYZ_003",
    "threshold_m": 2.0,
    "crossing_s": 153.19772338867188
  },
  {
    "mode": "VISUAL_XYZ_003",
    "threshold_m": 5.0,
    "crossing_s": 160.86262917518616
  },
  {
    "mode": "VISUAL_XYZ_010",
    "threshold_m": 0.25,
    "crossing_s": 28.03748917579651
  },
  {
    "mode": "VISUAL_XYZ_010",
    "threshold_m": 0.5,
    "crossing_s": 84.91932845115662
  },
  {
    "mode": "VISUAL_XYZ_010",
    "threshold_m": 1.0,
    "crossing_s": 93.5928385257721
  },
  {
    "mode": "VISUAL_XYZ_010",
    "threshold_m": 2.0,
    "crossing_s": 151.88659954071045
  },
  {
    "mode": "VISUAL_XYZ_010",
    "threshold_m": 5.0,
    "crossing_s": 157.73619413375854
  }
]
```

## Improvements

```json
{
  "VISUAL_XYZ_005": {
    "all_improvement": 0.00018844590687638885,
    "late_improvement": 0.00018662996905249862,
    "rotation_ratio": 1.0184336443717572,
    "late_local_improvement": 0.12452851056334235,
    "crossing_delayed": false
  },
  "VISUAL_XY_005": {
    "all_improvement": 0.00028049515660999447,
    "late_improvement": 0.0002863645022570527,
    "rotation_ratio": 1.0184382097023696,
    "late_local_improvement": 0.12177890977188355,
    "crossing_delayed": false
  },
  "VISUAL_XYZ_003": {
    "all_improvement": 0.007682216318377066,
    "late_improvement": 0.00768116476022751,
    "rotation_ratio": 1.0351364234365916,
    "late_local_improvement": 0.1079986904663921,
    "crossing_delayed": false
  },
  "VISUAL_XYZ_010": {
    "all_improvement": -0.0008279889133493423,
    "late_improvement": -0.0008284955587363196,
    "rotation_ratio": 1.004721521275032,
    "late_local_improvement": 0.12010095298322931,
    "crossing_delayed": false
  }
}
```
Primary XYZ/XY global RMSE changes are only about 0.019%/0.028%; 2m/5m crossings move about 1.1-1.3s, failing the global gate. Late local scan-interval RMSE reductions are about 12.45%/12.18%, supporting only LOCAL_ONLY_IMPROVEMENT. XY's tiny global advantage and slightly worse local RMSE do not establish a meaningful preference over XYZ. No recovery or closed-loop-runtime success is claimed.

## Update rates and local predictor

```json
[
  {
    "mode": "BASELINE",
    "segment": "all",
    "count": 4126,
    "local_t_rmse_m": 0.22830059885130433,
    "visual_updates": 0,
    "duration_s": 416.72996616363525,
    "update_hz": 0.0
  },
  {
    "mode": "BASELINE",
    "segment": "150-end",
    "count": 2644,
    "local_t_rmse_m": 0.28439004867209555,
    "visual_updates": 0,
    "duration_s": 266.72996616363525,
    "update_hz": 0.0
  },
  {
    "mode": "BASELINE",
    "segment": "0-50",
    "count": 491,
    "local_t_rmse_m": 0.016378843524973857,
    "visual_updates": 0,
    "duration_s": 50,
    "update_hz": 0.0
  },
  {
    "mode": "BASELINE",
    "segment": "50-100",
    "count": 496,
    "local_t_rmse_m": 0.02536538259618079,
    "visual_updates": 0,
    "duration_s": 50,
    "update_hz": 0.0
  },
  {
    "mode": "BASELINE",
    "segment": "100-150",
    "count": 495,
    "local_t_rmse_m": 0.03783490574087552,
    "visual_updates": 0,
    "duration_s": 50,
    "update_hz": 0.0
  },
  {
    "mode": "BASELINE",
    "segment": "150-200",
    "count": 496,
    "local_t_rmse_m": 0.2804334998374872,
    "visual_updates": 0,
    "duration_s": 50,
    "update_hz": 0.0
  },
  {
    "mode": "BASELINE",
    "segment": "200-250",
    "count": 495,
    "local_t_rmse_m": 0.2626243071415483,
    "visual_updates": 0,
    "duration_s": 50,
    "update_hz": 0.0
  },
  {
    "mode": "BASELINE",
    "segment": "250-300",
    "count": 496,
    "local_t_rmse_m": 0.18337968254148743,
    "visual_updates": 0,
    "duration_s": 50,
    "update_hz": 0.0
  },
  {
    "mode": "BASELINE",
    "segment": "300-350",
    "count": 496,
    "local_t_rmse_m": 0.15178198395810388,
    "visual_updates": 0,
    "duration_s": 50,
    "update_hz": 0.0
  },
  {
    "mode": "BASELINE",
    "segment": "350-end",
    "count": 661,
    "local_t_rmse_m": 0.41270766484953997,
    "visual_updates": 0,
    "duration_s": 66.72996616363525,
    "update_hz": 0.0
  },
  {
    "mode": "VISUAL_XYZ_005",
    "segment": "all",
    "count": 4126,
    "local_t_rmse_m": 0.20003530518575935,
    "visual_updates": 1802,
    "duration_s": 416.72996616363525,
    "update_hz": 4.324143081403505
  },
  {
    "mode": "VISUAL_XYZ_005",
    "segment": "150-end",
    "count": 2644,
    "local_t_rmse_m": 0.24897537949192305,
    "visual_updates": 1093,
    "duration_s": 266.72996616363525,
    "update_hz": 4.097777297843839
  },
  {
    "mode": "VISUAL_XYZ_005",
    "segment": "0-50",
    "count": 491,
    "local_t_rmse_m": 0.01541759269576045,
    "visual_updates": 234,
    "duration_s": 50,
    "update_hz": 4.68
  },
  {
    "mode": "VISUAL_XYZ_005",
    "segment": "50-100",
    "count": 496,
    "local_t_rmse_m": 0.02591092481796399,
    "visual_updates": 55,
    "duration_s": 50,
    "update_hz": 1.1
  },
  {
    "mode": "VISUAL_XYZ_005",
    "segment": "100-150",
    "count": 495,
    "local_t_rmse_m": 0.0378875167968674,
    "visual_updates": 420,
    "duration_s": 50,
    "update_hz": 8.4
  },
  {
    "mode": "VISUAL_XYZ_005",
    "segment": "150-200",
    "count": 496,
    "local_t_rmse_m": 0.24683837394037098,
    "visual_updates": 423,
    "duration_s": 50,
    "update_hz": 8.46
  },
  {
    "mode": "VISUAL_XYZ_005",
    "segment": "200-250",
    "count": 495,
    "local_t_rmse_m": 0.26154489307008566,
    "visual_updates": 67,
    "duration_s": 50,
    "update_hz": 1.34
  },
  {
    "mode": "VISUAL_XYZ_005",
    "segment": "250-300",
    "count": 496,
    "local_t_rmse_m": 0.16723650778625915,
    "visual_updates": 202,
    "duration_s": 50,
    "update_hz": 4.04
  },
  {
    "mode": "VISUAL_XYZ_005",
    "segment": "300-350",
    "count": 496,
    "local_t_rmse_m": 0.1649750333838723,
    "visual_updates": 55,
    "duration_s": 50,
    "update_hz": 1.1
  },
  {
    "mode": "VISUAL_XYZ_005",
    "segment": "350-end",
    "count": 661,
    "local_t_rmse_m": 0.3310571924694401,
    "visual_updates": 346,
    "duration_s": 66.72996616363525,
    "update_hz": 5.185076808693993
  },
  {
    "mode": "VISUAL_XY_005",
    "segment": "all",
    "count": 4126,
    "local_t_rmse_m": 0.20066256884146147,
    "visual_updates": 1802,
    "duration_s": 416.72996616363525,
    "update_hz": 4.324143081403505
  },
  {
    "mode": "VISUAL_XY_005",
    "segment": "150-end",
    "count": 2644,
    "local_t_rmse_m": 0.24975733859483487,
    "visual_updates": 1093,
    "duration_s": 266.72996616363525,
    "update_hz": 4.097777297843839
  },
  {
    "mode": "VISUAL_XY_005",
    "segment": "0-50",
    "count": 491,
    "local_t_rmse_m": 0.015357160444315024,
    "visual_updates": 234,
    "duration_s": 50,
    "update_hz": 4.68
  },
  {
    "mode": "VISUAL_XY_005",
    "segment": "50-100",
    "count": 496,
    "local_t_rmse_m": 0.02585400406967562,
    "visual_updates": 55,
    "duration_s": 50,
    "update_hz": 1.1
  },
  {
    "mode": "VISUAL_XY_005",
    "segment": "100-150",
    "count": 495,
    "local_t_rmse_m": 0.03810109704492298,
    "visual_updates": 420,
    "duration_s": 50,
    "update_hz": 8.4
  },
  {
    "mode": "VISUAL_XY_005",
    "segment": "150-200",
    "count": 496,
    "local_t_rmse_m": 0.24880026624064308,
    "visual_updates": 423,
    "duration_s": 50,
    "update_hz": 8.46
  },
  {
    "mode": "VISUAL_XY_005",
    "segment": "200-250",
    "count": 495,
    "local_t_rmse_m": 0.26260472994660394,
    "visual_updates": 67,
    "duration_s": 50,
    "update_hz": 1.34
  },
  {
    "mode": "VISUAL_XY_005",
    "segment": "250-300",
    "count": 496,
    "local_t_rmse_m": 0.16747283716272063,
    "visual_updates": 202,
    "duration_s": 50,
    "update_hz": 4.04
  },
  {
    "mode": "VISUAL_XY_005",
    "segment": "300-350",
    "count": 496,
    "local_t_rmse_m": 0.16486065975130604,
    "visual_updates": 55,
    "duration_s": 50,
    "update_hz": 1.1
  },
  {
    "mode": "VISUAL_XY_005",
    "segment": "350-end",
    "count": 661,
    "local_t_rmse_m": 0.33163550175247186,
    "visual_updates": 346,
    "duration_s": 66.72996616363525,
    "update_hz": 5.185076808693993
  },
  {
    "mode": "VISUAL_XYZ_003",
    "segment": "all",
    "count": 4126,
    "local_t_rmse_m": 0.20380761770658545,
    "visual_updates": 1802,
    "duration_s": 416.72996616363525,
    "update_hz": 4.324143081403505
  },
  {
    "mode": "VISUAL_XYZ_003",
    "segment": "150-end",
    "count": 2644,
    "local_t_rmse_m": 0.2536762958338357,
    "visual_updates": 1093,
    "duration_s": 266.72996616363525,
    "update_hz": 4.097777297843839
  },
  {
    "mode": "VISUAL_XYZ_003",
    "segment": "0-50",
    "count": 491,
    "local_t_rmse_m": 0.015106585341447641,
    "visual_updates": 234,
    "duration_s": 50,
    "update_hz": 4.68
  },
  {
    "mode": "VISUAL_XYZ_003",
    "segment": "50-100",
    "count": 496,
    "local_t_rmse_m": 0.02615281781038196,
    "visual_updates": 55,
    "duration_s": 50,
    "update_hz": 1.1
  },
  {
    "mode": "VISUAL_XYZ_003",
    "segment": "100-150",
    "count": 495,
    "local_t_rmse_m": 0.03880811867796314,
    "visual_updates": 420,
    "duration_s": 50,
    "update_hz": 8.4
  },
  {
    "mode": "VISUAL_XYZ_003",
    "segment": "150-200",
    "count": 496,
    "local_t_rmse_m": 0.2583087682736615,
    "visual_updates": 423,
    "duration_s": 50,
    "update_hz": 8.46
  },
  {
    "mode": "VISUAL_XYZ_003",
    "segment": "200-250",
    "count": 495,
    "local_t_rmse_m": 0.28024893234667236,
    "visual_updates": 67,
    "duration_s": 50,
    "update_hz": 1.34
  },
  {
    "mode": "VISUAL_XYZ_003",
    "segment": "250-300",
    "count": 496,
    "local_t_rmse_m": 0.1699008471027737,
    "visual_updates": 202,
    "duration_s": 50,
    "update_hz": 4.04
  },
  {
    "mode": "VISUAL_XYZ_003",
    "segment": "300-350",
    "count": 496,
    "local_t_rmse_m": 0.1828267158706518,
    "visual_updates": 55,
    "duration_s": 50,
    "update_hz": 1.1
  },
  {
    "mode": "VISUAL_XYZ_003",
    "segment": "350-end",
    "count": 661,
    "local_t_rmse_m": 0.31903115652564873,
    "visual_updates": 346,
    "duration_s": 66.72996616363525,
    "update_hz": 5.185076808693993
  },
  {
    "mode": "VISUAL_XYZ_010",
    "segment": "all",
    "count": 4126,
    "local_t_rmse_m": 0.20102347964931355,
    "visual_updates": 1802,
    "duration_s": 416.72996616363525,
    "update_hz": 4.324143081403505
  },
  {
    "mode": "VISUAL_XYZ_010",
    "segment": "150-end",
    "count": 2644,
    "local_t_rmse_m": 0.2502345328076299,
    "visual_updates": 1093,
    "duration_s": 266.72996616363525,
    "update_hz": 4.097777297843839
  },
  {
    "mode": "VISUAL_XYZ_010",
    "segment": "0-50",
    "count": 491,
    "local_t_rmse_m": 0.016120957881922283,
    "visual_updates": 234,
    "duration_s": 50,
    "update_hz": 4.68
  },
  {
    "mode": "VISUAL_XYZ_010",
    "segment": "50-100",
    "count": 496,
    "local_t_rmse_m": 0.02550868745402913,
    "visual_updates": 55,
    "duration_s": 50,
    "update_hz": 1.1
  },
  {
    "mode": "VISUAL_XYZ_010",
    "segment": "100-150",
    "count": 495,
    "local_t_rmse_m": 0.03713956783559252,
    "visual_updates": 420,
    "duration_s": 50,
    "update_hz": 8.4
  },
  {
    "mode": "VISUAL_XYZ_010",
    "segment": "150-200",
    "count": 496,
    "local_t_rmse_m": 0.23370162926974866,
    "visual_updates": 423,
    "duration_s": 50,
    "update_hz": 8.46
  },
  {
    "mode": "VISUAL_XYZ_010",
    "segment": "200-250",
    "count": 495,
    "local_t_rmse_m": 0.25422841972157234,
    "visual_updates": 67,
    "duration_s": 50,
    "update_hz": 1.34
  },
  {
    "mode": "VISUAL_XYZ_010",
    "segment": "250-300",
    "count": 496,
    "local_t_rmse_m": 0.17106061071929993,
    "visual_updates": 202,
    "duration_s": 50,
    "update_hz": 4.04
  },
  {
    "mode": "VISUAL_XYZ_010",
    "segment": "300-350",
    "count": 496,
    "local_t_rmse_m": 0.15108937718579915,
    "visual_updates": 55,
    "duration_s": 50,
    "update_hz": 1.1
  },
  {
    "mode": "VISUAL_XYZ_010",
    "segment": "350-end",
    "count": 661,
    "local_t_rmse_m": 0.3492829185684708,
    "visual_updates": 346,
    "duration_s": 66.72996616363525,
    "update_hz": 5.185076808693993
  }
]
```

## Update effects

```json
[
  {
    "mode": "VISUAL_XYZ_005",
    "metric": "innovation_norm",
    "mean": 0.19229872819216495,
    "rmse": 0.3347999391165762,
    "median": 0.056454365320084784,
    "p95": 0.8560642913456722,
    "max": 1.715202098529681
  },
  {
    "mode": "VISUAL_XYZ_005",
    "metric": "measured_innovation_norm",
    "mean": 0.19229872819216495,
    "rmse": 0.3347999391165762,
    "median": 0.056454365320084784,
    "p95": 0.8560642913456722,
    "max": 1.715202098529681
  },
  {
    "mode": "VISUAL_XYZ_005",
    "metric": "position_correction",
    "mean": 0.07960255716447288,
    "rmse": 0.1403039303744741,
    "median": 0.022798898391050496,
    "p95": 0.3412062530537273,
    "max": 0.8963524139678732
  },
  {
    "mode": "VISUAL_XYZ_005",
    "metric": "velocity_correction",
    "mean": 0.16890764917499376,
    "rmse": 0.2949249924420766,
    "median": 0.047948972939987525,
    "p95": 0.7262366555563203,
    "max": 1.8252065564577569
  },
  {
    "mode": "VISUAL_XYZ_005",
    "metric": "rotation_correction_deg",
    "mean": 0.34133375948582256,
    "rmse": 0.5969248298348809,
    "median": 0.09758183780836058,
    "p95": 1.4769138831102375,
    "max": 3.6376011564380772
  },
  {
    "mode": "VISUAL_XYZ_005",
    "metric": "update_ms",
    "mean": 0.014876568257491677,
    "rmse": 0.015285291671172258,
    "median": 0.014387,
    "p95": 0.015645,
    "max": 0.078572
  },
  {
    "mode": "VISUAL_XY_005",
    "metric": "innovation_norm",
    "mean": 0.1929269359852478,
    "rmse": 0.3351747713584983,
    "median": 0.05783271098378723,
    "p95": 0.8534881040812798,
    "max": 1.7331895831112563
  },
  {
    "mode": "VISUAL_XY_005",
    "metric": "measured_innovation_norm",
    "mean": 0.1899236660499046,
    "rmse": 0.33299807567646267,
    "median": 0.05379878350175341,
    "p95": 0.8515861608455906,
    "max": 1.6862458315289308
  },
  {
    "mode": "VISUAL_XY_005",
    "metric": "position_correction",
    "mean": 0.07884369184063723,
    "rmse": 0.13985127817125978,
    "median": 0.021210088998451115,
    "p95": 0.340517542695709,
    "max": 0.8923220672146094
  },
  {
    "mode": "VISUAL_XY_005",
    "metric": "velocity_correction",
    "mean": 0.16718511220405086,
    "rmse": 0.29357417741737263,
    "median": 0.046337489677883925,
    "p95": 0.7263110763357276,
    "max": 1.813304171509018
  },
  {
    "mode": "VISUAL_XY_005",
    "metric": "rotation_correction_deg",
    "mean": 0.3400364432479709,
    "rmse": 0.5950267303340494,
    "median": 0.09763343750514518,
    "p95": 1.4751009120773122,
    "max": 3.6126819145717417
  },
  {
    "mode": "VISUAL_XY_005",
    "metric": "update_ms",
    "mean": 0.01384979578246393,
    "rmse": 0.013905333190496902,
    "median": 0.013758,
    "p95": 0.014597,
    "max": 0.042185
  },
  {
    "mode": "VISUAL_XYZ_003",
    "metric": "innovation_norm",
    "mean": 0.16655704482491895,
    "rmse": 0.3141208043439378,
    "median": 0.04208219925460592,
    "p95": 0.7937347723948842,
    "max": 2.035711271244649
  },
  {
    "mode": "VISUAL_XYZ_003",
    "metric": "measured_innovation_norm",
    "mean": 0.16655704482491895,
    "rmse": 0.3141208043439378,
    "median": 0.04208219925460592,
    "p95": 0.7937347723948842,
    "max": 2.035711271244649
  },
  {
    "mode": "VISUAL_XYZ_003",
    "metric": "position_correction",
    "mean": 0.08559577120253185,
    "rmse": 0.1662819799226705,
    "median": 0.020340930635576227,
    "p95": 0.3931560521286606,
    "max": 1.2759207721472803
  },
  {
    "mode": "VISUAL_XYZ_003",
    "metric": "velocity_correction",
    "mean": 0.22163141858095262,
    "rmse": 0.4219582152854094,
    "median": 0.053803820534622834,
    "p95": 1.0604955590360292,
    "max": 3.065850616310832
  },
  {
    "mode": "VISUAL_XYZ_003",
    "metric": "rotation_correction_deg",
    "mean": 0.4676864587799748,
    "rmse": 0.8917899250231069,
    "median": 0.11126521500292086,
    "p95": 2.2767192783134207,
    "max": 6.345925409447845
  },
  {
    "mode": "VISUAL_XYZ_003",
    "metric": "update_ms",
    "mean": 0.01469305049944506,
    "rmse": 0.01487878375250948,
    "median": 0.014388,
    "p95": 0.015221549999999988,
    "max": 0.041905
  },
  {
    "mode": "VISUAL_XYZ_010",
    "metric": "innovation_norm",
    "mean": 0.1907555978236258,
    "rmse": 0.3068485475604822,
    "median": 0.07060601093545332,
    "p95": 0.7756722078929954,
    "max": 1.0878772948977515
  },
  {
    "mode": "VISUAL_XYZ_010",
    "metric": "measured_innovation_norm",
    "mean": 0.1907555978236258,
    "rmse": 0.3068485475604822,
    "median": 0.07060601093545332,
    "p95": 0.7756722078929954,
    "max": 1.0878772948977515
  },
  {
    "mode": "VISUAL_XYZ_010",
    "metric": "position_correction",
    "mean": 0.053212153588417396,
    "rmse": 0.08507359592586441,
    "median": 0.01887816056208992,
    "p95": 0.2043517082496467,
    "max": 0.3555525684314313
  },
  {
    "mode": "VISUAL_XYZ_010",
    "metric": "velocity_correction",
    "mean": 0.0881671039146256,
    "rmse": 0.14076047511810538,
    "median": 0.031977108058127326,
    "p95": 0.3495043808189846,
    "max": 0.5830601848473671
  },
  {
    "mode": "VISUAL_XYZ_010",
    "metric": "rotation_correction_deg",
    "mean": 0.16627238264310668,
    "rmse": 0.26609227726777107,
    "median": 0.06210623486960369,
    "p95": 0.6636021438712393,
    "max": 1.0943406799991362
  },
  {
    "mode": "VISUAL_XYZ_010",
    "metric": "update_ms",
    "mean": 0.014502308546059935,
    "rmse": 0.014536956306803937,
    "median": 0.014387,
    "p95": 0.015156,
    "max": 0.029613
  }
]
```
Update timing is C++ offline measurement-update wall time, including covariance checks; it excludes propagation, snapshot, input loading and CSV output. Prior visual frontend cost ~23.6ms is an offline Python/OpenCV measurement, not C++ runtime WCET.

## Provenance

Replay scratch data: `/tmp/p4_i4_visual_fusion_3itaz6bp`
Full trajectory/error traces remain in scratch. Git contains required aggregates, visual-update records, plots, summary and helpers only. Initially replay completed but reporting stopped because the report module was not yet created; report-existing resumed evaluation without repeating filter runs.
Compile: `See p4_i4_visual_fusion.py compile_helper`
Frozen visual SHA256: `962333e1a0a1b4d4824209fa64e85e3a31fd8eeda0813e41214ba85237813f49`
Saved runtime bag SHA256: `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`
Frozen baseline and user dirties untouched. No additional sigma or modes searched. STOP.
