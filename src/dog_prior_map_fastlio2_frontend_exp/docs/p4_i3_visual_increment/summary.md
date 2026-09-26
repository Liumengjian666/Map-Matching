# PAPER-P4-I3 — LiDAR-depth visual increment viability

Verdict: `SPARSE_VISUAL_SIGNAL`. Offline only; no runtime changes, NDT rerun, IKFoM replay or rosbag playback.

Start SHA: `03d38dd5a7df0c81ce1d5cf0ba5dff18c54dd451`; branch `paper`. Synthetic PnP direction and MEI rectification: PASS.
Supplement follows completed first-pass commit `8def495a82245d52772c3bc78334adfaf797cce9`; no reset or rewrite to the original algorithm baseline. Only instrumentation and post-hoc evaluation are supplemented.
## Camera and source lineage

Camera: `{"topic": "/cmu_sp1/camera_1/image_raw", "type": "sensor_msgs/Image", "frame": "d", "encoding": "bgr8", "width": 640, "height": 480, "count": 10017, "rate_hz": 23.998741394978317}`.
Canonical manifest `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_full_baseline/floor01_canonical_input_manifest.csv` explicitly identifies the following three contiguous source shards; all are required for the same R10B trajectory:
- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/raw/extracted/Multi_Floor_Rosbag/raw_data_core_2022-08-18-17-16-31_0.bag` (3422674923 bytes)
- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/raw/extracted/Multi_Floor_Rosbag/raw_data_core_2022-08-18-17-19-00_1.bag` (3442739076 bytes)
- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/raw/extracted/Multi_Floor_Rosbag/raw_data_core_2022-08-18-17-21-29_2.bag` (2783923015 bytes)
The R7H `floor01_runtime_provenance.md` records these shards -> canonical input -> R10B source. No bag was chosen by filename alone.

## Calibration and fixed conventions

Calibration PASS: official MEI model, K=[[771.6611713881042, 0.0, 317.73333375994946], [0.0, 688.7864250675316, 233.53737010585292], [0.0, 0.0, 1.0]], D(k1,k2,p1,p2)=[-0.04701957438555236, 0.28058825271927323, 0.0004063627333225579, -0.00015239154341021015], xi=1.5967783801524584.
OpenCV 4.2.0, ccalib omnidir, perspective rectification. Virtual K=[[297.160965789772, 0.0, 317.73333375994946], [0.0, 265.24651866020713, 233.53737010585292], [0.0, 0.0, 1.0]]; same image size and principal point; focal lengths fixed to gamma/(1+xi), the central angular scale, before inspecting GT. Identity rectification rotation.
T_imu_camera (p_I = T_IC p_C): `[[0.00373945, 0.0024439, 0.99999002, 0.17409098], [-0.99989234, -0.0141801, 0.00377374, 0.04962189], [0.01418918, -0.99989647, 0.00239061, -0.02152957], [0.0, 0.0, 0.0, 1.0]]`.
T_imu_lidar (p_I = T_IL p_L): `[[0.999945562, 0.009074807, 0.005149763, 0.08], [-0.009060897, 0.999955255, -0.002718066, 0.029], [-0.005174199, 0.002671256, 0.999983046, 0.03], [0.0, 0.0, 0.0, 1.0]]`.
Projection uses T_CL = inverse(T_IC) T_IL. Nearest positive Z per rounded pixel, then nearest projected point within 2 px. Its Z depth is assigned along the rectified feature ray. No depth completion.
Shi-Tomasi 500/0.01/10; KLT 21x21/maxLevel=3, criteria=(30,0.01), FB<=1 px; PnP EPNP RANSAC 100/2 px/0.99, >=30 correspondences and >=20 inliers, LM on RANSAC inliers; fixed per-transaction RNG seed. No parameter search.
## Time and evaluation semantics

Image-scan absolute mismatch ms mean/median/P95/max: 10.713091 / 10.430411 / 19.8795335 / 40.480952.
<=20 ms scan coverage: 3936/4127 = 95.371941%. Pairing uses adjacent original scans only; missing matches are never bridged.
PnP returns T_Ccur_Cref. The GT camera observation is inverse(T_WI_cur T_IC) (T_WI_ref T_IC) evaluated at actual image header timestamps. Camera residuals are retained separately.
Primary visual metrics conjugate inverse(PnP) to the IMU origin: D_I_visual = T_IC inverse(T_Ccur_Cref) inverse(T_IC), compared with inverse(GT_I_ref) GT_I_cur. GT follows the existing R10C IMU-origin interpretation. No global alignment is needed for increments.
IKFoM errors are copied from existing R10C for the same adjacent scan pair, with timestamp/dt assertions, not recomputed. Actual image intervals differ slightly from scan intervals (each endpoint <=20 ms); primary visual scoring uses image times. Additional visual_scan_gt columns report sensitivity using scan-time GT and the same fixed visual estimate.
Magnitude columns without camera in their name use the IMU origin; visual_camera/gt_camera magnitude columns use the camera origin. These must not be mixed.
Visual status is finalized before any GT is loaded. No GT extrapolation. Coverage denominator is ALL adjacent scan pairs, including synchronization failures; paired accuracy requires both methods and GT. GT availability never changes visual validity.

## All and late metrics

```json
{
  "all": {
    "segment": "all",
    "total": 4126,
    "attempted": 3751,
    "visual_valid": 1802,
    "coverage": 0.4367426078526418,
    "paired": 1802,
    "visual_t_rmse_m": 0.029304442897347738,
    "visual_r_rmse_deg": 0.7531981892919931,
    "ikfom_t_rmse_m": 0.24818593134766298,
    "ikfom_r_rmse_deg": 0.7101010271037413,
    "visual_scan_gt_t_rmse_m": 0.0341267076256552,
    "visual_scan_gt_r_rmse_deg": 0.8640972306841428,
    "visual_better_t": 1490,
    "ikfom_better_t": 312,
    "visual_better_r": 848,
    "ikfom_better_r": 954
  },
  "post150": {
    "segment": "150-end",
    "total": 2645,
    "attempted": 2402,
    "visual_valid": 1093,
    "coverage": 0.41323251417769374,
    "paired": 1093,
    "visual_t_rmse_m": 0.027546745736187844,
    "visual_r_rmse_deg": 0.7356761598226121,
    "ikfom_t_rmse_m": 0.31762127430235737,
    "ikfom_r_rmse_deg": 0.6685680878840835,
    "visual_scan_gt_t_rmse_m": 0.031503714417223584,
    "visual_scan_gt_r_rmse_deg": 0.8260903778058167,
    "visual_better_t": 1038,
    "ikfom_better_t": 55,
    "visual_better_r": 483,
    "ikfom_better_r": 610
  },
  "gates": {
    "A": false,
    "B": true,
    "C": true
  },
  "post150_translation_improvement": 0.9132717233859943,
  "post150_rotation_ratio": 1.100375822828929
}
```

## 50-second segments

| Segment | valid/total | coverage | Visual t RMSE | IKFoM t RMSE | Visual r RMSE | IKFoM r RMSE |
|---|---:|---:|---:|---:|---:|---:|
| 0-50 | 234/490 | 47.755% | 0.00942434 | 0.0144492 | 0.442124 | 0.421099 |
| 50-100 | 55/496 | 11.089% | 0.0234697 | 0.0211259 | 0.78332 | 0.708915 |
| 100-150 | 420/495 | 84.848% | 0.0398508 | 0.0395701 | 0.914496 | 0.914121 |
| 150-200 | 423/496 | 85.282% | 0.0270196 | 0.270894 | 0.908783 | 0.87383 |
| 200-250 | 67/495 | 13.535% | 0.0246936 | 0.265221 | 0.668547 | 0.68731 |
| 250-300 | 202/496 | 40.726% | 0.0421445 | 0.169788 | 0.691319 | 0.543568 |
| 300-350 | 55/496 | 11.089% | 0.0347011 | 0.178583 | 0.950948 | 0.726423 |
| 350-end | 346/662 | 52.266% | 0.0125747 | 0.43983 | 0.436685 | 0.361439 |

## Failure coverage

```json
[
  {
    "interval": "all",
    "threshold_m": 0.25,
    "count": 1086,
    "visual_valid": 551,
    "evaluated": 551,
    "visual_better": 551,
    "better_rate": 1.0
  },
  {
    "interval": "all",
    "threshold_m": 0.4,
    "count": 662,
    "visual_valid": 340,
    "evaluated": 340,
    "visual_better": 340,
    "better_rate": 1.0
  },
  {
    "interval": "150-end",
    "threshold_m": 0.25,
    "count": 1086,
    "visual_valid": 551,
    "evaluated": 551,
    "visual_better": 551,
    "better_rate": 1.0
  },
  {
    "interval": "150-end",
    "threshold_m": 0.4,
    "count": 662,
    "visual_valid": 340,
    "evaluated": 340,
    "visual_better": 340,
    "better_rate": 1.0
  }
]
```

## Frontend and wall time

Status counts: `{'VALID': 1802, 'SYNC_INVALID': 375, 'INSUFFICIENT_CORRESPONDENCES': 1906, 'PNP_REJECT': 43}`.
Attempted-pair statistics (not just successful poses):
```json
{
  "detected": {
    "mean": 295.02905891762197,
    "rmse": 339.01245544671934,
    "median": 266.0,
    "p95": 500.0,
    "max": 500.0
  },
  "klt_valid": {
    "mean": 193.12156758197813,
    "rmse": 240.24574917086065,
    "median": 158.0,
    "p95": 484.0,
    "max": 500.0
  },
  "klt_forward_valid": {
    "mean": 265.5609170887763,
    "rmse": 307.9996844995562,
    "median": 238.0,
    "p95": 499.0,
    "max": 500.0
  },
  "fb_valid": {
    "mean": 193.12156758197813,
    "rmse": 240.24574917086065,
    "median": 158.0,
    "p95": 484.0,
    "max": 500.0
  },
  "depth_associated": {
    "mean": 33.277525993068515,
    "rmse": 41.12216909716297,
    "median": 29.0,
    "p95": 78.5,
    "max": 109.0
  },
  "pnp_inliers": {
    "mean": 23.60677152759264,
    "rmse": 35.84775934662147,
    "median": 0.0,
    "p95": 72.0,
    "max": 109.0
  },
  "inlier_ratio": {
    "mean": 0.43753481892588425,
    "rmse": 0.6311059465447625,
    "median": 0.0,
    "p95": 1.0,
    "max": 1.0
  },
  "reprojection_rmse_px": {
    "mean": 0.45691485560693434,
    "rmse": 0.5211041165529747,
    "median": 0.4626436515877438,
    "p95": 0.8967515986294355,
    "max": 1.2703828459849937
  }
}
```
Runtime is offline wall time, with OpenCV and BLAS configured single-thread. Includes both image rectifications conservatively (even when the reference is cached), feature/KLT/depth/PnP and small orchestration overhead; excludes bag I/O, GT scoring and plotting. It is not C++ runtime WCET.
```json
[
  {
    "component": "feature_ms",
    "mean": 6.242740493391546,
    "rmse": 6.558278796837452,
    "median": 6.13659099326469,
    "p95": 9.716514497995377,
    "max": 25.74387501226738
  },
  {
    "component": "klt_ms",
    "mean": 9.390182214951038,
    "rmse": 10.986745585320813,
    "median": 8.048863004660234,
    "p95": 19.534403501893394,
    "max": 34.88883702084422
  },
  {
    "component": "depth_ms",
    "mean": 3.909261128069114,
    "rmse": 3.9942575388763375,
    "median": 3.7766329769510776,
    "p95": 4.865852490183897,
    "max": 13.418380985967815
  },
  {
    "component": "projection_ms",
    "mean": 2.3956962002390885,
    "rmse": 2.4547747760351304,
    "median": 2.289557975018397,
    "p95": 3.1664235139032826,
    "max": 7.250709022628143
  },
  {
    "component": "association_ms",
    "mean": 1.4957574898453658,
    "rmse": 1.550533602430133,
    "median": 1.4409089926630259,
    "p95": 1.7801309877540916,
    "max": 11.14900698303245
  },
  {
    "component": "pnp_ms",
    "mean": 0.4034392515963236,
    "rmse": 0.8158808263802879,
    "median": 0.0,
    "p95": 1.3870254915673286,
    "max": 11.134269996546209
  },
  {
    "component": "preprocess_ms",
    "mean": 3.653072053737272,
    "rmse": 3.842772869666,
    "median": 3.1315030064433813,
    "p95": 6.111273498390801,
    "max": 19.600649015046656
  },
  {
    "component": "total_ms",
    "mean": 23.620814138628834,
    "rmse": 24.56524339568647,
    "median": 22.647363017313182,
    "p95": 35.36149348656181,
    "max": 51.75838799914345
  }
]
```

## Input hashes

- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r10b_fix1_floor01_full_rerun_20260926/floor01_fix1_runtime_topics.bag` SHA256 `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`
- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_full_baseline/floor01_canonical_input_manifest.csv` SHA256 `7ff3f63fa2034b29d28c5de8922424e0ad608ddcc059c34edfd4c91def103b1e`
- `/home/jian/livox_ws/dog_loc_paper_ws/src/dog_prior_map_fastlio2_frontend_exp/docs/p3_r10c_failure_mechanism_1/p3_r10c_local_increment_analysis.csv` SHA256 `20b73837340d42ab20ede6ac6ef5efdbeb11ee97c057ffb070962e893c6a2ca3`
- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r10b_fix1_floor01_full_rerun_20260926/evaluation_inputs/predictor.csv` SHA256 `0d6b418a973eafad3746c56c423aadc3d8c80e85be7ea0614001786dc4ad771b`
- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r10b_fix1_floor01_full_rerun_20260926/evaluation_inputs/corrected.csv` SHA256 `ff61f3fc72ec2b0c4c9e7a99f54e0866d696bb8999cf1f7a16001cacd3a26416`
- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/gt/floor01_gt.txt` SHA256 `b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f`
- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_intrinsics.yaml` SHA256 `04c5865d2d1f3864c8b1a5f2348b8455d400394be94c58313e8d3323b446b059`
- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_extrinsics.yaml` SHA256 `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414`

## Nearest-time pose and directional supplement

Reconstructed 1802 valid paired increments from saved corrected/predictor poses; scalar translation/rotation residuals agree with the existing R10C values within 1e-5 m / 1e-4 deg.
For each sync-valid visual pair, independently select nearest saved corrected(ref image) and predictor(cur image), each <=20 ms, and require their IDs to equal the original adjacent scans. This is the task-authorized nearest-endpoint approximation, NOT exact image-time propagation. No interpolation across NDT correction discontinuities, no doubled scan interval, no new IKFoM run.
Absolute endpoint offset and interval-length difference (ms), over attempted pairs:
```json
{
  "ikfom_ref_image_offset_ms": {
    "mean": 10.141971564916023,
    "rmse": 11.63478060004368,
    "median": 10.201621,
    "p95": 19.027417,
    "max": 19.988987
  },
  "ikfom_cur_image_offset_ms": {
    "mean": 10.131967691282325,
    "rmse": 11.630092677620913,
    "median": 10.204294,
    "p95": 19.0441115,
    "max": 19.988987
  },
  "image_minus_ikfom_dt_ms": {
    "mean": 20.019413929618768,
    "rmse": 20.194868021970784,
    "median": 19.42347100000001,
    "p95": 21.473610500000003,
    "max": 31.42118000000002
  }
}
```
Residual E = inverse(GT_delta) * estimated_delta. e_visual_xyz/e_ikfom_xyz are E translation, in metres. Both are expressed in the common GT-current-scan IMU/body basis: image-time visual residual is rotated by Q=R_GT_scan_cur^T R_GT_image_cur; rotation residual is conjugated Q E_R Q^T. GT is used only in post-hoc scoring, never to alter a measured visual pose or validity.
roll/pitch/yaw are extrinsic xyz Euler angles (degrees) of the residual rotation, NOT differences between pose Euler angles and NOT the SO(3) geodesic angle. Component RMSE squares therefore need not sum to the squared geodesic RMSE.
```json
[
  {
    "segment": "all",
    "method": "visual",
    "count": 1802,
    "x_rmse": 0.02131520419520814,
    "y_rmse": 0.01342055651032026,
    "z_rmse": 0.014976685434160536,
    "roll_rmse": 0.494128033665079,
    "pitch_rmse": 0.4044684920884164,
    "yaw_rmse": 0.39936805525875146
  },
  {
    "segment": "all",
    "method": "ikfom",
    "count": 1802,
    "x_rmse": 0.2134094524441692,
    "y_rmse": 0.12359551974909865,
    "z_rmse": 0.02787130467590151,
    "roll_rmse": 0.4697400860185599,
    "pitch_rmse": 0.3738628850569315,
    "yaw_rmse": 0.37920567899759344
  },
  {
    "segment": "150-end",
    "method": "visual",
    "count": 1093,
    "x_rmse": 0.02103386446383668,
    "y_rmse": 0.01239815936443186,
    "z_rmse": 0.01275481833446805,
    "roll_rmse": 0.4800279916975253,
    "pitch_rmse": 0.3929831063438897,
    "yaw_rmse": 0.39536616012750453
  },
  {
    "segment": "150-end",
    "method": "ikfom",
    "count": 1093,
    "x_rmse": 0.2735560267104256,
    "y_rmse": 0.15809610504524932,
    "z_rmse": 0.032496087601121604,
    "roll_rmse": 0.4477628730757159,
    "pitch_rmse": 0.3515314527059933,
    "yaw_rmse": 0.3506534932116078
  },
  {
    "segment": "0-50",
    "method": "visual",
    "count": 234,
    "x_rmse": 0.007327422504414689,
    "y_rmse": 0.003636799232248677,
    "z_rmse": 0.00467981579722015,
    "roll_rmse": 0.3076831465325578,
    "pitch_rmse": 0.22794640824135953,
    "yaw_rmse": 0.22145735336528488
  },
  {
    "segment": "0-50",
    "method": "ikfom",
    "count": 234,
    "x_rmse": 0.012806917289992618,
    "y_rmse": 0.004765563688796327,
    "z_rmse": 0.004695995720500016,
    "roll_rmse": 0.29423541028466815,
    "pitch_rmse": 0.2326032139180865,
    "yaw_rmse": 0.1915949933323742
  },
  {
    "segment": "50-100",
    "method": "visual",
    "count": 55,
    "x_rmse": 0.01852272007648185,
    "y_rmse": 0.00868415437270678,
    "z_rmse": 0.01150299767440119,
    "roll_rmse": 0.6085077559314795,
    "pitch_rmse": 0.3967948085028095,
    "yaw_rmse": 0.2930777677169693
  },
  {
    "segment": "50-100",
    "method": "ikfom",
    "count": 55,
    "x_rmse": 0.01237949785009634,
    "y_rmse": 0.009112951885434591,
    "z_rmse": 0.01449161762593164,
    "roll_rmse": 0.5143977740283324,
    "pitch_rmse": 0.40908985396647213,
    "yaw_rmse": 0.2655304524427755
  },
  {
    "segment": "100-150",
    "method": "visual",
    "count": 420,
    "x_rmse": 0.02689106676597895,
    "y_rmse": 0.018854586589233138,
    "z_rmse": 0.022571218571943823,
    "roll_rmse": 0.5887967103220578,
    "pitch_rmse": 0.5004318151129904,
    "yaw_rmse": 0.4888220057628693
  },
  {
    "segment": "100-150",
    "method": "ikfom",
    "count": 420,
    "x_rmse": 0.023421032585562985,
    "y_rmse": 0.021731520905045945,
    "z_rmse": 0.02334508170830587,
    "roll_rmse": 0.5848741090267896,
    "pitch_rmse": 0.4754448983944168,
    "yaw_rmse": 0.5170002044097608
  },
  {
    "segment": "150-200",
    "method": "visual",
    "count": 423,
    "x_rmse": 0.015083015134688512,
    "y_rmse": 0.014647892593922044,
    "z_rmse": 0.016970582265923597,
    "roll_rmse": 0.6309776475593535,
    "pitch_rmse": 0.4719411678234007,
    "yaw_rmse": 0.452699973868767
  },
  {
    "segment": "150-200",
    "method": "ikfom",
    "count": 423,
    "x_rmse": 0.22009911360295026,
    "y_rmse": 0.15485512755633482,
    "z_rmse": 0.030985355291798886,
    "roll_rmse": 0.5988474074144993,
    "pitch_rmse": 0.43688351023734256,
    "yaw_rmse": 0.46271830536347697
  },
  {
    "segment": "200-250",
    "method": "visual",
    "count": 67,
    "x_rmse": 0.020929930024627005,
    "y_rmse": 0.008391071276847623,
    "z_rmse": 0.010064806934387988,
    "roll_rmse": 0.43041933833004875,
    "pitch_rmse": 0.39845858863291067,
    "yaw_rmse": 0.32100392420693824
  },
  {
    "segment": "200-250",
    "method": "ikfom",
    "count": 67,
    "x_rmse": 0.19578517046219696,
    "y_rmse": 0.16489935941571027,
    "z_rmse": 0.06941521270325372,
    "roll_rmse": 0.47935368732091765,
    "pitch_rmse": 0.3640026774678082,
    "yaw_rmse": 0.33308687946316634
  },
  {
    "segment": "250-300",
    "method": "visual",
    "count": 202,
    "x_rmse": 0.03829230414163109,
    "y_rmse": 0.014125509283061347,
    "z_rmse": 0.010503783428607487,
    "roll_rmse": 0.41345563133496666,
    "pitch_rmse": 0.3823932359135875,
    "yaw_rmse": 0.40116120409926703
  },
  {
    "segment": "250-300",
    "method": "ikfom",
    "count": 202,
    "x_rmse": 0.12648526160539486,
    "y_rmse": 0.11184317691540821,
    "z_rmse": 0.01789966928488599,
    "roll_rmse": 0.34498502317877905,
    "pitch_rmse": 0.33321073109968946,
    "yaw_rmse": 0.2555227625890409
  },
  {
    "segment": "300-350",
    "method": "visual",
    "count": 55,
    "x_rmse": 0.02433810040074849,
    "y_rmse": 0.0173803736709672,
    "z_rmse": 0.017599562269101317,
    "roll_rmse": 0.5502938851365164,
    "pitch_rmse": 0.3964287938525432,
    "yaw_rmse": 0.6658625015463101
  },
  {
    "segment": "300-350",
    "method": "ikfom",
    "count": 55,
    "x_rmse": 0.17536657231999228,
    "y_rmse": 0.017961294040434395,
    "z_rmse": 0.02856439103318848,
    "roll_rmse": 0.468324876857302,
    "pitch_rmse": 0.35572395541290647,
    "yaw_rmse": 0.4263935769147685
  },
  {
    "segment": "350-end",
    "method": "visual",
    "count": 346,
    "x_rmse": 0.009189042497141805,
    "y_rmse": 0.0067176319225931355,
    "z_rmse": 0.005344010861783033,
    "roll_rmse": 0.2395059902200306,
    "pitch_rmse": 0.27288666937426126,
    "yaw_rmse": 0.2426124047157635
  },
  {
    "segment": "350-end",
    "method": "ikfom",
    "count": 346,
    "x_rmse": 0.39435809849170916,
    "y_rmse": 0.19240550471973591,
    "z_rmse": 0.030204046538027433,
    "roll_rmse": 0.21465479935387033,
    "pitch_rmse": 0.2154775430538649,
    "yaw_rmse": 0.19534422785898328
  }
]
```
klt_forward_valid counts successful forward LK status flags. fb_valid additionally requires backward status, FB<=1px, finite image bounds (same as klt_valid). projection_ms includes cloud decoding, extrinsic projection and pixel z-buffer; association_ms includes KD-tree construction/query and feature-ray depth assignment. depth_ms retains their combined wrapper time.

## Scope and interpretation

Timestamp sensitivity (150s-end): scoring the unchanged visual estimate against scan-time GT gives rotation RMSE 0.826090 deg versus IKFoM 0.668568 deg, ratio 1.235611. The image-time rotation gate must not be presented as a timestamp-insensitive guarantee; the final sparse-coverage verdict is unchanged.
LiDAR depth comes from saved scan-end IMU-deskewed request clouds. It carries the existing inertial deskew assumptions, so visual estimates are not statistically independent of IMU. This is a fixed-parameter signal viability test on one sequence; no fusion or novelty claim.
OpenCV reference: https://docs.opencv.org/4.5.5/d3/ddc/group__ccalib.html .
Outputs: visual_increment.csv, segment_metrics.csv, sync_stats.csv, failure_subset.csv, runtime_metrics.csv, runtime_breakdown.csv, directional_metrics.csv and four plots named 01..04 (old plot aliases retained). Raw images, bags and point clouds are not included.
