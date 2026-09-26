# PAPER-P4-I3 — LiDAR-depth visual increment viability

Verdict: `SPARSE_VISUAL_SIGNAL`. Offline only; no runtime changes, NDT rerun, IKFoM replay or rosbag playback.

Start SHA: `03d38dd5a7df0c81ce1d5cf0ba5dff18c54dd451`; branch `paper`. Synthetic PnP direction and MEI rectification: PASS.
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
    "mean": 6.635077111651129,
    "rmse": 6.912233407202525,
    "median": 6.379151018336415,
    "p95": 10.17177599715069,
    "max": 28.564434993313625
  },
  {
    "component": "klt_ms",
    "mean": 9.516790855490466,
    "rmse": 11.134927638676077,
    "median": 8.15027198405005,
    "p95": 19.6480670128949,
    "max": 32.0718259899877
  },
  {
    "component": "depth_ms",
    "mean": 3.9843776616266866,
    "rmse": 4.074808363277758,
    "median": 3.8541559770237654,
    "p95": 4.999808501452208,
    "max": 13.490105018718168
  },
  {
    "component": "pnp_ms",
    "mean": 0.408222242449049,
    "rmse": 0.8239345381128531,
    "median": 0.0,
    "p95": 1.402355992468074,
    "max": 14.393789984751493
  },
  {
    "component": "preprocess_ms",
    "mean": 3.756524479518883,
    "rmse": 3.959293746529779,
    "median": 3.192823991412297,
    "p95": 6.197666487423703,
    "max": 21.94160598446615
  },
  {
    "component": "total_ms",
    "mean": 24.32496969271101,
    "rmse": 25.281143209137127,
    "median": 23.386982997180894,
    "p95": 36.46229751757346,
    "max": 57.77532598585822
  }
]
```

## Input hashes

- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r10b_fix1_floor01_full_rerun_20260926/floor01_fix1_runtime_topics.bag` SHA256 `860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db`
- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r7_floor01_full_baseline/floor01_canonical_input_manifest.csv` SHA256 `7ff3f63fa2034b29d28c5de8922424e0ad608ddcc059c34edfd4c91def103b1e`
- `/home/jian/livox_ws/dog_loc_paper_ws/src/dog_prior_map_fastlio2_frontend_exp/docs/p3_r10c_failure_mechanism_1/p3_r10c_local_increment_analysis.csv` SHA256 `20b73837340d42ab20ede6ac6ef5efdbeb11ee97c057ffb070962e893c6a2ca3`
- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/gt/floor01_gt.txt` SHA256 `b8db2491cdb3ca3194f653cab90f31eae70b457e91b2f87f30ed7fe2d0e2ce9f`
- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_intrinsics.yaml` SHA256 `04c5865d2d1f3864c8b1a5f2348b8455d400394be94c58313e8d3323b446b059`
- `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_extrinsics.yaml` SHA256 `fd9fbfb209d6715b4812a92d458cdeb32cbfd75160356a87802c2bda5dfa7414`

## Scope and interpretation

Timestamp sensitivity (150s-end): scoring the unchanged visual estimate against scan-time GT gives rotation RMSE 0.826090 deg versus IKFoM 0.668568 deg, ratio 1.235611. The image-time rotation gate must not be presented as a timestamp-insensitive guarantee; the final sparse-coverage verdict is unchanged.
LiDAR depth comes from saved scan-end IMU-deskewed request clouds. It carries the existing inertial deskew assumptions, so visual estimates are not statistically independent of IMU. This is a fixed-parameter signal viability test on one sequence; no fusion or novelty claim.
OpenCV reference: https://docs.opencv.org/4.5.5/d3/ddc/group__ccalib.html .
Outputs: visual_increment.csv, segment_metrics.csv, sync_stats.csv, failure_subset.csv, runtime_metrics.csv and four PNG plots. Raw images, bags and point clouds are not included.
