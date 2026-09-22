# PAPER-P2B Corridor01 baseline results

## 固定输入与边界

frozen paper HEAD：`8e0c1661a2ddac1d669d752d3b0a7ad771cb7409`。frozen delivery baseline 未修改；本阶段仅使用 paper workspace 的既有算法可执行文件和 adapter workspace。输入为官方 raw bag、normalized map、官方 VLP-16 converter、上游 rotational deskew；baseline 内部 `deskew_enable=false`，没有改变 NDT/EKF/OOSM 参数。

## 30 s smoke

结果目录：`/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/baseline_smoke_20260923_final/`。

```text
duration: 30.0 s
adapter scans: 297
NDT odom: 297
EKF corrected: 287
IMU: 5991
deskew success: 903/903 rows in the running adapter diagnostics (first row only missing=1 leading-gap flag)
NaN/Inf pose: none observed
NDT/EKF process crash: no
```

离线 GT 结果（评价起点 `t_init+5 s = 1517157224.18898`，不使用 bag record time）：

| source | aligned translation mean/RMSE/P95/max (m) | aligned rotation mean/median/P95/max (deg) | 结论 |
|---|---:|---:|---|
| NDT | 2.17697 / 2.36457 / 3.75326 / 4.36374 | 4.99943 / 2.17174 / 21.61029 / 24.18193 | smoke 连续 |
| EKF | 2.18706 / 2.37402 / 3.72444 / 4.52829 | 5.09837 / 2.26033 / 21.63885 / 23.97145 | smoke 连续 |

raw（无对齐）平移 median 约 15.17 m，说明这是 map/frame 初始差异，不能把 raw 数值当作算法精度。固定尺度 SE(3) 对齐只用于离线诊断，未做 scale correction。根据本阶段 gate，`SMOKE_STATUS = PASS`。

## Full Run A

结果目录：`.../results/frozen_baseline_runA_20260923/`；实际时长 279 s，2776 NDT/deskew rows，2765 corrected odom rows，55943 adapted IMU。

```text
NDT RSS mean 77035.8 KiB, peak 78424; CPU mean 8.32%, peak 10.9%
EKF RSS mean 13906.1 KiB, peak 14088; CPU mean 2.43%, peak 3.1%
adapter RSS mean 22817.2 KiB, peak 26720; CPU mean 8.84%, peak 11.5%
NDT aligned translation mean/RMSE/median/P95/max = 62.92530/70.98377/51.18971/129.42614/137.95245 m
NDT aligned rotation mean/median/P95/max = 109.96039/96.10124/177.08280/179.87805 deg
NDT 1-frame RPE median/P95 = 0.21497/0.53341 m; rotation median = 0.65549 deg
NDT 1-second RPE median/P95 = 2.00304/4.87725 m; rotation median = 1.32209 deg
EKF aligned translation mean/RMSE/median/P95/max = 62.90743/70.96218/51.24298/129.49911/137.87817 m
EKF aligned rotation mean/median/P95/max = 109.86678/96.09008/177.10603/179.95507 deg
```

完整序列后段存在明显累计漂移/框架不一致；它不是 smoke 阶段的启动崩溃。因为 Run A/B 未通过确定性 gate，不能把这些数值直接作为论文最终 baseline 结论。

## Full Run B 与重复性

Run B 结果目录：`.../results/frozen_baseline_runB_20260923/`；2777 NDT rows，2766 corrected odom rows，55957 IMU。Run B NDT aligned translation mean/RMSE/P95/max 为 `62.92530/70.98377/129.42614/137.95245 m`；EKF 为 `62.87815/70.93123/129.55496/137.86948 m`。

逐帧共同 NDT header timestamp 比较：

```text
common NDT timestamps: 2776
cloud hash mismatch: 2 (first frame 1517157219.1889789; one frame 1517157306.024142)
first startup initial-guess component difference: 0.94349 m
after startup branch, NDT pose max position difference: 0.03195 m
NDT quaternion component max difference: 0.01797
EKF corrected common samples: 1869
EKF corrected max position difference: 0.09112 m
```

```text
DETERMINISTIC = NO
CLASSIFICATION = PUBLIC_DATA_NONDETERMINISM / startup scheduling sensitive
```

Run B 不是另一个算法版本；它说明需要先处理首帧/回调调度和 cloud hash 差异，再进行论文级 failure screening。

## Failure screening 现状

当前不能把完整 Run A 的大误差直接标成自然退化失败：全局 aligned 误差从评价起点即受初始 frame/map 关系影响，且 A/B 还没有逐帧确定性。`failure_windows_runA.csv` 和 `failure_windows_runB.csv` 已保存为诊断材料，但不是最终 Mode-Structured 标签。下一步应优先做 `PAPER-P2B-INITIALIZATION-FIX`，而不是调 NDT 或进入新算法。

## 代码与主机保护

```text
sudo apt: NO
pip install: NO
conda: NO
delivery baseline modified: NO
paper core source modified: NO
paper config/launch/CMake/package modified: NO
Robustness_Metric cloned for audit only; official runner not executed because pyhocon is absent.
```
