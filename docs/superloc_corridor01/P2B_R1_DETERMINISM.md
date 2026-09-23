# PAPER-P2B-R1 deterministic replay audit

## Input determinism

Run C 和 Run D 播放前都检查了：

```text
60048c6369ea1065cc4dee2484ff38360620fa6b7321abc05b71e874f2b06345  corridor01_adapted_v1.bag
```

derived bag 只包含 `/superloc_adapter/points_deskewed`（2776）和 `/superloc_adapter/imu_lidar`（55946）。PointCloud2 与 IMU header 时间均严格单调、无非有限值和倒退时间。输入审计的完整 hash 在 `derived_bag_manifest.txt`。

**INPUT_DETERMINISM = PASS**。

## Run C / Run D

| run | result bag | NDT | corrected EKF | OOSM |
|---|---|---:|---:|---|
| C | `frozen_baseline_derived_runC_20260923` | 2776 | 2765 | APPLIED 2765, NO_HISTORY 11 |
| D | `frozen_baseline_derived_runD_20260923` | 2776 | 2764 | APPLIED 2764, NO_HISTORY 11, FUTURE_MEASUREMENT 1 |

两次 NDT `ndt_determinism.csv` 的 2776 行共同时间戳中，cloud hash、initial guess、raw NDT、fitness、converged、iterations、translation/rotation limited、final pose 全部 mismatch = 0，数值最大差 = 0。仅 `ros_now`/`wall_time` 因独立回放时钟不同而变化；四元数角度比较的浮点残差为约 `3.42e-6 deg`。

**NDT_DETERMINISM = PASS**。

## EKF timestamp mechanism and comparison

代码审计结果：普通 corrected publish 使用 `msg->header.stamp`；OOSM replay 完成后使用当前 `t_now`；IMU high-rate 使用 IMU header stamp。对应代码为 `fusion/ndt_observation.cpp:222,275`、`imu_processor.cpp:84` 和 `ros_output.cpp:10`。因此 corrected topic 不是严格等间隔、可能有重复 sensor stamp，也不能只按 exact timestamp intersection 判定。

C/D 的 corrected stamp 都没有负时间差，但分别有 18/17 个重复 stamp。采用“按 stamp 排序、重复 stamp 保留最后一条、在 2776 个共同 NDT stamp 上线性平移 + quaternion SLERP 插值”的比较方法，得到 2763 个共同有效 EKF 样本：

- position difference mean/RMSE/median/P95/max = `0.002683/0.015865/0/0.009110/0.329965 m`
- rotation difference mean/RMSE/median/P95/max = `0.027741/0.153248/0/0.096478/2.935237 deg`

这说明 EKF 的大部分输出近似确定，但仍有少数 OOSM 调度差异，故本阶段保守标为：

**EKF_DETERMINISM = NEAR-DETERMINISTIC**。

Run C 的资源采样文件为空；Run D 有采样，但采样器记录格式没有可靠的 PID/字段标题，因此不把它作为本阶段确定性结论。该缺陷不影响输入和 NDT 几何审计。
