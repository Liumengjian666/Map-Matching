# P47 源码改动与工具说明

## 代码改动范围

运行定位代码未改动：

- `dog_prior_map_ndt_node.cpp` 的 CONTROL NDT、Deskew 输入、EKF/OOSM、地图选择和 NDT 反馈路径保持原状。
- 没有修改 Coupled 数学内核；离线 probe 复用现有 `runWeakCoupledRefinement()`、`ObservablePclNdt::nativeJet()` 与注册几何代码。
- 没有让候选进入 EKF 或下一帧初值。

`src/dog_prior_map_localization/CMakeLists.txt` 在已存在的 `DOG_PRIOR_BUILD_RESEARCH_TOOLS`（默认 `OFF`）条件下新增两个离线 executable：

1. `p47_reconstruct_frozen_sources`：按 frozen manifest 重建点云 Source，重用 P46 Deskew/IMU propagation，并逐帧检查 stable hash。
2. `p47_coupled_shadow_probe`：重建 P46 的活动 Local Target，计算 Native jet、执行 Nominal parity gate，并对固定候选样本调用既有 Weak-only/Coupled 数学内核。

新增工具源码：

- `tools/freeze_samples.py`：只按预注册帧号和真实 tile 更新行冻结样本；不读取 GT。
- `tools/reconstruct_frozen_sources.cpp`：使用 P46 已有轨迹/IMU样本和 P46 Deskew 实现生成 Source；逐帧 hash 不匹配即失败。
- `tools/coupled_shadow_probe.cpp`：研究工具独立可执行；检查 P45 tile SHA、active tile ID、Source/Target hash/点数、pose 有限刚体性、solver status，并输出 Hessian、候选、parity 和耗时 CSV。
- `tools/evaluate_shadow_accuracy.py`：唯一读取 P43 条件性 GT 的步骤；先验证 freeze manifest 中所有输入和 Shadow 输出 SHA，再要求 GT 路径必须出现在 P43 冻结清单且 SHA 匹配，检验精确 GT 时间戳后输出单帧误差；拒绝覆盖既有评估文件。

研究工具构建命令：

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
cmake -S src/dog_prior_map_localization -B build-p47 -DDOG_PRIOR_BUILD_RESEARCH_TOOLS=ON -DCATKIN_ENABLE_TESTING=OFF
cmake --build build-p47 --target p47_reconstruct_frozen_sources p47_coupled_shadow_probe -j2
```

实际实验使用 P44/P45 已构建工作树中的相同源码并启用该选项；运行时需按本机 PCL/MVS 环境设置 `LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0`，仅解决动态库装载冲突，不改系统库。

## 源码出处

- CONTROL 中 Native derivative 的生产调用顺序在 `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp` 的 `runMatureIntegration()` 一带：`setInputSource`、真实 `align()`，然后在成熟集成中调用 Native jet。
- 已有 Weak/Coupled 算法定义在 `src/dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_weak_refinement.cpp`，强方向求解位于 `coupledStrongStep()`；Coupled arm 在弱位移后重新计算 jet，并使用零增量求解强方向残差。
- 本轮未修改以上两个运行/数学源文件。

`results.json` 中记录了二进制 SHA、冻结样本、输入与输出哈希、Nominal parity 误差、运行计数及限制。完整 probe 输出和大体量 Source/Tile 数据位于 HIKVISION 外置盘；大文件未加入 Git。
