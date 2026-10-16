# P47 复现协议

## 固定路径与数据

- 仓库：`/home/jian/livox_ws/p10_corridor01_oosm_r1_ws`
- 数据：`/media/jian/HIKVISION/paper rosbag/NewerCollege/01_short_experiment`
- P46 冻结运行：`/media/jian/HIKVISION/paper rosbag/NewerCollege/.p46_real_tile_switch/formal1800`
- P47 外部 artifact root：`/media/jian/HIKVISION/paper rosbag/NewerCollege/.p47_coupled_shadow_r1/frozen`
- 原始 Ouster IMU：`/media/jian/HIKVISION/paper rosbag/NewerCollege/01_short_experiment/raw_format/ouster_imu/data.csv`

提交/复核前先确认 P45 tile manifest、P46 diagnostics、输入 SHA 与 `sample_freeze.json` 相符。Manifest 中保存真实原始 PCD 路径、P46 frame index、stamp、Deskew 输入/输出和初值/结果；不应另行重新挑选帧。

## 生成与重建命令接口

研究工具依赖本仓库的 ROS Noetic/Catkin、PCL、Eigen 与 OpenSSL。CMake 源码中的研究工具构建选项默认关闭。本次使用的既有 Catkin build cache 明确设为 `DOG_PRIOR_BUILD_RESEARCH_TOOLS=ON`，实际构建命令是：

```bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
cmake --build build --target p47_reconstruct_frozen_sources p47_coupled_shadow_probe -j2
```

新 build tree 需要先按该工作区既有 Catkin/ROS 环境配置，并显式传入 `-DDOG_PRIOR_BUILD_RESEARCH_TOOLS=ON`；不要把此实验 cache 状态误认为默认启用。若系统存在 MVS SDK 的 libusb 符号冲突，运行时使用该工作区已验证的 `LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0`。

该版完整固定输入已存于归档。重新生成样本冻结记录时，`freeze_samples.py` 的参数顺序是：dataset root、P46 formal1800 run、P45 tile manifest、输出目录。Source 重建工具参数顺序为：冻结 `sample_manifest.csv`、原始 Ouster IMU CSV、Source 输出目录、Source parity CSV。工具会拒绝覆盖已有 parity 输出；复现实验应使用新且明确的输出目录，不覆盖本归档。

Shadow probe 参数顺序为：sample manifest、Source parity、tile catalog、Source 目录，以及 5 个分别用于 degeneracy、paired、candidate、runtime、Nominal parity 的新 CSV 输出路径。可执行文件拒绝覆盖已有 CSV。实际运行采用固定 `retry3` artifact 命名；输出 SHA 见 `shadow_output_freeze.json`。

GT 的唯一后冻结读取命令：

```bash
python3 docs/p47_newer_college_coupled_shadow/tools/evaluate_shadow_accuracy.py \
  --repo-root /home/jian/livox_ws/p10_corridor01_oosm_r1_ws \
  --artifact-root /media/jian/HIKVISION/paper rosbag/NewerCollege/.p47_coupled_shadow_r1/frozen \
  --freeze /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/docs/p47_newer_college_coupled_shadow/shadow_output_freeze.json \
  --gt /media/jian/HIKVISION/paper rosbag/NewerCollege/01_short_experiment/ground_truth/registered_poses.csv \
  --output /media/jian/HIKVISION/paper rosbag/NewerCollege/.p47_coupled_shadow_r1/frozen/conditional_gt_shadow_accuracy.csv
```

该脚本在读取 GT 前先逐个验证所有 frozen input/output SHA，并要求 P43 GT SHA 及精确时间戳匹配。它只评 frame 1 的 Nominal/Weak-only/Coupled 三个位姿。

评估器拒绝覆盖已有输出；重复运行时必须选择新的目标路径。

## 关键隔离保证

- `DOG_PRIOR_BUILD_RESEARCH_TOOLS` 默认 `OFF`。
- P47 probes 不接 ROS topic，不发布位姿，不写回运行时状态。
- Native jet 在 PCL `align()` 初始化后计算；每次 align 都参与计时和计数，并由 parity gate 与冻结 P46 Nominal 核验。
- 没有完整 P46 回放、没有 NDT 参数更改、没有候选反馈、没有 GT 选帧或 GT 调参。

归档内 CSV 是实际运行产物；HIKVISION 上的原始 PCD、IMU 与 P45 tile 不提交仓库。
