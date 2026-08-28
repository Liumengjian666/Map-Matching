# NDT 先验地图定位已尝试但未保留方法汇总

日期：2026-08-28  
数据集：`/home/jian/rosbag/loop3/bag/loop3_2026-08-11-16-09-07.bag`  
评估参考：FASTLIO2Location `/localization`，仅用于离线评估，不作为实时定位先验  
当前保留默认版本提交：`725aa18`，报告补图提交：`1615103`

## 1. 写这份报告的目的

这份报告单独记录到目前为止为了消除楼梯和重复拐角误匹配而尝试过、但最终没有放进默认算法的配置和方法。判断标准不是只看某一小段，而是尽量以 loop3 全包轨迹与 FASTLIO2Location 的离线对齐误差为主，同时观察楼梯、拐角、末端静止段是否出现跳变或漂移。

当前默认算法只保留被验证有效且运行稳定的主链路：单帧点云预处理、全局先验地图 NDT、上一帧定位结果传播初值、连续帧 step limit 限幅，以及定位结果发布。大量默认关闭且没有带来稳定收益的实验开关已经从 NDT 节点中清理。

## 2. 当前保留基线

当前效果最好的完整验证结果来自：`/home/jian/rosbag/loop3/ndt_finer_map_source_loop3_20260826_044853`。

| 指标 | 数值 |
| --- | ---: |
| mean | 0.417 m |
| RMSE | 0.618 m |
| median | 0.245 m |
| p90 | 1.243 m |
| p95 | 1.535 m |
| max | 2.694 m |
| corrected rate | 约 10 Hz |

该基线相比更早的粗体素参数，在楼梯和拐角处已经明显降低误匹配，但仍未达到全程 `max < 0.5 m` 的最终目标。

## 3. 参数调优类尝试

### 3.1 地图/当前帧体素和 NDT 分辨率

这类实验的目的是增加地图和当前帧点数，让楼梯、拐角、墙面边界等局部结构更丰富，从而降低重复结构误吸附。

| 方法/运行 | 关键配置 | mean | p95 | max | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| 旧基线 | source `0.35`，target `0.25`，resolution `1.0`，source points `900` | 0.646 m | 3.515 m | 5.608 m | 楼梯/拐角误匹配明显 |
| 当前保留 | source `0.25`，target `0.15`，resolution `0.8`，step `0.08`，source points `1400` | 0.417 m | 1.535 m | 2.694 m | 保留，整体最稳 |
| `ndt_fine_target_balanced_loop3_20260826_050015` | source `0.30`，target `0.15`，resolution `0.9`，source points `1200` | 0.552 m | 1.743 m | 5.268 m | 不如当前保留版本 |
| `ndt_finer_res07_loop3_20260826_051104` | source `0.25`，target `0.15`，resolution `0.7`，step `0.06`，source points `1400` | 0.809 m | 4.772 m | 9.139 m | 过细反而更容易被局部错误吸引 |

结论：减小体素和增加点数确实有效，但不是越细越好。`source=0.25`、`target=0.15`、`resolution=0.8` 是目前验证最稳的组合；继续减小 NDT resolution 到 `0.7` 会显著变差。

### 3.2 单帧 step limit 收紧/放宽

这类实验的目的是限制 NDT 结果相对上一帧结果的突变，避免在重复拐角中被错误全局位置吸走。

| 方法/运行 | 关键配置 | mean | p95 | max | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| 当前保留 | translation `0.5 m`，rotation `5 deg` | 0.417 m | 1.535 m | 2.694 m | 保留 |
| `ndt_finer_step035_loop3_20260826_152941` | translation `0.35 m` | 0.468 m | 1.567 m | 4.490 m | 收太紧导致拐角跟不上 |
| `ndt_finer_step075_loop3_20260826_154620` | translation `0.75 m` | 0.544 m | 1.557 m | 8.305 m | 放太宽会让错误吸附扩大 |

结论：step limit 是有效安全带，但只能抑制跳变，不能真正判断哪个重复拐角是正确的。全局只改一个平移阈值会出现两难：太小拐弯被截断，太大错误匹配放大。

## 4. 硬拒绝/冻结类门控

### 4.1 `ndt_acceptance_enable`、`ndt_accept_max_translation`、`ndt_accept_max_rotation_deg`

这几个参数曾重点测试。需要注意：`ndt_accept_max_translation` 和 `ndt_accept_max_rotation_deg` 不是单独生效的软约束，只有打开 `ndt_acceptance_enable: true` 后才会作为硬拒绝门控参与运行。

| 方法/运行 | 关键配置 | mean | p95 | max | corrected rate | 结论 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `ndt_accept_gate_120_12_loop3_20260827_174518` | translation `1.20 m`，rotation `12 deg`，fitness `3.0` | 44.891 m | 101.813 m | 109.825 m | 4.18 Hz | 大量拒绝导致定位冻结/发散 |
| `ndt_accept_trans180_nofitness_loop3_20260827_175624` | translation `1.80 m`，rotation `45 deg`，关闭 fitness 限制 | 0.437 m | 1.547 m | 3.755 m | 9.96 Hz | 比基线差 |
| `ndt_accept_trans205_nofitness_loop3_20260827_180710` | translation `2.05 m`，rotation `45 deg`，关闭 fitness 限制 | 0.434 m | 1.545 m | 3.755 m | 9.98 Hz | 比基线差 |

结论：硬拒绝门控不适合当前问题。部分看起来“跳得大”的帧实际上是基线恢复路径的一部分，拒绝后会造成后续更大误差；门控太严则直接降低修正频率，导致轨迹冻结和发散。因此默认不启用 acceptance gate。

### 4.2 large-jump guard + coast

| 方法/运行 | 关键配置 | mean | p95 | max | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| `ndt_finer_guard12_decay08_loop3_20260826_155711` | 大跳变检测后用恒速 coast，decay `0.8` | 0.746 m | 3.924 m | 7.077 m | coast 会累计漂移 |

结论：coast 可以暂时避免被单帧 NDT 拉走，但如果误判或连续几帧没有可靠修正，漂移会累计，反而比直接使用 step-limited NDT 更差。

## 5. 条件放宽/条件限幅类尝试

### 5.1 confident step limit

该方法尝试在 NDT fitness 较低、旋转较小、看起来“有信心”时临时放宽平移限幅，避免拐角处被 `0.5 m` 截断。

| 方法/运行 | 关键配置 | mean | p95 | max | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| `ndt_confident_step065_loop3_20260826_162040` | 低残差/小旋转时放宽到 `0.65 m` | 0.427 m | 1.541 m | 5.317 m | 最大误差增大 |
| `ndt_confident_step055_strict_loop3_20260826_163117` | 更严格条件下放宽到 `0.55 m` | 0.456 m | 1.558 m | 5.465 m | 仍不如基线 |

结论：NDT fitness 在重复结构中并不总能代表“匹配正确”。错误拐角也可能有较好的 fitness，所以用低残差作为放宽依据不稳定。

### 5.2 Z step clamp

| 方法/运行 | 关键配置 | mean | p95 | max | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| `z_step_limit_loop3_20260826_032709` | 对单帧 Z 方向变化单独限幅 | 3.120 m | 25.569 m | 33.549 m | 拒绝 |

结论：楼梯场景需要真实的 Z 变化。单独限制 Z 会让楼层变化跟不上，并且会把误差转移到横向轨迹，造成长段漂移。

## 6. 初值和运动先验类尝试

### 6.1 多初值 NDT candidate selection

该方法对应“如果多个位置都满足 NDT，就优先选靠近 IMU/前端传播初值的候选”的思路。实现上曾围绕运动传播初值生成多个局部候选，用综合代价选择：`NDT fitness + 最近地图残差 + 运动先验距离 + yaw 偏差`。

| 方法/运行 | 关键配置 | mean | p95 | max | corrected rate | 结论 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `candidate_prior_select_loop3_20260826_024729` | 多初值候选，触发过于频繁 | 35.723 m | 83.314 m | 92.934 m | - | 严重发散 |
| `candidate_prior_gated_loop3_20260826_025957` | 只在预测跳变时触发候选 | 0.710 m | 3.529 m | 7.125 m | - | 比旧基线差 |
| `ndt_prior_candidate_v1_loop3_20260827_073817` | permissive trigger | 24.801 m | 97.224 m | 106.016 m | 7.46 Hz | 严重发散 |
| `ndt_prior_candidate_v2_loop3_20260827_075039` | stricter trigger | 13.143 m | 69.408 m | 77.004 m | 8.77 Hz | 严重发散 |
| `ndt_prior_candidate_v3_loop3_20260827_080308` | 加 hard prior-distance gate | 11.472 m | 69.516 m | 81.974 m | 8.71 Hz | 严重发散 |

结论：思路本身合理，但当前 10 Hz 在线节点里额外跑多次 NDT 会降低修正频率，而且候选代价仍不能可靠区分重复走廊/重复拐角。候选离运动先验更近，也不一定是真实位置；一旦选错，会把后续传播链条带到错误区域。

### 6.2 直接使用 `/livox/imu` 陀螺传播初始姿态

| 方法/运行 | 关键配置 | mean | p95 | max | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| `imu_initial_candidate_xy_loop3_20260826_031436` | 将 IMU gyro 传播混入 NDT 初始 yaw/姿态 | 4.677 m | 29.370 m | 39.228 m | 拒绝 |

结论：直接积分原始 IMU 对 bias、时间同步和坐标系非常敏感。没有完整惯导预积分/滤波状态约束时，只把 gyro 增量硬塞给 NDT 初值，反而会破坏已有的上一帧位姿传播初值。

### 6.3 motion-prior guard

该方法不额外跑 NDT，只在 raw NDT 远离运动传播初值、且 NDT 残差优势不明显时，保留或偏向运动先验。

| 方法/运行 | 关键配置 | mean | p95 | max | corrected rate | 结论 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `ndt_motion_prior_guard_v4_loop3_20260827_081544` | motion prior guard，无额外 NDT | 0.496 m | 1.558 m | 5.476 m | 9.86 Hz | 拒绝 |

窗口对比显示，该方法在 345-351 s 窗口变化很小，但 398-403 s 和 565-576 s 窗口明显变差。说明仅靠“靠近运动先验”和“最近地图残差”仍不足以区分重复结构。

### 6.4 motion tube guard

该方法后来也作为默认关闭实验分支加入过：当 NDT 相对运动初值偏移超过平移、旋转或 Z 阈值，并且残差没有明显优势时，把结果投影回运动先验附近的“运动管道”。典型配置包括：

- `ndt_motion_tube_guard_enable: false`
- `ndt_motion_tube_trigger_translation: 0.45`
- `ndt_motion_tube_trigger_rotation_deg: 7.0`
- `ndt_motion_tube_trigger_z: 0.20`
- `ndt_motion_tube_max_translation: 0.25`
- `ndt_motion_tube_max_rotation_deg: 3.0`
- `ndt_motion_tube_max_z: 0.12`

结论：该方法本质上仍是“运动先验硬约束”。它可能会压制真实楼梯/拐角运动，也可能把必要的 NDT 修正截断。由于没有比当前基线更稳定的 full-bag 改善，后续已回滚并在清理版本中删除。

## 7. 时序一致性类尝试

### 7.1 short-window temporal consistency

该方法把远离传播初值的 NDT 修正视为风险帧，先暂时保留运动先验，只有最近几帧支持同一修正方向时才接受。

| 方法/运行 | trigger | mean | p90 | p95 | max | corrected rate | 结论 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `ndt_temporal_consistency_v1_loop3_20260827_160832` | 0.70 m | 0.444 m | 1.379 m | 1.545 m | 4.143 m | 9.99 Hz | 拒绝 |
| `ndt_temporal_consistency_v2_loop3_20260827_161953` | 1.10 m | 0.436 m | 1.446 m | 1.552 m | 3.468 m | 9.92 Hz | 拒绝 |
| `ndt_temporal_consistency_v3_loop3_20260827_163119` | 1.60 m | 0.419 m | 1.264 m | 1.535 m | 2.694 m | 9.95 Hz | 中性，无改善 |

结论：短滑窗不增加 NDT 计算量，算力上安全；但剩余拐角误差不是单帧离群点，而是连续几帧都局部合理的重复结构误匹配。因此简单时序一致性不能稳定区分正确/错误位置。

## 8. 双分辨率候选类尝试

该方法试图把旧版本粗 NDT 和当前细 NDT 结合：风险帧先跑当前细 NDT，再额外跑一套更粗的旧版 NDT candidate，用 `fitness + 残差 + 运动先验距离 + yaw + Z` 选择最终结果。典型配置包括：

- `ndt_dual_resolution_candidate_enable: false`
- `ndt_dual_resolution_source_voxel_size: 0.35`
- `ndt_dual_resolution_target_voxel_size: 0.25`
- `ndt_dual_resolution_resolution: 1.0`
- `ndt_dual_resolution_max_source_points: 900`
- `ndt_dual_resolution_prior_weight: 0.8`
- `ndt_dual_resolution_z_weight: 0.5`

结论：这个方法没有被保留。原因是它引入第二套 NDT target/source 和额外配准分支，复杂度明显上升，且在默认关闭时仍带来代码维护成本和潜在副作用。后续已通过 `0bbfe08` 回滚，并在清理提交 `725aa18` 中删除。

## 9. 旧版本回退验证

用户提到 `12154e5` 之前版本在部分拐角看起来更好，因此单独在旧版本 worktree 上跑过当前 loop3 全包验证。

| 方法/运行 | mean | p95 | max | 结论 |
| --- | ---: | ---: | ---: | --- |
| `commit12154e5_real_loop3_20260826_040054` | 4301.764 m | 12847.841 m | 13986.607 m | 不支持直接回退 |

结论：旧版本可能在某些局部视觉观察上更自然，但放到当前 full-bag 评估流程和坐标对齐方式下严重不稳定。因此不能直接回退到旧提交作为默认算法。

## 10. 已清理的默认关闭配置/代码

清理前曾存在大量默认 `false` 的实验开关，它们在验证后没有成为稳定收益项，清理提交 `725aa18` 已将 NDT 节点恢复为更简洁的默认链路。主要清理项包括：

| 已清理分支/配置 | 原始目的 | 未保留原因 |
| --- | --- | --- |
| `ndt_multiframe_source_enable` | 多帧源点云增加几何结构 | 全包中容易引入拖影，重复走廊更易误匹配，算力增加 |
| external initial guess | 使用外部位姿话题作为初值 | 不符合只用 LiDAR/IMU/image/先验地图的约束 |
| internal ICP odom | 用内部 ICP 前端辅助初值 | 室内重复结构下不够稳定，增加复杂度 |
| `ndt_acceptance_enable` | 对异常 NDT 帧硬拒绝 | 容易冻结或错过恢复帧 |
| `ndt_coast_on_reject_enable` | 拒绝后恒速滑行 | 会累计漂移 |
| `ndt_confident_step_limit_enable` | 低残差时放宽限幅 | fitness 不能可靠代表正确匹配 |
| `ndt_large_jump_guard_enable` | 大跳变保护 | 和 coast/拒绝结合后容易漂移或冻结 |
| `ndt_ambiguity_check_enable` | 搜索附近候选诊断歧义 | 只适合诊断，不能直接改善定位，增加算力 |
| `ndt_temporal_consistency_enable` | 短滑窗一致性验收 | 对连续合理的重复结构误匹配区分力不足 |
| `ndt_motion_prior_guard_enable` | 偏向运动传播初值 | 会压制必要 NDT 修正，部分窗口变差 |
| `ndt_prior_candidate_enable` | 多初值候选择优 | 算力下降且候选代价不可靠 |
| dual-resolution candidate | 粗细 NDT 候选融合 | 复杂度高，未验证出稳定收益 |
| motion tube guard | 把风险结果投影回运动先验管道 | 容易截断真实拐弯/上楼运动，未保留 |

## 11. 总体结论

1. 真正有效并保留的是：适度减小 source/map 体素、增加当前帧点数、NDT resolution 调到 `0.8`，并保留 `0.5 m / 5 deg` 的连续帧 step limit。
2. 没有效果或效果不稳定的主要方向是：硬拒绝、coast、单纯收紧/放宽阈值、直接 IMU 陀螺初值、多初值 NDT 候选、运动先验硬约束、短滑窗一致性和双分辨率候选。
3. 这些失败实验说明，剩余误匹配不是普通的单帧噪声，而是重复楼道/重复拐角中 NDT 局部最优本身具有迷惑性；单靠 NDT fitness、最近地图残差或单帧运动约束无法稳定解决。
4. 下一步更合理的方向是引入独立信息源或更高层约束，例如视觉位置识别/角点语义、楼梯高度语义、离线短窗联合优化，或只在少数高风险区域启用更强但可验证的候选判别。
