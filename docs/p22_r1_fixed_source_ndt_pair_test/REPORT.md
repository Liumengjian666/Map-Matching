# P22：固定 Source 的 NDT 双初值实验未建立

TASK: P22-R1-FIXED-SOURCE-NDT-INITIALIZATION-PAIR-TEST

FINAL_RESULT: **PAIR_TEST_NOT_ESTABLISHED**

START_SHA: `39c5eee473b379a1962d2333984ebd8492b9f36d`

CODE_SHA: `10f23927d222aaf6de4ded074aedf571de8f329c`

仓库：Liumengjian666/Map-Matching；分支：research/p9-r4-heldout-visual-evidence。
只使用隔离实验副本；未修改稳定机器狗或主论文工作区。
本报告不预先声称提交已经上传；实际 END_SHA 与推送状态由交付回执核实。

## 实际执行和停止门槛

冻结 8 帧后，对每帧完成一次 Source 恢复尝试；全部得到 1400 点，但
**0/8** 恢复点云通过 P19 stableCloudHash 验收。少于 4 帧，触发
SOURCE_RECONSTRUCTION_FAILED / COVERAGE_INSUFFICIENT，立即停止。

完整 NDT align：**0**。A 组优化器复现：未执行。B 组 EKF 初值构造：未执行。
GT 读取和误差评价：未执行。完整 ROS 回放：0；EKF 观测更新：0。
没有新增扫描、第三种初值、参数搜索或第二套恢复方法。
未生成 optimizer_parity、paired_ndt_results、step_limit_response 等未执行产物。

这不是 NDT 或 EKF 初值的负精度结果。它只说明本次限定恢复方法没有得到
原 P19 的相同 Source，故不能合法比较两个初值的优化响应。

## 输入和原参数

已核验冻结输入 SHA256：

- SSD 地图：`103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f`，4058751 bytes。
- V2 bag：`7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e`。
- visual.csv：`10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79`。
- P18 配置：`dc0850f91b43a4afed2e692644ac292db96a599e33fdafb414f3a986bc743220`。

NDT 冻结参数：resolution=0.8、step=0.08、epsilon=0.001、max_iterations=40。
Source voxel XYZ=0.25m，最多 1400 点，range=0.5–80m，无额外 Z 截断。
地图/target voxel XYZ=0.15m；Step Limit 平移 0.5m、旋转 5deg。
velocity_blend=0.6；scan reference=start；所有原配置和正式运行源码保持不变。
这些是原合同参数；本轮没有构建 NDT target 或执行优化，不能声明 target 点云复现已通过。

## 全部固定扫描和实际恢复结果

frame_index 是 P19 原记录；时间相对原评价起点 1517157224.188979。
所有选定帧无直接视觉应用，后七帧均为固定目标附近唯一最近非视觉帧。
选择冻结于任何 GT 读取之前；没有按误差筛选。

| frame | scan stamp | relative s | P19 hash | 恢复 hash | 原/恢复点数 |
|---|---|---:|---|---|---|
| 2 | 1517157219.289839 | -4.899139881 | d80390e3ce0d7677 | 2470a471839674ed | 1400/1400 |
| 284 | 1517157247.730654 | 23.541675091 | eddf619035f6142f | 3339821e929e1111 | 1400/1400 |
| 293 | 1517157248.6383331 | 24.449354172 | a3b7d078ddcdfb61 | 7a50db5838612150 | 1400/1400 |
| 296 | 1517157248.940913 | 24.751934052 | d821afba10e04398 | 01d0b95673517b01 | 1400/1400 |
| 308 | 1517157250.1511171 | 25.962138176 | 0d5d884de016936c | 4ab8bef58d5ea3ec | 1400/1400 |
| 347 | 1517157254.0844071 | 29.895428181 | eade330395549e4b | 5f8c82d3d8ee7f3a | 1400/1400 |
| 368 | 1517157256.2023449 | 32.013365984 | d5aac1979f3df7b5 | bd61719538fa79f3 | 1400/1400 |
| 396 | 1517157259.026242 | 34.837263107 | 5851dfe8307ce5c0 | fab2e810ec71ac4a | 1400/1400 |

每帧有序 float32 XYZ 的 SHA256 见 source_reconstruction.csv；原始 XYZT、
状态片段及恢复 XYZ 仅保存在 `.p22_r1_fixed_source_ndt_pair_test/`，不提交点云。
第 2 帧位于主评价区间外，仅为原计划中的诊断样本；本轮所有帧均未进入误差评价。

## 恢复方法与证据限制

P19 outputs.bag 只有诊断、NDT odometry、corrected 与 high-rate odometry，
没有 filtered/source 点云。原冻结输入 bag 含 396 扫描与 8005 IMU 消息。
本轮仅提取所选 8 扫描，使用 P19 自身公开高频状态片段和对应原 IMU。
没有使用 P21 分化历史。通过链接原 core/imu_deskew.cpp、state_history.cpp、
imu_propagation.cpp，调用既有逐点补偿/部分时间插值；没有重跑完整滤波轨迹。
finalizeCloud、voxelDown、preprocess、stableCloudHash 函数文本与当前生产节点一致，
由测试核实。Source probe 不含 NDT align 或 NDT 观测更新入口。

公开日志并非内部 deskew 历史的无损备份：

1. writeEkfPredictionLineage 将内部旋转转换为归一化四元数；恢复时再构建矩阵，
   不能保证原矩阵 bitwise 一致。
2. 公开高频状态是已发布的当前时刻状态；实际内部 state_history 可能经 OOSM
   擦除/重写中间状态。这些重写后的中间快照没有全部独立归档。

上述限制意味着不能将本轮恢复视图当成原内部历史，也不能把 8 个差异
唯一归因于 OOSM、四元数表示或其他单一机制。没有证明一切其他恢复方法均不可行。
本轮不扩展恢复调查。唯一直接缺少的资产是原扫描进入 NDT 前的有序 float32 XYZ
及其 width/height/is_dense/hash 元信息。

## 编译、测试和独立复核

只编译独立 source probe，未重新编译 NDT/EKF。环境没有 pcl_filters pkg-config
条目，改用已有 PCL 1.10 / Eigen include 与库路径后编译通过；不是数学修改。

```bash
g++ -O2 -g -Wall -O3 -DNDEBUG -I src/dog_prior_map_localization/include -I /usr/include/eigen3 -I /usr/include/pcl-1.10 src/dog_prior_map_localization/scripts/p22_fixed_source_pair/source_probe.cpp src/dog_prior_map_localization/src/core/state_history.cpp src/dog_prior_map_localization/src/core/imu_deskew.cpp src/dog_prior_map_localization/src/core/imu_propagation.cpp -lpcl_filters -lpcl_common -lpcl_search -lpcl_kdtree -o /tmp/p22_source_probe
python3 src/dog_prior_map_localization/scripts/p22_fixed_source_pair/test_contract.py
```

针对性测试 **5/5 PASS**：原函数一致、原参数一致、固定唯一非视觉选择、
无 align/filter update、实际停止回执无 GT/配对声明。
独立只读复核重查了 8 帧选择、点数、FNV、XYZ SHA、预处理和停止边界。
用户选择仅本轮独立审查；没有运行外部跨模型 CLI。
本工具仅针对冻结 bag 格式，不是通用点云恢复库。

单帧恢复进程（包含启动）mean=19.760291500ms，max=24.172847989ms，
P95 见 runtime_summary.csv。**不是 NDT 耗时**，峰值 RSS 未测。

## 科研判断与唯一 NEXT

Source一致：FAIL；Target冻结地图身份：PASS（target构建未执行）；
Previous Used/IMU旋转：已选取原 P19 记录（配对未执行）；
Optimizer parity：NOT_EVALUATED；Pair test：NOT_EXECUTED。

不能回答哪些帧 EKF 初值更准确、原生 NDT 是否保留其优势、Step Limit 是否改变
优化收益，不能新增完整轨迹精度结论，不能证明唯一根因。
GT 原定合同仍是 Control V2 0–3 秒固定 SE(3)、IMU 参考点、前缀对齐相对漂移，
但本轮没有使用 GT。

按科研总控合同结束初值替换研究，不追加恢复方法、回放或第三初值。
唯一 NEXT：请科研总控核查真实停止证据后，制定有界的 Coupled Subspace
Refinement 实验合同，针对弱约束/错误局部匹配检验修正机制；新实验应直接保存
实际 Source，避免再依赖历史公开状态恢复。本轮不实施下一阶段，也不宣称创新成立。
