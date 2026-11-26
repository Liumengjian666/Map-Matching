# R3 固定候选资产与受控验证计划

## 资产与边界

- **H307 原生竞争终值**。H307_N1406 与 H307_N1739 的原生奖励/源点数为 2.643805 和 2.629745，相对分差 0.532%。两者均 2,726/2,726 点有关联，原生局部优化均 3 次迭代并报告收敛。相隔 2.084859 m / 7.926857°。两者均超出参考容差，提供相近原生评分的不同终点。候选视角含相似的平面，特定重复结构的因果贡献仍需独立验证。
- **H307 判别性试验主对照**。Park 排名产生的 H307_P2648 经原生优化后进入参考容差，原生奖励 2.632594，低于 H307_N1406 的 2.643805。两位置相隔 2.011102 m / 2.938899°。这一对优先检验固定共享源点空间中的判别证据与随后可验证的真实结构对应关系。
- **H515 边界/误排序对照**。原始 H515_N2425 高分错误位姿为 0.27743 m / 5.51344°。H515_N2200 原生第 2 名，偏航在 −5° 网格边界，离线误差 0.05077 m / 1.12829°，奖励 1.756586 低于 1.799226。这份资产固定记录 Q1 网格状态。Q2 通用 C3 规则保留边界候选并执行一次标准原生精修。4 次迭代后奖励升至 1.801405，有效点降至 3,539/3,849。相对种子移动 0.197 m / 4.149°，超出 4° 模式保持门槛，参考误差变为 0.196 m / 4.787°。因此保留正确粗格作边界对照，该自然精修终值未通过模式保持及正确性。 [精修收据](H515_boundary_fit_receipt.csv) · [真实前后对比](r3_fixed_assets/H515_boundary_before_after.png)

[候选六自由度矩阵及计数](fixed_candidate_manifest.csv) · [候选间相对位姿](fixed_candidate_pairs.csv) · [Q1 精修记录](fixed_candidate_q1_refinements.csv) · [逐文件哈希](r3_asset_inventory.csv)

## 坐标与采样合同

两个样本都来自 Hard 深度相机序列。此资产包用符号 L 表示源点云实际坐标 depth_camera_link，统一对应历史定位接口的传感器坐标。该包使用相机源云及固定地图，固定候选图所需的唯一变换为地图与传感器之间的刚体位姿。每帧使用对应时间戳的 Q1 源快照。源点索引来自已核对哈希的 0.1 m 原生 PCL VoxelGrid（点云体素化滤波器）质心。P63 归档输入合同明确 Hard 为 Azure Kinect 瞬时深度快照，H307/H515 源头时间戳分别为 1690354450397440974 / 1690354464464374712 ns，未新增逐点时间去畸变。历史传播把 imu_link 变换至 depth_camera_link，外参 T_L_I 的 xyz/xyzw 为 [0.003, 0.004, −0.051, −0.476, 0.474, 0.524, 0.525]，四元数归一化。此静态局部图导出不读取 IMU，不重复应用外参。历史参考来自带地图锚定的通用迭代最近点与惯性解，缺少独立动捕验证。 [P63 输入合同](/home/jian/.local/share/Trash/files/Map-Matching-R14-first-loss-heldout/docs/p63_r10_hdl_imu_full_hard/INPUT_AND_FRAME_CONTRACT.md)

$T_{ML}$ 作用方向是传感器至地图。对同一块既有 Hard 地图，用 $T^{-1}_{ML}$ 转入候选视角，再以 8 m 闭球等距裁剪。两边使用同一个预先存在的原始图和同一裁剪规则，保留其全部点。原生高斯栅格始终取自共同完整地图，经相同逆变换输出中心和协方差 $R^{-1}\Sigma(R^{-1})^\top$，避免在各候选裁剪图重新估计体素。原生栅格 ID 保持其世界图语义。每个局部地图做 $T$ 和 $T^{-1}$ 往返核对。两份地图的无约束配准、可见性判别或遮挡补全均未执行。

PCD（点云数据文件）地图对叠加用蓝/橙表示 A/B，用绿色表示实际源云。逐点奖励差图以 $\Delta s_i=s_i(A)-s_i(B)$ 定义，最大化奖励下蓝偏 A，红偏 B。颜色仅表示匹配证据。区域固定为传感器系的 0.5 m 半开立方体，零点和边界不随候选变化。每个候选对的未缩放原生奖励、空间箱贡献和相消量均已导出。

## 逐点判别诊断

| 对照 | A / B 支持点数 | 近中性点数 | 奖励净差 $\sum_i \Delta s_i$ | 近中性点净差 | 点级奖励相消 |
|---|---:|---:|---:|---:|---:|
| H307 原生竞争终值 | 1,246 / 1,352 | 128 | +38.33 | −0.21 | 98.19% |
| H307 正确对错误终值 | 1,314 / 1,227 | 185 | −30.56 | −0.49 | 97.54% |
| H515 正确边界对错误首选 | 1,393 / 1,697 | 759 | −164.12 | −0.33 | 88.17% |

支持点以 $|\Delta s_i|>0.05$ 的符号划分。相消以 $1-|\sum_i \Delta s_i|/\sum_i |\Delta s_i|$ 计算。把近中性门槛改为 0.01 或 0.10，H307 正误对的近中性净差为 +0.03 / +0.48，H515 为 −0.16 / −0.87。前五个空间区只占各区绝对净贡献总量的 13.4% / 13.3%，绝对点贡献最强的 1% 也只占 4.2% / 6.0%。两例均呈现空间分散、正负相消和净排序偏差。按这批固定门槛，近中性点群没有主导误排。需要 R3 识别空间区域的真实几何语义和可重复性。

[匹配贡献表](discriminative_score_diagnostics.csv)

## 经批准后的试验顺序

1. 固定原始候选及共享源点 ID，分别复核同位姿自比较与相同物理面的小扰动对照。使用实际点云共同可见区域，记录视野、扫描空洞、遮挡、体素跨界及两边局部支持。
2. 从同一实际源快照估信息，剔除不可比较的地图缺面及低点数片区。每个判别区域都需要保留位置误差、地图误差和观测噪声预算，单独观察厘米级法向误差对结论的影响。
3. 保留 H307 两个错误终值作为模式分离阴性对照，用 H307 正误对检验差异证据方向。H515 作为边界失败对照。若统一规则下正确格点精修消失，保留其粗格身份。
4. 将几何差异检测与真实扫描可重复性绑定。逐点原生贡献只能帮助定位诊断区域。两对高相消的正误对目前均未发现预注册近中性小差值占据总误排的证据。
5. 为后续选点独立冻结邻域规则和阈值。若已知候选残余误差在目标法向超过拟检测尺度，先做有界精修及误差传播试验。

## 本地复现与查看

工作区保持原始 Q1 数据在原路径。以下示例从本交付目录运行，输出到新的实验目录。

```bash
mkdir -p /tmp/p64_q2_fixed_repro/docs
bash build_fixed_candidates.sh /tmp/p64_q2_fixed_repro/fixed_candidate_export
python3 -B fixed_candidate_assets.py plan /tmp/p64_q2_fixed_repro/plan.tsv
env -u LD_LIBRARY_PATH OMP_NUM_THREADS=1 /tmp/p64_q2_fixed_repro/fixed_candidate_export /tmp/p64_q2_fixed_repro/plan.tsv /tmp/p64_q2_fixed_repro/native
python3 -B fixed_candidate_assets.py analyze /tmp/p64_q2_fixed_repro/native /tmp/p64_q2_fixed_repro/plan.tsv /tmp/p64_q2_fixed_repro/docs /tmp/p64_q2_fixed_repro/pcd
env -u LD_LIBRARY_PATH pcl_viewer /tmp/p64_q2_fixed_repro/pcd/H307_native_competitors_maps_and_scan.pcd
env -u LD_LIBRARY_PATH pcl_viewer /tmp/p64_q2_fixed_repro/pcd/H515_boundary_rank_native_reward_delta.pcd
```

已生成本地 PCD 位于 /home/jian/p64_r2_q2_ws/run/r3_pcd，归档也保留可先用 gzip 解压的 PCD。全图、完整源云、处理后原生逐点数据的哈希可由索引复核。资产导出对 5 个固定姿态各执行一次 Native、Park 和原生只读求导核对，无精修调用。

[H307 竞争终值叠加](r3_fixed_assets/H307_native_competitors_overlay.png) · [H307 正误终值叠加](r3_fixed_assets/H307_correct_wrong_overlay.png) · [H515 边界对叠加](r3_fixed_assets/H515_boundary_rank_overlay.png) · [H515 累积贡献](r3_fixed_assets/H515_boundary_rank_regional_rewards.png)

H515 的 Q2 前后对照可由已封存搜索输出复现。把如下输入放在发布的同一哈希关系下，输出放在含本节固定候选资产的新的 docs 和 pcd 目录。

```bash
python3 -B plot_h515_boundary.py /home/jian/p64_r2_q2_ws/run/online_seal.json /home/jian/p64_r2_q2_ws/run/official/fit.csv /tmp/p64_q2_fixed_repro/docs /tmp/p64_q2_fixed_repro/pcd
```
