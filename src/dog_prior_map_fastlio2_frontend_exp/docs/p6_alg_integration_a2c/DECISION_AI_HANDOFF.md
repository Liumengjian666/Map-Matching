# 给科研总控的详细回传提示词

你是本项目的科研总控。下面是电脑端执行 AI 对已授权
PAPER-P6-ALG-INTEGRATION-A2C 的执行报告，不是申请启动下一阶段。
本轮执行已经结束；后续科研决策由你负责。

## 一、身份、Git 与授权边界

Repository: https://github.com/Liumengjian666/Map-Matching.git

Branch: `research/p6-i6d-full-algorithm`

START_SHA: `7329f74e5691c08379251046cce268477677f4f3`

接口代码提交: `982aa1c8dc58f6476d3596da68a62a575cea5451`

CODE_SHA: `c78e01fc38d583d8839257c172c41eef8fc34e0e`

END_SHA 是包含本文件的最终文档交付提交，完整 SHA 和远程核实结果由
随附终端回报提供，避免在提交中填写自身哈希的循环定义。

Workspace: `/home/jian/livox_ws/dog_loc_p6_i6b_ws`

本轮只实施真实 producer 接线与轻量验证。未运行完整 Floor01、Corridor01、
rosbag 或 GT 调参；未改冻结 baseline、旧正式 FULL 默认行为、其他工作区；
未擅自接入正式 FULL，也未开始下一任务。

交付目录在 GitHub 仓库对应分支：

`src/dog_prior_map_fastlio2_frontend_exp/docs/p6_alg_integration_a2c/`

## 二、完成了什么

新增独立 opt-in 模式 `FULL_FIXED_LAG_V2_EXPERIMENTAL`。它把静态初始化、
窗口 IMU 预积分、NDT 初值、真实 PCL NDT、Geometric U_obs、窗口协方差驱动的
U_nonlocal、方向选择 LiDAR、相对视觉 cross-state factor、联合优化与已有
Schur 边缘化接成同一条实验生产链。没有复制或重写 PCL NDT。

IKFoM 只在初始化函数中存在。初始化桥返回后对象已销毁；后续状态 owner
唯一为 FixedLagEventAdapter。所有 nominal/probe NDT 初值都由窗口预测姿态
乘真实 T_imu_lidar 得到，未回写 EKF 再循环生成窗口 prior。

## 三、八项指定回答

1. **A2B stale A_exact：已关闭。** 风险记录不再保存权威旧 Jacobian。
   视觉入窗时查找不晚于 current 的最近风险，要求风险状态仍在活动窗口，
   读取当前优化状态，重算 A_exact、Wp、Qw；入窗后冻结 Qw。
2. **窗口自己的 pre-measurement covariance：已有。** 使用包含现有 prior、
   IMU、LiDAR、visual 的真实联合 H，求 H X=E，取最新 15D marginal。
   不添加协方差专用 damping；不可用时明确返回 UNAVAILABLE。
3. **U_nonlocal 是否仍依赖旧 EKF P：否。** trigger、谱分解、probe 方向和
   幅度都用窗口 P_map6。协方差不可用时停止 probe，不回退旧 EKF P。
4. **P0/P1/P2/P3：原语义保持。** BASE/no NIS、ADAPTIVE/no NIS、
   BASE/selected NIS、ADAPTIVE/selected NIS，共用一套 producer。
5. **NDT 初值：100% WindowState。** prepareStateAt 在 NDT 前运行；该接口
   只添加预测节点和 IMU 因子，不先注入待检 LiDAR 或视觉测量。
6. **视觉：仍是 cross-state。** 真正连接 reference/current 两个窗口状态，
   残差 p_cur-p_ref-R_ref*z，保留两个端点 Jacobian 与交叉信息。
7. **handoff 后 IKFoM：完全停止。** 新 producer 中没有该对象，无预测、
   pose update、projected update、位置 update 或窗口反馈调用。
8. **未进入这条 producer 的模块：** live ROS/raw 图像深度前端、基于新窗口
   状态重新生成 deskew source cloud、正式长序列验证、全局重定位。
   本轮消费现有 prepared metric visual 和 cloud 数据，不重跑视觉前端。

## 四、关键数学定义

### 1. 23D 到 15D 初始化是条件模型，不是等价全后验

在真实 IKFoM translation unit 内使用 MTK::getStartIdx 并校验 pinned layout。
窗口顺序为 `[right rotation, map position, velocity, bg, ba]`。对于重力 S2
切空间，使用 `Pcond=Pxx-Pxg solve(Pgg,Pgx)`。固定外参交叉块必须接近零，
Pcond 必须 finite/symmetric/SPD，再用 LLT 生成初始 information，gradient=0。

这意味着固定 initialized gravity 条件下的 15D prior，不是将 uncertain
gravity 简单丢弃，也不是声称保留完整 23D posterior。测试中非零重力交叉
协方差的 conditioning difference norm 为 0.0309436。

### 2. 右扰动与 map-left covariance 严格区分

窗口 R_new=R Exp(dtheta_body)，nonlocal chart 为 Exp(dphi_map)R，因此
dphi_map=R*dtheta_body。G 的 rotation block 为 R、position block 为 I，
P_map6=G P15 G^T，保留 cross blocks。map6 顺序 `[rotation map,position map]`
与测量噪声 `[position map,rotation right/body]` 不混用。

### 3. Selected NIS 是入窗之前的检查

从实际 window LiDAR factor 得到 r_s、J_s、R_s；用 pre-measurement P15
构造 S_s=J_s P15 J_s^T+R_s，使用已有 chiSquare99Threshold(rank)。
拒绝时不添加 LiDAR 因子，但保留诊断风险及 transaction 消费记录。
视觉 route 明确知道 LIDAR_MEASUREMENT_REJECTED，不冒充全局校正成功。

### 4. 视觉方向测试的精确含义

0.4 rad LiDAR 测量使窗口风险状态优化后明显变化；完整 A_after-A_before
范数 >0.1，实际 admission A 与重算参考之差 <1e-12，Qw projector 正确。

需要注意：在当前冻结测量、外参和 Uw 下，A 的平移行是
`[skew(R_measured*t_IL), length_scale*I]`，不依赖估计状态，所以 Wp/Qw
不一定因窗口优化而变化。测试满足指令原文要求的新旧 A_exact 可测差异，
没有人为制造 Qw 改变。重算及冻结语义都已落地，边缘化风险状态禁止路由。

## 五、调度与输入

按 sensor stamp 严格排序；相同 stamp 为 LiDAR scan → visual current →
visual reference。IMU 只追加到当前事件第一右边界，不预灌后续几秒。
原 ref/current/depth stamp、参考 IMU frame metric translation、质量元数据
均保留。visual R=0.05² I 是工程模型，不是像素 Fisher covariance。

所有 LiDAR terminal stamp 都要单调，包括 not converged、support invalid、
measurement reject。时间回退不推进 transaction watermark、不写 risk history。

imu.csv 等 RuntimeParameters 的四项噪声直接传入窗口，gravity 来自真实
initialized snapshot。没有替换为窗口默认常数；也未宣称噪声谱已统计标定。

新轨迹 CSV 的 pose 是 map_T_imu，不能未经外参转换当成 LiDAR 轨迹评价。

## 六、构建和测试结果

Release 构建 PASS，14/14 CTest PASS；原有 10 项回归全部保留。
Debug 针对性构建 PASS，3/3 targeted CTest PASS，未关闭 Eigen 断言。
git diff --check PASS。

新增证据覆盖：真实 MTK 初始化映射、非零 gravity 条件项、联合 H 全逆参考、
SO3 map-left FD、无 damping 的 singular rejection、入窗前 NIS 和拒绝后
相对视觉 fallback、风险状态重算/边缘化拒绝、同时间与异步调度、四策略
PCL 生产 fixture，以及旧 FULL 源码/后 handoff 零调用审查。

真实 PCL 小点云 fixture（非真实数据精度实验）：

| 策略 | 事件 | LiDAR | Visual | Probe pairs | NDT calls | NDT ms | Total ms |
|---|---:|---:|---:|---:|---:|---:|---:|
| P0 | 7 | 3 | 0 | 1 | 5 | 52.1791 | 81.2164 |
| P1 | 7 | 3 | 0 | 1 | 5 | 52.9817 | 81.7748 |
| P2 | 7 | 3 | 0 | 1 | 5 | 52.9942 | 82.1768 |
| P3 | 7 | 3 | 0 | 1 | 5 | 54.6594 | 84.2122 |

此 fixture 的 LiDAR 正常，视觉被正确拒绝为不需要融合，不能声称视觉
改善定位。实际方向视觉与 rejected-LiDAR full relative factor 在 adapter
测试中运行。所有 fixture event 的 pre-measurement covariance 可用，
因果 IMU 第一右边界检查通过，post-handoff IKFoM calls=0。

上述时间只是本机合成 fixture，既不是公开数据统计，也不是 WCET。
本轮完整序列 NDT align 调用数为 0；表中 NDT 是小内存点云调用。

环境中 SDK libusb 符号冲突通过测试进程局部 LD_LIBRARY_PATH 解决，未改系统。
最终测试在正确 build cwd 运行，最初误在源目录执行而找到 0 tests 的尝试
没有算作 PASS。协方差数值检查改为精确坐标均衡和 backward error，而非
加 damping。未扩大科研接受阈值，未删除任何失败用例。

## 七、旧正式路径和交付

只移除新 includes、模式 dispatch、usage 后，整个旧 runner 与 START_SHA
源码逐字节相等，SHA256 为
498ec598db3aaec84b74391292d9db7928b2337944be01287621502135abed34。

交付含 summary、初始化/协方差/调度/U_obs-U_nonlocal/NIS/视觉方向/单 owner
审计、源码 SHA、修改清单、构建测试结果和 fixture 小 CSV。
代码与文档分开提交，没有 bag、PCD、images 或数万行历史轨迹混入。

## 八、科学结论和边界

RESULT=PASS，含义为指定工程链路和轻量合同验证通过；不是定位精度、
全序列稳定性或论文方法有效性的 PASS。联合协方差是局部线性化近似；
历史 marginal prior 的固定线性化近似仍存在。prepared visual 和 source
输入链的运行级时效性、真实长窗性能及统计标定没有在本轮得到证实。

READY_FOR_FORMAL_EXPERIMENT=NO。

执行 AI 已停止，不自行运行正式公开数据集、不进入下一阶段。
