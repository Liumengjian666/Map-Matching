# P32-R3冻结预算（执行/GT之前）

START_SHA：b5814cff0fcdab6f3d210915ecaf4452f46aa26d。
沿用研究分支和P10工作区，受保护工作区/历史缓存/报告不改，不自动push。
不读取GT，不运行893秒，不调整P31算法参数或噪声。

## 实际旧方法迁移

读取P2B_ADAPTER_REPORT及现有initializer/normalizer完整源码。
将原initializer、normalizer、holdout和adapter数学来源保留为本轮副本。
初始化器只增加诊断：seed/final矩阵、SE(3)数值残差、0.5m覆盖率、
平移/旋转修正与每候选耗时；原局部子图、ICP参数、候选及排序不改。
一次首5秒scan-to-scan ICP+24次原proper-axis scan-to-map ICP，非单种子NDT。

原真实规则为R_seed_i=R_yaml*A_i，t_seed_i=t_yaml：右乘旋转且平移固定。
不包含整体世界左乘Rz90°同时旋转t的假设，不擅自加入该假设。
只解析生效矩阵，不试逆、注释矩阵或平移网格。

## 输入与因果边界

L0特指原第3扫描，header1645999726479818106ns；NDT/ICP首扫描绑定该ns。
init cutoff=1645999731479818106ns，即L0+5秒。
只接收start>=L0且完整scan end<=cutoff的扫描，保留此前真实IMU边界
样本与截止之前的真实IMU；不把跨cutoff扫描截断或读取截止后IMU积分。
R2已抽查XYZ/time为float32、time约0..0.100839s；保持header/seconds解释。

复用P2B初始化输入的gyro-trapezoidal+SO(3)插值+旋转去畸变数学，
使用RC1原标定数值和既有proper-rotation舍入处理。scan-to-scan ICP
负责首段各扫描的完整相对运动；不把rotation-only称为P31完整Deskew。
当前旧adapter的full-SE3反馈模式会等待此前NDT，不在bootstrap中使用。
不使用EKF公开轨迹重建，不生成虚假odometry，不伪造缺失IMU。
旧initializer收到的bag topic仍为/superloc_adapter/points_deskewed。

## 准入与后续预算

保留旧排序：先converged，再fitness小。旧日志/YAML即使写PASS也不是
本轮验收：最佳候选必须finite、proper到1e-5容差、converged、fitness<=10、
NN median<=1m、P90<=3m且0.5m覆盖>=50%。这是预设非GT工程screen，
不是搜索调参；失败不重跑、不改阈值、不换候选、不归一化地图。

通过才一次5–10秒holdout，原ICP传播/几何检查不改；保留所有样本。
窗口匹配finite、无失败、无严重跳变，aggregate NN median<=1m、
P90<=3m作为P2B历史结果量级参考；不根据holdout重新选候选。
未通过则停止，未执行项NOT_RUN，不伪造结果。

通过后才施加T_N_M=inverse(T_M_L0)，保存点数/bbox/SHA与往返误差。
初始化首5秒为离线预热，结果最早cutoff后可用，不回写为L0时刻在线状态。
holdout决定正式准入最早在L0+10秒；正式F/G从合法启动时刻开始，使用
此前sensor-only运动传播初值，不使用未来验证数据或GT作滤波初值。
仅F30s/G30s各一次，算法、噪声、固定lag、step limit、velocity_blend等不改。
G用自身真实Camera特征；旧PnP路径关闭。完整893s和GT精度评价均禁止。

结果区分：方法迁移/初始化几何、地图归一化、F/G工程运行、GT参考资格。
缺少GT参考点不单独否定合法非GT工程运行；也不宣布认证精度或泛化。
