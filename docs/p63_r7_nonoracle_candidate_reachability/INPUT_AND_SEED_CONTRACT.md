# P63-R7非Oracle可达性预注册

START `79657c8c4cba6985af7a51ee759fe38746c24acc`。仅Koide indoor_easy_01 raw78–83（Azure Kinect，非MID360）。本轮是固定平移多初值成熟基线，不是创新算法，不是连续定位恢复，也不是六个独立场景。

## 运行输入隔离

只生成新的允许字段`INPUT.json`，不沿用含Oracle的R6 INPUT。输入准备只读取R5 INPUT_FREEZE的地图/扫描身份、R5封存ONLINE_RESULTS的guess/原生终值/质量，以及R6 PREPROCESS_BEFORE的Source几何SHA。C++只接收新INPUT及原始PCD，排序工具只读此文件和新原生结果。

不读取参考轨迹、R6 INPUT、POSE_MATRICES或Oracle原生输出生成种子和排序。先保存所有候选和原生Gaussian选择结果并SHA封存，再由独立评价工具读取参考；GT标签不得回写排序。

M为原地图数字坐标，L为depth_camera_link，所有位姿为`T_M_L`。整数header/XYZ不改，无外参重估、坐标拟合、UTM、Deskew或时间偏移。原地图93146点/文件SHA `8a3a7b0a9be71a5a76a284079766ca0c91037ba2a43b12dc93b13be14eda7707`；原0.1m VoxelGrid目标89656点/SHA `dd839ccce78afebd3c7b055b5e9cff4b021f70a8cd0ad2d7323d899079d3c533`。Source复用R6同一消息→同帧复制→0.1m VoxelGrid路径，点数及SHA逐帧与R6重建证据相同。

保留来源限制：R5没有保存现场post-filter Source SHA；R6基于原输入字节/同库同路径和点数确定性重建，本轮进一步与R6保存的Source SHA逐点核对，不冒充R5现场测量。

## 冻结种子及选择

每帧`T_pred`是R5实际传入NDT的float guess，非R6参考初值。Candidate 0为R5已执行的原生终值，不重跑。

Candidate 1–8：地图XY方向0/45/90/135/180/225/270/315°，按该顺序，距离均0.75m。

`R_seed=R_pred; p_seed=p_pred+0.75[cos(theta),sin(theta),0]`。

等价于地图系纯平移左乘`Trans_M(delta) T_pred`，不是机体系右乘平移；保持Z和姿态逐bit不变。双精度加法后按原NDT接口转换float32；全部48个实际矩阵运行前保存在INPUT，并在C++中复核公式。半径来自已暴露开发误差尺度，非独立泛化预注册。

NDT_OMP及原共享库与R6相同：resolution1m、DIRECT7、epsilon.01、max35、step.1、outlier.55。不改核心、不增加Yaw候选。六帧各8次，总新增align≤48；UKF0、ROS0、原在线重跑0。

在Candidate 0–8中，只对收敛、有限位姿且Gaussian分数有限者排序：分数降序，**完全同分按ID升序，0优先**。原生`getTransformationProbability()`是唯一选择量。NN fitness、25m地图原点裁剪的原生inlier和matching_error均仅记录，不参与选择。没有去重后再排序、阈值准入或GT筛选。

## 封存后评价

继承R29/R5的官方`T_map_lidar`参考与时间插值：线性位置/SLERP，bracket≤150ms、最近端点≤50ms，不对齐轨迹。参考属于地图锚定离线GICP+IMU，非独立动捕。

逐候选成功标签：位置≤.15m、旋转≤2°，且比同帧Candidate 0位置误差改善≥.30m。统计新增1–8的可达性与封存Gaussian选择的实际成功，分别报告四类结果。GT最优候选只作离线上界，不能冒充选择结果。

- 可达且实际选中成功≥4/6：`REACHABLE_AND_NATIVE_RANKING_SUPPORTED`。
- 可达≥4/6但实际选中成功<4/6：`REACHABLE_BUT_SELECTION_FAILURE`。
- 可达<4/6：`FIXED_TRANSLATION_SEARCH_INSUFFICIENT`。
- 输入/参数/Source/Target/执行异常：`CONTRACT_OR_EXECUTION_BLOCKED`，保留实际调用，不补齐重试。

终值去重仅离线描述：按ID升序，若与已有任一代表同时≤.10m且≤1°视为重复，否则新增代表；非传递贪心统计不代表严格局部极小值数量，不影响原排序。

## 成本和停止

记录每次align、只读评分、全部诊断、每帧八次总compute和事件墙钟、独立进程累计RSS峰值。评分/哈希开销不藏进“原生compute”。离线条件不证明10Hz。

所有六帧先通过零align预检，冻结提交后唯一formal_01执行。每次align记BEFORE/AFTER，非有限核心或身份违约立即停止，不改种子重试；程序180s超时。至少20GiB余量，无下载/删除/历史改写/稳定机器狗写入。本轮结束停止，不自动开展新算法或第49次align。
