# 地图、初始化和评价参考点合同

## Floor01

定位前已有原官方地图和 H1 SSD 冻结地图；本轮没有修改或新建地图。
官方标定给出 `p_imu=T_imu_lidar*p_lidar` 与 `p_imu=T_imu_camera*p_camera`。
历史 R7C 完成 Floor01 GT 对官方 SubT-MRS 同序列的2061/2061精确
lineage，GT参考点为 IMU。此项不是本轮 GT 评价。

未闭合的是 H1 初始化的语义：R7G 仅通过发布矩阵正/逆两方向的
地图包围盒和首帧匹配选择了 H1，明确标为
`OPERATIONAL_INIT_SELECTED_BY_MAP_CONSISTENCY`，没有确认发布矩阵
方向。H1确实是非GT工程初始化，不能称其不存在或自动不可接受。
但有限/收敛NDT本身未闭合完整的运行body/初始化/地图坐标链；
支持该坐标链的工程证据也可满足合同，不要求必须由官方说明。
没有开展新的 H1/H2 搜索或 GT 初始化。

## Corridor02

raw bag、原地图、相机/IMU/LiDAR标定和初始 YAML 均实际存在。
bag 的 record/header 时钟不同，需要格式适配，不能构成误判数据缺失。
官方 extrinsic 的 LiDAR→IMU 方向已有文档，但不能仅据官方定位代码
输出 LiDAR pose，就断言发布 GT 轨迹一定是 LiDAR pose。

未闭合字段：

1. `corridor02.yaml` 有效 world_darpa 矩阵对应何种 T_target_source，
   传感器/body关系是什么，如何关联首个运行扫描。
2. `corridor02_gt.txt` 轨迹的传感器参考点与该标定链的关系。

已知地图→GT绝对变换不是相对漂移评价的必需条件，不把它作为独立
hard gate。可以在合法初始/参考点合同闭合后，在输出冻结后按事先
冻结的前缀拟合共享相对评价变换。不能用这一步生成运行初值。

## 唯一最小数据阻塞与 NEXT

补齐一份**可追溯的 Corridor02 坐标／初始化说明**，明确发布GT
参考点、初始变换的source/target、传感器/body关系和首扫描关联。
可以来自官方配置、exporter或有依据的工程坐标链；字面时间戳字段
不是必需条件。有依据的首扫描关联即可满足该项时间语义。
不需要再下载 raw bag，也不要求为了相对漂移评价新建绝对地图→GT变换。
初始位姿必须不由本轮 GT 反推。该语义资产是当前最小可执行的补齐项，
它只解决已知语义缺口，不自动证明全段覆盖、输入哈希及独立性等均通过。

不得用默认identity、GT首帧、经验残差最小或新的候选搜索替代此说明。
本轮停止；后续是否重新准入由科研总控决定。

参考本仓库现有证据：

- `src/dog_prior_map_localization/docs/superloc_floor01/P3_R7G_FLOOR01_CAUSAL_CLOSED_LOOP_BOOTSTRAP.md`
- `src/dog_prior_map_localization/docs/superloc_floor01/P3_R7C_FLOOR01_PREREQUISITE_AND_LINEAGE_AUDIT.md`
- `src/dog_prior_map_localization/docs/superloc_corridor02/corridor02_reference_audit.md`
- `src/dog_prior_map_localization/docs/superloc_corridor02/corridor02_initial_pose_audit.md`

旧审计的“bag未下载”已由本轮真实读取纠正；其未闭合语义不能自动
因 bag 后来出现而变成 PASS。
