# 经典先验地图定位公开基准：文献核查与数据合同准备完成

TASK：P62-CLASSIC-PRIOR-MAP-LOCALIZATION-BENCHMARK-PLAN。
START：c237c1c09ca5c823eb7b1279a8742ba7762dccf9。
分支：research/p62-classic-prior-map-benchmark-plan。
工作区：/home/jian/livox_ws/p62_classic_prior_map_benchmark_ws。
最终归档身份为交接所列本地HEAD；未push。

## 本轮实际工作

按科研总控最新澄清，R30尚未执行。本轮只完成公开文献、数据合同准备和证据归档；不再找R30文件，不运行新的Outdoor定位，不下载大体积载荷，不改C+G，不复现经典算法。

1. 逐项核对作者PDF Table II、作者项目页、Zenodo原始说明及当前官方HDL/FAST-LIO Localization README。
2. 六行公开数值全部一致。ATE中心值和±项的具体统计定义、evo对齐参数、重启后轨迹拼接/剔除规则等仍为NR。不能根据evo默认值猜测。
3. 确认论文会在方法失锁后重启，并统计corruptions。该公开指标不能直接当作我们连续无重启完整轨迹的公平RMSE排名。
4. 只读取Outdoor01/02四个ZIP的目录尾、metadata.yaml和Zenodo文件信息；保留CRC、来源和SHA。GT复用R29已下载文件，仅读取时间与文件身份，不用位姿求新误差。
5. 梳理官方单位外参/地图系LiDAR参考关系、ROS2消息定义、原始数据待验证项、存储及完整轨迹评价计划。

## 论文可引用的公开表

|作者实际运行方法|Outdoor01 ATE [m]（原样）|失锁/腐化次数|Outdoor02 ATE [m]（原样）|失锁/腐化次数|
|---|---:|---:|---:|---:|
|HDL Localization|5.592 ± 5.252|5|7.137 ± 6.833|5|
|FAST-LIO Localization|6.983 ± 6.693|4|4.849 ± 4.692|4|
|Koide ICRA2024方法|0.959 ± 0.838|0|0.597 ± 0.401|0|

出处为作者PDF第6页Table II。±含义未明确认证，不改写为mean±std或RMSE。不是FAST-LIO2纯里程计表，也不是我们新运行的数字。完整逐字段来源与NR项目见CLASSIC_BASELINE_LITERATURE.md和PUBLISHED_BASELINE_RESULTS.csv。

## 输入合同准备的实际结果

- 仅主序列Outdoor01/02，对应outdoor_hard_01a/b、02a/b及map_outdoor_hard.ply。官方传感器是MID360，GT是地图系LiDAR位姿，Outdoor T_lidar_imu为官方单位变换。
- 元数据CustomMsg数5225/5206，PointCloud2数5147/5127，IMU数136422/132454；GT分别5147/5127行。不能把数量对应当作实际header逐条闭合证明。
- a/b记录时间存在0.889904866s和1.285481515s间隔。必须继续核查实际传感器header，不可压缩时差或伪造IMU。
- 四ZIP约5.79GB，展开约11.24GB；root不够同时保存两份并保留20GiB安全余量。外部盘约449GB可用，足够规划独立资产目录；未实际下载这些载荷。
- 当前Livox驱动IMU发布函数为数值直传；仅凭ROS Imu消息类型不能认证原始加速度单位。点内时间、XYZ、raw unit和转换必须在实际数据到位后核验。
- 地图、GT及外参的官方定义可建立目标链，但原始消息/地图文件及运行中的方向、时间、初始化和融合最低合同尚未通过实际验收。状态为准备完成、运行未准入，不能全部写PASS。

## 科学结论

目前可以形成带协议限定的Published Classic Baseline表，暂时只有B级公开背景比较。没有我们的Outdoor结果，因此没有新增精度优势、额外恢复收益或计算成本测量。R29完整失败证据未删除、隐藏或改名；R23–R25内部消融降为辅助机制证据。

本轮不生成OUTDOOR01/02_FULL_RESULTS.csv或真实轨迹图，避免把未运行占位符冒充实验。具体尚未满足项与论文判定见FINAL_PAPER_SCIENTIFIC_ASSESSMENT.md。

唯一NEXT：科研总控单独安排R30基础定位链路验收，再授权完整Outdoor01、Outdoor02实验。执行在本轮归档后停止，不自动开展算法研发。
