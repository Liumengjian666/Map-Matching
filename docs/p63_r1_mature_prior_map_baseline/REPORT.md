# P63-R1：原版 Super-LIO 冻结先验地图基线

本轮结论：**原版 Release 编译、外部 PCD 导入和地图冻结通过；条件性历史零外参冒烟测试提前失败，尚不能验收为科研定位主干。** 没有开展算法创新、没有移植 C+G、没有启动第二套框架或重跑失败片段。

| 项目 | 实测事实与判定 |
|---|---|
| Git START | `3f0d24f1a930a2365e7c918c8444afc44c039998` |
| 本地分支 | `research/p63-r1-mature-prior-map-baseline` |
| 执行工作区 | `/home/jian/livox_ws/p63_r1_super_lio_baseline_ws` |
| 官方 ROS1 源码 | `Liansheng-Wang/Super-LIO` / `ros1` / `60b57aaac8dc397f80c56364e7ccb008c300cc29`，GPLv3，作者 tracked-source 修改为 0 |
| 实际有输入运行 CODE | `5ae6d0c600e1140b996e83a237d81cffb956969d`；前一监督器导入错误在启动 ROS 和发布数据前中止 |
| END | 包含本归档的最终 commit，完整 SHA 见最终交接；`git rev-parse HEAD` 可核实；未 push |
| 地图 | 原文件 459154 点；原生 OctVox 151401 点；probe 与真实跟踪前后几何 SHA 相同 |
| 输入／外参等级 | 用户明确授权的历史 `T_I_L=(I,0)` Smoke；没有独立物理标定，地图生产配置的文件级绑定也不完整 |
| 预注册／实际输入 | 固定首 400 扫描；停止时已发布 107 LiDAR、2125 IMU；并未完成 400 扫描 |
| 原生初始化 | NDT 1 次＋ICP 1 次；ICP fitness 0.10959，通过作者初始化门；此分数不是位姿精度 |
| 原生跟踪 | 95 次原生 ESKF Observe 执行／状态输出；75 次至少 4 个有效平面匹配，最后连续 20 次不足 4 个，按冻结停止规则中止 |
| 输出诊断 | 位姿有限；首末估计位置相距 225.114052 m，传感器时间跨度 9.400211 s；这不是 ATE 或真实位移 |
| 重力一致性 | 首 10 扫描、185 IMU 重建初始化输入；记录的 ICP 姿态变换平均比力方向后，与原框架地图 +Z 相差 38.021059°；仅是兼容性线索，未确定唯一根因 |
| 单次 process 时间 | Mean/P95/Max 0.973411 / 1.672614 / 5.267051 ms，仅 95 次有输出的 process；另有启动配准约 3.289708 s |
| 进程内存 | 定位 observer 峰值 190496 KiB；RViz 390512 KiB；自有 ROS 进程树采样同时峰值 712264 KiB，**不包括另起的输入重放进程** |
| 可视化 | 实际启动 RViz，配置地图、点云、位姿／轨迹和 TF；提前失败未取得截图。新增自动输入 one-click 组合只做解析检查，未再次回放 |
| 正式精度 | ATE/RPE `NOT_EVALUABLE`；没有合法独立 GT、外参未验收 |

## 证据入口

- [源码与许可证审查](P63_FRAMEWORK_SOURCE_AUDIT.md)、[地图冻结合同](P63_MAP_FREEZE_CONTRACT.md)。
- [传感器、坐标和时间合同](P63_SENSOR_FRAME_TIME_CONTRACT.md)、[编译身份及命令](P63_BUILD_REPORT.md)。
- [原始结果及失败证据](P63_ORIGINAL_BASELINE_RESULTS.md)、[运行成本](P63_RUNTIME_MEMORY.md)、[RViz 状态和命令](P63_RVIZ_ACCEPTANCE.md)。
- [后续接口和唯一 NEXT](P63_NEXT_INNOVATION_INTERFACE.md)、[机器可读结果](results.json)。
- 原始输出逐字压缩在 `raw/`，SHA 对照在 `evidence/RAW_OUTPUT_MANIFEST.json`；完整 95 行状态在 `ORIGINAL_TRAJECTORY.csv`；没有删除失败帧。

## 责任边界

连续跟踪使用作者的点到面迭代 ESKF，并非持续原生 PCL NDT。NDT＋ICP 只用于启动。原论文介绍的 LIO 机制不能自动证明官方仓库 relocation 扩展在本地外部地图上的可靠性。只读 observer 通过继承读取原状态和地图，链接未改作者库，未重写估计器虚函数；实际运行的是该带日志 observer，不是把作者 `relocation_node` 可执行文件直接运行的结果。

当前不能证明 Super-LIO 不适用，也不能证明替代框架必能解决问题。失败发生在历史外参／地图世界重力关系尚未验收的条件下。唯一建议：请总控安排一次有界的外部地图重力轴、初始化姿态及设备外参来源闭合，再决定是否需要坐标适配或切换框架。本轮不修改重力、外参或初值重跑。
