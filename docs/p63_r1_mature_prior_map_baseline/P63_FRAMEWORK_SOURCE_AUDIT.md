# 官方源码与算法责任边界

官方源：<https://github.com/Liansheng-Wang/Super-LIO/tree/60b57aaac8dc397f80c56364e7ccb008c300cc29>，分支 `ros1`。本地独立 clone 为 `/home/jian/livox_ws/p63_r1_super_lio_data/Super-LIO`。源码身份及 binary SHA 见 `BUILD_FREEZE.json`，最后检查见 `evidence/FINAL_IDENTITY_CHECK.json`。作者 tracked diff 和 status 均为空；仅在 clone 自己的 `.git/info/exclude` 忽略研究包和只读地图符号链接。不是对子模块算法打补丁。

论文：Wang 等，*Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System With a Compact Mapping Strategy*，IEEE RA-L 11(3):2666–2673，2026，DOI [10.1109/LRA.2026.3653372](https://ieeexplore.ieee.org/document/11347459/)。论文对象主要是 LIO；本轮测试官方仓库额外提供的先验地图 relocation 路径，不据标题推断其已通过外部 PCD 定位基准。

许可证为 GPLv3（原 `LICENSE` 和 package.xml 均核查）；归档选定未改源码及原 LICENSE 于 `upstream_source/`，研究 helper 包同样声明 GPL-3.0。完整可重现来源是固定上游 commit，不将原作者模块当成本项目原创。

| 实际源码 | 已核实行为 | 本轮处理 |
|---|---|---|
| `src/super_lio/src/apps/relocation_node.cpp` | 建立 ROSWrapper 和 SuperLIOReLoc，500 Hz spinOnce/process | 作者 executable 编译；日志 observer 保持同一主顺序 |
| `src/super_lio/src/lio/super_lio_reloc.cpp`：init/map_init | PointXYZI 读取外部 PCD；去 NaN；构造 V3 并插入 OctVox；后续 loaded guard | 直接读取现有 FAST-LIVO2 XYZRGB 地图；不覆盖／重建原 PCD |
| 同文件 kf_init | 累计 10 扫描与至少 20 IMU；平均加速度确定水平初态；人工位置和 yaw；原生 NDT→ICP；通过后赋值 ESKF 初态 | 原数学与参数保留；原点人工初值，不读取轨迹 |
| 同文件 UpdateMap | `if(!g_update_map) return;` 后才允许插入当前扫描 | 原配置从 true 改 false；另设 save_map=false |
| `super_lio.cpp`：Propagation_Undistort | 原 IMU 传播、T_I_L、逐点时间和去畸变 | 未改 |
| 同文件 Observe | OctVox 查询／平面估计、残差和 Jacobian；迭代 ESKF UpdateObserve | 未改，无独立自建滤波器或 NDT 后端 |
| `ESKF.cpp` | 原状态传播、协方差和迭代观测更新 | 未改；旧 P62 EKF/OOSM 未接入 |
| `ROSWrapper.cpp/.h` | Livox CustomMsg 与 IMU 输入、时间同步和原生话题发布 | 原序列化字段不变；driver2 与 driver1 的 MD5 相同，真实运行已接收 |
| `OctVoxMap.hpp` | 插入时做紧凑化、邻域搜索、getMap；resetMap 存在但跟踪无调用 | 冻结后几何摘要实测不变 |
| `relocation.yaml` / `relocation.launch` | 默认 update_map=true；例示非零 T_I_L；地图路径受 ROOT 前缀约束 | 研究 YAML 覆盖冻结参数／历史身份假设；只读 map symlink，不改作者配置 |

初始化参数来自作者：NDT resolution=1.0 m、max iterations=25、epsilon=1e-4；ICP max correspondence=4 m、iterations=40、epsilon=1e-4，fitness≤1.5。**没有移植 P62 的 0.8 m/40 iteration NDT 或 ±5°恢复。** 正式连续配准是原生平面 ESKF，不能称“持续 NDT 定位”。

研究文件在 `src/p63_super_lio_baseline/`：只读 observer、目标冻结 probe、输入重放、启动 gate、TF 可视化桥、YAML／launch／RViz。observer 没有 override 作者估计器／配准／地图更新虚函数。probe 的工程 fixture 只用于实际 PCD 的关闭插入测试；600 次 synthetic UpdateMap 不是 600 帧真实定位。

定位阶段全部地图写路径、实际初始化和重力前提详见另外两份合同。本轮没有新增科研模块，也未开始 FAST_LIO_Relocation 适配。下一阶段算法插点仅列位置，不开发新方法。
