# P46 源码改动

定位器、NDT 数学、EKF、IMU 去畸变、Tile 参数、NDT resolution、Source 点数上限、初始化 YAML、OOSM 和输入发布器均未修改，也没有重新编译定位二进制。

唯一已跟踪源码改动：

- `src/dog_prior_map_localization/scripts/monitor_p45_local_map_run.py`
  - 明确接收 `--timeseries`，以 0.1 s 采样写进程角色 RSS/VmHWM、系统 MemAvailable 和 launch 后代进程 CPU/RSS 时间序列。
  - 从 roslaunch 的父子进程树跟踪节点，避免各节点脱离进程组后漏计；分类统计 NDT、EKF、publisher、地图显示器及 RViz。
  - 允许 P45 Local launch 和 P44 Global launch 共用监控器，便于同条件资源比较。

P46 代码未修改以下路径：`dog_prior_map_ndt_node.cpp`、EKF/OOSM、Deskew、Tile 管理核心、NDT 参数和 launch 配置。P46 新增的冻结、评估、哈希与统计脚本均为离线验证工具，不进入定位运行时。

构建二进制 SHA256（两组相同，沿用 P45 已构建程序）：

- NDT：`c5e028637a38fb8877486c9c7ef783e40de071e61c981b874d82db8bfcbb266c`
- EKF：`930a3ca96e178d63afd58874ca3974dadcbd87908e58e39a3d15c85807d25d2d`
- Sequence publisher：`024b8f679c16a325acb73bf6b4e91698f4a7a025dd47ed90de3cfb6fef565e66`

运行配置、初始化文件、两个 launch、完整 Global PCD 及 Tile manifest 的哈希列于 `runtime_statistics.json`。本轮验证的是现有 P45 Local Target 实现，不是新地图算法。
