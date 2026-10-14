# 源码范围与复用

仅新增 `src/dog_prior_map_localization/scripts/p32_init_transfer/` 离线工具。
没有改定位节点、EKF、MSCKF、Deskew、DCReg、多候选、launch或运行配置。
没有编译新的 NDT/EKF 节点，也没有修改 P31 C 任何物理算法参数。

- `legacy/superloc_first_segment_initializer.cpp`：原程序逐字副本，实际执行。
- `legacy/superloc_map_normalizer.cpp`、`superloc_holdout_sanity.cpp`、
  `full_se3_deskew.hpp`：原来源副本，初始化未准入，后续程序未执行。
- `initializer.cpp`：只包装原 ICP 调用并增加候选矩阵、0.5m覆盖与耗时诊断。
- `prepare_input.py`：冻结 scan3 起始的完整5秒扫描和真实IMU边界样本；
  复用既有 R2 `cloud_array` 数据解码，不使用其单种子 NDT 初始化。
- `gyro_input.hpp`、`input_bridge.cpp`：适配既有 P2B 旋转补偿数学；
  避免当前 full-SE3 adapter 必须等待历史 NDT 的 bootstrap 循环依赖。
- `math_test.cpp`：24唯一proper轴、固定平移、实际active YAML解析、
  gyro插值、静止点、禁止外插的定向测试，不执行 ICP。
- `run_once.py`：一次性目录保护、执行前哈希冻结、真实 CPU/RSS、
  固定准入判断。原日志的 PASS 不等于任务验收 PASS。
- `make_review_input.py`：外部复核的限定输入生成，不执行外部调用。
- `guard_test.py`：一次外部复核后的输入绑定、最终矩阵和依赖来源检查
  回归测试，仅临时fixture，不运行ROS或ICP。

外部复核后对prepare_input/run_once的修改只补离线工具的来源与准入
守卫，没有修改原ICP、候选生成、参数或定位节点。实际实验用CODE
aaef3d58ded3561710a8148807c700d0c24e2289；该次脚本快照及原哈希均保留。
修复工具没有重跑初始化；后续guard commit与实际实验版本分开记录。

统一 CMake 编译中仅修复诊断模板的 Eigen 推导类型，改为显式 Matrix4f；
未修改优化器。MVS 的旧 libusb 搜索环境由本离线进程绑定系统 libusb；
不更改系统安装或任何受保护工作区。

本轮复制的是本项目已有工程源，不宣称新的原创算法或完整第三方模块
移植。当前 adapter 源已晚于 P2B 更新为 full-SE3 模式；本轮复用其明确
旋转积分/杆臂方程，不宣称其现版本二进制就是 P2B 历史运行二进制。
