# 源码修改清单

- `src/dog_prior_map_localization/src/newer_college_first_scan_publisher.cpp`：在保留 P41 单帧路径的基础上增加连续扫描模式。只选择规范时间戳 PCD；用 uint64 ns 排序和精确起点定位；读取原始 `t:uint32` 并新增秒单位 Deskew 字段；按扫描内真实 IMU 发送并等待 EKF/NDT 水位、精确 NDT 诊断时间戳及 OOSM 校正确认；输出逐帧回执和轨迹 CSV。校正消息与 IMU 水位比较容纳 1 µs 的 ROS double 秒/整数纳秒往返量化误差，并要求每扫描确有新的 corrected 消息。
- `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp`：新增可选 IMU 消费水位 `std_msgs/Header` 诊断输出，用于发布器背压；默认空 topic 时不发布。NDT 数学、目标函数、参数和输出修正未改。
- `src/dog_prior_map_localization/CMakeLists.txt`、`package.xml`：新增连续发布器 target 和直接 `std_msgs` 依赖；保留 P41 单帧 target。
- `src/dog_prior_map_localization/config/p44_newer_college_base_oracle_start.yaml`：冻结唯一首帧条件性 Oracle 状态，无 GT 文件输入。
- `src/dog_prior_map_localization/launch/newer_college_300frame.launch`：复用 P41 runtime YAML 与现有 split NDT/EKF；CONTROL、P41 NDT/Deskew/OOSM 参数；可选 RViz/地图发布；headless 完成即退出，RViz 模式下输入发布器完成后保留查看窗口。
- `src/dog_prior_map_localization/rviz/p44_newer_college_300frame.rviz`：显示官方先验地图、Deskew 当前扫描、NDT 对齐扫描、NDT pose/path、EKF 高频和 corrected path，固定坐标系 `newer_college_map`。
- `docs/p44_newer_college_300frame/evaluate_trajectory.py`、`summarize_runtime.py`：GT 只在运行输出冻结之后离线读取；生成精度、轨迹误差、RPE、点云输入哈希及运行汇总。

没有改 NDT 优化器、EKF/IMU 数学、OOSM 调度、标定、地图、DCReg、视觉、Coupled 或 Recovery。P41 冻结的 resolution 0.8 m、source/target voxel 0.25/0.15 m、step 0.08、epsilon 0.001、40 iterations、source cap 1,400、step limit 0.5 m/5°、`velocity_blend=0.6` 均未改。P41 的标准协方差路径继续使用；实验 square-root 开关保持关闭。

运行后仅对 publisher 失败日志文案作了泛化，未改运行数学；RViz launch 生命周期的小改动只影响 `rviz:=true` 的查看窗口。实际 300 帧 run 的 publisher 二进制 SHA256 为 `993c5bee1f2d745c152dc88522d4cad24be8c50e76f9285796f8ee5ce5a0fce61`，最终当前工作区构建二进制为 `024b8f679c16a325acb73bf6b4e91698f4a7a025dd47ed90de3cfb6fef565e66`；NDT/EKF 二进制在运行与最终构建中相同。两项运行后差异均不改变 300 帧算法输出。最终构建 SHA 详见 `REPORT.md`。
