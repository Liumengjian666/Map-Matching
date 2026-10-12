# P25 源码变更清单

- `dog_prior_map_fastlio2_frontend_exp/scripts/p4_i3_visual_frontend.py`：从 PnP Jacobian、特征几何及深度噪声假设生成带交叉项的 6×6 相对位姿协方差。
- `dog_prior_map_localization/scripts/p17_visual/prepare.py`：协方差字段、P17 pose parity 和传感器时间来源输出。
- `dog_prior_map_interfaces/msg/VisualRelativeMeasurement.msg`：真实相对 SE(3)、端点时间、质量与协方差 ROS 消息。
- `core/selective_relative_visual_update.*`：DCReg 弱方向投影的相对位姿残差、保守伪端点协方差、固定 CI 子空间更新及 PSD/有限值守卫。
- `skf_visual_measurement.*`：带协方差/质量/因果时间检查的离线 CSV catalog。
- `fusion/skf_visual.cpp`、EKF node/core/header、IMU fixed-lag scheduler：传感器时间排队、历史状态克隆、相对视觉更新、OOSM/IMU replay 与诊断。
- `core/psd_covariance_damping.hpp` + `fusion/ndt_observation.cpp`：SKF-enabled 分支将 NDT 更新后的位姿方差缩减改成 `D P D` PSD-preserving 同余缩放；这是随 SKF 开关启用的数值保护，也构成精度归因的实现级混杂。
- `dog_prior_map_ndt_node.cpp`、`mature_ndt_algorithms.cpp`、launch/config：受控发布 SKF 测量、DCReg 门控与 A/B 参数。
- P25 scripts：受限回放、冻结检查、结果评价脚本、失败态只读归档器；此次没有调用 GT evaluator。

本轮修改没有重写原 NDT 目标函数、EKF IMU 传播方程、OOSM replay 算法或 velocity blend；增加了视觉固定滞后事件及上面说明的协方差数值保护。定位算法新增的是选择性相对视觉更新原型。当前最终结论仍为数值合同失败，代码不应视作已通过部署验收。
