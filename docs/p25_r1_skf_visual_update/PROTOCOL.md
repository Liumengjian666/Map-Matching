# P25-R1 执行协议与偏差记录

## 冻结设计

对同一 Corridor01 冻结地图、V2 bag、P4/P17 视觉相对观测、NDT/IMU/EKF/OOSM 配置进行 A/B：A 为 DCReg + Multi-NDT 开启、Visual Initial Guess 关闭、SKF 关闭；B 仅打开 `/mature_integration/skf_enable`。`velocity_blend=0.6`、固定滞后 150 ms、`camera_enable=false`。相同视觉相对观测不同时供 NDT 初值和 SKF 使用。GT 只能在两组完整冻结且有效后离线读取。

## 运行顺序与停机规则

1. 复用 P4 前端、官方 Corridor01 标定和 P17 冻结 pose；生成带 6×6 协方差的离线 CSV，并先做 pose parity。
2. 构建独立 catkin workspace；运行针对性单元测试。
3. 做 B 短段 smoke，验证真实 ROS 发布、固定滞后应用和无非有限状态。
4. 完整 A；完整 B；每组至多一次，无参数搜索。
5. 先检查完整性与 SKF covariance 状态；只有合同通过才允许加载 GT。

B 完整回放有 55 条 `NON_PSD_CI_COVARIANCE`，因此在离线 GT 阶段前停止。没有运行 `freeze_outputs.py` 的 GT 解锁阶段或 `evaluate_p25.py`；未生成精度指标，也没有重跑整段。

## 结果偏差 / 限制

- P4 covariance 是重投影 Jacobian + 显式像素/深度噪声 floor 与膨胀模型；`0.75 px / 0.5 px / max(0.05 m,1% range) / ×4 / 0.02 m / 0.5°` 均为保守先验，不由 GT 调整，未作完整传感器协方差标定。
- raw camera bag 为 6,473,379,689 B；P4 阶段只做受限头部提取，未重算完整 raw-camera SHA。冻结地图与 V2 bag 使用各回放启动收据的 SHA；归档时未再次扫描大型 bag。
- 离线视觉桥接记录 `translation_deskew=NOT_ESTIMATED`；使用短时平均运动端点近似，不是在线全相机处理。
- 历史参考端点跨时刻交叉协方差不可用，以 Cauchy 上界和 CI 作保守处理；不能当作完整 SKF 原论文的精确滤波器。
- `NON_PSD_CI_COVARIANCE` 是数值拒绝，不等于滤波状态污染。首个拒绝时输入协方差已对称但非 PSD；本轮没有确定其唯一上游成因。

## 可复现产物

完整轻量运行 CSV、ROS 参数、execution receipt 与日志位于 `runs/`；ROS launch 日志以可无损还原的 gzip 形式保存。大 bag/地图未复制至归档。`input_freeze.json`、`parameter_parity.json`、`artifact_hashes.json` 保存输入与输出哈希。GT 在本轮未载入。
