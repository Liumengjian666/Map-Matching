只读反证复核。只允许读取限定包：
/home/jian/livox_ws/p61_r7_geode_runs/r7_review_v1.tar.gz
先核验 SHA256 必须为 26b1cae8fb0895bccfa958c509061cc487e91dca350be2c4feea5603c039b00a。
若不一致立即停止。可通过 Python tarfile 在内存读取选定成员，禁止解包写盘。
不要读取其他运行、GT、凭据或历史对话。不要调用网络，不要运行 ROS、NDT、下载器、测试或编译器，不要改文件。

Adversarial review. Find what is wrong with this artifact. Assume the author
is overconfident. Look for unstated assumptions, edge cases, hidden coupling,
ordering violations and ways the contract can fail. Do NOT validate or
summarize; find Critical/Required issues or explicitly state none found.

ARTIFACT: 包内 R7 geode/scan_end_bridge_enable 默认关闭分支、冻结输入准备、
replay_twenty.py、test_time_boundary.cpp 及 PROTOCOL.md。
只检查相关新增 GEODE 分支、原生产IMU/Deskew/OOSM调用上下文、回放ACK；
不要扩大审计现有Coupled、视觉和其他算法。

CONTRACT:
固定20个真实扫描2986..3005，外置Xsens。无GT，无在线建图，无候选/Recovery。
沿用现有15维EKF标量状态，未导入FAST-LIO23维协方差；使用明确标记的现有固定初始P。
原uint32 offset_time保留；精确输出header与预测用header_ns+max_offset_ns。
Deskew reference_pose是IMU pose，乘T_I_L后成为同一scan-end LiDAR NDT初值。
NDT只做每帧一个原align，实际Gaussian0.8，先resolution再target。
已有时间问题允许一次针对性修复：rollback head到measurement先用真实held head
调用同一个propagateImu推进，再施加测量、保存measurement时刻快照、重放剩余真实IMU。
不能用future tail预测measurement；默认关闭不改变旧路径；不声称分段与旧未分段数值等价。
现LEGACY_HISTORY的scan-end partial也是causal head，两个experimental轨迹模式false。
回放必须先等真IMU处理，再发cloud，关联同measurement_ns的fusion回执以及本帧发出后
新收到的corrected，才允许下一帧。corrected从double重建stamp，与原IMU允许<1us表示差。
不得用静态/预录轨迹冒充连续估计。

请提供具体成员/行号、反例和最小修复建议。只输出与本合同有关的真实漏洞；
未知执行结果不得当作已经发生的失败或通过。
