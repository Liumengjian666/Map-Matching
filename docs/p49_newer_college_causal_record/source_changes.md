# P49 源码变更清单

本轮只加入 CONTROL 下的因果观测记录，没有启用 Coupled 在线反馈，没有改 EKF/OOSM 更新数学、IMU Deskew、NDT 优化器或冻结触发阈值。

| 文件 | 变更 | 在线行为影响 |
|---|---|---|
| `src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp` | 加入默认关闭的 Record-Only 参数、fresh CSV 检查、EKF high-rate 样本元数据与因果 prediction/Anchor/trigger 记录；输入拒绝路径和预测-before-publish 时间合同；明确禁用会增加优化调用或改变测量的实验分支。 | CONTROL 下只旁路订阅并记录。Record-Only 不推进 EKF watermark、不由 EKF callback 驱动 pending-cloud pump，也不改变 scan 调度或 CONTROL measurement。 |
| `src/dog_prior_map_fastlio2_frontend_exp/include/.../coupled_ndt_weak_refinement.hpp` | 提取纯函数 innovation 计算及冻结阈值比较供记录和 refinement 共用。 | 不调用 NDT、score、jet 或 candidate；保持严格 `>0.12 m OR >3 deg`。 |
| `src/dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_weak_refinement.cpp` | 生产 Shadow refinement 改用共享纯 gate，避免观测与既有算法的阈值实现漂移。 | 不改变原数学或阈值。 |
| `src/dog_prior_map_localization/launch/dog_prior_map_localization_split.launch` | 暴露 Record-Only 开关与记录目录参数，默认关闭。 | 普通 roslaunch 默认行为不变。 |
| `src/dog_prior_map_localization/launch/newer_college_local_map.launch` | 透传两个关闭默认参数。 | 默认配置不变。 |
| `src/dog_prior_map_localization/launch/newer_college_causal_record.launch` | 新的冻结 Newer College 观测入口，固定 CONTROL、本轮数据范围和分离记录目录。 | 不包含候选反馈。 |
| `src/dog_prior_map_localization/CMakeLists.txt` | 注册严格触发门槛/无效数值输入合同测试。 | 测试构建目标，不进入运行节点。 |
| `src/dog_prior_map_localization/tests/p49_causal_trigger_contract_test.cpp` | 检验 0.12m/3deg 边界采用严格大于，及非有限/无效变换拒绝。 | 定向单元测试。 |
| `src/dog_prior_map_localization/scripts/monitor_p45_local_map_run.py` | 允许监控器启动 P49 launch。 | 仅进程监控与资源采样。 |
| `docs/p49_newer_college_causal_record/verify_smoke_contract.py` | 校验冻结输入、逐帧诊断与完整轨迹 source 集合；拒绝缺失/重复键、缺失/额外轨迹行和非有限 pose。 | 仅离线验收。 |
| `docs/p49_newer_college_causal_record/summarize_record_run.py` | 校验 1800 帧计数/时间序/parity，汇总 prediction、Anchor、trigger、资源和 SHA256。 | 只读 P46/P49 输出；不读取 GT。 |

构建目标及验证：`catkin_make --pkg dog_prior_map_localization -j2` 成功；`p49_causal_trigger_contract_test` CTest 1/1 通过；launch XML 与 roslaunch 解析通过。构建后二进制 SHA256：NDT `47fdc06728ab9e832e148560de8f5de53879331bee6abfd36bb1e6ec88684f7e`，EKF `930a3ca96e178d63afd58874ca3974dadcbd87908e58e39a3d15c85807d25d2d`，publisher `024b8f679c16a325acb73bf6b4e91698f4a7a025dd47ed90de3cfb6fef565e66`。

外部只读反证后确认并修正的三处：P49 不能改变 pending scan 的 watermark/回调调度；因果交付校验必须在 Anchor 传播之前；parity 必须比较全部 trajectory source 行并拒绝缺失或重复。修正后 smoke 与 1800 帧正式记录均使用最终上述 NDT/ EKF 运行二进制和最终 NDT node source hash `941c15a4455c47b8d9c9aa742dc1dcdbde82cc2bac81fb4a54075e98f307b609`。
