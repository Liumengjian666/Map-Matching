# 定向只读复核与处理

使用增量实现/Git工作流保存独立离线实验；代码质量复核和一次有界反证复核没有扩展算法范围。

Claim：两个初值只取Y及完整数值逆，Source和统计合同与P33/P36一致，GT不生成初值。
重要性：任何隐含GT乘法、错误参考时间或更换点云都会使本轮问题失效。
给reviewer的是artifact+contract，没有附该claim或作者对结果的预期。

Fresh-context reviewer `/root/p38_initial_pose_review` 只读源码、小型freeze和矩阵字节，
没有执行测试、NN、配准、大bag读取或其他代理。

第一项finding（valid/actionable）：准备程序加载的P33 JSON含旧GT矩阵，因此
`GT_pose_read_before_candidate_freeze=False` 这一旧字段字面过强。
处理：区分 `GT_text_file_poses_parsed_before_candidate_freeze=False` 和
`P33_archive_loaded_before_freeze_including_unused_GT_matrices=True`；协议同步明确。
旧输出留在本地 `input_original_audit_label`，修正后输出与旧候选/XYZ经cmp字节相同。
没有更改选帧、初值或算法；修正前没有执行几何。

复核更正及collector后未发现其他实质问题，确认：两槽映射Y/inverseY；点序字节一致；
2NN/0align计数；冻结门槛和A优先；XY/XZ/YZ统一物理尺度。另一个注释用语小问题
也改为“GT text-file poses”，不改变程序行为。

结果产生后主执行端只将collector门槛布尔字段名由含糊的
`initial_pose_geometric_support` 改为 `preregistered_strict_geometry_gate_pass`，
并同步改名已归档JSON键。它只表示严格门槛，不否定实际粗重合；数值、门槛、原始CSV
及计数均未改变，未因此重复运行collector或NN。

本轮已显式提供可选跨模型意见；截至归档未获本轮精确命令和限定输入授权，未执行外部CLI。
历史任务授权不复用于本轮。单模型复核完成后停止审查循环。

结果反证边界：正向A的粗重合是真实正面线索；未通过额外保守门槛不等同于Y不适合NDT。
GT数值差不称为定位误差，不把目录用途自动升级成source/target/时间认证。
