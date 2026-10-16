# P49 Record-Only 设计

## 边界

本模式以 `coupled_mode=CONTROL` 为强制条件，默认关闭。启用后只订阅高频 EKF Odometry、维护现有因果历史、计算与生产路径相同的 scan-time EKF prediction / Anchor 生命周期 / innovation 触发并写记录。不得运行 Weak-only、Coupled、候选评分、额外 NDT align 或反馈。

生产 NDT 原始结果、Step Limit、EKF/OOSM 测量路径仍使用 CONTROL 结果。Record-Only 的 Anchor 状态只供记录分支，不改变 NDT measurement。

## 单线程因果时序

NDT 节点入口使用 `ros::spin()`，回调串行。点云处理回调内运行 NDT align 时，EKF high-rate 回调不会并发写入样本历史。记录同时比较所选样本 `callback_accept_wall_sec` 与本帧 `ndt_align_start_wall_sec`；不满足严格先后关系即将该帧判为因果无效。样本 ROS header stamp 还必须不晚于 scan reference，且通过既有最大年龄/body-twist prediction 合同。

## 无额外优化

Record-Only 仅调用共享创新计算（SE(3) 位姿差）和已有 Anchor advance/settle 状态转移。没有构造候选，也不调用 `runWeakCoupledRefinement()`。NDT 节点日志和 P46 帧诊断用于确认每帧只有原 CONTROL align。

## 日志

运行时在独立且此前不存在的目录创建：

- `prediction_records.csv`
- `anchor_records.csv`
- `trigger_eligibility.csv`
- `record_overhead.csv`

原有诊断 CSV 仍由冻结 P46 launch 配置生成。`source_hash64` 与 `target_cloud_hash64` 是代码中的稳定 FNV-64 诊断摘要，不是密码学哈希；原始输入和 Tile manifest 使用 SHA256 冻结。

每条消息的 `state_revision` 不由已发布 Odometry 暴露，因此记为 `NOT_EXPOSED`；记录订阅端历史 revision、header seq、publisher 和墙钟接收/接受时刻。可在运行后用 EKF lineage 的精确 state stamp 独立关联其 revision，但不能反向改变在线 prediction。
