# 因果 EKF Prediction 合同

生产 helper `ekfPredictionAtOrBeforeLocked()` 以 scan reference ns 做 `upper_bound`，选取不晚于目标时刻的最近历史样本；样本必须是合法刚体位姿且年龄不超过冻结 20 ms。对于正年龄，调用现有 `predictMapTLidarWithBodyTwist()`，使用 Odometry 的 `child_frame_id` 轴约定速度传播；零年龄使用精确样本。

Record-Only 不重建、不插值未来样本。除 stamp 合同外，还要求消息 callback 接受时间不晚于本帧 NDT align 开始时间。由于 NDT 节点使用串行 `ros::spin()`，此字段可审计预测数据在本帧长耗时 align 前已经进入历史。查询发生在 NDT align 后、同帧 NDT Odometry publish 前；它不可能读取本帧 NDT 发布结果。

拒绝条件包括无样本、未来首样本、刚体位姿无效、过期、twist 外推失败、publisher/frame/child 不匹配、消息在 align 开始后才接受、或 query 与本帧 NDT publish 顺序不满足。无合法 prediction 时不填补数值，不将 NDT 初值或 GT 代作 prediction。

`map_T_lidar` 是运行时 pose 合同；EKF Odometry 的 `frame_id`、`child_frame_id` 与实际速度轴均逐帧保存。`ekf_sample_state_revision` 未通过消息字段提供，写作 `NOT_EXPOSED`，不是推断值。
