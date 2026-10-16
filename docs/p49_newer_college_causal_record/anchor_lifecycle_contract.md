# Anchor 生命周期合同

Record-Only 复用生产 `advanceCoupledAnchorFromPrediction()`、`advanceCoupledAnchor()` 与 `settleWeakRefinementAnchor()`，没有平行实现：

- 有合法预测时，以相邻 scan-time prediction 的相对位姿推进现有 Anchor。
- 传播时间间隔必须严格递增且不超过 0.25 s；真实状态函数还检查位姿、Anchor 年龄及生命周期。
- Anchor 原始寿命沿用 2.0 s，不延长。
- 若预测失效，沿生产路径失效并记录原因。
- 仅在不触发的普通稳定帧（或现有生产路径因创新不可计算而将其当作未触发）时，用 causal prediction settle；触发帧不新建 Anchor。
- Record-Only 不产生 `PendingCandidate`，不做 branch admission，不调用候选反馈。

所谓“pair eligible”只表示：成功 NDT、有效因果 prediction、真实冻结阈值触发、该帧传播后的 Anchor 合法且未冻结、Anchor propagated stamp 等于 scan reference、生命周期有效、并且不是首帧。它表示后续 Shadow 数据合同可能成立，不代表已运行精修或 Feedback 轨迹中 Anchor 历史成立。
