# 最近邻几何选点源码核查

实现属于 **A 与 C 的组合**。距离项比较候选地图，当前帧回波提供射线和双侧门控，未用当前点对两个候选地图的最近邻残差之差作为分值。

| 步骤 | 源码证据 |
|---|---|
| 原生 0.10 m 均匀滤波后的源点用原生点号。未经再次下采样的世界地图使用候选雷达到世界矩阵的逆变换，同一 0.4–8 m 球形裁剪 | [make_view 和 compute](../p64_r3_q1_observation_contribution/geometry.py#L82) |
| 对当前帧单位射线用 cKDTree（空间近邻索引），每地图点以 7.5 cm 支持球求首交，按交深和图点号取最早命中 | [first_intersections](../p64_r3_q1_observation_contribution/geometry.py#L52) |
| good 条件为双方首交各距真实回波≤0.30 m，不能存在更近遮挡点，近界与原点位于支持球时否决。缺图或深度超门均标未知 | [make_view](../p64_r3_q1_observation_contribution/geometry.py#L82) · [compare_pair](../p64_r3_q1_observation_contribution/geometry.py#L215) |
| 取同一射线首交支持球的图点中心 qA、qB，分别查询对方裁剪图的最近邻。dᵢ=max(minₓ∈MB‖qA−x‖, minᵧ∈MA‖qB−y‖)。≤3 cm 共同，≥5 cm 显著，其余未知 | [双向查询](../p64_r3_q1_observation_contribution/geometry.py#L221) |
| 双侧支持且显著者按 dᵢ 降序，数值相同按点号升序；取 ceil(N×50%)，短缺以 0.30 m 体素交错序去重补齐。共用点号掩码、原生无关联奖励为零、求和，再另列共有分母 | [排序回填](../p64_r3_q1_observation_contribution/contribution_selection.py#L53) · [预算](../p64_r3_q1_observation_contribution/contribution_selection.py#L129) · [奖励累计](../p64_r3_q1_observation_contribution/contribution_selection.py#L266) |
