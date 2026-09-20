# CODE-ARCH-1：estimator core types

本阶段只做保持行为不变的类型边界整理。目标是让估计器数据结构脱离 ROS Node 类的声明位置，同时保持现有节点仍然拥有并读写这些状态。

## 提取内容

新增 `include/dog_prior_map_localization/core/estimator_types.hpp`，仅依赖 Eigen，迁移了原 `dog_prior_map_ekf_node.hpp` 中已有的：

- `Matrix15d`
- `Vector15d`
- `Matrix3x15d`
- `ImuSample`
- `FilterStateSnapshot`

字段、默认值、矩阵尺寸、状态顺序均保持不变。`FilterStateSnapshot` 仍表示当前估计器的 `p, v, R, ba, bg, P`；误差状态仍按现有 `[p, v, theta, ba, bg]` 的 15 DoF 约定使用。未对坐标系语义作超出源码证据的声明。

## 明确不变

- 没有修改任何 `.cpp` 文件。
- 没有修改 IMU 传播、NDT、视觉、OOSM rollback/replay 或输出逻辑。
- 没有修改 ROS topic、参数、launch、时间戳语义或矩阵计算。
- `DogPriorMapEkfNode` 仍拥有运行时估计器状态；本阶段只移动类型定义。

## 为什么暂不抽 OOSM manager

`saveStateSnapshot()`、`restoreStateSnapshot()`、历史查找、回滚和重放仍然与当前节点状态及回调顺序紧密耦合。现在抽出它们会同时改变调用边界和时间行为，超出本阶段的风险预算。因此 OOSM manager 留给后续独立阶段，并要求每一步都进行确定性回放验证。

## 后续候选边界

下一层可以在不改变算法的前提下，围绕显式的时间戳记录、测量记录和状态快照建立纯数据接口；随后再单独隔离 NDT timing/map 与 OOSM。视觉 frontend 在进入正式融合前仍应只返回不可变测量结果，不能直接写 EKF 状态。
