# P49 — Newer College 因果 Coupled Anchor 记录

## 结果

**FINAL_RESULT: `CAUSAL_RECORD_VALID_NO_ELIGIBLE_PAIRS`**

Record-Only 合同、1800 帧 CONTROL 运行及 P46 输出 parity 均通过；但冻结触发条件 `translation > 0.12 m OR rotation > 3 deg` 在 1799 个有效因果预测帧中从未触发，合法非首帧配对为 0。因此没有运行 Weak-only、Coupled 或 Equal-Budget 精修，不读取 GT，不产生候选反馈。本轮到此停止，不启动 P50。

## 冻结运行

- 数据：Newer College 2020 / `01_short_experiment`，P46 相同连续 1800 帧。
- 传感器时间：`1583836591182590976 ns` 至 `1583836771078157312 ns`；发布器实际完成 1800/1800 帧。
- 状态合同：`CONDITIONAL_BASE_FRAME / ORACLE_START_ONLY`，仅首帧初始化使用条件性参考；运行中不读取 GT。
- 运行入口：`newer_college_causal_record.launch`，`coupled_mode=CONTROL`、Record-Only 开启、`rviz=false`。
- 运行源码基线 HEAD：`0c03b24c2b6908ecde57749799c8dc31a043d07d`；运行二进制 SHA256 见 `input_freeze.json`。
- 原始记录保存在 HIKVISION：`/media/jian/HIKVISION/paper rosbag/NewerCollege/.p49_causal_record/formal1800_run1/`。文件路径、大小及 SHA256 见 `record_bundle_manifest.csv`；本地文档不复制大型逐帧记录。

## 运行合同与因果证据

- 1800/1800 NDT 收敛，1800/1800 OOSM `APPLIED`，最大 pending overflow 为 0。
- 1799 帧具有有效 EKF prediction 且满足消息 callback 在本帧 NDT align 开始之前已接受、样本 stamp 不晚于 scan reference、prediction query 早于同帧 NDT odometry 发布。首帧因没有更早的合法 EKF 样本，标为 `NO_CAUSAL_EKF_SAMPLE`，未伪造 prediction。
- Prediction age：mean 2.4274 ms，P95 4.0160 ms，max 9.9960 ms。
- Innovation：平移 mean/P95/max = 0.02217/0.03884/0.11963 m；旋转 mean/P95/max = 0.43390/0.51336/1.14415 deg。两项最大值均未超过严格触发门槛。
- 生产 Anchor 状态被记录：before valid 1715 帧、传播后 valid 1632 帧、settle 后 valid 1716 帧、settle 后 frozen 0 帧；invalid reason 为 `EXPIRED=83`、首帧 `CAUSAL_EKF_DELIVERY_INVALID=1`。该首帧通用失效码对应预测不可用；其 Anchor 初始状态为 missing。记录使用生产生命周期，不为触发而构造 Anchor。
- 记录中的 Anchor age mean/P95/max = 1.128/2.100/2.201 s；P95/max 包含已过期并被标记 invalid 的记录，不代表这些 Anchor 仍符合有效寿命合同。有效 pair 合同要求 Anchor 合法、未冻结、已传播至当前 scan 且未过期；本次触发为零，因此无 Anchor 可配对。
- 触发：首帧 0、非首帧 0；合法非首帧配对：0。资格原因：`NOT_TRIGGERED=1799`、`PREDICTION_UNAVAILABLE=1`。
- Coupled jet/value/attempted/feedback 计数全部为 0；本轮没有调用精修器、候选评分或额外 NDT align。

## CONTROL 不变性

修复后的 verifier 使用 P46 与 P49 外盘原始 CSV 对全窗口重算：

- 1800 帧 timestamp、Source hash、Tile IDs、target 点数、NDT 收敛/迭代、OOSM 状态、initial/raw/final 位姿及 fitness 全部逐帧一致；最坏平移、旋转、fitness 差均为 0。
- 轨迹对照 21600/21600 行：EKF high-rate 18000、NDT 1800、EKF corrected 1800；缺行/额外行均为 0，最坏平移/旋转差均为 0。
- 20 帧 smoke 也严格通过。验证器负向注入中，删除轨迹行及复制轨迹行都会失败退出。

复核包 SHA256 为 `04945992b9e6ca0c2f556367448e8c5efa1697d1ccbb7dbaa5cb8554e66b9b5f`。外部只读反证指出原实现的调度差异、无效来源先推进 Anchor、以及 parity 验证遗漏轨迹对照；相关问题已按最小范围修正。该限定包未含 P46/P49 完整原始轨迹，外部复核未能独立重算 21600 行；全量数值 parity 已由本地 verifier 对真实外盘记录完成。

## 资源

- 正式运行 wall time：233.697 s（约 180 s 传感器时间），不是实时性验收。
- NDT align mean/P95/max：8.783/11.295/311.287 ms；Deskew mean/P95/max：20.912/26.105/37.455 ms。
- Record write wall mean/P95/max：0.355/0.764/1.975 ms；CPU mean/P95/max：0.090/0.130/0.347 ms。
- NDT VmRSS/VmHWM：369908 KiB；EKF VmRSS/VmHWM：23804/23888 KiB；publisher VmRSS/VmHWM：44212/45980 KiB。
- 同步采样的 roslaunch 定位进程树峰值 RSS 估计：509140 KiB；CPU 时间：118.69 s。各进程 RSS 不应和系统总内存混为一谈。
- 5 次真实在线 tile 更新（另有一次初始加载）；tile 数 21–50，local target 552716–2313118 点。该项是继承 P46 的真实运行记录，不是本轮 Coupled 实验变量。

## 证据边界

本结果证明 Record-Only 在该冻结 1800 帧窗口取得了有效因果 prediction/Anchor 生命周期记录，并保持 CONTROL 定位输出与 P46 全窗口一致。它没有提供任何满足冻结触发阈值的非首帧样本，因而没有回答 Coupled 是否优于 Weak-only 或等计算预算的局部精修。P43 Base-frame GT 仍是条件性参考；本轮未读取 GT，因此没有精度结论。Record-Only 的无候选 Anchor 历史也不等价于未来 Feedback 轨迹中的 Anchor 历史。

## Git 与归档

运行时本地/远端 HEAD 均为 `0c03b24c2b6908ecde57749799c8dc31a043d07d`；P49 修改在其后加入。源码、测试、运行配置和本报告的提交 SHA 与 push 状态以执行回执为准。P48 未提交归档及所有其他历史缓存均保留，未纳入 P49 文件集合。

运行原始逐帧文件不会随仓库提交；按 `record_bundle_manifest.csv` 在对应 HIKVISION 路径校验 SHA256 即可复核。

## 唯一 NEXT

科研总控决定是否在不改变冻结触发阈值的前提下，另行批准具有自然触发事件的场景/数据采集；执行端不扩帧、不换场景、不调阈值，也不开始 P50。
