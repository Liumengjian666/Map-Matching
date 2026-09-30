# PAPER-P6-ALG-INTEGRATION-A3A

本轮完成稀疏协方差热路径和 Corridor01 原始 LiDAR 输入资格收口。没有运行真实定位回放、GT 评价、参数调优或正式实验；旧 FULL 默认行为及 frozen baseline 保持。

START_SHA: `bf4e103e28588c103991cb3521d06cb1a0e120ee`

CODE_SHA: `9d8a794dac7f06b0e52dc2f8c4aadcbfc5138a0c`

END_SHA 与 push/remote 核验结果见最终交付消息。原始数据导出执行的脚本 Git SHA 为 `09a812e01eeea5b579006569a9d6f270764e4779`；最后一个代码提交只补充 V3 初始化期 raw schedule 处理，未改变导出字节或 decoder。

## 十项答复

1. **latest marginal 是否不再 dense 生产求解？是。** 同一 block assembler → SparseMatrix 平衡 → 未阻尼 sparse LLT → 15 RHS。没有完整联合 H⁻¹、dense fallback、jitter 或 IKFoM P 回退；dense oracle 仅供测试/debug。
2. **48-node synthetic 耗时？** 本轮同图 dense oracle 117.091 ms，sparse 4.16642 ms；A2D 历史 dense 约 113 ms。只是单次 synthetic complexity sanity，不是正式 CPU/WCET 结论。
3. **non-LiDAR 是否停止求 P？是。** scan-start/reference/current 全部 0 次请求；诊断明确 NOT_REQUESTED_NON_LIDAR_EVENT/NaN。V2 3/7 events、V3 3/10 events 请求 P，且均为 current LiDAR factor 入窗前。
4. **Corridor RAW_TIMED_SENSOR 是否真实生成？是。** 独立持久化目录 `.../Corridor01/results/p6_a3a_v3_input/`；2777 scans、79,932,911 points、3,197,316,440 bytes。导出验证与独立 validate-only 均 PASS。
5. **逐点时间证据？** 固定官方 ros-drivers/velodyne 提交 `29abd0e…` 的 VLP16 firing table/unpack，真实记录 packet.stamp + firing offset。没有用 point index 或 bag record time。硬件时钟来源及同步精度未独立证明，manifest 强制 false。
6. **Corridor visual 满足 RAW_SENSOR_LOCAL_DEPTH？本轮不能证明，因此 NO。** rot_point 数值只用 raw IMU rotation+固定标定，但发布/共享有效性 gating 依赖旧 NDT odometry。482 条 sidecar 均 UNKNOWN，formal V3 全部拒绝，不“改名”旧数据。
7. **legacy estimator state 泄漏进入 V3 正式输入？本轮交付的 raw LiDAR 无。** 导出没有 estimator/map/Window pose，legacy cloud 仍拒绝；有控制流依赖的旧视觉产品也没有获准入窗。不能把这个结论扩大成完整 FULL 实测已通过。
8. **Floor01 为什么不可 formal？** 旧 XYZ 是 LEGACY_STATE_DERIVED_SE3_DESKEW，旧视觉是 LEGACY_STATE_DERIVED_DEPTH，旧 bundle 缺逐点时间。原始 `/cmu_sp1/velodyne_packets` 可能可恢复，但本轮只审计，没有重新导出或重标来源。
9. **使用 GT？NO。** 未读取 GT，没有 ATE/RPE 或 GT 调参。
10. **已运行正式定位实验？NO。** 只有 synthetic 算法测试和真实原始传感器解码/完整性检查，真实数据 NDT align=0。

## 保留合同与验证

Release build PASS；原 21 项 CTest 全保留，加 sparse marginal 与 input contract 共 **23/23 PASS**。Debug 定向 **7/7 PASS**；Python 数据合同 **15/15 PASS**。P15/P_map6 covariance parity 满足 1e-9，backward error 满足 1e-10。重复 Schur、A1-R1 信息守恒、23D→15D fixed-gravity prior、P0/P1/P2/P3、selected pre-measurement NIS、A_exact、directional/cross-state visual、Window sole owner 和 post-handoff IKFoM=0 回归均保留。

内部定向反向审查促成了分解排序门槛一致性、非 LiDAR 调用计数、动态库实际绑定身份、输出覆盖保护、manifest 时间/计数检查和 scan-start minimum 验证。发现均修复并加检查；跨模型外部复核按用户选择在最终汇总后手动进行，尚未执行，不能写成已经外部验收。

## 边界

READY_FOR_SHORT_REAL_LINK_TEST=**YES（raw LiDAR input/build eligibility only）**。

READY_FOR_FORMAL_EXPERIMENT=**NO**。

视觉独立深度来源、packet clock/sensor sync 的独立精度证明以及真实短链路运行效果均未完成，不宣称本轮改善定位精度。完成 A3A 后停止，没有进入 A3B。

详细回传材料：[DECISION_AI_HANDOFF.md](DECISION_AI_HANDOFF.md)。
