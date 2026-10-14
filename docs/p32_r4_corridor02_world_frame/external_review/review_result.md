**未发现可由所给材料证实的实质性问题。本次是按合同停止，程序没有运行报错。**

- **实际停止原因：四个预筛选全部不合格。** Q0、+90、−90、180 的初始 median 分别约为 292.944、47.298、363.551、159.844 米，8 米覆盖率全部为 0，均不满足 `median<=8 && coverage>=.5`。[候选证据](/tmp/p32-r4-cli-review.aVFvMC/docs/p32_r4_corridor02_world_frame/runs/world_once/world_candidates.csv:2)、[筛选条件](/tmp/p32-r4-cli-review.aVFvMC/src/dog_prior_map_localization/scripts/p32_world_frame/world_test.cpp:93)

- **候选构造和停止行为符合合同。** 四个 Q 是指定世界左乘旋转，种子使用 `Q*R`、`Q*t`；历史主函数被重命名，没有执行其中的右乘候选搜索。运行记录为四次筛选、零次地图 ICP，退出码为 0，下游全部未执行。[候选构造](/tmp/p32-r4-cli-review.aVFvMC/src/dog_prior_map_localization/scripts/p32_world_frame/world_test.cpp:8)、[历史入口隔离](/tmp/p32-r4-cli-review.aVFvMC/src/dog_prior_map_localization/scripts/p32_world_frame/world_test.cpp:2)、[调用账本](/tmp/p32-r4-cli-review.aVFvMC/docs/p32_r4_corridor02_world_frame/runs/world_once/world_test.log:1)、[退出记录](/tmp/p32-r4-cli-review.aVFvMC/docs/p32_r4_corridor02_world_frame/runs/world_once/execution.json:9)、[下游状态](/tmp/p32-r4-cli-review.aVFvMC/docs/p32_r4_corridor02_world_frame/runs/world_once/admission.json:6)

- **未执行路径：未发现实质性防护缺陷，但不能声称已获运行验证。** 源码按 `(初始median,Qid)` 排序并最多保留两个候选；最终准入同时检查收敛、有限 fitness、全部几何阈值及合法 SE(3)。本轮没有进入这些 ICP 分支。[排序和预算](/tmp/p32-r4-cli-review.aVFvMC/src/dog_prior_map_localization/scripts/p32_world_frame/world_test.cpp:97)、[最终准入](/tmp/p32-r4-cli-review.aVFvMC/src/dog_prior_map_localization/scripts/p32_world_frame/world_test.cpp:107)、[矩阵合法性](/tmp/p32-r4-cli-review.aVFvMC/src/dog_prior_map_localization/scripts/p32_world_frame/world_test.cpp:21)

结论仅限于这四个冻结假设未通过本次筛选，不能据此排除其他合法变换，也不能判定官方 YAML 或地图有错。