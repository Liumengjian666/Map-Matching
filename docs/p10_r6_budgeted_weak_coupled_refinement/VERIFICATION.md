# R6 验证记录与边界

## 编译与测试

- Standalone P7/P10 CMake Release：`/tmp/p10_r2_build.CdJ4jf`。
- FAST-LIO2 依赖 commit：`7cc4175de6f8ba2edf34bab02a42195b141027e9`。
- P7/P10 测试 7/7 PASS（V0 和 V1），包含真实 displaced-jet 合成目标、强方向
  能量变化、保持弱坐标、总位姿边界、半步后在实际位置重新构造 jet、数值失败
  回退、普通帧零预算、弱/耦合消融一致性、锚点保留/过期/原 R5 消费行为。
- P9 Release build 和 41/41 self-tests PASS；没有运行真实 P9 oracle/B12。
- 当前修改是独立研究 runner 和 opt-in 计算模块；未运行生产 ROS launch，
  不能把 standalone 测试写成生产部署或实时最坏时延认证。

## 回放与信息隔离

V0 和唯一 V1 均各运行 3 个完整 4127 帧模式，按 Shadow、Weak-only Feedback、
Coupled Feedback 顺序。规则/代码/input/binary 在执行前冻结；三种输出完成并
冻结后先执行工程检查，再读 GT。V1 是看过 V0 的开发改进，不是独立验证。
Control 复用 R3 全段档案，不重跑。全部帧保留；GT 仅覆盖 4126 帧，最后一帧
不外推。实际 corrected IMU trajectory 是主评价，raw measurement 单独记录。

GT 前逐帧检查：最多 2 jet/3 value/0 extra align；普通或锚点缺失帧额外为零；
每帧仅 1 nominal align；刚体/finite/更新成功/物理边界；实际测量与候选绑定；
真实后续预测改变；独立 anchor chart/rho/objective/强方向 score 一致性；
Shadow 原始 source/nominal/filter trajectory 8254/8254 exact parity。
V1 额外检查每个有效锚点的 after origin/prediction/stamp 均未吸收当前反馈。

## 审查与修复

按独立反证审查技能进行了小范围 fresh-context 只读审查。初版修复了锚点
chart 方向、总步长不应改变弱坐标、half weak 后必须重新构造 displaced jet、
NaN 不能伪装为质量改善、旧网格日志 ID 不得伪造等问题，均在真实回放前。
V1 审查补充 after anchor 原点/预测日志与末帧核查。没有执行外部跨模型 CLI。
技能促使这些守卫和测试落地；审查通过本身不是精度提升证据。

V0 后处理 JSON 序列化失败（NumPy int64）。保留 partial 文本和错误日志，
只修复序列化与只读 summary repair。原回放、规则、GT CSV 没有重跑或替换。
原始未完成 JSON 字节保存在持久 cache 的 `attempt_0/evaluation.partial.original.txt`；
Git 中 partial 文本仅删除其最后一行尾随空格以满足 diff --check，未伪装成合法 JSON。
曾有一次错误 cwd 的 ctest 未找到测试，不计为通过；最终使用真实 build cwd
运行并保存 7/7 与 41/41 PASS 日志。

V1 Weak-only TX1014 的 nominal 达 80 次迭代上限，effective=0；原 C++ 正确
保留有限预测而没有注入无效测量。Python 驱动器错误要求所有帧 update_success=1，
暂停了第三个尚未运行的模式。修复仅涉及 harness/评价器，不改二进制或科学规则。
4/4 harness 回归测试 PASS。resume receipt 绑定原二进制/input/规则及前两 job
输出 SHA，跳过已完成 job，仅运行未启动的 Coupled。GT 前同时检查无效帧
zero attempted/jet/value/feedback、清锚点、lidar_update_applied=0、corrected pose
exactly 等于 predicted pose。不能把这帧写成 nominal SUCCESS 或滤波更新成功。

## 保护与交付

稳定 workspace、历史 R1–R5 archive、原 raw 输入和 `.git` 只读限制均保持。
实际 Git 工作副本为 `/tmp/dog_loc_paper_r4_ws.Fq21k2`；原工作区的用户
untracked AGENTS.md/Testing/IDE 文件不混入科研提交。完整中间缓存与版本二进制
位于原工作区 `.p9_experiment_cache/p10_r6_budgeted_weak_coupled_refinement/`，
属于允许写入的持久 ext4。最终使用独立 bundle 交接，没有主动 push。
