# 独立反证审查与修复记录

用户选择仅当前独立审查，无外部CLI或跨模型调用。
采用doubt-driven-development、code-review-and-quality、incremental-implementation、
git-workflow技能：先读真实残差/外参接口，再小组件与实际坐标测试，冻结科学
源码后运行；工程与科研结论分开。共享references文件缺失，按主技能继续。

## 运行前几何审查（fresh context）

Actionable：原nominal chart在非零候选处需要SO(3) left Jacobian运输，不能
将重心化切空间直接用于nominal Q。已补J_left(theta)并用原chart有限差分。
Actionable：必须反算实际输出covariance验证strong block，不验证自构造中间量。
Actionable：双精度covariance输入日志；未触发R0；候选与实际消费测量/矩阵分开。
全部运行前修复，8/8测试通过；第二轮审查无剩余阻塞。
Trade-off：固定R API不在IKFoM内部逐迭代运输，记录为局部一阶启发式。
Trade-off：重根下逐基对角规则存在基依赖，记录具体反例；不擅改冻结公式。

## 执行与GT前评价审查（另一fresh context）

Actionable：jump统计也属于GT前工程输出，将最终freeze移到统计完成之后。
Actionable：未消费候选时必须从registration双精度raw Pose3d重建测量；不能
以旧float诊断矩阵套1e-8容差。保留严格容差，未放宽。
Actionable：Shadow合法零反馈，修正量分布null，不伪造零样本数值。
Actionable：所有具名表无条件检查4127顺序ID，不能只对恰好4127的表检查。
主进程修复后，reviewer拦截CSV写入、不读GT，独立复验12381守卫、8254一致性、
1079covariance、357配对通过。paired covariance最大差3.65e-16。
审查未编辑文件；无剩余阻塞后收束，不追加理论认证项目。

## 汇总工具修复（无科研影响）

最终archive出现dict重复stop_after_this_task关键字异常；保留已经产生的
paired_candidates_posthoc_gt.csv，汇总重启读取该CSV，不重跑NDT、不覆盖输出、
不再读取GT。此为序列化修复，runtime binary、规则、blind freezes完全未变。
真实科研运行始终仅三次；运行后针对性算法修复次数0。
