# 执行前本地反证复核

2026-10-20。用户选择继续本地验证，不执行外部CLI。

- 数学只读复核：真实PCL路径数值对照、G的非零姿态有限差分、共同权重成对优势、无资格对手仍参与比较、训练/验证边界及heldout隔离；修正后未发现执行阻断。复核者没有运行align，测试结果来自主执行日志。
- 驱动只读复核：发现并在真实执行前修正CRLF最后一列读取、非法恢复候选继续执行、缺少完整输入SHA守卫。修正后未发现执行阻断。
- 主执行额外修正：超时/计算后身份门失败也保存失败回执；内存字段明确为Linux getrusage(RUSAGE_SELF).ru_maxrss，单位KiB，不称为/proc VmHWM。
- 已实际执行的C++测试：`unit_native.log`输出`CONTRACT_TESTS_PASS categories=18 synthetic_native_aligns=2 real_aligns=0`。这是一个覆盖18类断言的程序，不是18次真实数据实验。2次合成原生NDT分别用于研究封装与原生PCL数值对照，独立于真实100帧预算。
- 修改RSS列名只触及CSV标签，重新构建shadow，数学测试二进制未改动；不会为了验证标签再次执行合成align。

生产工作区保持只读。后续实际输入哈希、二进制、源码提交及运行终态以run_freeze.json和run_completion.json为准。测试通过不代表候选恢复有效或定位精度通过。
