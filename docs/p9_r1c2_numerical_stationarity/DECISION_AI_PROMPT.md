【科研总控复核输入】
TASK: PAPER-P9-R1C2-NUMERICAL-STATIONARITY-CERTIFICATE-CLOSURE
请直接检查 THEORY.md、REPORT.md、results.json、stationarity_roots.csv 和 double_stationary_endpoints.csv。
FINAL_RESULT=FLOAT_NUMERICAL_STATIONARITY_FLOOR_CONFIRMED
完整 certificate=11/14 logical；6/9 independent。
TX616 六个 forward 只算一个独立 root；P10 reverse support 未闭合；P09 两个 endpoint 原 float FD 失败仍不放行。
double 固定 support 参考梯度约1e-10；投回float仍约2e-5至7e-5；未放宽原solver阈值，未调用NDT或continuation。
唯一下一步=SUPPORT_AWARE_CONTINUATION_R1C3，仅从完整认证子集开始。
环境 .git 只读，本轮分支和commit尚未创建；请在可写环境完成附带Git命令后再给出实际SHA。
