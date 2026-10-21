# 实现差异与运行入口

R15C 仅在研究归因工具中新增 `GRADIENT_ONLY`，没有修改 R15B/R6–R14B 源码或生产定位程序。

## 文件

- `tools/gradient_only.hpp`：在相同冻结子空间和 `lambda` 下用 LDLT 分别求 `ds_G`、`ds_X`、`ds_C`，检查有限性、分解恒等式、正定性及相同单位信赖域上界。
- `tools/run_attribution.cpp`：复用 R15B 固定关联/动态 PCL、R9 Coverage V2、R12 V3 选择器及冻结的 R13/R14B 输入。读取冻结 `T0/TP/Hss/Hsw/gs/dw`；新候选只构造 Gradient-only，逐行确认 R15B Nominal、Weak-only、Coupled 动态目标及留出损失重现。对所有方法输出动态分数、活动验证块、Overlap、V3 正式准入和共同成对优势。全程序没有 NDT align 或 EKF 调用。
- `tools/test_gradient_only.cpp`：公式分解、零交叉项、单位信赖域、非正定强块、非有限梯度拒绝测试。
- `tools/verify_r15b_outputs.py`：验证上游冻结的四张模型表。
- `tools/freeze_model_outputs.py`：在读取 Newer College 参考位姿之前封存 R15C 模型输出及二进制身份。
- `tools/evaluate_reference.py`：仅在封存后使用固定的 20 个 Newer College 参考时刻计算 Reference-Relative Pose Error。
- `tools/summarize_r15c.py`：从已封存 CSV 计算主/辅 alpha、数据组、共同最坏权重 D 及运行成本汇总。

## 复现顺序

```bash
cmake -S docs/p62_r15c_coupling_attribution/tools -B /tmp/p62_r15c_coupling_build_01
cmake --build /tmp/p62_r15c_coupling_build_01 -j2
/tmp/p62_r15c_coupling_build_01/test_gradient_only
python3 docs/p62_r15c_coupling_attribution/tools/verify_r15b_outputs.py /home/jian/livox_ws/p62_r15b_piecewise_coupled_ws/docs/p62_r15b_piecewise_coupled
```

正式无 align 归因命令及运行日志见 `run_01/formal_run.log`。模型输出先由 `freeze_model_outputs.py` 独占创建 seal；随后只运行一次 `evaluate_reference.py`。结果 JSON 可由 `summarize_r15c.py` 从封存输出再生。

PCL 1.10 原生 Gaussian 目标和 R15B 现有实现被复用；没有另写近似 NDT 或替换其分数。
