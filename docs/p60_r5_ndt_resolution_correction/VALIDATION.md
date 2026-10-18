# 验证与反证复核

## 自动验证

1. `test_corrected_grid`：不调用align，仅初始化synthetic grid。
   构造旧R4对象，证明getter=0.8不能通过actual cells guard；新对象XYZ=.8f严格通过。
   step/epsilon/iteration/outlier参数与R4相同，setInputSource后grid保持一致。
2. `test_sampling.py` 3项、`test_statistics.py` 6项，冻结R4代码未修改，全部通过。
3. `test_frozen_results.py` 4项：输出字节身份、真实XYZ网格、输入及调用预算、分离参考和冻结判据计数，全部通过。
4. 600个完整6×6矩阵NumPy/eigen向量独立复核通过，精确残差见results.json。
5. 输入SHA/输出SHA/二进制SHA分开保存；输出冻结发生在分析之前。

## 新上下文只读反证复核

采用 doubt-driven-development 技能，断言范围限于“本次网格修正和统计确实满足冻结对照合同”。
复核对象：corrected_ndt.hpp、corrected_screen.cpp、analyze_results.py及未修改R4数学／预处理依赖。
复核者不读取GT、不访问/media、不写文件、不运行NDT。

复核结果：没有发现具体可操作合同违约。
逐项核实配置先后和actual leaf、300次identity align、600次native jet、两套参考、输入及输出哈希、SPD和判定门槛。
因此无需为了复核重新运行实验或修改科学规则。

已接受且明确披露的限制：

- 旧固定姿态可能不再是新网格的驻点，Hessian不是统计信息矩阵保证。
- 曲率尺度依赖冻结的0.8平移归一化。
- 原始相邻扫描未经去畸变，无GT，不能推出定位误差或协方差。

已向用户提供可选跨模型复核入口。未执行外部CLI，不能将本次内部独立复核描述为跨模型复核。
需要外部CLI时必须另行确认精确命令与限定输入；不将历史授权复用于本轮。
