# 有界复核

使用 code-review-and-quality 与 doubt-driven-development；范围仅为本轮变换、输入合同、
计数和归档工具，不重新审计整个定位仓库。

Claim：新 A/B 的唯一数值变化是官方 Y 的两个左乘方向，输入及统计合同继承 P33。
重要性：右乘、错误求逆或点云变化都会使方向对照失效。
交给 fresh-context reviewer 的只有 artifact+contract，未附此 claim 或作者推断。

独立只读 reviewer `/root/p36_transform_review` 的实际结论：

> 未发现实质问题。只读核对了左乘及完整逆、整数纳秒、原始 XYZ/time 字节校验、
> P33 插值复用、继承／新算 SHA 标记，以及 collector 的 H1→A、H2→B 映射、
> C0/C1 矩阵核对和预处理字节一致性检查。依赖导入未触发前序脚本 main。
> 未运行测试、几何计算或读取大型 bag。

主执行端对照实际 artifact 核查，未发现需修正的 substantive finding；不进行新的审查循环。
三个测试覆盖左乘对旋转/平移的作用、非交换性、完整逆保留舍入系数、反射/非有限/形状拒绝。
实际准备及 collector 又检查三帧源字节、GT、矩阵和输出计数。图像已实际查看，未做二次对齐。

交互中已明确提供可选的 Codex CLI/Gemini CLI/手工跨模型第二意见。
截至本归档，未收到本轮精确命令与限定输入的授权，没有执行外部 CLI，
不能将历史任务的 CLI 授权复用于本轮。此项是可选复核，不影响已经发生的六组固定统计。

结果解读保留反例：A 的相对改进很大，但所有帧 0.5m/2m overlap 仍为0，不能称为三维几何通过。
不把 `Planar=DARPA` 或 GT 参考点的未认证事实补成已知关系，不以 runtime 初始化替代官方语义证明。
