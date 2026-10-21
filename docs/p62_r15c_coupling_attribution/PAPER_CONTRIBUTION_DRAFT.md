# 论文贡献草稿（仅供科研总控审查）

## 现有方法的具体空缺

在 R15B 的固定先验地图 NDT 后处理原型中，Weak-only 到 Coupled 的改善曾经看起来较一致；但 Coupled 同时包含普通强方向梯度项和强弱交叉项，不能把整体差异归因给 `Hsw`。本轮要回答的窄问题是：在弱向修正、强块、正则化和信赖域完全相同时，额外交叉项究竟贡献多少真实收益？

## 本轮改动

本轮没有提出新的完整定位算法，而是把

\[
d_s^C=-(H_{ss}+\lambda I)^{-1}(g_s+H_{sw}d_w)
\]

明确分解成 Gradient-only 与耦合增量：

\[
d_s^G=-(H_{ss}+\lambda I)^{-1}g_s,\qquad
d_s^X=-(H_{ss}+\lambda I)^{-1}H_{sw}d_w.
\]

并在相同冻结真实输入下直接比较 Gradient-only 与 Coupled 的动态 PCL Gaussian 目标、Coverage V2 留出损失及受限 benchmark-reference 相对误差。

## 与最近工作的区别

DCReg 最接近“显式交叉块/Schur 处理”的数学组件；SuperLoc 研究预测风险和里程计先验的融合优化；X-ICP、GenZ-ICP 分别处理局部可定位方向及退化鲁棒 ICP；VICET 已涉及固定 HD 地图 NDT。R15C 的小差异是同一个冻结 NDT 子空间候选上的交叉项边际归因，而不是 Schur 补、特征值退化检测、运动先验或地图 NDT 本身。完整范围见 [`NOVELTY_COMPARISON.md`](NOVELTY_COMPARISON.md)。

## 数学合理性

对正则化的强块 Hessian 方程，Coupled 相对 Gradient-only 的强块原始差值满足

\[
d_s^C-d_s^G=-(H_{ss}+\lambda I)^{-1}H_{sw}d_w.
\]

该式只说明交叉项会造成什么更新，不保证真实动态关联 NDT 目标或未参与优化的结构留出损失下降。因此两类目标被分开实测，且仍保留原先的 `D>0.02` 门槛。

## 实验实际显示

alpha=0.5 的 25 帧中，动态目标 Coupled 更低 16 帧，但留出均值只更低 13 帧；Newer College 子集留出 Coupled 反而在 9/20 帧更低。共同最坏权重 `D>0.02` 为 0/25。Coupled 与 Gradient-only 的均值位姿差约 0.973 mm / 0.00749°。R14B 20 帧地图锚定参考相对误差显示 Coupled 平移 RMSE 较 Gradient-only 仅低约 0.063 mm，旋转 RMSE略高；这不构成可解释的定位精度改善。

因此暂不建议把“耦合项带来选择性 NDT 恢复”写成论文贡献。可以把 R15C 写成一个机制归因消融，说明 R15B 的总体变化不能归功于 `Hsw`。形式退化方向在这 25 帧中为 0，R15C 全部是探索性谱尾探针；Newer College 参考是 map-anchored benchmark reference，不是独立动捕真值。

## 额外计算成本

新 Gradient-only 候选的动态目标加留出评价均值约 1.740 ms/帧/alpha（P95 1.934 ms）；强块求解均值约 0.00555 ms。两张固定地图各自只初始化一次 Gaussian Grid；R15C 不调用 NDT align、不运行 EKF。该离线消融不证明在线实时性。
