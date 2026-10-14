# 有界独立只读复核

Artifact：world_test.cpp、原包含的initializer、run_once.py及冻结合同。
Reviewer：p32_r4_world_review，fresh-context；无GT、无实验、无源码修改。

唯一有效可修复发现：legal()未检查齐次末行，可能接受非SE(3)矩阵。
执行前已补exact末行检查和反例测试；实际math_test通过。
输入实际路径/SHA、原ICP依赖、一次性目录及<=2ICP回执未发现其他实质问题。

随后按代码质量技能核对范围、数据边界、预算和性能统计。
用户选择追加并确认本轮精确命令和输入后，外部CLI实际调用一次，
gpt-6.1-sol/xhigh、read-only、approval=never、退出0。
限定输入SHA：dc4fbc32c89640b8f8b1ea8cb5e5e6cea7ddcaa7d3b260f7df873e685d7dd2af。
原始输入、输出、日志及执行回执保存于external_review；旧授权不复用。

外部复核没有发现限定材料中可证实的实质问题。主代理对照实际代码
核对了左乘R/t、四候选几何和零ICP停止条件，结论与回执一致。
它也明确指出未执行ICP/holdout/normalization/F/G分支并无运行验证，
不能将静态检查通过写成工程运行PASS。没有因此重跑实验或修改结果。

外部原输出的临时路径链接仅是stdin材料的文件标签，不是实际创建的
源码文件路径，也不是额外数据验证证据；原输出保持不改。
本次只读复核不等同于官方世界坐标语义认证。
