# 测试与审查

首轮 `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s docs/p62_r5_exp18_coordinate_closure/tests -v`：7/7 PASS，0.066s。

涵盖：YAML/MOLA一致、官方URDF雷达轴、0.253mm差异保留、非交换位姿往返、I/base的pi旋转不可互换、1094/1093首帧边界、非法四元数拒绝。
这些是纯数学和冻结元数据测试，不是物理标定、GT精度或定位有效性证明。

新增网络工具检查HTTP状态、单文件3MB上限、独占写入；只保存证据，不执行来源内容。一次错误目录404已保留，随后定位到mola-cli-launchs正确路径；一次不必要的MOLA总仓库稀疏clone在90秒超时，未得到HEAD、未消费其任何源码，没有继续重试。两个官方小型description源码clone成功；实际使用文件均另按固定commit保存SHA快照。

代码质量审查范围：只新增本任务docs目录，不安装依赖，不改生产源码，不读取大型输入内容。已知限制：网络采集程序面向一次性独占路径，重复执行须用新目录；不覆盖已有证据。

待追加：最终决策的fresh-context反证复核及处置。
