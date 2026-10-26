# 本轮必要验证与执行记录

- 实际命令：`/usr/bin/python3 docs/p63_r2_frame_provenance_closure/tools/check_frames.py`。
- 成功执行CODE：`b8339e277971c87c71d3a0be664e650120f0f722`。
- 初次分析在选取旧日志dict记录时误查字典键，报StopIteration；只修复为检查text字段。没有传感器／优化器调用，没有输出任何应用变换；失败和修复保留Git历史，未改参数、均值或输入。
- 对固定输入验证FromTwoVectors旋转的正交性、行列式、比力+Z／重力-Z方向、前后组合逆关系、正确符号下角度一致性。全部数值见FRAME_NUMERICS.json；属于代数验证，不是定位性能实验。
- 原日志矩阵打印舍入导致非严格正交，保留并报告误差，未以SVD拟合或定位误差优化修正。
- Python AST通过；15份所选输入前后SHA一致；稳定机器狗快照、R1工作树和两套框架源码状态保持不变。FAST-LIVO2原有未提交改动保留，Super-LIO保持pristine。
- 源码摘录保留原作者尾随空格，故全树diff-check会提示这类空格；不改原样证据。排除SOURCE_EXCERPTS后的研究文件diff-check另行通过。
- 五轴自审：数学符号／方向和输入身份明确；离线脚本与估计器隔离；没有新状态或在线门限；只存相关日志摘录而不存ROS环境转储；有限旧文件读取和一次成功坐标计算，没有新数据或完整bag哈希。
- 本轮NDT=0、ICP=0、ESKF=0、传感器重放=0、地图／外参修改=0、下载=0、删除=0。无Release重编译需求，未运行历史实验替代必要测试。
