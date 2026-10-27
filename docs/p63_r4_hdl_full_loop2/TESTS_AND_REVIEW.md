# 必要检查与复核

采用git-workflow、incremental-implementation及code-review-and-quality技能约束范围；本轮没有启用外部AI、CLI第二意见或额外算法实验。检查范围是读取／诊断不改变原生状态和数据身份，不把工程测试解释为科研精度通过。

|检查|证据／结果|
|---|---|
|运行前全部原始LiDAR身份|10905条；bag完整SHA；严格递增header、有限XYZ、19-byte点记录；首400与R3完全一致|
|参数、桥接和ELF|`LAUNCH_BRIDGE_EQUIVALENCE.json`、`SOURCE_FREEZE.json`；原生参数相同、桥接字节相同、六份R3库无重建|
|新调度器|`tools/test_contract.py`重放400条旧诊断记录；筛选／增量一致至1e-12；完整行读取及半行保留通过；align0|
|真实运行逐帧对应|3377个输入／native／callback／adapter／status／odom同时间戳；调用序号1–3377；16项后验检查全部通过|
|封存后统计工具|`summarize.py`对每字段报告总数、有限数、非有限数；已知平移／同姿态检查通过；未重新优化|
|原始档案|每个gzip解压后的SHA与实际原文件一致；manifest列原始路径、bytes、SHA及压缩件SHA|
|地图|原始文件SHA前后一致；3377次target XYZ／点数相同|
|原生源码和实际编译物|四上游pristine／Git SHA不变，六库SHA相同；原作者核心修改0|
|稳定机器狗|HEAD、index、关键源码及完整-uall状态摘要前后一致；初次收尾检查误用默认status折叠未追踪目录，改用原-uall命令即完全复现旧摘要，没有改动文件|
|R3前400轨迹一致性|同一R4必经前缀的全部位姿分量与R3已存值相同；不是另一条重复运行或GT评价|
|GUI|真实运行scan20及停止scan3377截图均显示冻结地图／当前扫描；不以可视一致认定精度|

## 实际错误保留

预启动manifest缓冲未flush导致拒绝：旧摘要与完整CSV摘要不同。使用同样缓冲写入精确复现旧摘要后，仅加flush并更正摘要；完整输入manifest内容不变。先提交修复CODE，再启动唯一正式传感器运行。拒绝时NDT0、发布0、ROS0，`full_trial_01`尚未创建。`evidence/PRE_LAUNCH_REJECTION.json`和`logs/formal_full.log`保留。

封存后导出原生位姿增量时，一次tuple/list拼接类型错误使分析CSV写入中断；统一位姿差函数返回list后重新生成全部分析CSV。该错误发生在ROS退出、运行回执封存之后，不影响原始轨迹／耗时，新增NDT0。没有把导出错误当作新的定位失败或重新回放理由。

## 源码复核结论

- 正确性：实际运行只更换调度和读取，不改作者predict/correct、align、参数或目标。外部筛选与原生UKF更新明确分开。
- 可读性／结构：所有新增代码留在独立package和R4档案。离线分析不导入ROS，不写原始输入或原生状态。
- 安全：使用明确bag/map/输出路径，独占运行目录；无删除／push／凭证输出；私有ROS master隔离输入。未归档包含完整环境快照的roslaunch调试目录。
- 性能：复用原生Release二进制，无额外align。增量CSV读取避免全段O(N²)解析；诊断和GUI成本显式保留，未冒称不带诊断的性能。
- 科研解释：合同B仅由连续外部支持不足成立；没有证据证明绝对位置错误。不得把正常收敛和小进程RSS直接写成论文鲁棒性或实时性通过。

原始完整命令在`raw/COMMAND.json`，调度命令在`RVIZ_ACCEPTANCE.md`。分析命令为：

```bash
/usr/bin/python3 docs/p63_r4_hdl_full_loop2/tools/summarize.py
/usr/bin/python3 docs/p63_r4_hdl_full_loop2/tools/archive_receipts.py
/usr/bin/python3 docs/p63_r4_hdl_full_loop2/tools/plot_results.py
```

这些命令只读取已封存输出，不启动第二次定位。图、CSV及报告结果都来自全部3377个实际输出，包含22个外部无效帧。7528个未处理扫描没有填充成成功样本。
