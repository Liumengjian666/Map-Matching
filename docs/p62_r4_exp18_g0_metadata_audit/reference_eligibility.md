# 参考资格：不得计算定位误差

本轮实际语义读取了两个 GT 文件：

1. `/home/jian/livox_ws/p62_r2_hilti_exp18_g0_data/assets/ground_truth/exp18_corridor_lower_gallery_2_imu.txt`
2. `/home/jian/livox_ws/p62_r2_hilti_exp18_g0_data/assets/ground_truth/exp18_corridor_lower_gallery_2_imu_3dof.txt`

它们的大小、SHA 在前后资产快照中逐项一致。GT reads 不记为 0；这里统计的是2个唯一语义输入文件，不是系统底层read调用次数。

## 稠密参考

资格保持 `CONDITIONAL_REGISTERED_REFERENCE`。
[官方论文 Sec.V-C](https://arxiv.org/abs/2208.09825) 描述以测绘 TLS 地图为先验，将去畸变 LiDAR 扫描通过 point-to-point ICP 注册，使用 VILENS 与 IMU 预积分处理运动畸变。它是有用的条件性开发参考，不独立于本测试 LiDAR 和同源 TLS，不能当作完全独立的稠密绝对真值。

实际 789 行、每行8列，时间 1649856227723471000–1649856314319139000ns；数值有限、无重复/倒退时间。后四列范数约1仅是数值检查，不认证其排列、姿态方向或 I/base 语义。没有插值或外推任何参考姿态。

## 稀疏位置

实际3行8列，后四列均为0,0,0,1，未作为姿态真值使用。
文件注释是 `# timestamp_s id x y z`，与8列正文不一致；正文没有文字ID。`sparse_label_audit.csv` 将位置列保持为原始数值，并把 bag marker 只记作按时序/时间关联的候选标签，未伪称原文含P30/P7/P36。

| 行 | 文本时间s | bag候选marker | bag/IMU支持 | 稠密参考时间支持 |
| --- | --- | --- | --- | --- |
| 1 | 1649856228.9143598 | P30 | 有 | 有 |
| 2 | 1649856287.0620654 | P7 | 有 | 有 |
| 3 | 1649856335.6804302 | P36 | 有 | **无** |

marker record时间与文本换算时间差依次为53、-128、34ns，完整整数值见表。没有用最近扫描替代标签，也没有设定或放宽匹配容差凑样本。
第三个点在本 bag 支持内，但在 dense 末端之后，不能写成“3/3由稠密参考支持”，也没有擅自删除该点。

论文的独立测量过程是将金属测量尖端放在已测绘十字点，而发布文件/FAQ标称IMU框架。目前缺少该发布文件的 measurement_tip→IMU 处理说明，以及与 E57 根坐标的数值基准关系。因此资格是 `SURVEY_DERIVED_POSITION_ONLY_WITH_REFERENCE_POINT_CONTRACT_OPEN`，不认证可直接应用到当前 IMU 位姿的绝对误差。

没有利用任何参考数据建图、生成初值、拟合变换、选择坐标轴或选择算法结果。ATE/RPE及其他定位误差计算次数为0。
