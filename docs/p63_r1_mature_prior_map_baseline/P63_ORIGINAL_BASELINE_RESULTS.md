# 条件性原版基线：提前失去跟踪

正式状态：`SMOKE_TEST_FAILED_EARLY`；`RESEARCH_BACKBONE_NOT_YET_ACCEPTED`。用户已批准历史零/I假设 Smoke，但这不构成物理外参验收。按预注册首 400 扫描只跑一次；没有改窗口、初值、重力或配准参数重试。

实际命令（已执行，固定输出目录现已存在，脚本会拒绝重复覆盖）：

```bash
/usr/bin/python3 /home/jian/livox_ws/p63_r1_super_lio_baseline_ws/docs/p63_r1_mature_prior_map_baseline/tools/run_smoke.py
```

监督器在专用 localhost ROS master 11631 启动 `p63_loop2_frozen_map.launch`，随后发布原 bag 前缀，开启 RViz；精确命令及执行时 Git HEAD 在 `raw/loop2_smoke_01/RUN_COMMAND.json.gz`。本地原运行目录 `/home/jian/livox_ws/p63_r1_super_lio_data/loop2_smoke_01` 保留，所有关键日志逐字压缩归档。

| 时间顺序 | 真实记录 |
|---|---|
| 外部地图加载 | 459154 PCD 点压缩为151401 OctVox点；不插入当前扫描 |
| 原生初始化输入 | 首10扫描，58407保留点、185 IMU；人工原点/yaw0＋作者重力估计 |
| 启动优化 | NDT1次、ICP1次，ICP fitness=.10959，原生初始化宣布成功 |
| 连续跟踪 | 95 次作者 ESKF Observe 执行与状态输出，不是95次独立地图观测质量通过 |
| 有效几何支持 | 75输出至少4个有效平面匹配；第76个跟踪输出开始不足4，累计连续20个后停止 |
| 退出时输入 | 107 LiDAR和2125 IMU已发布；发布不等于全部处理；最后输入扫描未形成跟踪输出 |
| 退出方式 | observer设置tracking_failure_stop，返回2；required节点使roslaunch正常结束码0，不能用roslaunch的0判为成功 |
| 目标身份 | 实际跟踪前后151401点、相同几何SHA；并非重新建图或仅靠新扫描运行里程计 |

完整原生轨迹见 `ORIGINAL_TRAJECTORY.csv`：最后估计位置 [-97.3976898,-196.0295715,-51.2203560]m；相对首次估计位置的范数225.114052m，覆盖9.400211s原生输出时间。这只是异常估计轨迹的数值，不是独立GT误差，也不能当真实机器人速度。所有95行保留，没有裁去失效部分。

plane_matches 是作者最后匹配掩码的只读计数；final_cached_plane_rms 由最终姿态与缓存平面算出，76条有限，平均0.154840m、P95 0.199859m；其中1条不足4点，不能仅看此均值认定可靠定位。它不是重新做全地图评估，也不证明正确性。

实际同时存在 IMU、原生去畸变、固定地图平面约束及 ESKF 更新，但轨迹不稳定；不能因程序运行和有限输出宣称地图定位通过。正式 ATE/RPE没有合法独立GT，写 `NOT_EVALUABLE`。历史FAST-LIVO2轨迹不参与在线初始化／选择，也未用于事后对齐。

后验只读初始化诊断显示38.021059°重力方向分歧，详见传感器合同。这个线索有助于决定唯一下一问题，不能根据它改外参或把失败归因为已证实的作者数学错误。即便ICP残差较低，也不能替代世界重力及物理外参的一致性证明。
