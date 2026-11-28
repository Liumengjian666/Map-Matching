# Hilti-Oxford 参考和实测事件

- 稠密 `exp18_corridor_lower_gallery_2_imu.txt` 有 789 条。作者以注册的 Z+F 地面扫描、逐帧点面 ICP 与 VILENS 融合获得连续六自由度轨迹，论文报告通常 1–2 cm。它是基于几何匹配的参考。时间范围 1649856227.723471–1649856314.319139 s，有 30 个记录间隔超过 0.15 s，最大 1.300 s。本项目不跨越这些间隔插补连续评价。 [论文第 V 节](https://arxiv.org/pdf/2208.09825)
- 稀疏 `exp18_corridor_lower_gallery_2_imu_3dof.txt` 有 3 个独立测针控制位置，匹配录包 `/Point_name` 中 P30、P7、P36。索引块实际 ROS 时间与发布十进制时刻差最大 128 ns。第 3 个点超出稠密参考尾端。 [时间戳审计](SURVEY_EVENT_CLOCK_AUDIT.csv)
- 官方评价脚本对稀疏参考使用 YAML 中约 0.205 m 的 (T_{IP}) 平移杆臂。稠密 IMU 参考对应零杆臂。作者挑战脚本允许至多 2 s 的时刻关联和后验刚体对齐。先验地图固定世界评价采用已认证的固定变换，未使用后验对齐。 [官方评价源码](https://github.com/Hilti-Research/Hilti-SLAM-Challenge-2022/blob/e4aacf7c111ab832a0bdffe3f069255454068540/evaluation-2022/evaluation.py#L192)
- 假设发布稠密世界与稀疏世界同系，对两个共时点实施时间插值和测针杆臂，差距仍为 0.595301 m 和 0.286632 m。不加杆臂的诊断距离为 0.402608 m 和 0.114231 m。事件时间核查和标定复用无法给出 (T_{MG_s})。这不是正式定位误差，也不足以定位差异来自地图配准、轨迹生成或世界系定义。禁止用这两个测试点自由拟合并回报同批数据性能。 [无拟合原点核查](REFERENCE_ORIGIN_CROSSCHECK.csv)
