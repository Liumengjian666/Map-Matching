# 时间合同：未具备实测准入输入

状态：TIME_CONTRACT_NOT_ESTABLISHED；不是发现了具体时间违规。

[官方平台说明](https://hilti-challenge.com/dataset-2022#hardware)给出相机FPGA同步、相机/LiDAR采用PTP和传感器对齐<1ms。这是发布方平台级证据，不代替Exp18实际字段审计。

固定版本完整bag缺失；两个partial未作为rosbag打开。以下全部NOT_EVALUABLE：

- 真实topic/类型、PointCloud2字段名/类型/偏移/step。
- 逐点时间单位、header语义、scan_start/end。
- header相对bag record时差。
- IMU实际frame_id、时间单调、重复、缺口、频率。
- 每个完整扫描区间的IMU包围与覆盖。
- 实际录制驱动版本、时间戳转换与该文件的一致性。

TIME_COVERAGE.csv只保留明确NA原因记录，不虚填0帧故障或正常覆盖。没有从partial截取首帧后将其当作完整输入验收。

只读GT时间格式检查见reference_schema.json与SPARSE_LABEL_AUDIT.csv；原始时间文本经Decimal转整数ns，未用四舍五入double推断纳秒精度。没有搜索时间偏移、拟合bias或运行任何运动积分。
