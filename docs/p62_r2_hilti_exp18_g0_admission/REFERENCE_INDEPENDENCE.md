# 参考独立性与三个标签

本轮实际读取：assets/ground_truth/exp18_corridor_lower_gallery_2_imu.txt与对应_imu_3dof.txt，均实算大小/SHA匹配合同。只核验格式、支持时间和来源，不计算定位误差。文件摘要和绝对路径见ASSET_MANIFEST.json。

[数据集论文Sec.V-B/C](https://arxiv.org/html/2208.09825#S5.SS2)区分：稀疏位置来自地面十字控制点；稠密位姿由去畸变测试LiDAR到注册TLS先验的点点ICP生成，VILENS/IMU因子参与运动校正。故后者只允许标记 **CONDITIONAL_REGISTERED_REFERENCE**，发布方1–2cm描述不是本项目已认证精度。

稀疏测量与TLS地图共用测量基准，并不由本项目NDT生成；但只有位置。_3dof文件3行均带4个姿态占位值0,0,0,1，不是独立姿态测量。文件头5列与实际8列不一致，不能根据头部把第二列强行解释为整数ID。精确列消费者与IMU/tip参考语义仍需核验。

三个时间戳均逐条保留在SPARSE_LABEL_AUDIT.csv，未用最近帧替代。第3个在稠密参考末端之后，但bag时间范围未知，不能据此删除。三者的实际传感器可配对性都NOT_EVALUABLE，未执行任何插值或外推。

当前不能用于独立稠密6DoF主张；也不在本轮替总控判定这些限制必然不可接受。无合法非GT初值已建立：未采用任何参考点/姿态初始化，未手填单位矩阵，未测试粗定位器。INITIALIZATION_NOT_ESTABLISHED。
