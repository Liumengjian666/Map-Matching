# 坐标来源审计：未闭合

统一T_A_B为B→A。M为采用E57站位变换导出的地图系，W为参考世界系，I为原始IMU系，L为Pandar系，P为measurement_tip。当前没有实际E57，故M的数值原点/轴向尚未检查。

理论链T_M_L=T_M_W T_W_I T_I_L仅为恒等式，不证明任一实物变换已知。合同把T_M_W文字写为“地图到世界”，与公式/记号方向相反；本轮采用W→M，未知数值仍NA。

## 证据

- [官方论文Sec.V-A–C](https://arxiv.org/html/2208.09825#S5)：控制点取自注册TLS，稠密参考注册测试LiDAR到同一先验。支持共同来源，不自动证明发布E57与发布参考的世界变换为恒等。
- [官方FAQ](https://github.com/Hilti-Research/hilti-slam-challenge-2022#how-are-the-frames-defined-on-the-sensor-setup)：将I定义为Alphasense IMU，并称GT在此系。
- [官方phasma URDF固定版本](https://github.com/Hilti-Research/phasma_description/blob/79002026e9b229cf9b4e95a403eaec4f00acfd79/urdf/phasma.urdf.xacro#L45)：将base注明为GT框架，并给出parent=alphasense_imu、child=base、rpy=(-π,0,0)、零平移。因此按URDF父子变换语义T_I_base=Rx(-π)。**不能在未核验Exp18导出程序时直接将此矩阵应用到GT**。
- 同URDF把Pandar挂在tip下，给出T_tip_L的平移[-.060,0,.095]m及零rpy。它不是T_M_W。tip与747B YAML的measurement_tip是否完全同义尚未认证。
- [该URDF引用标定](https://github.com/Hilti-Research/phasma_description/blob/79002026e9b229cf9b4e95a403eaec4f00acfd79/urdf/calibrations/20220322_calibration.yaml)只包含imu_to_cam0…4的xyz/rpy，不能用它解释另一份lidar_calibration.yaml的四元数排列。

两份实际Git小文件的字节数/SHA与commit见source_identity.json；未用最新版驱动冒充录制版本。一次GitHub API目录请求403 rate limit后，仅通过公开Git传输读取官方源文件，没有绕过认证。

## 数值测试边界

可确认YAML数值有限、四元数数组长度4，但范数不决定排列或方向。未对未知T_M_W、T_I_L或T_W_I进行伪造往返“PASS”。已知URDF纯数值旋转可作代数检查，但不能认证物理精度或本序列GT约定。

未完整读取E57，未检查其coordinateMetadata/每站pose、未导出PCD、未运行ICP/Umeyama、未用GT选轴。因此结论为COORDINATE_CONTRACT_NOT_ESTABLISHED，而非已证明外参错误。
