# 坐标记号与固定假设

T_target_source将source点转换到target。
W：GT数值表达的世界；B：假设GT机体参考点；I：RC1 IMU；
L：cmu_rc1_velodyne；M：发布PCD XYZ；D：YAML名字中darpa的未知世界语义。
不将D、W、M、B自动合并。

H1：T_BI=Identity，T_WI=T_WGT，T_WL=T_WGT*T_I_L。
H2：GT直接是LiDAR，T_WL=T_WGT。
两者都是待检验假设，不是已认证的GT exporter事实。

原RC1 T_I_L：

```text
[ 0.999954875339377  0  0.009499857104811  0
  0                  1  0                  0
 -0.009499857104811  0  0.999954875339377  0
  0                  0  0                  1 ]
```

平移严格0，旋转约Y轴+0.0095rad（0.544309906deg）。不用逆方向，
不做SVD修改原标定。GT quaternion仅为建立旋转归一化，不改原文件。

C0：T_MW=I；C1：T_MW=T_yaml；C2：T_MW=inverse(T_yaml)。
最终T_ML=C*T_WGT*[T_I_L或I]。矩阵顺序如上；H1并非把相对增量
伪装成绝对位姿，而是本轮获准的离线GT辅助诊断。
全部静态C和标定见world_matrices.json；30个实际T_ML见geometry_comparison.csv。
active YAML原舍入值保留，C2是真4×4逆，不以转置代替逆，不读注释候选。

因T_I_L零杆臂，H1/H2在同C下的传感器原点位置完全相同，只差小旋转。
本次配对原点差为0，median差最大0.000873566m，P90差最大0.003997803m。
但6组合都没有8m内地图对应，因此不能在错误世界关系下据细微分数
判定Body=IMU或GT=LiDAR。GT_ORIGIN_NOT_IDENTIFIABLE_FROM_GEOMETRY。

NN数据是RAW整帧在header位姿下的点到map几何；不是ATE，也不是定位
轨迹收益。raw扫描畸变未被消除，限制保留，不用GT修补点运动来争取PASS。
本轮GT没有进入正式初始化、运行YAML、NDT align、EKF或F/G。
