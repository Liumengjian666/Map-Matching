# 四个世界左乘转换：假设而非官方语义证明

记 `T_target_source` 为 source 到 target。D 是数值种子的假设世界系，
M 是发布PCD的XYZ系，L0为原scan3。仅检验：

`T_M_L0=[Q,0]*T_yaml`，`R_candidate=Q*R_yaml`，`t_candidate=Q*t_yaml`。

四个Q采用exact整数proper旋转，不引入三角函数近零项：

```text
Q0 = [ 1  0  0;  0  1  0; 0 0 1]  yaw 0°
Q1 = [ 0 -1  0;  1  0  0; 0 0 1]  yaw +90°
Q2 = [ 0  1  0; -1  0  0; 0 0 1]  yaw -90°
Q3 = [-1  0  0;  0 -1  0; 0 0 1]  yaw 180°
```

生效seed t为[39.929523,-176.217148,-5.225427]。
四个变换后t分别为：

```text
Q0: [  39.929523, -176.217148, -5.225427]
Q1: [ 176.217148,   39.929523, -5.225427]
Q2: [-176.217148,  -39.929523, -5.225427]
Q3: [ -39.929523,  176.217148, -5.225427]
```

真实float32初始4×4矩阵记录在initial_matrices.json和world_candidates.csv；
不改变原YAML精度，不读注释矩阵，不做求逆/平移偏置/任意yaw/右乘轴组合。

P32-R3的候选仅右乘旋转且t不变；本轮明确改为左乘整个变换。
但D是否确实是该字段所定义的世界、数值seed是否是L0 pose仍未官方认证。
几何成功只能支持工程假设，不能单独证明官方语义；本次没有几何成功。

## 相同几何方法

Source直接读取原R3 local PCD，SHA与R3 pre_gt_freeze一致，2569点不重新voxel。
Target使用同一原PCD和原voxel(map,0.20f)，2378870点。
复用原PCL KdTreeFLANN<PointXYZ>，float平方距离、sqrt、sorted[N/2]和
floor(.9*(N-1))，并对全部有限点计<=0.5m/<=8m覆盖。
没有近似索引替换、地图裁剪、Source重建或额外点云筛选。

四次筛选均失败，8m覆盖均为0，故没有调用原地图ICP。
+90°数值种子虽然落入整个地图bbox，实际表面NN median仍47.298m，
不能以bbox包含作为初始化PASS或坐标语义证明。

没有评价其它合法世界变换，也没有证明其不存在。
没有产生T_M_L0，故无holdout、normalization、启动状态或F/G结果。
