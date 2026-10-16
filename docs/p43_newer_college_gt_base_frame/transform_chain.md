# 帧定义与变换链

约定 `T_A_B` 将 B 坐标变换到 A：`p_A = T_A_B p_B`。`M` 为与官方 5 cm PLY 同 XYZ 的 P41 无损 PCD 表示，`C` 为 RS_C1 左相机 optical 轴，`B` 为机器人 Base 轴，`I` 为 Ouster IMU，`L` 为 Ouster LiDAR。

## 两个且仅两个被测试的姿态解释

**A / Optical 假设**：发布 GT 行解释为 `T_M_C`。

```text
T_M_L = T_M_C × T_C_I × T_I_L
p_M   = T_M_L × p_L
```

**B / Base 假设**：发布 GT 行解释为 `T_M_B`。

```text
T_M_L = T_M_B × T_B_C × T_C_I × T_I_L
p_M   = T_M_L × p_L
```

`T_C_I` 来自 Kalibr 字段 `cam0.T_cam_imu`（IMU → RS_C1 optical）。`T_I_L` 来自 Ouster 标定字段 `os1_lidar_to_os1_imu`（LiDAR → IMU）。未重复施加任何标定。

## Base 与相机 optical 轴关系

根据官方轴定义：Base X forward、Y left、Z up；相机 optical X right、Y down、Z forward。二者物理原点共点，所以平移为零：

```text
T_C_B = [ 0 -1  0  0 ]    T_B_C = inverse(T_C_B) = [ 0  0  1  0 ]
        [ 0  0 -1  0 ]                            [-1  0  0  0 ]
        [ 1  0  0  0 ]                            [ 0 -1  0  0 ]
        [ 0  0  0  1 ]                            [ 0  0  0  1 ]
```

`T_C_B T_B_C = I` 的 Frobenius 误差为 0；两旋转的正交误差为 0、行列式均为 +1。该轴旋转只在 Base 假设中出现一次。

## 标定矩阵（米）

Kalibr `T_C_I`：

```text
[ 0.709921628386 -0.704141666247  0.013992691116  0.015627525087 ]
[ 0.024600026822  0.004936229792 -0.999685186604 -0.019816480433 ]
[ 0.703850921879  0.710042356124  0.020826240168 -0.075441430050 ]
[ 0               0               0                1              ]
```

Ouster `T_I_L`：

```text
[ -1  0  0 -0.006253 ]
[  0 -1  0  0.011775 ]
[  0  0  1 -0.028535 ]
[  0  0  0  1        ]
```

标定文件同时提供的 `T_L_I` 与 `T_I_L` 互逆；本次只在右侧使用 `T_I_L`。

## GT、时间与输出姿态

GT CSV 列顺序由官方说明和真实文件头确认是 `sec,nsec,x,y,z,qx,qy,qz,qw`。使用 SciPy `Rotation.from_quat([qx,qy,qz,qw])`，归一化偏差均小于 `2.81e-7`。三帧扫描 header 与 GT 行时间戳逐纳秒相等，没有插值。

Base 假设得到的 `T_M_L` 中，LiDAR +Z 轴相对地图 +Z 的倾角分别为 2.035°、2.040°、1.425°；Optical 假设为 88.899°、87.709°、88.779°。这与地图中地面/竖直结构的 Z-up 几何一致性相符，但仍属于几何证据。

全部三帧的 6 个完整 `T_M_L` 矩阵保存在 [input_manifest.json](input_manifest.json)，原始 CSV 行也记录在其中，便于复算。
