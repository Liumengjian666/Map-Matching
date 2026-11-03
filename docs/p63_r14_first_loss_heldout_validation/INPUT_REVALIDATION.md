# R14 输入身份复核

## 结论

P63-R10 的 Koide `indoor_hard_01` 固定输入可以复用：2375 个实际 Source 的原生预处理记录与 R10 封存输出逐字节一致，Target 的点数和 XYZ SHA256 也完全一致。此次复核仅执行 PCD 读取、原 XYZ 身份校验和 0.1 m `VoxelGrid`；原生 NDT `align` 调用数为 0。

## 冻结输入身份

| 项目 | 身份 |
|---|---|
| 数据集 | Koide `indoor_hard_01`，R10 冻结扫描 raw5–raw2379 |
| Source 数 | 2375 |
| 原始地图文件 | `/media/jian/HIKVISION/livox_ws/p63_r8_indoor_hard_holdout_data/r29_hard_tiles_exact_xyzi.pcd` |
| 原始地图点数 | 96324 |
| 原始地图 SHA256 | `50bba09fde204a55be8261e69ea539ae69c65dd44ec7ca1eeed5b066ff8da098` |
| PCL 0.1 m Target 点数 | 93223 |
| Target XYZ SHA256 | `166c4d2a4f5d24a4c79d4c90a8a275e0fe905fcc9650d559aabb775780e81654` |
| 复核后的 Source CSV SHA256 | `771c023f80a58d77c17eb6789e4220d7ed762578ee328b712c15806d399cea2e` |
| 复核后的 Target JSON SHA256 | `e86e8d3fa5bcb7c4d3713c0e894bb218d59b79a4ceeb3d86ba3b2eedc095bc46` |

R14 的 [`INPUT_FREEZE_LOCAL.json`](../../research/p63_r14_first_loss_heldout_validation/data/INPUT_FREEZE_LOCAL.json) 与 R10 `INPUT_FREEZE.json` 除地图文件路径换成当前 USB 挂载路径外完全相同。两份原生预处理输出与 USB 上 R10 封存的 `preflight/SOURCES.csv` 和 `preflight/TARGET.json` SHA256 完全一致。原 R10 文件、地图、数据集及 P63-R10 归档均未改写。

## 预检程序身份与执行

- 复用源码：`src/kernel_preflight_r10.cpp`，SHA256 `6be777cc600e02448f34ca2842eefb38a3b3b09312ab654273ba24fc425f3cf9`，与 R10 封存源码相同。
- 复用的已封存预检 ELF：`/media/jian/HIKVISION/livox_ws/p63_r10_imu_full_hard_data/kernel_preflight`，SHA256 `aa6fee25c8910277984af76861edc4c7341397977f4276456d8b5b2bb17a8d97`，与 R10 `BUILD.json` 相同。
- 运行命令：

  ```bash
  LD_LIBRARY_PATH=/opt/ros/noetic/lib:/media/jian/HIKVISION/livox_ws/p63_r3_hdl_data/catkin_ws/devel/lib \
    /media/jian/HIKVISION/livox_ws/p63_r10_imu_full_hard_data/kernel_preflight \
    /home/jian/Map-Matching-R14-first-loss-heldout/research/p63_r14_first_loss_heldout_validation/data/INPUT_FREEZE_LOCAL.json \
    /home/jian/Map-Matching-R14-first-loss-heldout/research/p63_r14_first_loss_heldout_validation/data/preflight_output
  ```

- 结果：`PASS 2375 native preprocessing identities; target=93223 align=0`。
- 完整日志：`research/p63_r14_first_loss_heldout_validation/logs/native_preprocess_preflight_retry.log`。

## 库路径问题与保留证据

本机继承的 `LD_LIBRARY_PATH` 将 Hikrobot MVS 的旧版 `libusb-1.0.so.0` 放在 PCL 运行库之前，导致 PCL 报 `undefined symbol: libusb_set_option`。系统版 libusb 含所需符号；用上面限定的 ROS 与 NDT_OMP 路径运行已封存预检 ELF 后成功。没有更换、复制或改写系统库及相机驱动库。源码编译通过；一次本地链接尝试也记录在 `logs/link_preflight.log`，因为同一环境问题未产出新 ELF，故实际复核使用 SHA 与 R10 完全相同的预检二进制。

## 严格对照的边界

空间互斥只能保证当前帧的验证 Source 点没有进入这一帧 NDT 优化。官方地图与参考轨迹的生成说明未给出地图是否剔除了该序列的原始扫描，因此本实验不能宣称验证 Source 与 Target 地图相互独立。heldout 残差只可解释为“当前 NDT 未使用的源点对既有地图的后验几何支持”，不能视为独立传感器或独立真值。
