# 官方坐标轴语义证据

## 结论先行

Newer College 2020 原始论文和官方 `frontier_description` 仓库都明确把 Ground Truth 描述为 `base` 帧位姿；官方平台资料又说明 Base 原点与左相机光学中心重合，但 Base 使用机器人轴（X forward、Y left、Z up），而 `RS_C1` 被称为左相机 optical frame。当前官网 GT 页面则把 stereo GT 标为 `RS_C1`。这些官方文本对“同一个发布 CSV 的四元数是否已采用 optical 轴”存在命名/版本表述张力。

因此：官方文献对 Base 解释有直接且强的支持；几何也在预选的三个时间点一致支持 Base 轴。此次范围内未找到生成 `registered_poses.csv` 的 exporter 代码，故不把几何结果升级为该 CSV 序列化语义的源码级认证。

## 官方来源

1. 原始数据集论文 [The Newer College Dataset: Handheld LiDAR, Inertial and Vision with Ground Truth, arXiv:2003.05691v2](https://arxiv.org/html/2003.05691v2)：Sec. III 将 `RS_C1` 称为被视为 base frame 的左相机坐标系；Sec. V 说明 GT poses 相对于 base frame，且该 base 位于左相机中心。论文还说明 GT 核心通过把 Ouster 扫描 ICP 配准到 Leica prior map 生成，再转换/外推到相机位姿。
2. 官方 [Frontier Description README（`newer-college-dataset` 分支）](https://github.com/ori-drs/frontier_description/blob/newer-college-dataset/README.md)：明确写明 `base` 与左相机 optical frame 共点，`base` 采用 X forward、Y left、Z up，并称 Ground Truth expressed in this frame。
3. 当前官方 [Ground Truth 页面](https://ori-drs.github.io/newer-college-dataset/ground-truth/)：称 stereo setup 的 poses expressed in `RS_C1` frame；当前官方 [Stereo-Camera Platform 页面](https://ori-drs.github.io/newer-college-dataset/stereo-cam/platform-stereo/) 将 `RS_C1` 称为左相机 optical frame，并紧接着说明 Base 与该 frame 共点、但采用机器人轴约定。
4. 官方平台 URDF 链接位于上述 README 指定的 `newer-college-dataset` 分支。其 README 对 GT/Base 语义有直接说明；本轮未发现发布该 CSV 的具体 exporter/writer 实现，不能用 URDF 单独证明每个 CSV 四元数写出前是否做过轴转换。

## 证据边界

- “共点”只证明原点相同，不证明姿态轴相同。本轮模型令 `T_C_B` 的平移为零，但保留 Base 到 optical 的固定轴旋转。
- 当前官网对 `RS_C1` 的标签不能简单等同为 optical 姿态已经写入 CSV，因为原论文与官方 URDF README 明确把 GT 称为 Base frame；反过来，也不能忽略当前官网页面的 RS_C1 表述。
- 三帧点云几何支持 Base 轴解释，但几何一致性本身不是文件格式/生成代码的直接语义证明。
- 因此正式状态采用 `BASE_FRAME_GEOMETRICALLY_SUPPORTED`：官方文档强烈支持 Base，三帧几何重复支持 Base；CSV exporter 级别的最后一环仍未获得源码证据。
