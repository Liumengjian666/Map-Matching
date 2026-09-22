# SuperLoc initialization-config matrix

审计日期：2026-09-23。来源目录：`/media/jian/HIKVISION/paper rosbag/SuperLoc`。

| sequence | init 文件 | calibration | map/GT/raw | YAML keys | 结论 |
|---|---|---|---|---|---|
| Corridor01 | `Corridor01/initial_pose/corridor01.yaml` | extrinsics + intrinsics | map、GT、raw 均在本机 | `extrinsicRotation_world_darpa`、`extrinsicTranslation_world_darpa` | 官方先验搜索种子；不是已证明的 `T_map_lidar` |
| Corridor02 | `Corridor02/initial_pose/corridor02.yaml` | extrinsics + intrinsics | init/calibration 在本机；本阶段未运行其 bag | 同上，另有 4 组注释掉的 4x4 候选矩阵 | 只完成 archive 对照 |
| Floor01 | 未找到 | 未找到 | 未找到 | — | 本机 archive 缺失 |
| Floor02 | 未找到 | 未找到 | 未找到 | — | 本机 archive 缺失 |
| Cave01 | 未找到 | 未找到 | 未找到 | — | 本机 archive 缺失 |
| Cave02 | 未找到 | 未找到 | 未找到 | — | 本机 archive 缺失 |
| Cave03 | 未找到 | 未找到 | 未找到 | — | 本机 archive 缺失 |
| Cave04 | 未找到 | 未找到 | 未找到 | — | 本机 archive 缺失 |

## Key 对照

Corridor01 的有效 YAML key 只有：

```text
extrinsicRotation_world_darpa: 3x3 OpenCV matrix
extrinsicTranslation_world_darpa: 3x1 OpenCV matrix
```

Corridor02 的有效 key 相同；文件中还保留了 4 组注释的 4x4 矩阵候选。两份文件都没有 `init_x`、`init_y`、`init_z`、`roll`、`pitch`、`yaw`、`world`、`map`、`laser`、`lidar` 或 `imu` 字段。

## 官方源码交叉检查

用户给出的旧路径 `/media/jian/HIKVISION/comparison algorithm/SuperLoc` 不存在；本机实际官方代码目录是：

```text
/media/jian/HIKVISION/comparison algorithm/SuperLoc_SuperOdom
branch: ros2
HEAD: f10e65cd50007767b22e4c401689665e20d827d6
tag: v1.0
```

对该仓库全部 commit 的 `world_darpa`、`extrinsicRotation_world_darpa`、`extrinsicTranslation_world_darpa` 搜索均无命中。源码真正读取的是 `laser_mapping_node.localization_mode`、`read_pose_file` 和 `init_x/y/z/roll/pitch/yaw`，并直接形成 `T_w_lidar`。因此 Corridor YAML 与当前 SuperOdom 的 `start_pose.txt` 接口不是同一已证实格式。

## 分类

```text
CORRIDOR01_INIT_CLASSIFICATION = OTHER_OFFICIAL_FRAME_TRANSFORM_UNRESOLVED
```

证据：`world_darpa` 在官方初始化 archive 中出现，但当前官方源码不读取该 key；本地数据没有 TF，也没有官方给出的 map↔DARPA 矩阵方向说明。P2B 使用它作为搜索种子，不把它伪装成 GT 或直接初始位姿。
