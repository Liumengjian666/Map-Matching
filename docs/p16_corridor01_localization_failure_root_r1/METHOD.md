# 离线分解方法与复现

这里只分析冻结输出，不拟合新变换、不运行配准或ROS。

## 几何与时间

NDT输出`map_T_lidar`，按历史冻结外参转为IMU原点：

`map_T_imu = map_T_lidar * inverse(imu_T_lidar)`。

使用原`evaluate_frozen_v2.py`及其几何函数读取GT、外参、四元数、插值。
读取原posthoc_metrics内SE(3)变换，GT/外参SHA不匹配即停止。
各阶段使用自己的物理时间：scan-start、rollback_stamp或state_stamp。
只在原0–35秒GT覆盖区间评价；不用启动时缺失GT进行外推。

`frame_stages.csv`每个扫描保留七个阶段和误差、限幅、速度诊断。
窗口按NDT测量时间选择扫描；其中replayed节点实际约晚0.15秒，故它的
窗口指标不等同于high-rate实际时间窗口。`analysis.json`另行提供
published_high_rate，其3–35秒RMSE重现历史指标，误差<1e-9m。

## 有界反事实

唯一离线反事实是九个已反馈扫描的**同历史代数替换**。
对Weak-only当前raw应用同一历史previous_pose的0.5m/5deg限幅，得到
没有当前候选的nominal测量，与实际used比较。其余历史保持不变。

- 重建`v_blended = .4*v_rollback + .6*(p_used_IMU-p_previous_IMU)/dt`。
- 直接候选影响`.6*(p_used_IMU-p_nominal_IMU)/dt`。
- 下一NDT平移初值为`p_cur+R_cur*R_prev.T*(p_cur-p_prev)`。
- 不重求NDT，不重放IMU，不产生另一条完整反事实轨迹。

`NDT_CORRECTION_APPLIED`在速度混合前；故表中v_blended是从真实源码
公式重建的数值，不能称为直接记录的混合后速度。
Control与Weak-only后续点云/初值分化，不能当作同输入因果消融。

## 复现命令

在实际隔离副本执行（只需ROS Python依赖，不需要roscore/socket）：

```bash
source /opt/ros/noetic/setup.bash
cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws
python3 src/dog_prior_map_localization/scripts/p16_corridor_failure/test_analysis.py
python3 src/dog_prior_map_localization/scripts/p16_corridor_failure/analyze.py \
  --repo /home/jian/livox_ws/p10_corridor01_oosm_r1_ws \
  --out /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/docs/p16_corridor01_localization_failure_root_r1 \
  --gt '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/gt/corridor01_gt.txt' \
  --extrinsics '/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/calibration/corridor01_extrinsics.yaml'
```

脚本输出三份离线统计文件；不读取bag点云、不修改已有运行文件。
只使用已冻结样本和0–35秒范围，没有新在线候选或GT反馈。
