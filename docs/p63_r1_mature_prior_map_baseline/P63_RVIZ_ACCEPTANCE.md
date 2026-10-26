# 一键展示入口与实际验收等级

实际失败Smoke自动启动了RViz（有进程和RSS记录），加载自带 `prior_localization.rviz`，FixedFrame=world。无需手工添加显示项：固定地图 `/lio/global_map`、当前世界云 `/lio/cloud_world`、原生 `/lio/odom`／`/lio/path`、独立可视化TF。原生pose是IMU位姿，TF桥无反馈，不将其擅称LiDAR原点。

**实际可视化验收未完成。** 计划在超过250输出时自动截取自有RViz窗口；实际95输出已失败，因此没有截图和稳定300–600帧展示。不能用launch解析或RViz进程存在替代视觉／定位验收。

## 已执行与仅配置检查

- **已执行**：`tools/run_smoke.py` 调用 `p63_loop2_frozen_map.launch`＋原始输入player，含RViz；唯一真实回放失败，如原始日志记录。
- **仅解析检查**：新 `p63_loop2_smoke_oneclick.launch` 把相同组件和自动输入player组合到一次roslaunch；`--dump-params`通过，Python wrapper生成成功；没有第二次回放。
- 该入口要求显式传入sensor_contract与 `allow_historical_identity_smoke:=true`；默认false拒绝历史占位外参。当前只有条件性合同，不是物理标定已通过的正式入口。

可直接运行的**条件性失败复现配置命令**如下，需全新output_dir；本轮未再次执行此组合。它不是已经通过的定位验收命令，也不自动授权下一轮调试：

```bash
bash --noprofile --norc -c 'source /opt/ros/noetic/setup.bash && source /home/jian/livox_ws/p63_r1_super_lio_data/Super-LIO/devel/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-} && ROS_MASTER_URI=http://127.0.0.1:11631 ROS_IP=127.0.0.1 roslaunch p63_super_lio_baseline p63_loop2_smoke_oneclick.launch sensor_contract:=/home/jian/livox_ws/p63_r1_super_lio_baseline_ws/docs/p63_r1_mature_prior_map_baseline/config/SENSOR_CONTRACT.json output_dir:=/home/jian/livox_ws/p63_r1_super_lio_data/manual_repro_01 allow_historical_identity_smoke:=true rviz:=true'
```

入口使用已有原bag的有限首400扫描，不依赖新裁剪bag；不发布GT。需要本机图形会话可用和≥20GiB剩余。不得复用loop2_smoke_01或已有轨迹目录。sensor_contract绑定原map和实际observer二进制SHA，错误身份会在原程序启动前拒绝。

原作者未启用的TF由只读桥生成 world→p63_imu→livox_frame；不是重复发布旧P62的校正状态，不更改估计器。没有在稳定机器狗路径增加launch或参数。
