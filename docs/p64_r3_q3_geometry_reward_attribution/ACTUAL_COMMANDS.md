# 执行与复现

仓库中的阶段脚本使用 /home/jian/p64_r3_q1_ws 和 /home/jian/p64_r3_q2_ws 的原始归档与 IILABS 先验资产。评分不需要重新跑 ROS bag，因果窗口使用历史原生运行输入和原有状态传播单文件程序。可从归档目录逐个执行复现脚本的命令，重跑需新目录。

~~~bash
bash /home/jian/p64_r3_q3_ws/docs/p64_r3_q3_geometry_reward_attribution/reproduce.sh /home/jian/p64_r3_q3_ws/run/q3_repro_new
pcl_viewer /home/jian/p64_r3_q3_ws/docs/p64_r3_q3_geometry_reward_attribution/point_clouds/H307_delta.pcd /home/jian/p64_r3_q3_ws/docs/p64_r3_q3_geometry_reward_attribution/point_clouds/H307_max_diff.pcd
pcl_viewer /home/jian/p64_r3_q3_ws/docs/p64_r3_q3_geometry_reward_attribution/point_clouds/H515_support_comparison.pcd
pcl_viewer /home/jian/p64_r3_q3_ws/docs/p64_r3_q3_geometry_reward_attribution/point_clouds/C3_offset_10cm.pcd
pcl_viewer /home/jian/p64_r3_q3_ws/docs/p64_r3_q3_geometry_reward_attribution/point_clouds/W2_32_candidate_overlay.pcd
pcl_viewer /home/jian/p64_r3_q3_ws/docs/p64_r3_q3_geometry_reward_attribution/point_clouds/B25_candidate_overlay.pcd
~~~

点云奖励：绿为正项，红为负项，灰为近中性。支持比较：绿为空间掩码，红为差异掩码，蓝为交集。尺度控制：绿为双侧支持，红为检测标记，灰为不支持。Mid-360 叠加：灰为图、绿为原生首选、红为弱分离终值；选点点云绿为保留，红为未保留但双支持，灰为未保留。
