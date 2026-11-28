# R1 实验命令

从 c103ad754729e1c1bf92741fd892d24f69b89279 建立独立研究分支，保留历史缓存。已执行官方公开 HTTP Range 续传。初始 8/10 workers，之后调整到 20/12，再到 20/32，变更时保留所有已取得块及符合 Content-Range 的在途块。尚未到齐时不执行正式定位。

```bash
cd /home/jian/p64_r3_q10_ws && python3 docs/p64_r3_q10_public_mechanism_validation/fetch_ranges.py run/assets map --workers 20
cd /home/jian/p64_r3_q10_ws && python3 docs/p64_r3_q10_public_mechanism_validation/fetch_ranges.py run/assets bag --workers 32
cd /home/jian/p64_r3_q10_ws/run/assets && printf '%s\n' '98373b52f207b9ace3914792b5604a6642f89441190cabca6acbaf1d17249280  exp18_corridor_lower_gallery_2.bag' '46575d3eaf3169a24e24ba0013cc48610125ac842d6745e5e8156c6aecc59756  sheldonian_full.e57' | sha256sum --check
cd /home/jian/p64_r3_q10_ws && bash docs/p64_r3_q10_a_r1_complete_hilti/build_equivalence.sh /home/jian/p64_r3_q9_ws/run/hdl_upstream run/r1/equiv_build_repeat
cd /home/jian/p64_r3_q10_ws && bash docs/p64_r3_q10_a_r1_complete_hilti/build_secondary_e57.sh run/r1/pye57-src run/r1/pye57_rebuild /home/jian/.local/lib/python3.8/site-packages/torch/include
cd /home/jian/p64_r3_q10_ws && bash docs/p64_r3_q10_a_r1_complete_hilti/replay_pair.sh run/r1/equiv_build run/input_prefix10 run/r1/prefix_equiv_repeat 11740 PARTIAL_MAP_ENGINEERING_PREFLIGHT
cd /home/jian/p64_r3_q10_ws && OPENBLAS_NUM_THREADS=1 /usr/bin/python3 docs/p64_r3_q10_a_r1_complete_hilti/validate_patch_tools.py /home/jian/p64_r3_q9_ws/run/pairs docs/p64_r3_q10_a_r1_complete_hilti run/r1
cd /home/jian/p64_r3_q10_ws && /usr/bin/python3 docs/p64_r3_q10_a_r1_complete_hilti/reference_checks.py run/assets/exp18_corridor_lower_gallery_2_imu.txt run/assets/exp18_corridor_lower_gallery_2_imu_3dof.txt run/eval_preflight_final/trajectory_imu.txt docs/p64_r3_q10_a_r1_complete_hilti
```

完整数据流程分别验证资源身份，执行完整解码。第三条命令先检查冻结首帧重合，若不通过则只运行一次预注册隔离标定。若通过，再依次运行 10 帧、100 帧和完整序列。

```bash
cd /home/jian/p64_r3_q10_ws && bash docs/p64_r3_q10_a_r1_complete_hilti/full_map_qualification.sh
cd /home/jian/p64_r3_q10_ws && bash docs/p64_r3_q10_a_r1_complete_hilti/full_bag_replay.sh
cd /home/jian/p64_r3_q10_ws && bash docs/p64_r3_q10_a_r1_complete_hilti/run_qualified_baselines.sh
```

默认完整流程在各固定产物目录已存在时拒绝覆盖。重复运行要使用新的工件目录。界面命令和正确输入见 [ROS1 指南](RVIZ_REPLAY_GUIDE.md)。预检原版节点源码 de2dc7769aa2412876d05ae6161408d3295b6a3c，官方 Hilti 评价源码 e4aacf7c111ab832a0bdffe3f069255454068540，地图主读取器 e74d8b0b5ca69a135fb96c23ce9fff2f390dc196，第二读取器 pye57 v0.4.15（5ea1c0dece0bbd22fed6c54baeea6b31fb98a802）使用固定 libE57Format 1914b8ea972251d3bb49a33828497dde683205d9。地图/点云和录包衍生产物来自 Hilti Research 和 Zhang 等及其 CC BY-NC-SA 3.0 数据发布。
