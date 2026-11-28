# 运行命令

以下均可在 /home/jian/p64_r3_q9_ws 独立执行。再次复跑更换输出目录。请参考 [下载回执](PUBLIC_ASSET_DOWNLOAD_RECEIPT.md) 校验官方地图等资产。

    cd /home/jian/p64_r3_q9_ws && python3 docs/p64_r3_q9_public_benchmark_fine_geometry/asset_probe.py run/drive_probe_repeat.csv run/probe_repeat
    cd /home/jian/p64_r3_q9_ws && python3 docs/p64_r3_q9_public_benchmark_fine_geometry/range_fetch.py run/assets/Corridor01 300388352
    cd /home/jian/p64_r3_q9_ws && source /opt/ros/noetic/setup.bash && python3 docs/p64_r3_q9_public_benchmark_fine_geometry/bag_prefix.py run/assets/Corridor01 run/corridor_prefix_repeat.bag 95 --wait
    cd /home/jian/p64_r3_q9_ws && bash docs/p64_r3_q9_public_benchmark_fine_geometry/build_hdl.sh run/hdl_upstream
    cd /home/jian/p64_r3_q9_ws && bash docs/p64_r3_q9_public_benchmark_fine_geometry/replay_corridor.sh run/corridor_prefix_95s.bag run/assets/Corridor01/static run/velodyne/velodyne_pointcloud/params/VLP16db.yaml run/corridor_replay_repeat run/corridor_docs_repeat run/hdl_upstream
    cd /home/jian/p64_r3_q9_ws && source /opt/ros/noetic/setup.bash && OPENBLAS_NUM_THREADS=1 /usr/bin/python3 docs/p64_r3_q9_public_benchmark_fine_geometry/paired.py run/pairs_docs_repeat run/pairs_processing_repeat
    cd /home/jian/p64_r3_q9_ws && source /opt/ros/noetic/setup.bash && OPENBLAS_NUM_THREADS=1 /usr/bin/python3 docs/p64_r3_q9_public_benchmark_fine_geometry/evaluate_pairs.py run/pairs_docs_repeat run/pairs_processing_repeat
    cd /home/jian/p64_r3_q9_ws && OPENBLAS_NUM_THREADS=1 /usr/bin/python3 docs/p64_r3_q9_public_benchmark_fine_geometry/plot_patches.py run/pairs_docs_repeat
    cd /home/jian/p64_r3_q9_ws && pcl_viewer docs/p64_r3_q9_public_benchmark_fine_geometry/visuals/W2_local_prior.pcd docs/p64_r3_q9_public_benchmark_fine_geometry/visuals/W2_source_selected_world.pcd docs/p64_r3_q9_public_benchmark_fine_geometry/visuals/W2_patch_target_world.pcd
    cd /home/jian/p64_r3_q9_ws && pcl_viewer docs/p64_r3_q9_public_benchmark_fine_geometry/visuals/Corridor01_official_prior_roi.pcd docs/p64_r3_q9_public_benchmark_fine_geometry/visuals/Corridor01_reconstruction_roi.pcd docs/p64_r3_q9_public_benchmark_fine_geometry/visuals/Corridor01_UKF_path.pcd

正式配对报告的扫描、初值、地图和匹配输出先哈希封存，此后离线读取 IILABS OptiTrack 参考。ROS1 原版 HDL 定位节点的源码固定为 de2dc7769aa2412876d05ae6161408d3295b6a3c。VLP16db 角度与时序参照 ros-drivers/velodyne 的 29abd0e1361cb7f5eda451d2b51c35eeca45e0d5。研究基线依赖 Q6 已有二进制。原版节点使用包自带 NDT_OMP 注册选项，隔离主站、标准 ROS 消息和 Q9 封存输入。适配的 Velodyne 解码器并非 ROS 节点的原封不动执行。

研究定位 270 帧已结束后，第一次诊断汇总因 NumPy 整数的 JSON 序列化退出。修复汇总后复用已封存研究输出，并执行原版节点，未重跑研究轨迹或改参数。实际可分开复现如下。

    cd /home/jian/p64_r3_q9_ws && source /opt/ros/noetic/setup.bash && OPENBLAS_NUM_THREADS=1 /usr/bin/python3 docs/p64_r3_q9_public_benchmark_fine_geometry/corridor_results.py run/corridor_replay/input run/corridor_replay/research run/corridor_diagnostics_repeat
    cd /home/jian/p64_r3_q9_ws && bash docs/p64_r3_q9_public_benchmark_fine_geometry/replay_hdl.sh run/hdl_upstream run/corridor_replay/input run/hdl_repeat 11729

另外以同包前两帧独立检查实际载入的原版库，读取 /proc 中映射路径并收到同戳位姿，完整循环结果保持原值。最终用于复跑的是含该校验的脚本。
