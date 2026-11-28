# 执行及复跑

以下是实跑命令的可复制写法。第二次运行请使用新的空输出目录。数据与先前研究二进制按哈希冻结。

    cd /home/jian/p64_r3_q8_ws && source /opt/ros/noetic/setup.bash && OPENBLAS_NUM_THREADS=1 /usr/bin/python3 docs/p64_r3_q8_public_degenerate_fine_geometry/prepare.py /home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag /home/jian/geode/Shield_tunnel1_gamma/inspection/scan_ledger.csv /home/jian/geode/Shield_tunnel1_gamma/inspection/imu_records.csv run/inputs
    cd /home/jian/p64_r3_q8_ws && bash docs/p64_r3_q8_public_degenerate_fine_geometry/build_curve.sh run/build
    cd /home/jian/p64_r3_q8_ws && bash docs/p64_r3_q8_public_degenerate_fine_geometry/baseline.sh run /home/jian/p64_r3_q6_ws/run/local_build/v1
    cd /home/jian/p64_r3_q8_ws && source /opt/ros/noetic/setup.bash && OPENBLAS_NUM_THREADS=1 /usr/bin/python3 docs/p64_r3_q8_public_degenerate_fine_geometry/match.py run docs/p64_r3_q8_public_degenerate_fine_geometry run/build/native_curve
    cd /home/jian/p64_r3_q8_ws && source /opt/ros/noetic/setup.bash && OPENBLAS_NUM_THREADS=1 /usr/bin/python3 docs/p64_r3_q8_public_degenerate_fine_geometry/assess.py run docs/p64_r3_q8_public_degenerate_fine_geometry run/access/geode_ref_endpoint_0
    cd /home/jian/p64_r3_q8_ws && source /opt/ros/noetic/setup.bash && OPENBLAS_NUM_THREADS=1 /usr/bin/python3 docs/p64_r3_q8_public_degenerate_fine_geometry/followup.py run docs/p64_r3_q8_public_degenerate_fine_geometry
    cd /home/jian/p64_r3_q8_ws && pcl_viewer docs/p64_r3_q8_public_degenerate_fine_geometry/visuals/G1_prior.pcd docs/p64_r3_q8_public_degenerate_fine_geometry/visuals/G1_direction_kept.pcd docs/p64_r3_q8_public_degenerate_fine_geometry/visuals/G1_direction_removed.pcd docs/p64_r3_q8_public_degenerate_fine_geometry/visuals/G1_weak_axis.pcd
    cd /home/jian/p64_r3_q8_ws && pcl_viewer docs/p64_r3_q8_public_degenerate_fine_geometry/visuals/G1_ndt_cells.pcd docs/p64_r3_q8_public_degenerate_fine_geometry/visuals/G1_fine_scan.pcd docs/p64_r3_q8_public_degenerate_fine_geometry/visuals/G1_fine_map_correspondences.pcd

运行 [reproduce.sh](reproduce.sh) 可开始新输出目录复跑。Q8 不提供新的 ROS1 launch。
