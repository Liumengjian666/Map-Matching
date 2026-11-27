# 实际命令

在 /home/jian/p64_r3_q7_ws 执行。数据及二进制依赖在归档中的对应路径。每次复跑使用新的输出目录。

    bash /home/jian/p64_r3_q7_ws/docs/p64_r3_q7_directional_fine_geometry_coupling/build.sh /home/jian/p64_r3_q7_ws/run/rebuild
    OPENBLAS_NUM_THREADS=1 python3 /home/jian/p64_r3_q7_ws/docs/p64_r3_q7_directional_fine_geometry_coupling/directional_q7.py /home/jian/p64_r3_q7_ws/run/direction_repeat
    OPENBLAS_NUM_THREADS=1 python3 /home/jian/p64_r3_q7_ws/docs/p64_r3_q7_directional_fine_geometry_coupling/run_continuous.py /home/jian/p64_r3_q7_ws/run/causal_repeat /home/jian/p64_r3_q7_ws/run/rebuild/q7
    bash /home/jian/p64_r3_q7_ws/docs/p64_r3_q7_directional_fine_geometry_coupling/run_dcreg.sh /home/jian/p64_r3_q7_ws/run/rebuild/q7 /home/jian/p64_r3_q7_ws/run/causal_repeat/DOG_natural_input /home/jian/p64_r3_q7_ws/run/dc_repeat
    OPENBLAS_NUM_THREADS=1 python3 /home/jian/p64_r3_q7_ws/docs/p64_r3_q7_directional_fine_geometry_coupling/detectors.py /home/jian/p64_r3_q7_ws/run/causal /home/jian/p64_r3_q7_ws/run/detectors_repeat
    OPENBLAS_NUM_THREADS=1 python3 /home/jian/p64_r3_q7_ws/docs/p64_r3_q7_directional_fine_geometry_coupling/fine_q7.py /home/jian/p64_r3_q7_ws/run/fine_repeat /home/jian/p64_r3_q7_ws/run/direction
    OPENBLAS_NUM_THREADS=1 python3 /home/jian/p64_r3_q7_ws/docs/p64_r3_q7_directional_fine_geometry_coupling/repeat.py /home/jian/p64_r3_q7_ws/run/contour_repeat.csv
    source /opt/ros/noetic/setup.bash && OPENBLAS_NUM_THREADS=1 python3 /home/jian/p64_r3_q7_ws/docs/p64_r3_q7_directional_fine_geometry_coupling/archive.py /home/jian/p64_r3_q7_ws/run /home/jian/p64_r3_q7_ws/run/review_results
    pcl_viewer /home/jian/p64_r3_q7_ws/docs/p64_r3_q7_directional_fine_geometry_coupling/W2_selected.pcd /home/jian/p64_r3_q7_ws/docs/p64_r3_q7_directional_fine_geometry_coupling/W2_removed.pcd
    pcl_viewer /home/jian/p64_r3_q7_ws/docs/p64_r3_q7_directional_fine_geometry_coupling/W2_contour_raw.pcd /home/jian/p64_r3_q7_ws/docs/p64_r3_q7_directional_fine_geometry_coupling/W2_fine_correspondences.pcd

最早的基本连续对照用 run/build/q7。增加自然 DCReg 对照后，最终代码使用 run/build_verified/q7。DCReg 作者实现固定提交 8ce8451b15491a4bbe17cf85ab02a8bed6696861。现成 Q6 原生方向二进制、原始扫描和 Q5 X-ICP/DCReg 核心测试工具均只读使用。系统依赖 ROS1 Noetic、PCL、Eigen 与 SciPy。输出封存之前不读取测试参考。操作范围为离线实际定位，本轮无新 roslaunch。
