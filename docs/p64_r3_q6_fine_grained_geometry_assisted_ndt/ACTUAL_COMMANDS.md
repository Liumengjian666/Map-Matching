# 复现入口

在 /home/jian/p64_r3_q6_ws 执行。编译依赖沿用 Q5 冻结文件，运行写入新目录。下列 Linux 命令可直接复制。

    bash /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/build.sh /home/jian/p64_r3_q6_ws/run/rebuild_q6
    bash /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/run_regression.sh /home/jian/p64_r3_q6_ws/run/regression_repeat /home/jian/p64_r3_q6_ws/run/rebuild_q6/v1
    bash /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/build_directional.sh /home/jian/p64_r3_q6_ws/run/rebuild_directional
    OPENBLAS_NUM_THREADS=1 python3 /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/directional.py /home/jian/p64_r3_q6_ws/run/directional_repeat /home/jian/p64_r3_q6_ws/run/rebuild_directional/v1
    OPENBLAS_NUM_THREADS=1 python3 /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/study.py fixed /home/jian/p64_r3_q6_ws/run/frozen_repeat
    OPENBLAS_NUM_THREADS=1 python3 /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/study.py submaps /home/jian/p64_r3_q6_ws/run/submaps_repeat
    OPENBLAS_NUM_THREADS=1 python3 /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/m3c2_multiscale.py /home/jian/p64_r3_q6_ws/run/m3_scale_repeat
    OPENBLAS_NUM_THREADS=1 python3 /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/refinement.py /home/jian/p64_r3_q6_ws/run/refinement_repeat /home/jian/p64_r3_q6_ws/run/rebuild_q6/v1 /home/jian/p64_r3_q6_ws/run/frozen_match
    bash /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/run_v1.sh /home/jian/p64_r3_q6_ws/run/rebuild_q6/v1 /home/jian/p64_r3_q3_ws/run/W2 /home/jian/p64_r3_q6_ws/run/modea_repeat LOCAL_SHADOW 32 25 S3
    bash /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/run_combined.sh /home/jian/p64_r3_q6_ws/run/combination_repeat /home/jian/p64_r3_q6_ws/run/rebuild_q6/v1 /home/jian/p64_r3_q6_ws/run/dog_deskew_final
    bash /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/build_deskew.sh /home/jian/p64_r3_q6_ws/run/dog_deskew_repeat
    env -u LD_LIBRARY_PATH LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 LD_LIBRARY_PATH=/opt/ros/noetic/lib /home/jian/p64_r3_q6_ws/run/dog_deskew /home/jian/rosbag/dog/mapping_20261031_160829/rect_record_2026-10-31-16-08-29.bag 1793434704688463600 1793434709588238800 /home/jian/p64_r3_q5_ws/run/dog_input/anchor.tsv /home/jian/p64_r3_q6_ws/run/dog_deskew_repeat_input
    pcl_viewer /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/DOG30_directional_retained.pcd /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/DOG30_directional_removed.pcd
    pcl_viewer /home/jian/p64_r3_q6_ws/docs/p64_r3_q6_fine_grained_geometry_assisted_ndt/W2_fine_correspondences.pcd

原版 LF-GICP 工件在 run/LF_GICP。上游从已记 SHA 用 lf_gicp/cpp/build.sh all 构建，python3 -m pytest -q tests 回归，随后 lf_reference.py 做真实静态对照。archive.py 在输出封存后用 ROS Noetic Python 读取参考。逐点完整记录在 run/directional_study，发布 CSV 带明确抽样步长，同时发布示例点云与真实匹配对应。 [冻结协议](frozen_protocol.md) · [方向协议](directional_protocol.md)
