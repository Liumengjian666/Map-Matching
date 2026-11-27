# 复现与打开点云

从独立工作区运行，给出尚不存在的新目录。默认使用 R3-Q1 历史逐点缓存、冻结先验地图、Q2 的 Protocol B 封存短窗和同一 NDT_OMP 源码。环境 Linux、PCL 1.10，固定 OpenBLAS 单线程。Q1 资格检查二进制复编译时只把输入帧数从 12 改为 3。几何诊断、原生审计、筛选掩码、原始评分和 Mid-360 终值产物均可追溯。运行时保留的源点、地图及二进制缓存通过 artifact_sha256.csv 索引。

~~~bash
bash /home/jian/p64_r3_q2_ws/docs/p64_r3_q2_multi_hypothesis_geometry/reproduce.sh /home/jian/p64_r3_q2_ws/run/new_reproduction
pcl_viewer /home/jian/p64_r3_q2_ws/docs/p64_r3_q2_multi_hypothesis_geometry/point_clouds/H307_retained_removed.pcd
pcl_viewer /home/jian/p64_r3_q2_ws/docs/p64_r3_q2_multi_hypothesis_geometry/point_clouds/H515_original_density.pcd /home/jian/p64_r3_q2_ws/docs/p64_r3_q2_multi_hypothesis_geometry/point_clouds/H515_voxel10_density.pcd
pcl_viewer /home/jian/p64_r3_q2_ws/docs/p64_r3_q2_multi_hypothesis_geometry/point_clouds/H309_M6.pcd
pcl_viewer /home/jian/p64_r3_q2_ws/docs/p64_r3_q2_multi_hypothesis_geometry/point_clouds/B25_nominal_and_weak_terminal.pcd
~~~

保留/删除点云绿色为保留，红色为删除。密度对照图红色为几何区别，绿色为双侧支持，灰色为未知。B25 图灰色为地图、绿色为原生首选、红色为弱终值。共享连续权重使用同点号的压缩数组和实际散点图。Mid-360 强竞争选点图记 NOT_RUN；现有 B25 图展示实际终值而非选点结果。

正式的第一次几何复核用 float32 图坐标复算 O164 时，1/9053 点与原 Q1 float64 首交比较不同，触发严格输入一致性检查。随后改用与 Q1 相同的 float64 转换入口，基线分组逐点一致。提前启动的共享跑因缺少未写出的 O164 权重停止。两次失败均在读取参考或写正式结果前。无门限或原生评分定义变化。
