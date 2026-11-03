# P63-R14 第一阶段：当前运行身份与首次异常定位

**阶段结论：当前 RViz 截图的首次异常帧为 `INCONCLUSIVE`。** 截图与具体 rosbag/运行目录没有可核对的绑定，不能把 P63-R10 的 raw224/raw309 直接说成截图中的异常帧。

## 当前运行的最佳候选身份

本次可找到的最近一次论文算法试跑是 `/home/jian/rosbag/dog/paper_algorithm_trial_20261103_1154`。`RUN_INFO.md` 将源工作区记为 USB 上的论文工程，分支 `research/p9-r4-heldout-visual-evidence`，基准提交 `cd85a78abdeff49abaece571a9ba6ca9bda362d8`。实际运行使用了从该工作区复制并重新编译的本地副本。源工作区当时存在未提交修改，但没有封存当时的源码树哈希，因此基准提交不能单独代表实际运行源码；当前可核对到的运行二进制 SHA 如下。

| 项目 | 实际可核对身份 |
|---|---|
| 隔离运行工作区 | `/home/jian/rosbag/dog/paper_algorithm_trial_20261103_1154/workspace` |
| 算法 | 论文工作区的 split NDT 节点 + IMU/EKF；相机、视觉方向融合和退化状态机关闭 |
| Launch | `dog_prior_map_localization/dog_prior_map_localization_split.launch`；试跑时 `rviz:=false camera_enable:=false`，另起 RViz |
| NDT 配置 | resolution 0.8 m、source voxel 0.25 m、target voxel 0.15 m、max iterations 40、step size 0.08、epsilon 0.001；启用 0.5 m/5° step limit；deskew 关闭 |
| 输入 | `/home/jian/rosbag/dog/mapping_20261031_160829/rect_record_2026-10-31-16-08-29.bag`；SHA256 `d137b4442a2d6ff163672fd22b1ef35f52335e5c81354bc06b393c1801b46658` |
| 输入话题 | bag 有 `/livox/lidar`、`/livox/imu`、`/image_left/image_rect`、`/image_left/camera_info_rect`；该次只回放雷达和 IMU，图像未进入算法 |
| bag 范围 | 1121.128269 s，251,991 条消息；实际在约 338.6 s 停止，约 30.2% |
| 先验地图 | `/home/jian/rosbag/dog/rect_record_20261029_fastlivo2/rerun_20261102_182309/pcd/rerun_20261102_182309_all_downsampled_points.pcd`；478,545 点；SHA256 `cd3fe063af0f8f3848055dd8c5fbe049ca6d909aa73b70ab0fd85699a45a4721` |
| 参数文件 | `config/paper_run.yaml`；SHA256 `47ff81a8656653a431b8d0ea4ae2ad200529ef34352e80d69a495f64bdf43da0` |
| RViz 文件 | `rviz/dog_machine_dog_display.rviz`；SHA256 `97dc1addbaa8f6962eb5eabb1f73d5b07c4225ddbd4581aaee77d77e4a3dd9ac` |
| 运行二进制 | NDT `dog_prior_map_ndt_node_cpp`: `d20bb90e87ad2adb8ca13e9b265419d380b1ea2fd8dc77b9854a09d910c7cc6c`；EKF `dog_prior_map_ekf_node_cpp`: `5c5259b18537bea5259afe16c88f3c37ee8904df20ce468e6b957f5f539e4d8c` |
| ROS 时间 | ROS master `127.0.0.1:11346`，`/use_sim_time=true`，rosbag 使用 `/clock` 和 1× 回放 |

工作区及稳定版本核对：机器狗工作区 `/home/jian/dog_visual_loc_ws` 当前分支为 `feature/quadruped-localization-interface`，HEAD `41999ea`；该试跑没有从该稳定工作区启动。论文主工作区位于 USB，记录的源基准提交如上。该试跑也不是 P63-R10 的 HDL 运行。

## RViz 绿色轨迹和坐标系

该候选试跑所用 RViz 配置把绿色 Path 绑定到 `/dog_livo/path_corrected`，不是原始 NDT 路径；源码中该 Path 由 EKF 节点发布。参数把 `map_frame` 设为 `camera_init`、`base_frame` 设为 `livox_frame`。NDT 里程计输出 `/dog_livo/ndt_odom` 的语义是 `T_camera_init_livox`，其消息 `frame_id=camera_init`、`child_frame_id=livox_frame`。绿色校正轨迹的 Path `frame_id` 也设为 `camera_init`。RViz 固定坐标系同为 `camera_init`。

因此，在这一候选运行身份下，绿色轨迹代表 EKF 校正轨迹；RViz 配置中的红色当前帧点云是 `/dog_livo/points_aligned`。代码与配置的 frame 约定彼此一致。运行未录制 TF 或输出话题 bag，先验 PCD 本身也不携带 ROS `frame_id`，所以没有证据可以事后验证该次地图发布消息的真实 frame、运行时 TF 树，或 Oct29 地图与 Oct31 采集起点之间是否需要非单位变换。这里是未闭合项，不等同于已经发现坐标错误。

## 为什么不能给当前截图报出首次错误时间戳

目前没有与截图绑定的截图文件名、RViz session 记录或输出 bag。候选运行目录中只有 1 Hz 左右的 `paper_runtime.csv`（速率、NDT 计数、队列和历史长度），没有逐帧 NDT 初值、原始终值、EKF 位姿、残差或已保存的 `/dog_livo/path_corrected`。该试跑约有 3,364 次 NDT correction 记录，但这些累计计数不能恢复哪一帧第一次折返/绕圈。

bag 只包含 LiDAR、IMU 和图像/相机参数，没有独立 `/odom`、关节、足端接触或可靠物理轨迹真值；这次又没有处理图像。因此无法确认真实机器狗运动是否包含 RViz 里的折返，也不能据截图判断它是真实轨迹、frame/地图错位、绘图坐标问题还是估计器异常。

要对当前截图确定首次异常，至少缺少一项确定运行身份的材料（截图所对应的 bag/运行目录/启动配置），以及同一次运行按时间戳记录的 NDT 初值、原始终值、step-limit 后观测、EKF 更新前后位姿和 TF。现有候选日志不满足这些要求。没有基于此缺口重播或补造时间戳。

## P63-R9 至 R13 的独立历史反例

以下是 Koide `indoor_hard_01` 上 P63-R10 的历史失败，不是当前机器狗截图的归因。其原生 NDT 是 NDT_OMP，参考为地图锚定的离线 GICP+IMU 轨迹，不是独立动捕。坐标合同和参考插值合同已核验，但绝对真值独立性有限。

| raw | 历史意义 | IMU 预测误差 → NDT 终值误差（位置 m / 姿态 °） | NDT 与原生质量 |
|---:|---|---|---|
| 150 | 未失锁时的有益修正 | 0.098370 / 2.806436 → 0.017078 / 0.640271 | 保留为有益反例 |
| 224 | 首个孤立的显著姿态误差 | 0.104014 / 2.139838 → 0.085861 / 5.021638 | 第一处历史单帧高误差；不是持续失锁 |
| 309 | 首次持续失锁段开始 | 0.249361 / 2.107554 → 0.980368 / 14.017120 | converged；inlier 0.997835、native probability 2.692876、全点 NN fitness 0.017092 m²，质量外观良好但修正确有害 |
| 314 | 有益恢复修正 | 0.632389 / 10.366148 → 0.173088 / 3.335585 | 0.475812 m / 7.049293° 的大修正有效；不能用“大修正即拒绝” |
| 485 | 未失锁时姿态获益 | 0.081170 / 4.249620 → 0.101239 / 1.963711 | 姿态改善、位置略差；保留为多维折衷反例 |
| 515 | 第二次持续失锁开始 | 0.218922 / 2.874659 → 0.277434 / 5.513442 | 修正较小但有害；inlier 0.776306、probability 1.799226、fitness 0.191222 m² |

raw309 的高 Inlier/低 Fitness 无法认证全局位置正确；raw314 的质量分数与它非常接近而结果有益。raw515 的低质量分数能描述更差的局部拟合，却不能解释 raw309 的高质量有害修正。R10 没有计算或保存 NDT Hessian，因此不能声称历史 Hessian 指标已经识别这几帧。Gaussian/probability、Inlier 和 NN Fitness 都是目标局部拟合描述；R10 的 Inlier 还有地图原点 25 m 统计范围限制，不是参考误差估计。

HDL 源码的更新顺序为 `registration->align()`、读取 `getFinalTransformation()` 形成观测，随后调用 `ukf->correct(observation)`（R10 复用的 `pose_estimator.cpp` 约 188–208 行）。raw309 和 raw515 的 UKF 输出紧随 NDT；先验状态不是独立传感器。R13 全段风险排序只有描述性统计意义，不能构成安全准入阈值。当前唯一尚未验证、且与本任务相符的额外几何量是：在本次 NDT 优化前空间互斥留出的源点，对预测位姿和 NDT 终值分别计算地图残差；它是否有区分能力仍待实验。

## 第一阶段判定

- 当前 RViz 截图首次异常：**INCONCLUSIVE**，运行身份和逐帧输出未闭合。
- 历史 P63-R10 首个孤立高误差：raw224，stamp `1690354444797300442 ns`（相对 R10 raw4 anchor 15.133556493 s）。
- 历史 P63-R10 首次持续失锁：raw309，stamp `1690354450530800303 ns`（相对 anchor 20.867056354 s）。
- 当前没有证据把截图轨迹异常归因到 NDT、坐标系、时间同步或真实运动中的任何一项。
- 本报告后续阶段将使用 P63-R10 冻结公开输入作受控原型实验；这不能补认当前截图身份。

## 证据路径

- 本次候选试跑：`/home/jian/rosbag/dog/paper_algorithm_trial_20261103_1154/RUN_INFO.md`、`logs/paper_runtime.csv`、`config/paper_run.yaml`、`rviz/dog_machine_dog_display.rviz`。
- 当前候选运行源码/启动：`workspace/src/dog_prior_map_localization/launch/dog_prior_map_localization_split.launch`、`src/dog_prior_map_ndt_node.cpp`、`src/dog_prior_map_ekf_node_core.cpp`。
- 历史坐标和首次失败证据：`docs/p63_r10_hdl_imu_full_hard/INPUT_AND_FRAME_CONTRACT.md`、`REPORT.md`、`FULL_TRAJECTORY.csv`、`NATIVE_NDT.csv.gz`；raw309/raw314/raw515 因果反例在 `docs/p63_r11_harmful_ndt_observation/`；全段历史对照在 `docs/p63_r13_full_remaining_risk_ranking/`。
