# ROSBAG-LARGE-FILE-CLEANUP

结果：SAFE_DUPLICATE_CLEANUP_COMPLETED。仅删除明确完整内容重复副本；没有删除唯一原始数据、裁剪数据或整数据集目录。

## 实际释放空间

- 清理前df可用：22,934,634,496 bytes（21.360 GiB）。
- 清理后df可用：91,418,374,144 bytes（85.140 GiB）。
- **df可用空间实际增加：63.780 GiB（68,483,739,648 bytes）**。
- 删除10个大文件：4份完整重复bag（其中3份同一loop2旧恢复副本）及6份重复PCD；涉及2组bag内容、4组PCD内容。
- 删除裁剪bag：0；删除完整数据集目录：0；原始唯一bag删除：0。
- df为前后净值，包含同期其他磁盘活动和审计产物，不把逻辑大小之和冒充实际释放量。完整原始df/du回执已保存。

## 盘点和判定

全目录同设备普通文件中，大于300MiB的文件共51个，清理后41个。GB/GiB两种门槛统计见SIZE_COUNTS_DETAILED.csv；扩展名、magic、mtime、st_blocks实占、nlink与初步来源见ROSBAG_LARGE_FILES.csv。

对相同大小组完整顺序计算SHA256；仅精确SHA匹配才作为重复证据。两份同大小loop3 result/orig bag的SHA不同，均保留。没有从名字推断数据完全相同。FASTLIVO2-dogmapping.zip、唯一历史结果、.bag.active及日志未获得可靠重复或完整再生合同，保留为UNCERTAIN。

引用范围：R6–R14实际研究工作区，以及稳定/轻量/论文工作区的源码、配置、脚本、manifest和报告。第二轮扫描记录4562条引用、286个文件，无超限跳过。10个拟删文件的精确路径及特定父目录令牌无匹配；指定原始bag有200条引用。原始引用与历史提及分开保留在REFERENCE_AUDIT.json。这是限定静态证据，不能证明任意外部脚本永远不会使用旧路径。

所有删除按A类证据：保留副本完整SHA及长度匹配、独立inode、进程占用检查通过、非符号链接/挂载点或跨设备路径、文件版本不变。删除前再次完整哈希目标与保留源，逐条INTENT/UNLINKED/VERIFIED_DELETED持久日志；没有通配符递归删除，也没有移到同盘回收站。没有根据B类可重建性推测删除唯一派生文件。

## 实际删除的bag

- `/home/jian/rosbag/loop2/stage1_raw_20260918/loop2_raw.bag`；保留 `/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag`。
- `/home/jian/rosbag/loop2/stage2b_raw_recovered_20260918/loop2_raw.bag`；保留 `/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag`。
- `/home/jian/rosbag/loop2/stage3a7_raw_recovery_20260920/loop2_raw.bag`；保留 `/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag`。
- `/home/jian/rosbag/realtime_map_match/realtime_map_match_20261007_143634.bag`；保留 `/home/jian/rosbag/FASTLIVO2-dogmapping/realtime_map_match/bag/realtime_map_match_20261007_143634.bag`。

其余6份PCD完整路径及保留副本见DELETED_FILES.csv。数据内容可按RECONSTRUCT_EXACT_COPIES.sh.txt中的明确cp命令重新生成；本轮未执行恢复。不保证恢复原inode/mtime，只保证可重建同SHA载荷。旧报告和路径记录没有改写。

## 清理前最大的20个文件（按实占排序）

| 完整路径 | 逻辑GiB | 实占GiB |
|---|---:|---:|
| `/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag` | 18.326 | 18.326 |
| `/home/jian/rosbag/loop2/stage1_raw_20260918/loop2_raw.bag` | 18.326 | 18.326 |
| `/home/jian/rosbag/loop2/stage2b_raw_recovered_20260918/loop2_raw.bag` | 18.326 | 18.326 |
| `/home/jian/rosbag/loop2/stage3a7_raw_recovery_20260920/loop2_raw.bag` | 18.326 | 18.326 |
| `/home/jian/rosbag/usb_dataset_backup_2026_07_30/fastlivo_3dgs_outputs/output/loop1_bag/loop1_2026-07-29-15-09-26.bag` | 13.599 | 13.599 |
| `/home/jian/rosbag/FASTLIVO2-dogmapping.zip` | 7.767 | 7.767 |
| `/home/jian/rosbag/FASTLIVO2-dogmapping/realtime_map_match/bag/realtime_map_match_20261007_143634.bag` | 6.189 | 6.189 |
| `/home/jian/rosbag/realtime_map_match/realtime_map_match_20261007_143634.bag` | 6.189 | 6.189 |
| `/home/jian/rosbag/FASTLIVO2-dogmapping/realtime_map_match/bag/realtime_map_match_2026-10-07-15-08-49.bag` | 5.772 | 5.772 |
| `/home/jian/rosbag/loop2/dcreg_loop2_baseline_20260917_01/result.bag.active` | 4.883 | 4.883 |
| `/home/jian/rosbag/public_datasets/IILABS3D/iilabs3d_dataset/benchmark/livox_mid-360/loop/livox_loop_2025-02-06-18-24-40.bag` | 3.322 | 3.322 |
| `/home/jian/rosbag/loop5/three_algorithm_rerun_20260905/fastlivo2_aligned_full_20260905/fastlivo2_aligned.bag` | 3.056 | 3.056 |
| `/home/jian/rosbag/fastlivo2_degen_tests/loop4_degen_lioonly_2026_08_07_18_09_26/pcd/loop4_degen_lioonly_2026_08_07_18_09_26/loop4_degen_lioonly_2026_08_07_18_09_26_2026_08_07_18_20_33_all_raw_points.pcd` | 2.673 | 2.673 |
| `/home/jian/rosbag/loop2/visual_loc_rviz_20260910/try14/launch.log` | 2.485 | 2.485 |
| `/home/jian/rosbag/usb_dataset_backup_2026_07_30/fastlivo_3dgs_outputs/output/loop1_bag/loop2_2026-07-29-15-24-39.bag` | 2.330 | 2.330 |
| `/home/jian/rosbag/loop3/fastlivo2_official_loop3_20260825_131103/fastlivo2_official_odom.bag` | 1.232 | 1.232 |
| `/home/jian/rosbag/loop3/fastlivo2location_backend_only_loop3_20260817_141325/fastlivo2location_backend_only_result.bag` | 1.004 | 1.004 |
| `/home/jian/rosbag/loop3/fastlivo2location_backend_only_loop3_20260817_141325/fastlivo2location_backend_only_result.orig.bag` | 1.004 | 1.004 |
| `/home/jian/rosbag/usb_dataset_backup_2026_07_30/usb_trash_recovered_files/output/loop1_bag/loop1_2026-07-29-15-04-28.bag` | 0.876 | 0.876 |
| `/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_raw_points_with_red_mapping_path.pcd` | 0.676 | 0.676 |

## 清理后最大的20个文件（按实占排序）

| 完整路径 | 逻辑GiB | 实占GiB |
|---|---:|---:|
| `/home/jian/rosbag/loop2/raw_restore_20260914_stage0/loop2_raw.bag` | 18.326 | 18.326 |
| `/home/jian/rosbag/usb_dataset_backup_2026_07_30/fastlivo_3dgs_outputs/output/loop1_bag/loop1_2026-07-29-15-09-26.bag` | 13.599 | 13.599 |
| `/home/jian/rosbag/FASTLIVO2-dogmapping.zip` | 7.767 | 7.767 |
| `/home/jian/rosbag/FASTLIVO2-dogmapping/realtime_map_match/bag/realtime_map_match_20261007_143634.bag` | 6.189 | 6.189 |
| `/home/jian/rosbag/FASTLIVO2-dogmapping/realtime_map_match/bag/realtime_map_match_2026-10-07-15-08-49.bag` | 5.772 | 5.772 |
| `/home/jian/rosbag/loop2/dcreg_loop2_baseline_20260917_01/result.bag.active` | 4.883 | 4.883 |
| `/home/jian/rosbag/public_datasets/IILABS3D/iilabs3d_dataset/benchmark/livox_mid-360/loop/livox_loop_2025-02-06-18-24-40.bag` | 3.322 | 3.322 |
| `/home/jian/rosbag/loop5/three_algorithm_rerun_20260905/fastlivo2_aligned_full_20260905/fastlivo2_aligned.bag` | 3.056 | 3.056 |
| `/home/jian/rosbag/fastlivo2_degen_tests/loop4_degen_lioonly_2026_08_07_18_09_26/pcd/loop4_degen_lioonly_2026_08_07_18_09_26/loop4_degen_lioonly_2026_08_07_18_09_26_2026_08_07_18_20_33_all_raw_points.pcd` | 2.673 | 2.673 |
| `/home/jian/rosbag/loop2/visual_loc_rviz_20260910/try14/launch.log` | 2.485 | 2.485 |
| `/home/jian/rosbag/usb_dataset_backup_2026_07_30/fastlivo_3dgs_outputs/output/loop1_bag/loop2_2026-07-29-15-24-39.bag` | 2.330 | 2.330 |
| `/home/jian/rosbag/loop3/fastlivo2_official_loop3_20260825_131103/fastlivo2_official_odom.bag` | 1.232 | 1.232 |
| `/home/jian/rosbag/loop3/fastlivo2location_backend_only_loop3_20260817_141325/fastlivo2location_backend_only_result.bag` | 1.004 | 1.004 |
| `/home/jian/rosbag/loop3/fastlivo2location_backend_only_loop3_20260817_141325/fastlivo2location_backend_only_result.orig.bag` | 1.004 | 1.004 |
| `/home/jian/rosbag/usb_dataset_backup_2026_07_30/usb_trash_recovered_files/output/loop1_bag/loop1_2026-07-29-15-04-28.bag` | 0.876 | 0.876 |
| `/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_raw_points_with_red_mapping_path.pcd` | 0.676 | 0.676 |
| `/home/jian/rosbag/loop2/loop2_map_match_experiment_2026_08_08_17_36_19/outputs/loop2/pcd/loop2_all_raw_points_with_red_mapping_path.pcd` | 0.676 | 0.676 |
| `/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_raw_points.pcd` | 0.675 | 0.675 |
| `/home/jian/rosbag/loop2/loop2_map_match_experiment_2026_08_08_17_36_19/outputs/loop2/pcd/loop2_all_raw_points.pcd` | 0.675 | 0.675 |
| `/home/jian/rosbag/fastlivo2_original_vs_improved_loop4_2026_08_07_21_25_03/loop1_raw_official_vs_improved_2026_08_07_22_26_58/official_original/pcd/official_loop1_all_raw_points.pcd` | 0.538 | 0.538 |

## 不确定而保留的文件

共33项。均未删除，具体原因见CLEANUP_PLAN.csv：

- `/home/jian/rosbag/FASTLIVO2-dogmapping.zip`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/FASTLIVO2-dogmapping/realtime_map_match/bag/realtime_map_match_2026-10-07-15-08-49.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/fastlivo2_degen_tests/loop4_degen_lioonly_2026_08_07_18_09_26/pcd/loop4_degen_lioonly_2026_08_07_18_09_26/loop4_degen_lioonly_2026_08_07_18_09_26_2026_08_07_18_20_33_all_raw_points.pcd`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/fastlivo2_degen_tests/loop4_degen_projection_2026_08_07_20_40_37/pcd/loop4_degen_projection_2026_08_07_20_40_37/loop4_degen_projection_2026_08_07_20_40_37_2026_08_07_20_51_41_all_raw_points.pcd`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/fastlivo2_degen_tests/loop4_degen_projection_soft_2026_08_07_20_55_03/pcd/loop4_degen_projection_soft_2026_08_07_20_55_03/loop4_degen_projection_soft_2026_08_07_20_55_03_2026_08_07_21_06_08_all_raw_points.pcd`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/fastlivo2_original_vs_improved_loop4_2026_08_07_21_25_03/loop1_raw_official_vs_improved_2026_08_07_22_26_58/official_original/pcd/official_loop1_all_raw_points.pcd`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/fastlivo2_original_vs_improved_loop4_2026_08_07_21_25_03/official_original/pcd/official_original_all_raw_points.pcd`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/dcreg_loop2_baseline_20260917_01/result.bag.active`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/deskew_faststate_topicready_full_1090s_20260913/state.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/loop2_map_match_experiment_2026_08_08_17_36_19/outputs/loop2/pcd/loop2_all_raw_points.pcd`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/loop2_map_match_experiment_2026_08_08_17_36_19/outputs/loop2/pcd/loop2_all_raw_points_with_red_mapping_path.pcd`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/loop2_map_match_experiment_2026_08_08_17_36_19/outputs/loop4/pcd/loop4_all_raw_points.pcd`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_raw_points.pcd`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_raw_points_with_red_mapping_path.pcd`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/stage1_1b_path_on_camera_on_20260918/result.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/stage1_1b_path_on_camera_on_repeat2_20260918/result.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/stage2d1_full2_20260919/result.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/stage2d1_full_20260919/result.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/stage2e_a_runA2_20260919/result.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/stage2e_a_runA_20260919/result.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/stage2e_a_runB_20260919/result.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/stage2e_a_runC_20260919/result.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/stage2e_start_full_runD_20260919/result.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/stage2e_start_full_runE_20260919/result.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop2/visual_loc_rviz_20260910/try14/launch.log`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop3/fastlivo2_official_loop3_20260825_131103/fastlivo2_official_odom.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/loop5/three_algorithm_rerun_20260905/fastlivo2_aligned_full_20260905/fastlivo2_aligned.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/p11_solid_r1_20261011/p11_solid_full_ancestry.bundle`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/public_datasets/IILABS3D/iilabs3d_dataset/benchmark/livox_mid-360/loop/livox_loop_2025-02-06-18-24-40.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/usb_dataset_backup_2026_07_30/fastlivo_3dgs_outputs/output/loop1_bag/loop1_2026-07-29-15-09-26.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/usb_dataset_backup_2026_07_30/fastlivo_3dgs_outputs/output/loop1_bag/loop2_2026-07-29-15-24-39.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/usb_dataset_backup_2026_07_30/fastlivo_3dgs_outputs/output/loop1_eval/pcd/loop1_fastlivo_2026_07_30_15_43_29_all_raw_points.pcd`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。
- `/home/jian/rosbag/usb_dataset_backup_2026_07_30/usb_trash_recovered_files/output/loop1_bag/loop1_2026-07-29-15-04-28.bag`：Unique/undetermined content; historical mention alone is not a preservation or deletion decision。

## 冻结科研依赖与工程保护

三个指定路径保持存在且完整SHA正确：

- loop2/raw_restore_20260914_stage0/loop2_raw.bag：d17bee3f8697dcb01cb4b9f6ea2483aa42a1a31ae277659c8a7869d9c69f13ae。
- loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd：016b7ae3ae494ad5d462a66d888a96b96dda5fe29d6058059aa2fa6bdf6b09ca。
- loop2/final_delivery_run_A_20260922/ndt_determinism.csv：e5f749cbbc7fc514326cc61626945f4ac6347722e4ad769e676984e1d214102a。

R13/R14全部归档校验项、HEAD和工作树状态前后相同。稳定工程HEAD、refs、reflog、index、状态和快照文件SHA相同。所有未删大文件版本签名保持不变。唯一GT/标定等小文件不在任何删除清单中，未读取内容或修改。未触碰R10保护备份、Git历史、源码或参数。

18项安全测试通过；首次测试兼容性失败、第一次引用扫描中断和工具选项错误均保留，不伪造首轮通过。独立本地反证审阅共3个有界周期，发现问题已修正；最后未发现新增实质问题。用户选择不调用外部模型。守卫不能保证防御任意非合作写入者在系统调用间隙修改文件，检查范围和残余竞态见SAFETY_REVIEW.md。

## R14与唯一NEXT

当前容量显著改善，足以重新考虑同一Wheel-float01的容量准入，但**没有因此认证精确下载/处理峰值、资产访问、坐标或时间合同**。唯一NEXT：将本清理回执交科研总控，由其决定是否批准恢复同一R14资产准入；本轮不自动下载或运行R14。

本轮审计产物位于独立目录，不修改任何研究分支，未形成本轮Git commit，也未push。没有可诚实提供的本轮分支上传命令；不得用旧研究分支上传无关修改。
