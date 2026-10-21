# R11授权对象保护重试

实际结果：PRESERVATION_PASS，实际进程退出0。旧部分备份未覆盖；R10原ARCHIVE_ASSET_DRIFT_DETECTED和此前PRESERVATION_BLOCKED均保留。

源 `/home/jian/livox_ws/dog_visual_loc_ws`，HEAD41999ea700c66c4cadf0eca9e0c5d73caa2783fd。
新备份 `/home/jian/livox_ws/r10_stable_preservation_20261021/object_export_retry_01/`。
使用只读pack-objects --revs --stdout（没有重试unadvertised fetch），
再在rescued.git执行index-pack --stdin及update-ref。
development.bundle由其导出，restored.git仅从bundle独立裸克隆，无alternates。
两个裸仓库fsck均成功；以下commit/tree逐项一致：

| Commit | Tree |
|---|---|
|5e0bd7a98456fbf4453cf2e63f421ffdf649347f|0e9a95d9719cb4c2826d3e2274b6208ea00fe554|
|82d7a5eddce666464e34a28c29768ea152049d72|85e140f627376c99a64e9cb9009c997c25525571|
|41999ea700c66c4cadf0eca9e0c5d73caa2783fd|0f6dfa27838a37db8f9f41deb9839b48cd089acb|

保护引用只建立在独立备份中：refs/heads/preserved_5e0、preserved_82d、current_at_backup。
全部3个提交和其完整可达对象恢复成功；新产物已哈希复验。
旧备份1651项清单及清单自身（共1652文件）前后一致；包括旧index、二进制补丁和1618个未跟踪文件（含ignore）。
源HEAD/branch/refs/index/reflog/status及全部1766文件身份前后一致，没有恢复旧HEAD。
保护结束可用空间22973038592bytes，始终保留20GiB安全余量；未删除任何旧资产。

实际命令、stdout/stderr、两个完整快照、旧备份前后清单、新SHA清单均保留在新备份目录。
R11的PRESERVATION_EXECUTION.json记录实际exit0和最终回执SHA；运行前再次验证这些文件和源快照，
不把回执存在等同于完整保护通过。大型对象包/裸仓库/Bundle保留内盘，不提交Git。
