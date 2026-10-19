# 只读复现与实际执行顺序

工作区：`/home/jian/livox_ws/p61_r11_geode_clock_bias_provenance_ws`。
以下程序不启动ROS节点；import rosbag仅用于离线读库。不要source定位launch或播放bag。

实际执行：

```bash
git -C /home/jian/livox_ws/p61_r10_geode_gyro_increment_audit_ws status --short
git -C /home/jian/livox_ws/p61_r10_geode_gyro_increment_audit_ws rev-parse HEAD
git -C /home/jian/livox_ws/p61_r10_geode_gyro_increment_audit_ws ls-remote origin refs/heads/research/p61-r10-geode-gyro-increment-audit
cd /home/jian/livox_ws/p61_r11_geode_clock_bias_provenance_ws
/usr/bin/python3 docs/p61_r11_geode_clock_bias_provenance/check_assets.py before
/usr/bin/python3 docs/p61_r11_geode_clock_bias_provenance/freeze_evidence.py
/usr/bin/python3 docs/p61_r11_geode_clock_bias_provenance/freeze_lookup.py
/usr/bin/python3 -m unittest discover -s docs/p61_r11_geode_clock_bias_provenance/tests -v
/usr/bin/python3 docs/p61_r11_geode_clock_bias_provenance/inspect_records.py
```

最后一条实际在8fe5d08241be44c821b55f9d773e0eb11656eea8执行，最终JSON元数据
序列化失败；已保存CSV及不完整JSON全部保留，详见EXECUTION_NOTES.md。修正后没有再次执行
它；从保存CSV恢复最终JSON：

```bash
/usr/bin/python3 docs/p61_r11_geode_clock_bias_provenance/complete_saved_audit.py
/usr/bin/python3 docs/p61_r11_geode_clock_bias_provenance/check_assets.py after
/usr/bin/python3 docs/p61_r11_geode_clock_bias_provenance/finalize_archive.py
git diff --check
sha256sum -c docs/p61_r11_geode_clock_bias_provenance/checksums.sha256
```

complete_saved_audit实际版本fe652728f的完整SHA见analysis_manifest.json。
finalize_archive使用运行前本地已提交版本，最终提交只归档，不冒充消息读取代码。

所有写结果的脚本使用exclusive create；不要在冻结目录直接重复运行以覆盖产物。
新复核实例必须复制这些R11分析脚本到独立目录运行，HERE按脚本目录确定，REPO按仓库根
确定；原始输入路径保持只读。已保存CSV可以直接检验，不必重新提取消息。
本轮未安装新依赖，使用现有/usr/bin/python3、numpy、rosbag读取库。

时差为header_ns-record_ns，统计中P95采用NumPy线性经验分位数；选定记录精确查询，
不将record时间窗当header时间窗。历史seed均值只复核源码当时初始化算术，不用作新bias。
任何缺失、SHA变化、重复选定lookup或消息不一致立即停止；禁止以删文件方式通过。

没有NDT/EKF/定位回放复现命令，因为本轮没有也不允许执行它们。
