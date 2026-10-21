# 离线证据复核（不重试下载）

工作区 `/home/jian/livox_ws/p62_r14_m3dgr_pilot_ws`。

```bash
cd /home/jian/livox_ws/p62_r14_m3dgr_pilot_ws
python3 docs/p62_r14_m3dgr_mocap_pilot/tests/test_storage.py
cd docs/p62_r14_m3dgr_mocap_pilot
sha256sum -c checksums.sha256
```

一次正式前检已经执行 `python3 docs/p62_r14_m3dgr_mocap_pilot/preflight.py`。不要重跑；其哨兵拒绝已有回执时再次执行。审计程序仅获取小型官方元数据和一次HEAD，不下载bag/GT。

最终本地归档执行 `python3 docs/p62_r14_m3dgr_mocap_pilot/finalize.py`，只检查现有证据与保护快照并生成本轮输出，不执行网络/定位操作。脚本用独占创建保护关键回执，不覆盖原结果。

预检源码在CODE_SHA `5bb6ad76596a5046cb7461f3d7b84ed5d09b3cc5`。定位研究二进制、C++运行身份、NDT/GT误差命令均NOT_RUN。
