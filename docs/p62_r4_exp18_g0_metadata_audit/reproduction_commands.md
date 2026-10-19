# 实际执行命令与只读复核

工作区：`/home/jian/livox_ws/p62_r4_exp18_g0_metadata_audit_ws`。

以下是本轮历史命令，不是重新执行批准。输入快照和CSV用独立一次性输出；现有冻结结果不得覆盖，复做需新输出目录和新的有界审计合同。脚本带固定UTC截止时间。

```bash
python3 -m unittest discover -s docs/p62_r4_exp18_g0_metadata_audit/tests -v
python3 docs/p62_r4_exp18_g0_metadata_audit/assets.py before
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u docs/p62_r4_exp18_g0_metadata_audit/bag_audit.py
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 docs/p62_r4_exp18_g0_metadata_audit/capture_sources.py
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u docs/p62_r4_exp18_g0_metadata_audit/e57_audit.py
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 docs/p62_r4_exp18_g0_metadata_audit/summarize_metadata.py
python3 docs/p62_r4_exp18_g0_metadata_audit/assets.py after
```

冻结文件与结果表只读复核（从工作区根执行）：

```bash
sha256sum -c docs/p62_r4_exp18_g0_metadata_audit/checksums.sha256
cmp docs/p62_r4_exp18_g0_metadata_audit/asset_identity_before.json docs/p62_r4_exp18_g0_metadata_audit/asset_identity_after.json
```

完整原始包、E57路径和预期SHA见assets.py及身份快照；它们位于R3 verified目录，不复制进Git。已用解析器位于R2独立target，未执行pip安装。所有ROS使用仅限rosbag离线API，不存在rosbag play或ROS节点启动命令。
