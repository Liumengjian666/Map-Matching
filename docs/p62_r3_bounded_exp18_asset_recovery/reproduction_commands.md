# R3 复核与历史命令

以下只读回执复核会生成新的同内容 `receipt_audit.json`，不联网、不解析 bag/E57，不改大资产：

```bash
cd /home/jian/livox_ws/p62_r3_bounded_exp18_asset_recovery_ws
python3 docs/p62_r3_bounded_exp18_asset_recovery/audit_receipts.py
python3 -m unittest discover -s docs/p62_r3_bounded_exp18_asset_recovery/tests -v
sha256sum -c docs/p62_r3_bounded_exp18_asset_recovery/checksums.sha256
```

checksums 清单中的文件路径相对工作区根目录。若重新生成审计 JSON，内容确定且应保持清单匹配。合成测试使用内盘独立临时夹，自动清理的是测试自身创建的小型临时数据，不是历史资产。

实际执行的恢复命令，代码版本 `3d54bb08d9f712724c911cd5f8931e6063e90fb8`：

```bash
cd /home/jian/livox_ws/p62_r3_bounded_exp18_asset_recovery_ws
python3 -u docs/p62_r3_bounded_exp18_asset_recovery/recover.py --run
```

此命令仅作为历史复现记录。**不要再次执行**：本轮授权已用完，程序也会因既有独立输出目录拒绝重跑，并有固定 UTC 截止时间。未请求新的下载批准。

不需要 ROS、PCL、E57 库、外部 CLI 或新增依赖。Python 标准库用于请求、哈希、文件守卫和回执检查。
