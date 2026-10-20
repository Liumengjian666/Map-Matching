# 复核命令（不启动定位）

以下命令在研究worktree根运行；无需重下载、重解析大文件。

```bash
git -C /home/jian/livox_ws/p62_r5_exp18_coordinate_closure_ws status --short
git -C /home/jian/livox_ws/p62_r5_exp18_coordinate_closure_ws diff 49a6733fb63b60fff06f001c4efc1e9c5441cbea --name-only
cd /home/jian/livox_ws/p62_r5_exp18_coordinate_closure_ws
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s docs/p62_r5_exp18_coordinate_closure/tests -v
sha256sum -c docs/p62_r4_exp18_g0_metadata_audit/checksums.sha256
sha256sum -c docs/p62_r5_exp18_coordinate_closure/checksums.sha256
```

原实际数值生成命令为 `PYTHONDONTWRITEBYTECODE=1 python3 docs/p62_r5_exp18_coordinate_closure/audit_contracts.py`，文件独占写入，不应直接在完成目录重跑。复算可导入`matrices()`、`profile_transform()`、`boundary_policy()`输出至终端或新的显式目录，不能覆盖冻结结果。

实际采集源URL及SHA在issue_identity.json、source_manifest.json、consumer_source_manifest.json。采集脚本是一次性独占写入，完成后不为“复现”而覆盖旧证据或重取可变Issue。

只读计算示例：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -c 'import sys;sys.path.insert(0,"docs/p62_r5_exp18_coordinate_closure");from audit_contracts import matrices,boundary_policy;print(matrices());print(boundary_policy())'
```
