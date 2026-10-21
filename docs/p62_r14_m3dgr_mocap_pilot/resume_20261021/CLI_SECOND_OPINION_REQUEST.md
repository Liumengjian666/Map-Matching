# 用户选择后的外部第二意见准备（未运行模型）

用户已选择Codex CLI第二意见。仅完成本地 `/usr/bin/codex --version` 和 `codex exec --help` 验证，版本codex-cli0.159.2，尚未调用外部模型。使用OpenAI Docs与本机帮助核对参数，来源：[官方CLI说明](https://learn.chatgpt.com/docs/developer-commands?surface=cli)。

输入清单及逐项SHA见CLI_REVIEW_REQUEST.json。只包含acquire.py、test_acquire.py、OFFICIAL_ACCESS_RECEIPT.json，不包含稳定工程/保护快照、bag、GT、地图、凭据或历史归档。完整提示词已内嵌这3项，位置：

`/home/jian/livox_ws/p62_r14_resume_cli_review_20261021_bdQHpf/PROMPT.md`

SHA256：`2d04f6160305fe1d12a34b180facef43f9e0ccb247adba66420e028618fa531b`。

精确命令已另外发起确认，等待用户明确批准，选择CLI本身不等于批准以下调用：

```bash
timeout --signal=TERM --kill-after=10s 300s codex exec --ignore-user-config --ignore-rules --ephemeral --skip-git-repo-check --sandbox read-only -c 'approval_policy="never"' -C /home/jian/livox_ws/p62_r14_resume_cli_review_20261021_bdQHpf --output-last-message /home/jian/livox_ws/p62_r14_resume_cli_review_20261021_bdQHpf/SECOND_OPINION.md - < /home/jian/livox_ws/p62_r14_resume_cli_review_20261021_bdQHpf/PROMPT.md
```

仅用现有CLI认证，不提供或修改API密钥及环境变量；忽略用户配置，使用CLI默认模型，不承诺它一定与当前模型不同。提示词禁止工具、其他文件读取、网络、测试执行、改文件及子代理。read-only控制模型命令写权限，不等于仅3文件可见的操作系统隔离；若出现越界工具行为应停止，不能宣称输入隔离获得强制认证。

输出只允许写第二意见文件与调用回执。失败不自动重试、不切模型、不重试资产请求。外部意见不解除本轮ASSET_ACCESS_REQUIRES_USER_ACTION，也不授权任何定位或R15。

本说明晚于原checksums.sha256封存；通过补充清单与Git记录身份，不覆盖旧封存结果。
