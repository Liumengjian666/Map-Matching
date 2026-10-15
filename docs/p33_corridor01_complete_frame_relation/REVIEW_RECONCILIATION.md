# 独立复核与处理

先进行fresh-context单模型只读复核，仅允许读取准备程序、两项测试、合同和小型冻结输入。
没有发现这些限定文件中的明确实质违约；其结论不认证范围外依赖。
依赖的TUM解析/SLERP与标准缓存转换由主执行端读取，并复用既有实现。

用户随后选择追加Codex CLI，并再次确认本轮精确命令和输入SHA。
实际仅调用一次，exit0；本机codex-cli0.159.2，gpt-6.1-sol/xhigh，provider=openai，sandbox=read-only，
ignore-user-config、ephemeral；保留现有登录，不传递API key、Bag或大点云。
本地执行日志未见模型工具事件，不提交原始CLI日志。

输入7685字节，SHA256：
`cb6dbdd015bc0900d39e5c7061f160fc3d300cb345b1343d2093ce60ce1af5a9`。
输出SHA256：
`8b08d967d391bedf01afc07f5ea4ec2a55b440f07f5a39cd670d342d106858f6`。
全文分别为EXTERNAL_REVIEW_INPUT.md、EXTERNAL_REVIEW_RESULT.md。
归档输出仅移除Markdown行末双空格以通过diff检查；未改词句，原始输出保留在本地缓存。
上列8b08…为原始输出SHA，归档格式化副本SHA另记于results.json及artifact_hashes.json。

逐项处理：

| 意见 | 分类 | 处理 |
| --- | --- | --- |
| G=M不足以确定GT传感器，CSV b不能自动等同I | 有效、已保留的证据限制 | 表格明确三种物理参考点未认证，不宣布G≠M |
| 失败可能涉及覆盖、时间、发布版本，0覆盖只属于保留点，NN不是位姿误差 | 有效限制，补强文本 | 报告明确1400点/voxel目标及不能唯一归因；不扩大实验 |
| 两种外参解释只改变点位置很小，不能由微小NN差选择参考点 | 有效限制 | 加入80m内约0.421m点位移上界，不认证任何GT参考点 |
| 首消息时间差不是同步偏移；近67与精确67必须分开 | 有效限制、文本已基本区分 | 补写非校时说明；保留两套完整矩阵，未互换、未重跑 |
| YAML差不能证明初始化器故障；Identity只定义O | 有效限制、已保留 | 不宣布初始化源码错误，区分工程表达与物理GT认证 |
| 选择器没有显式拒绝重复header，缺邻帧证明 | 有效、可执行 | 新增strictly-increasing守卫和定向测试，检查实际2777索引均唯一，保存3组相邻帧距离；没有改变选择或重跑NN |
| 浮点秒相等不能证明原始整数点时间无损 | 有效表述边界 | 只声明经既有量化转换后有序XYZ/time相等，不声明原始offset_ns无损 |
| 限定输入未写GT发布时间差 | 输入提取缺口，不是新增数据故障 | 正式报告已记录1385条数值相同、1284时间相同、其余最大239ns；不将其升级为物理同步证明 |

实际三帧准备程序执行时SHA：
`5449512a8af5b4718e79bf0709a1be8429012b44a7e751a1501113c44dc4c195`。
复核后仅增header守卫的最终prepare.py SHA：
`a144739a2e9db5c8a05d100b995a81880bf6840a8b4f35cdb5e9a163ff069550`。
没有重新运行实际Bag准备或NN；实际索引和既有冻结选择由只读补证验证。
最终3项定向测试通过。收集程序的测试计数字段由2更新为3，未改变任何几何数据。

首次准备失败来自本轮工具把缓存uint32 offset_ns直接解释成float秒；在任何NN前停止，
改用pack_causal_slice.make_cloud转换。测试夹具的transaction_id缺项也已修复。
上述均未修改数据或运行算法，不把工具错误写成数据集失效。

确认后实际执行命令（一次）：

```bash
/usr/bin/timeout 480 /usr/bin/codex exec --ignore-user-config --ephemeral --sandbox read-only --skip-git-repo-check --cd /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.p33_frame_relation.a9l5g3/cli_review_workspace --model gpt-6.1-sol --config 'model_reasoning_effort="xhigh"' --color never --output-last-message /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.p33_frame_relation.a9l5g3/external_cli_review.md - < /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/docs/p33_corridor01_complete_frame_relation/EXTERNAL_REVIEW_INPUT.md > /home/jian/livox_ws/p10_corridor01_oosm_r1_ws/.p33_frame_relation.a9l5g3/external_cli.log 2>&1
```

OpenAI Docs用于确认非交互只读调用方式；实际可用参数以本机help核对。
这是额外意见，不是新的官方坐标语义证据。
