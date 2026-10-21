# 获授权的一次CLI第二意见及逐项处理

用户先选择Codex CLI，再明确确认限定3文件、SHA和精确命令。随后仅执行该命令一次，进程退出0；CLI0.159.2，provider=openai，model=gpt-6.1-sol，approval=never，sandbox=read-only，tokens used=7621。未确认它与本会话采用不同架构，因此只称独立CLI第二意见。

原始意见保留在external_review/SECOND_OPINION.md，不删除反例、不把它改成通过。执行前后输入包5项校验一致，日志未出现工具调用；这不是操作系统级3文件可见性隔离证明。此前CLI_SECOND_OPINION_REQUEST/CLI_REVIEW_REQUEST中的PENDING是请求时历史状态，最新状态见EXTERNAL_REVIEW_RECEIPT.json。

## 1. TLS错误不足以证明浏览器/登录必要（High）

分类：合同表述需要澄清，不接受其“当前状态必定错判”的推论。

用户原合同的停止条件包含“或无法取得真实下载地址”，并不要求证明登录是唯一原因。当前确实未取得HTTP响应或真实地址，ASSET_ACCESS_REQUIRES_USER_ACTION符合该停止条件。REPORT与USER_ACTION_REQUIRED已经明确：TLS EOF不证明必须登录或官方链接全球失效。唯一NEXT允许直接提供本地合法载荷/真实URL，并非要求以登录修复TLS。保留此解释限制，不把错误归因于网站权限。

## 2. runpy会执行helper模块顶层，受限输入不足以证明安全（High）

分类：有效的来源与接口限制；不能仅凭run_name证明任意模块安全。

本地只读检查实际helper：`/home/jian/livox_ws/p62_r12_feasibility_pool_ws/docs/p62_r12_feasibility_preserving_pool/execute.py`，当前SHA256 `6b4ad4d89b3868c5d903829809802214215abe5431c2d55ac498af987ead366a`。顶层为标准库导入、路径/常量与函数定义，实验入口位于__main__守卫内；本次只调用stable_snapshot。静态结构摘要另存，不运行任何历史prepare/build/bind/run。真实生产及旧R14快照相等。

helper未包含在授权外部3文件中，外部审阅不能独立认证它；本轮也没有在获取前另行绑定helperSHA，不能事后冒充完整运行时依赖认证。若将来复用，须先提取独立只读快照接口并在新合同中冻结身份。当前观察不能证明任何未知或未来修改过的helper安全。

## 3. 元数据/JSON写入未逐次执行配额守卫（Medium）

分类：有效潜在缺口，**没有追溯修正实际运行源码**。

bag流式写入有硬上限及余量检查；但write()与非bag样本保存不逐次检查日志配额/20GiB。若其他进程在初始容量门后大量占用磁盘，存在写入后跨越预留的路径。因此15项测试不能认证所有写入路径。当前实际TLS在响应前失败，没有HTML样本或bag写入；开始/结束容量均远大于预留，元数据远小于4GiB，未观察到本轮预算违规。

明确标记 `ACQUISITION_PROGRAM_NOT_CLEARED_FOR_GENERIC_REUSE`。未来任何新传输前必须补齐所有写入路径的统一配额检查及相应测试；不能去掉哨兵直接复用本程序。本轮按用户要求停止，不为修此潜在路径重试网络、重跑旧前检或伪造新成功。

## 4. 官方域名别名与同share_id的语义去重（Medium）

分类：有效的语义定义/测试限制，但非本次实际重复请求证据。

当前限制是初始GET一次、URL循环/默认端口/降级拒绝；没有将所有官方别名按share_id合并。首次302跳转到另一官方域名也可能是合法迁移，不能仅凭同share_id证明已重复读取相同HTML。若未来合同要求按share身份去重，须预先定义别名映射并测试；不能临时靠猜测改变下载规则。实际回执HTTP状态null、redirects_followed为空，没有发生该路径。

## 正式结论与边界

保留：ASSET_ACCESS_REQUIRES_USER_ACTION；只有本机TLS链路未取得载荷这一个实际阻塞。存储是当前观察到的容量通过，不是通用下载器配额认证；源码通用复用门未通过。没有bag/GT/地图/NDT/误差的新结果，没有科学假设得到验证。

第二意见后没有重复任何HTTP或模型调用，没有修改实际acquire.py/test_acquire.py，实际CODE_SHA仍为7eaa7309dd0f7ce0f4564ab3bac2fb459794946e。用户下一步只需交付同一序列的合法原始bag/GT或真实URL；坐标、时间、独立性及实验仍须分阶段验收。本轮不进入R15。
