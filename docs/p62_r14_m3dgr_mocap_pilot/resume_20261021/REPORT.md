# R14授权续行：ASSET_ACCESS_REQUIRES_USER_ACTION

唯一实际阻塞：没有取得真实官方bag下载地址。存储阻塞已经解除，后续坐标、时间及算法阶段未运行，不额外虚构其他失败。

固定基线 `9125c4f9b313994b08572bec143e20b7b81d1d3b`；实际获取程序CODE_SHA `7eaa7309dd0f7ce0f4564ab3bac2fb459794946e`；同一R14分支及工作区，旧失败归档保持原样。原分支开始时本地与远端均为基线SHA，工作树干净。

## 已验证事实

- 下载目标 `/home/jian/livox_ws/p62_r14_m3dgr_data/resume_20261021` 与研究工作区同文件系统。启动可用 `91412652032` bytes；预留20GiB，授权bag6/GT辅助1/地图临时20/日志结果4GiB共31GiB，预算门通过。64MiB提前停止余量固定，不根据结果改变。
- 限定3个目录的序列标识文件名搜索没有发现候选载荷。未声称对所有改名文件作穷尽哈希搜索。
- 复用官方固定版本 `e0cf7d59c9a5a3df515624034698d976abc26549` 的序列行与摘要记录；预期bagSHA `366171a5f1d0c1e7bd602e2b664d85dbd89dd08570c4165142ef9849029c1cf6` 仅为预期身份，不是已验证本地文件。
- 只做一次本机Alipan初始GET：`https://www.alipan.com/s/aVQ24Hm4Xcm`。没有再次请求旧OneDrive链接，没有运行旧preflight.py。
- 本机错误：`URLError: <urlopen error EOF occurred in violation of protocol (_ssl.c:1145)>`；HTTP状态未知，未收到HTTP响应或有效载荷。耗时 `0.333548` 秒。它不证明官方链接全球失效，也不证明必须登录。
- 浏览工具另对官方固定说明及Alipan入口进行了有限读取；分享页在该工具中不可读。这不是下载成功或认证结论。
- bag下载0 bytes，本地bag大小/实际SHA未知。GT未请求本机下载或解析，来源与独立性仍未验收。

## 后续全部NOT_RUN

没有建立T_W_B/T_B_L/T_O_B或时间关系；没有地图、测试帧、wheel预测、NDT、EKF或误差评价。S1–S6均NOT_EVALUABLE，不能回答5°误拒或预测/NDT谁更准确。算法时延与RSS未测。

获取进程峰值RSS `530665472` bytes，包含只读保护快照，不是NDT运行内存。实际请求耗时另记ACCESS_RUNTIME.csv，算法RUNTIME_AND_MEMORY.csv仅表头。

15项下载守卫合成测试通过；初次反证发现的总超时、循环/默认端口重定向问题已修正。没有把测试通过写成真实数据或算法通过。真实生产与全部旧R14文件SHA/Git快照前后相同。无清理、无删除、无新数据集、无R15。

## 唯一NEXT

请用户通过官方网页取得同一Wheel-float01 bag及GT，或提供合法可访问的真实载荷URL。入口、容量限制、落盘路径和所需操作详见USER_ACTION_REQUIRED.md。收到材料后仍必须先核验SHA/格式及参考、外参、时间和预测合同；本轮不自动重试或运行实验。
