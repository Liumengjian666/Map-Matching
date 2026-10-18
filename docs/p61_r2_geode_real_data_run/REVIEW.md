# 验证与边界复核

- 13项Python单测通过：存储错误归属／普通D态、恢复证据分离、真实消息layout、拒绝错误schema、
  文件截断、完整严重判据、稀疏／连续区间区分、归档字节身份及真实调用数量。
- 1项C++实际target_cells_网格测试通过，无align。旧错误初始化对象被guard拒绝，新XYZ严格为.8f。
- 57条实际CustomMsg与原生反序列化器交叉检查，全部180523条原始记录计数与索引一致。
- 120个实际Hessian由NumPy独立复核谱及最弱向量残差；不是只信任报告分类。
- 120次align＋120次只读native jet，360条实际网格审计，无候选、额外align或反馈。
- 一个Smoke/Screen重叠pair按输出做精确parity，无追加运行。

按code-review-and-quality技能进行本地五轴检查：
正确性区分6DoF比值分量与联合严重标签；架构只新增docs研究工具；
输入schema／路径／原文件保护采用失败关闭；资源、尝试和样本数量受限；未引入依赖或生产分支。
Git／增量技能使转存、适配、探针及准入工具分开提交。
没有外部CLI、跨模型或新上下文代理复核，不能声称存在这些验证。

official/livox_lddc.cpp是公开语义参照，原始CRLF保持不动；局部.gitattributes设置-text/-diff防止Git重编码。
它不参与编译，不把当前公开驱动版本当成GEODE采集版本认证。
没有运行完整ROS构建，因为生产源码和启动链未改。

限制仍包括：未去畸变、局部相邻扫描Target而非独立先验图、无GT、100稀疏样本、
未知全局／物理轴、未认证IMU外参和GEODE时钟转换。NDT收敛和Hessian SPD不证明定位正确。
