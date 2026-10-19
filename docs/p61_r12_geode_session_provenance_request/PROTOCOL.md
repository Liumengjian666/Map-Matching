# P61-R12 有界溯源请求合同

仅盘点已有材料并准备供用户审核的正式请求，不发送、不push、不启动下一轮。
R11本地与远端实际均为75b88b88f5c1bb2f978b065bcf3ee1391ec07ce3，工作树干净。
R12从该提交创建独立worktree和研究分支；不修改R11分支或既往回执。

目标绑定：Shield_tunnel1_gamma原始bag与R8本地派生输入各自独立SHA/大小。
原始bag的SHA是本地已冻结文件身份，不冒称作者公布的官方校验和。
R8输入是本地派生文件，不要求作者拥有它或提供相同哈希。
作者可用发布文件指纹、采集会话标识及可验证的转换链确认对应关系；如果原始采集bag
与发布bag不同，不强求字节一致，必须说明二者关系，不混接来源。

盘点范围固定为existing_evidence.csv的22个文件：两份bag仅只读SHA/大小；
20份已有说明、来源元数据、配置、冻结初始化和审计归档只读检查。
不解析新的传感器消息、不读取GT、不联网搜索新驱动或接触数据作者。
Git ls-remote只查询科研仓库分支，不代表联系数据作者。

request_checklist.csv为18项问题（S1、A1–A6、B1–B6、C1–C5）。
缺失是限定盘点中未发现能闭合问题的独立会话证据，不能推断作者从未记录。
所有请求项均允许NOT_RECORDED/UNKNOWN/NOT_APPLICABLE/EXISTS_BUT_RESTRICTED；
不要求补造历史材料，不要求为本请求进行新采集、拟合、优化或校准。

独立同步材料须能追溯设备时钟→转换代码→ROS时间链，并有独立误差依据。
偏置稳定性须有独立校准/验证来源，不能使用同一LIO估计器输出代替。
在没有新且可绑定的材料时，R10/R11结论保持不变。
本轮仅准备接收规则，不据尚未收到的材料更新任何历史结论。

GT reads=0；ROS localization nodes=0；rosbag play=0；new NDT align=0；
EKF runs=0；new trajectory estimates=0；clock-offset search=0；gyro-bias fit=0；
production code changes=0；messages sent=0；external CLI=0。
不访问HIKVISION，不修改地图、初始化、外参或运行配置。
