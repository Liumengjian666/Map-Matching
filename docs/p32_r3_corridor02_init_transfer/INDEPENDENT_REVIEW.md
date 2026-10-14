# 有界只读复核

fresh-context reviewer：`p32_r3_transfer_review`。
限定于原首段初始化完整复用、候选作用顺序、输入桥接与5秒边界。
不启动ROS/ICP、不读取GT、不修改源码、不增加审计阶段。

复核结论：没有发现本轮实质问题。原 initializer 与副本逐字一致；
包装器调用同一Base::align；局部ICP、24个 R_yaml*A_i、固定t和排序不改。
gyro梯形积累、SO(3)、slerp及旋转杆臂与现有 adapter 对应实现一致。

指出并核对的边界：

1. 旧loader是string-find，理论上可能误命中同名注释。实际冻结YAML
   前置注释只有矩阵数值，没有同名key；本次active parse真实测试PASS。
   不修改旧parser，不尝试注释矩阵。
2. 桥接SVD没有原adapter的det<0处理。本次原RC1 R determinant为
   1.0000000000000009，反射分支不触发；记录适配范围，不泛化到反射标定。

编译中Eigen诊断类型已定向改为显式Matrix4f，不改变ICP。
按代码质量技能核对准确性、接口范围、有限输入/预算、秘密隔离和
性能报告；没有提交Bag/PCD，运行节点和受保护工作区不改。

外部CLI：用户确认本轮精确命令和限定输入后，实际执行一次。
限定输入 SHA256：
`31ae8b1a277363df066538676a37b677408e7edb580b2c523c0a5374814ab104`。
gpt-6.1-sol/xhigh、read-only、approval=never，退出0；原输入、输出、日志
及回执完整保存在external_review。没有再次外部调用。

三项发现均分类为 **valid + actionable**：

1. 准入只校验seed，未校验result。现在同时校验最终4×4的finite、
   齐次行、det及orthogonality。本次24个result严格等于各自seed，
   没有进入成功准入路径，未把此潜在缺陷判为失败原因。
2. 清单记录绝对路径，原代码没有证明其等于bridge消费目录。现在
   精确绑定scans/IMU/calib/pointfiles集合、目录和字节数，校验消费字节。
   本次实际消费目录与原清单完全相同，补核查全部哈希一致。
3. 来源冻结未覆盖外部导入的R2 decoder。今后提取前记录self/decoder，
   提取后与运行前复核，源码冻结包含decoder。本次该源码与START
   commit一致，但这是事后补核查，不能冒充执行前已冻结该依赖。

新增的guard定向测试拒绝非法final矩阵、A清单校验B目录、消费文件变化
和decoder变化；全部通过。旧实际执行的prepare_input/run_once已另存
immutable快照，保留原冻结SHA；修复后没有重跑输入提取或任何ICP。
新的guard未再次提交外部CLI，因此不能称“修复版本外部复核通过”。
详细补核查见supplementary_input_binding.json。
