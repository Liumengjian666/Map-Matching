# 旧程序来源与实际使用状态

| 文件 | 原目录 | SHA256 | 本轮状态 |
|---|---|---|---|
| superloc_first_segment_initializer.cpp | superloc_adapter_ws/src/superloc_corridor_adapter/src | deb58aeb3bd76f802568469f50d33f91b8026924b3fe299fa93b8fa8f9dc266d | 原main实际执行；原历史manifest一致 |
| superloc_map_normalizer.cpp | 同上 | 5c38a755fc015abffabafab7ed71f343716ca29028b9f93b4ce35fba24abe808 | 逐字副本，未运行 |
| superloc_holdout_sanity.cpp | 同上 | 4e4c1b0e62a53629cb95828064a3adc960949ee0b5cce6ecfc3c4424d0572807 | 副本，未运行 |
| full_se3_deskew.hpp | superloc_adapter_ws/src/superloc_corridor_adapter/include/superloc_corridor_adapter | 0cba8177a3542bbc9589e0e9418bc1db1727a66c572cad18eac592fe419f596e | 副本，仅来源保存，未用于新bootstrap |
| superloc_sensor_adapter.cpp | superloc_adapter_ws/src/superloc_adapter/src | 58c4073839247098b6f0f545065056be647336cc64631563f5d070d7dcd3b4e8 | 只读数学参考，现版本需要full-SE3反馈，不运行 |

原initializer的完整源位于本轮scripts的legacy目录；P2B报告和历史
source manifest位于 `docs/superloc_corridor01/`。没有复制/修改受保护
机器狗工作区或主论文工作区的源码。

本轮初始化工具二进制：

- initializer：`7111a9792ad0601941a57831f1c7fbf5a3db3e1e086ad8db4f8b38b73c858ba4`
- input_bridge：`2a81ffd45683b4097321e0c8c9ffd63d0f236b2a5f0ba545fa752c8158163dbe`
- math_test：`2840c48a41389df6dcd562c03a6ab2d0d3ec62c842a3145066964ae308dc289e`

这是离线初始化程序哈希，不是新的运行NDT/EKF二进制；后两者没有
在本轮编译或执行。原始地图、派生输入bag和local PCD留本地缓存。
