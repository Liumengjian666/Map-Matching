# 源码范围

CODE commit：c36f6aa0790f11bf6a8caf08321ec3751b4be7a8。
只增加`src/dog_prior_map_localization/scripts/p32_gt_frame/`：

- `prepare.py`：GT TUM/整数时间/插值，active YAML与原RC1矩阵，最早5条合法raw扫描；输出本地二进制和绑定receipt。
- `gt_geometry.cpp`：固定2×3×5个NN诊断；记录30个T_ML和全部几何结果；无ICP/NDT依赖。
- `run_once.py`：原输入、消费文件、源码、协议和二进制冻结；one-shot目录；计数/退出/成本记录。
- `math_test.py`：有限合成夹具的时间、姿态和变换方向测试，不生成观测或定位结果。
- `CMakeLists.txt`：独立PCL io/filters/kdtree程序，不重新构建定位节点。

复用已有`p32_corridor02/prepare_probe.py`的`cloud_array()`和`sha()`。
没有调用其main、proper_transform/SVD或旧初始化逻辑；该文件未改，依赖SHA已冻结。

本归档新增`finalize_evidence.py`只汇总实际30行输出并检验绑定，
`prepare_external_review.py`只生成经确认的有限外部审查输入，
`archive_external_review.py`只归档一次CLI输出和执行receipt。
它们不新增NN、ICP、NDT或ROS调用。

未改成熟NDT、EKF、MSCKF、Deskew、DCReg、多候选、OOSM、任何运行参数或标定。
原GT/PCD/Bag/XYZ二进制只在数据盘或独立本地缓存；不入Git。
未对FAST-LIVO2/OpenVINS源码做新的拷贝或许可证变更。
原P32-R2/R3/R4与全部历史缓存保持原样。
