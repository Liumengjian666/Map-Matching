发现三处实质问题：

1. **准入检查的是种子，遗漏最终解。** `initializer.cpp` 的 `seed_SE3_float_tolerance` 仅检查 `guess`；`run_once.py` 将它作为 `legal_seed` 门槛，却不检查 `result` 的有限性、旋转行列式及正交误差。即使最终变换违反冻结协议的 proper／`1e-5` 要求，也可能被标记为准入。本轮未收敛，因此未触发这条错误准入路径。

2. **输入哈希没有绑定实际消费的文件。** `prepare_input.py` 在 `files_sha256` 中保存绝对路径；`run_once.py` 校验这些路径，而 `input_bridge.cpp` 读取传入的 `inputs` 目录。将提取目录 A 复制到 B、修改 B 的 IMU 后，清单仍可校验未改动的 A，桥接却消费 B。这样有限、时间连续但已被替换的 IMU 可以绕过冻结检查，破坏“真实原始输入”的约束。

3. **源码冻结遗漏实际参与转换的依赖。** `prepare_input.py` 从目录外的 `../p32_corridor02/prepare_probe` 导入 `cloud_array` 和 `sha`；`run_once.py` 的源码哈希范围仅为自身目录下的 `sources.rglob('*')`，未覆盖该依赖。决定 XYZT 提取方式的代码可以改变而不触发历史源码不匹配检查，因此整个 bootstrap 输入路径的同源结论缺少绑定依据。

所附运行结果只能支持：本次冻结的 24 个种子全部未收敛，最终矩阵均未离开种子，初始化失败。上述缺陷尚不能被认定为此次失败的原因；约 292 米的最佳 NN 中位数也不能证明数据集或地图错误。