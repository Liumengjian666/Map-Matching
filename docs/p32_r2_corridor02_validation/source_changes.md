# 源码修改清单

CODE_SHA：48569e92d075c5f14598761ad2e349cce88ee167。

新增scripts/p32_corridor02/：

- prepare_probe.py：有界真实header前缀、active YAML、RC1标定、float32点解码。
- first_geometry.cpp：唯一种子的一次原生NDT及NN几何检查，不接入运行节点。
- run_geometry.py：核对Source身份/点数、输入冻结、调用回执，不读取GT。
- test_adapter.py：5项数据接口定向测试。
- CMakeLists.txt：独立PCL检查程序构建，显式系统libusb。

原NDT/EKF/MSCKF/SKF/Deskew/OOSM运行源码和物理参数均未修改。
没有runtime YAML、视觉提取、传感器全段桥接或运行部署，因为init gate失败。
没有在受保护工作区编译或修改文件。原P31/P32-R1归档不变。
