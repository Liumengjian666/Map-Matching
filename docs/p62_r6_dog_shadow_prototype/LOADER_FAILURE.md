# 第一次启动的动态库失败（保留，不覆盖）

实际运行源码4592128cf21c0c22d51c6a88fdb9a10ec0d64497。原目录`/home/jian/livox_ws/p62_r6_dog_shadow_runs/`中的RUN_STARTED.json、run_freeze.json、formal.log、run_completion.json永久保留。

第一次进程退出127，0.06636135099688545秒，在C++ main之前动态链接失败：系统PCL IO需要`libusb_set_option`，但实际解析为`/opt/MVS/lib/64/libusb-1.0.so.0`，其动态符号表没有该符号。`/lib/x86_64-linux-gnu/libusb-1.0.so.0`具有该符号。`ldd`与`nm -D`核实了这一差异。

没有创建formal目录、没有Source预处理、更没有NDT调用。完整输入及稳定源码前后SHA/状态通过。不得将此次启动算作100帧实验完成，也不得隐瞒失败。

只对研究子进程设置LD_PRELOAD为已核验的系统libusb和LD_BIND_NOW=1；不改系统配置、生产环境、优化器或二进制。先以无参数方式测试真实shadow进程动态加载和argc拒绝路径，期待exit1及STOP usage，新增align=0。随后使用全新attempt02目录，不覆盖第一次回执。100帧窗口及所有预算/参数不变。
