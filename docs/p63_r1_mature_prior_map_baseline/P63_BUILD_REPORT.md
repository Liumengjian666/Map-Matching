# 原版独立 Release 构建与身份

使用官方 ros1 SHA `60b57aaac8dc397f80c56364e7ccb008c300cc29`，独立 catkin root `/home/jian/livox_ws/p63_r1_super_lio_data/Super-LIO`。只读使用本机 ROS Noetic、GCC9、PCL/Eigen、TBB2020.1、glog0.4.0；未 apt 安装／升级系统依赖，未在稳定机器狗工作区编译。

可重现编译命令（在这个 catkin root 执行）：

```bash
source /opt/ros/noetic/setup.bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 --make-args -j2
```

作者 CMake 使用 C++17、Release -O3、pthread、fexceptions、LTO；没有换优化级别反复试精度。`logs/upstream_release_build_02.log` 为原版成功构建，`logs/probe_observer_build_03.log` 为原库及研究读取／冻结验证工具的成功构建。`BUILD_FREEZE.json` 保存六份实际二进制 SHA，最终检查六份完全相同。

| 产物 | SHA256 |
|---|---|
| 原版 `devel/lib/super_lio/relocation_node` | ca6607d9e9e3f9c9436fd2c4563d2684e62cdb6c210305e1b38f03fa421951fb |
| 原版 `devel/lib/liblio.so` | c436f658eacf4528006917794653953564c8f444833b97bbf453039523aafcdf |
| 实际运行 `devel/lib/p63_super_lio_baseline/p63_original_observer` | ce69ff472b1043f5b6ceb38aa58ae723eeacf1f5a2e0c4b5c034a18e5c664070 |
| 冻结验证 probe | 5e0aabc1eceb120eb7b501ae61fbecdefb717dd44e8d07b31c67da4912803472 |

observer 链接未改 liblio/basic，沿原 500 Hz spinOnce/process 顺序，仅导出 protected 状态／匹配／地图摘要和故障终止；无估计器虚函数 override。实际有输入运行 CODE 为 `5ae6d0c600e1140b996e83a237d81cffb956969d`。之后仅增加结果归档、one-click 输入组合和已有文件保护，不是第二次定位试验。

## 实际失败与最小处理记录

1. `upstream_release_build_01.log`：PCL IO 链接缺 `libusb_set_option`。继承的相机 SDK 环境优先加载 `/opt/MVS/lib/64/libusb-1.0.so.0` 旧库；系统 `/usr/lib/x86_64-linux-gnu` 的库具有所需符号。仅为构建／执行进程前置系统库路径，未改源码或全局环境。
2. `probe_build_01.log`（另有同一日志的 probe_build.log 副本）：研究 helper include 的作者头需要 ROOT 宏。自己的 CMake 从 `lio` target 读取 source dir 定义 ROOT；不是改算法以通过编译。
3. `probe_observer_build_02.log`：一次误在研究仓库 root 配置，未编译即失败；小型 build/devel 保留。随后在正确 catkin root 构建通过。
4. `zero_input_import_failure.log`：监督器误用旧 helper 类名，**在启动 ROS 和读传感器前报错**。使用已有 OwnedProcesses 的真实名称，提交 `5ae6d0c6...` 后只进行一次有输入试验；这个 helper 仅管理进程，不复用 P62 估计器。
5. `oneclick_wrapper_generation.log`：运行后新增 Python input wrapper 的 CMake 生成成功，不重新处理传感器。one-click launch `--dump-params` 在 `oneclick_launch_parse.log` 通过。

## 必要验收和审查

实际 PCD map probe 和真实 smoke 都执行；前者 PASS，后者 FAIL，不用编译通过替代定位验收。最终 Python AST、launch XML、13 份 gzip 逐字还原 SHA、95 条输出有限／时间单调、六份二进制冻结、作者源码 pristine、稳定工程 before/after 全部一致，见 `evidence/FINAL_DELIVERY_CHECKS.json`／`FINAL_IDENTITY_CHECK.json`。

五项源码自审：正确性（observer 不改选择／状态，停止条件只中止整次测试）、可读性（估计器与研究工具分离）、架构（原作者库为唯一 LIO）、安全（不提交环境转储和认证信息，原 bag/map 只读，已有轨迹禁止覆盖）、性能（时间测量范围显式；诊断日志成本另列）。未使用外部 AI/CLI 第二意见，未增加大规模合成测试或第二套算法。

归档保真说明：上游原样源码和原始构建日志含原有尾随空格；全归档diff-check会报告这些空格。本轮没有清理它们来伪造“原样”SHA。排除这两类证据后的研究源码／报告／台账staged diff-check通过。
