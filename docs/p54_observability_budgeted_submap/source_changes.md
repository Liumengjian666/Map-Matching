# P54 源码与隔离边界

## 修改

- `src/dog_prior_map_localization/CMakeLists.txt`：仅在已有 `DOG_PRIOR_BUILD_RESEARCH_TOOLS` 门控内增加 P54 离线 probe 与自检入口。主实验输出冻结后，再增加仅用于描述字节统计的 `p54_tile_descriptor_profile` 目标；该辅助目标没有生产调用路径，也未安装到 runtime。
- `docs/p54_observability_budgeted_submap/tools/p54_budgeted_submap_probe.cpp`：独立离线程序，读取冻结 P47 Source 与 P45 Tile，描述 Tile、选择候选、检查覆盖并按覆盖门执行固定参数 PCL NDT。没有 Coupled、EKF、OOSM、Deskew 或反馈代码路径。
- `docs/p54_observability_budgeted_submap/tools/freeze_p54_outputs.py`：构造不读 GT 的输入/结果 SHA 清单。
- `docs/p54_observability_budgeted_submap/tools/evaluate_p54_gt.py`：验证 pre-GT freeze 和 P43 GT hash 后，才评价冻结候选。
- `docs/p54_observability_budgeted_submap/tools/summarize_p54_results.py`：生成固定统计汇总。
- `docs/p54_observability_budgeted_submap/tools/p54_tile_descriptor_profile.cpp`：后冻结一次性元数据统计器；以同一 `makeDescriptor()` 计算 51 个 Tile 的 payload bytes，不做选择、NDT 或 GT 读取。

工具构建仍需显式 `-DDOG_PRIOR_BUILD_RESEARCH_TOOLS=ON`；P54 不改变仓库的默认生产构建门控。运行时只通过进程级 `LD_LIBRARY_PATH` 避免 MVS SDK 私有 libusb 优先加载，没有改动系统库。

## 未改变

- `dog_prior_map_ndt_node.cpp` 和生产在线定位路径。
- NDT 参数/目标函数、Source 预处理、P45 Tile、地图坐标、P46 状态或轨迹。
- EKF/OOSM、IMU Deskew、耦合/Recovery 算法。
- GT、原始扫描和传感器标定。

P53 原工作区中的未提交源码、归档和二进制追溯记录均保持原样。P54 使用从已提交基线 `a93d41d46ac60adc04552d76a09861e08f7ed87d` 建立的独立 worktree；其实际 P54 可执行文件 SHA 在 `input_freeze.json` 与 `REPORT.md` 中记录，不冒充 P53 冻结二进制。

P54 主实验 freeze 时 CMake SHA 为 `d237eba5dbe98071e099f58e769e838035ed28b1f8e588c56fac1ee757da5a3d`。增加描述 profile 辅助目标后的当前 CMake SHA 为 `be45f27d8e9402ec7028dbd832d971a02d2ca5ce02bcd053560ddcb5412d51c0`；主 probe 源码仍为 SHA `1b49f377d115082330efb17a88fbaa945bcf1ba1fd4d1beb6afacf295ab8c9db`，主 probe 二进制仍为 SHA `5646dab51e063628a27129d5f17e8e41761a68c776a2972a27bbb610a594daac`。后冻结辅助 profile 源码/二进制 SHA 和输入输出 SHA 见 `tile_descriptor_profile_provenance.json`。
