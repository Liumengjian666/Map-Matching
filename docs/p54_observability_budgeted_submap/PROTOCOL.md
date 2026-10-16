# P54 执行协议

## 冻结资产

- Git 基线：`a93d41d46ac60adc04552d76a09861e08f7ed87d`
- 固定样本：P47 `sample_manifest.csv` 中全部 154 行；不增加或删除帧。
- P45 Tile：`/media/jian/HIKVISION/paper rosbag/NewerCollege/.p45_local_map/tiles/`
- P47 frozen source/catalog：`/media/jian/HIKVISION/paper rosbag/NewerCollege/.p47_coupled_shadow_r1/`
- P46 Local/geometry/NDT 诊断：`/media/jian/HIKVISION/paper rosbag/NewerCollege/.p46_real_tile_switch/formal1800/`
- 两个 Target 上限：1,200,000 与 1,500,000 点。
- Source 支撑距离和覆盖门：2 m、95%。

所有 P47 输入文件、P45 Tile 文件均按既存 SHA/点数验证。Source 是 P47 已重建且与 P46 stable cloud hash 一致的冻结 PCD；本轮不重新去畸变、不读取 GT 选样。

## 两阶段运行

1. `select`：读取冻结样本、P46 active Tile 列表、Source 和离线描述符；输出完整选择清单与信息/代理覆盖统计。不执行 NDT，不读取 GT。
2. `align`：核对 154×4 选择行与 P47 样本、时间戳、Source hash 和活动 Tile membership；按实际选择重建 Target，测量精确预测位姿 2 m 支撑率；仅精确覆盖至少 95% 的候选执行一次原参数 PCL NDT `align()`。选择/NDT 输出随后由 pre-GT freeze 清单按 SHA 封存；覆盖失败记录拒绝原因，不重选。
3. `freeze_p54_outputs.py`：冻结样本、154 个原始 PCD、154 个重建 Source、51 个活动 Tile、P45/P46/P47 输入、程序源码、CMake、执行二进制、全部选择结果及 NDT 输出。冻结清单中明确 `gt_opened: false`，并逐项重新核验哈希。
4. `evaluate_p54_gt.py`：在重验所有 372 项冻结文件与 P43 GT hash 后，才读取 GT。只用 P43 条件性 Base-frame 变换链评价 P46 frozen NDT Nominal、Distance-Based 与 Information-Aware 结果；要求精确 GT 时间戳，不插值、不拟合轨迹，不反向影响样本或选择。

最终归档审核发现初次 `align` 命令为比较表中的 P46 初始覆盖列读取了 `global1800/geometry_diagnostics.csv`。该列不参与选择/准入/align/GT，也未影响上述决策；已保留原冻结文件与 SHA，并以无 GT 后处理脚本从 P46 Local `formal1800/geometry_diagnostics.csv` 修正归档主覆盖表。修正未重跑 NDT，完整 provenance 见 `coverage_reference_correction.json`。

主实验和条件性 GT 评价冻结完成后，另生成一次 `tile_descriptor_metrics.csv`。该只读辅助工具调用主 probe 的同一 `makeDescriptor()`，使用 P47 冻结样本与 Tile catalog，只描述 51 个活动 Tile，不打开 GT、不选 Tile、不运行 NDT。其输出 SHA 与主实验 `input_freeze.json` 不属于同一个冻结集合；单独的来源、二进制、输入及输出 SHA 记录在 `tile_descriptor_profile_provenance.json`。此项补测不修改任何主实验数据。

## 预算与算法

两个方法的候选集合、输入点、预测位姿、覆盖门和点预算完全相同。算法、阈值和确定性 tie-break 见 [`THEORY.md`](THEORY.md)。NDT 参数沿用 P41/P46：分辨率 0.8 m、步长 0.08、变换 epsilon 0.001、最大迭代 40、outlier ratio 0.55。不得改变 Source/Target voxel 或观测范围。

## 失败与缺失

- Source/Tile 哈希、点数、输入时间戳不一致：停止并标记合同错误，不能替换输入。
- 精确覆盖不足：保留失败帧，NDT align 调用数为 0。
- NDT 未收敛或结果非有限：保留结果并计入失败率。
- 不做参数搜索、回退到完整 Target、自动追加 Tile 或 GT 引导补图。

本轮不接入运行时地图管理，不以点数上限推导实际 RSS/HWM 已降低。

## 实际复现命令

P54 worktree 位于 `/home/jian/livox_ws/p54_observability_budgeted_submap_ws`。该 worktree 使用显式 `DOG_PRIOR_BUILD_RESEARCH_TOOLS=ON` 编译离线工具；源码默认构建门控仍为 OFF。当前系统环境的 MVS SDK 将其私有 libusb 放在默认搜索路径前，因此只对 P54 命令设置系统目录优先的 `LD_LIBRARY_PATH`，没有改写/删除系统动态库。

```bash
cd /home/jian/livox_ws/p54_observability_budgeted_submap_ws
LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu \
  cmake --build build --target p54_observability_budgeted_submap_probe -j2
LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu \
  devel/lib/dog_prior_map_localization/p54_observability_budgeted_submap_probe --self-test
```

固定输入、选择与对齐输出保存在：

```text
/media/jian/HIKVISION/paper rosbag/NewerCollege/.p54_observability_budgeted_submap/run1/
```

对应 `select` / `align` 参数和所有逐帧结果见本归档 `input_freeze.json`、`selection_manifest.csv`、`align_results.csv` 与 `runtime_comparison.csv`。唯一后冻结 GT 的实际执行为：

```bash
python3 docs/p54_observability_budgeted_submap/tools/evaluate_p54_gt.py \
  --freeze docs/p54_observability_budgeted_submap/input_freeze.json \
  --p43 docs/p43_newer_college_gt_base_frame/input_manifest.json \
  --sample docs/p54_observability_budgeted_submap/sample_manifest.csv \
  --align docs/p54_observability_budgeted_submap/align_results.csv \
  --gt '/media/jian/HIKVISION/paper rosbag/NewerCollege/01_short_experiment/ground_truth/registered_poses.csv' \
  --accuracy-csv docs/p54_observability_budgeted_submap/localization_comparison.csv \
  --metrics-json docs/p54_observability_budgeted_submap/accuracy_metrics.json
```

逐 Tile payload 补测的复现命令（只读 P47 清单和 Tile，不读 GT、不调用 NDT）：

```bash
LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu \
  devel/lib/dog_prior_map_localization/p54_tile_descriptor_profile \
  '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p47_coupled_shadow_r1/frozen/sample_manifest.csv' \
  '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p47_coupled_shadow_r1/frozen/tile_catalog.csv' \
  '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p54_observability_budgeted_submap/run1/descriptor/tile_descriptor_metrics.csv' \
  '/media/jian/HIKVISION/paper rosbag/NewerCollege/.p54_observability_budgeted_submap/run1/descriptor/tile_descriptor_profile_manifest.json'
```
