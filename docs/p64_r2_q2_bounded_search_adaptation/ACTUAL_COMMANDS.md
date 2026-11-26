# 执行命令与复现

实际运行目录 /home/jian/p64_r2_q2_ws。Q1 证据目录 /home/jian/p64_r2_q1_ws/run。源、目标图与初始化矩阵从 Q1 封存取用，保持原路径和归档哈希。Q2 试验以全新输出目录为目标。

```bash
q=/home/jian/p64_r2_q1_ws
d=docs/p64_r2_q2_bounded_search_adaptation
P64_Q2_BUILD=/home/jian/p64_r2_q2_ws/run/build_final bash "$d/build.sh"
export OMP_NUM_THREADS=1
env -u LD_LIBRARY_PATH run/build_final/search validate "$q/run/input/online.tsv" "$q/run/official/grid.csv" unused
env -u LD_LIBRARY_PATH /usr/bin/time -v timeout 120 prlimit --as=3221225472 -- run/build_final/search gate "$q/run/input/online.tsv" "$q/run/gate/native_score_consistency.csv" run/official_score_parity.csv
env -u LD_LIBRARY_PATH /usr/bin/time -v timeout 1200 prlimit --as=3221225472 -- run/build_final/search search "$q/run/input/online.tsv" "$q/run/official/grid.csv" run/official "$q/run/official/fit.csv"
cp "$q/run/input/online.tsv" run/online.tsv
python3 -B "$d/seal_q2.py" "$q" run
python3 -B "$d/evaluate_q2.py" --online run/online.tsv --manifest "$q/docs/p64_r2_q1_candidate_search/qualification_sample_manifest.csv" --grid run/official/grid.csv --retained run/official/retained.csv --fits run/official/fit.csv --runtime run/official/runtime_breakdown.csv --reference "$q/run/reference.tsv" --seal run/online_seal.json --q1-run "$q/run" --out run/evaluation
```

第一次正式启动在读取旧网格的列索引时退出。该尝试在创建输出目录之前停止，生成候选及调用优化次数均为 0。修正采用字段名定位并新增完整网格读取校验，评分和参数保持不变。Gate A 在两个二进制上分别通过，结果字节一致。实际环境为 Ubuntu Linux 5.15、AMD Ryzen 7 5800H、PCL 1.10、g++17、OpenMP 单线程。无随机种子。

复现可运行 bash docs/p64_r2_q2_bounded_search_adaptation/reproduce.sh /home/jian/p64_q2_reproduction_01。该脚本要求路径尚不存在。固定候选导出及 PCD 验收见 [R3 准备计划](R3_PREPARATION_PLAN.md)，30 帧因果短窗另见 [Protocol B](protocol_b_initialization.md)。

封存后补充候选保留和精修终值合并统计的实际命令为 python3 -B docs/p64_r2_q2_bounded_search_adaptation/supplementary_analysis.py run /home/jian/p64_r2_q1_ws docs/p64_r2_q2_bounded_search_adaptation。该脚本只读取既有结果。新输出已封存，重新执行时请用空的输出目录。
