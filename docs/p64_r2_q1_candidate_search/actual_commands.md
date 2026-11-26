# 实际运行命令与复现

独立工作区为 /home/jian/p64_r2_q1_ws。实际运行在目录根。以下为执行的阶段命令，日志与全量逐点证据在该工作区的 run 目录。所有生成阶段使用只读历史数据，并为运行另建输出目录。

```bash
git -C /home/jian/p64_r1_ws worktree add --no-checkout -b research/p64-r2-q1-candidate-search /home/jian/p64_r2_q1_ws 3b47f06b6ffc09d70710b8af12581d0ee38c7fef
python3 docs/p64_r2_q1_candidate_search/prepare_inputs.py prepare --freeze
bash docs/p64_r2_q1_candidate_search/build.sh
env -u LD_LIBRARY_PATH OMP_NUM_THREADS=4 /usr/bin/time -v timeout 180 prlimit --as=3221225472 -- run/build/qualification gate run/input/online.tsv run/gate
env -u LD_LIBRARY_PATH OMP_NUM_THREADS=1 /usr/bin/time -v timeout 120 prlimit --as=3221225472 -- run/build/qualification smoke run/input/online.tsv run/smoke
env -u LD_LIBRARY_PATH OMP_NUM_THREADS=1 /usr/bin/time -v timeout 720 prlimit --as=3221225472 -- run/build/qualification grid run/input/online.tsv run/rest
python3 docs/p64_r2_q1_candidate_search/seal_experiment.py grid run
python3 docs/p64_r2_q1_candidate_search/peaks.py run/official/grid.csv run/official/modes.csv run/official/fit_seeds.tsv
env -u LD_LIBRARY_PATH OMP_NUM_THREADS=1 /usr/bin/time -v timeout 120 prlimit --as=3221225472 -- run/build/qualification fit run/input/online.tsv run/fit run/official/fit_seeds.tsv
cp run/fit/fit.csv run/official/fit.csv
python3 docs/p64_r2_q1_candidate_search/seal_experiment.py seal run
/usr/bin/python3 docs/p64_r2_q1_candidate_search/prepare_inputs.py references --online-seal run/online_seal.json --output run/reference.tsv
python3 -B docs/p64_r2_q1_candidate_search/evaluate.py --online run/input/online.tsv --manifest docs/p64_r2_q1_candidate_search/qualification_sample_manifest.csv --grid run/official/grid.csv --modes run/official/modes.csv --fits run/official/fit.csv --curvature run/fit/curvature.csv --reference run/reference.tsv --seal run/online_seal.json --out run/evaluation
```

实际 Gate 1 后以同一验证条件的 Python 命令生成 gate/identity.json，封装后的复现脚本使用 seal_experiment.py gate。实际计时 stdout/stderr 写入 run 目录的对应文件。没有随机抽样，需要相同输入顺序、浮点构建和固定姿态。

复现应换一个新的绝对路径，如 bash docs/p64_r2_q1_candidate_search/run_qualification.sh /home/jian/p64_q1_reproduction_01。程序拒绝覆盖既有阶段目录。
