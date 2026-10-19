# 复现与执行边界

工作树：`/home/jian/livox_ws/p62_r1_novelty_benchmark_lock_ws`

```bash
cd /home/jian/livox_ws/p62_r1_novelty_benchmark_lock_ws
/usr/bin/python3 -m unittest discover -s docs/p62_r1_novelty_benchmark_lock/tests -v
/usr/bin/python3 docs/p62_r1_novelty_benchmark_lock/archive_evidence.py
cd docs/p62_r1_novelty_benchmark_lock
sha256sum -c checksums.sha256
```

archive_evidence.py 读取已经冻结的两份抓取回执及小型文献/元数据缓存，检查其SHA并生成**新P62归档**，不读任何bag/PCD/旧实验原始数据。缓存根目录 `/home/jian/livox_ws/p62_r1_source_cache` 未入Git；使用源URL重新下载的main文件可能变化，必须保留与旧缓存SHA不同的事实，不可覆盖旧文件。初始capture脚本以exclusive-create保护已抓取文件，不能在原缓存上重复运行；网页访问失败也保留回执。

本轮没有可声称已经验证的Hilti运行命令；G0未通过，不提供猜测topic/四元数/时间单位的launch。下一轮必须先实际验证文件后再生成受控运行命令。

编译环境未改变：未编译C++、未安装依赖。测试使用已有系统 `/usr/bin/python3` 和NumPy。单元测试为人工数组，不读取传感器或GT。

Git START：`17d633000312227d7ced9f8f190bc72f6f520d7d`。执行前远端R12与START一致，查询未发现P62远端分支；实际最终提交由结束回执给出，避免文档自指哈希。未push。
