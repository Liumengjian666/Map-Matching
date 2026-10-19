# 复核命令（不自动重新下载）

```bash
cd /home/jian/livox_ws/p62_r2_hilti_exp18_g0_ws
/usr/bin/python3 -m unittest discover -s docs/p62_r2_hilti_exp18_g0_admission/tests -v
/usr/bin/python3 docs/p62_r2_hilti_exp18_g0_admission/audit_available.py
```

第二条只核验现有小资产与partial的文件身份，不把partial作为bag解析，不运行定位；会重新生成本轮派生审计文件。正式归档SHA检查见checksums.sha256。

原执行命令：

```text
/usr/bin/python3 -u docs/p62_r2_hilti_exp18_g0_admission/acquire_assets.py
kill -INT 119990
/usr/bin/python3 -u docs/p62_r2_hilti_exp18_g0_admission/acquire_ranges.py
/usr/bin/python3 docs/p62_r2_hilti_exp18_g0_admission/publish_small.py
```

kill只针对本轮当时已确认的下载子进程，**不可复制旧PID再次执行**。原单连接SIGINT为主动性能切换，非USB/网络故障。Range命令失败后未重跑；脚本已有receipt时拒绝重新执行，不删除receipt绕过保护。

版本区别：1519e8d679c55bedac841d8e88f4ac3218d787da记录实际启动传输源码。当前acquire_ranges.py缓存分支已改用publish_small，原运行未执行该分支；实际小文件由独立staging发布。下载路径只在内盘，不读写HIKVISION。

解析器准备命令（本轮已执行，不需重复）：

```text
/usr/bin/python3 -m pip install --target /home/jian/livox_ws/p62_r2_hilti_exp18_g0_data/parser --no-deps --only-binary=:all: --retries 0 --timeout 30 pye57==0.4.5
```

此为兼容本机Python3.8的固定wheel，仅安装到独立内盘目录，无全局依赖替换；E57不存在，**没有执行E57解码或点云导出**。rosbag/Python/NumPy/YAML沿用已安装环境。
