# 清理后的独立证据归档

原始执行目录：`/home/jian/livox_ws/rosbag_large_file_cleanup_20261021_Rwmrar`。

清理和保护验收先完成，随后只将小型报告、CSV、源码及审计回执复制到独立Git工作区：

`/home/jian/livox_ws/rosbag_large_file_cleanup_archive_ws`

仓库：`https://github.com/Liumengjian666/Map-Matching`。

新分支：`maintenance/rosbag-large-file-cleanup-20261021`。

归档基线：`9125c4f9b313994b08572bec143e20b7b81d1d3b`。

本文件是后续归档记录，不追溯修改原始CLEANUP_REPORT/results中“清理完成时尚无Git提交”的事实。最终归档提交由外部回传完整SHA，不将归档提交冒充实际删除时的CODE_SHA。删除时程序未处于本轮Git分支，源码内容身份由原checksums.sha256保存。

原checksums.sha256覆盖38项清理产物，在复制前后均须验证；本新增归档说明不在原清单中，由Git对象保存身份。没有复制bag、PCD或其他大型传感器资产。

安全测试终端回执摘录（不是重新运行）：18项，Ran 18 tests in 0.048s，OK。测试源在test_safety.py；已记录首轮失败及修正。真实删除10文件，df净增加63.780452728271484 GiB；清理结束时可用91418374144 bytes。随后几MiB的归档/Git写入会轻微影响当前df读数，不能将其与上述验收时点混淆。

本地归档，不push；未清理、重写或删除任何Git分支或历史，未修改稳定机器狗工程。原始执行目录保留。请勿再次执行safe_delete.py --apply：已有单次执行标记会拒绝重跑。
