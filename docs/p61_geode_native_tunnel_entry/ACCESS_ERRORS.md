# 访问与工具失败证据

## 直接决定停止的事件

目标bag大文件确认页正常可访问。一次正常确认请求后：
HTTP200、text/html、2040bytes、Quota exceeded、bag magic失败。
详见access_evidence/access_receipt.json和response_body.bin。
没有再次确认、自动重试、恢复续传或绕过配额。

共享gamma配置和Shield_tunnel1.txt下载各一次也返回Quota exceeded HTML。
它们不是YAML／GT有效载荷，均未用于算法。

## 非决定性的小型资料访问问题

- GitHub树API返回403，原文指明API rate limit exceeded；保存为official/repository_tree.json及HTTP回执。
  不将该JSON错误对象作为成功文件列表，没有配置凭据或循环请求。
- README中所述script/gamma2gt_leica.py在固定版本按该路径读取返回404。
  错误体另存official/gamma2gt_leica_404.txt，不冒充GT exporter源码。
- 一次稀疏clone工具调用失败：

```text
git clone --depth 1 --filter=blob:none --sparse https://github.com/PengYu-Team/GEODE_dataset.git /home/jian/geode_p61_data/official_repo
fatal: cannot change to 'https://github.com/PengYu-Team/GEODE_dataset.git': 没有那个文件或目录
error: 无法初始化稀疏检出
```

Git版本2.25.1；没有创建可用checkout，具体工具根因未确证。
没有修复全局Git设置或反复clone。小型官方资料通过固定commit raw URL成功读取，
故该工具错误不是bag获取失败原因，也不影响已归档YAML身份。

## 资源边界

仅元数据、小型官方文件和HTML错误证据，系统内盘约73GiB仍可用。
没有下载4.5GB传感器包、地图、其他序列或整套GEODE。
没有读取传感器／GT有效载荷，没有运行ROS、NDT、Deskew、EKF、OOSM或Coupled。
