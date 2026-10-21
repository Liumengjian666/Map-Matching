# 唯一实际阻塞与用户操作

状态：`ASSET_ACCESS_REQUIRES_USER_ACTION`。

存储门已通过。本机Alipan访问在TLS阶段出现EOF，未取得HTTP响应或真实bag地址；不能据此断言官方分享失效或一定需要登录。没有禁用TLS校验、使用代理绕过、提取登录凭据或调用非官方镜像。

## 固定官方来源

[官方固定版本序列表](https://github.com/sjtuyinjie/M3DGR/blob/e0cf7d59c9a5a3df515624034698d976abc26549/README.md)，只选择Wheel-float01行。

- bag：[Alipan](https://www.alipan.com/s/aVQ24Hm4Xcm)；[OneDrive](https://1drv.ms/u/c/2b4bfc0edf421186/EYYRQt8O_EsggCvoDwAAAAABPfiAyZaMZKeDieNO6DWjNQ?e=UkQKrT)。
- 对应GT：[Alipan](https://www.alipan.com/s/ZsdFZF1ueRW)；[OneDrive](https://1drv.ms/t/c/2b4bfc0edf421186/EYYRQt8O_EsggCsmEAAAAAABrgAChsxs_7wLR3YhrvfGGw?e=1dSi4p)。

这些链接来自同一固定官方序列行，不是新镜像。OneDrive链接本轮未重新请求。

## 请完成一次人工获取

1. 在你自己的浏览器打开官方入口；如果网站要求登录，使用你自己的合法账号或官方客户端。不要向执行AI提供密码、Cookie或其他认证秘密。
2. 只下载Wheel-float01原始bag及该行对应GT。开始前查看网站实际文件大小：bag必须小于6GiB、GT与辅助文件合计小于1GiB，并保留20GiB安全余量；异常大或进入预算末64MiB时停止。不要下载整个数据集，不要自行解压大目录。
3. bag放到：

   `/home/jian/livox_ws/p62_r14_m3dgr_data/resume_20261021/bag/Wheel-float01.bag`

   GT保留官方原始文件名，放到：

   `/home/jian/livox_ws/p62_r14_m3dgr_data/resume_20261021/gt/`

   两个目录已创建，当前为空。不要用网页另存为文件冒充bag。
4. 回传实际文件路径、来源及网站显示大小；或者提供网站合法生成、可直接读取的真实载荷URL。不要先更改GT列、时间、坐标或外参。

bag预期SHA256：

`366171a5f1d0c1e7bd602e2b664d85dbd89dd08570c4165142ef9849029c1cf6`

可以只读核验：

```bash
sha256sum /home/jian/livox_ws/p62_r14_m3dgr_data/resume_20261021/bag/Wheel-float01.bag
```

取得文件不等于允许跳过下一道门。执行AI必须先核验完整bag格式、GT身份、T_W_B/T_B_L/T_O_B、时间及wheel预测合同，才能按原R14合同建图和运行一次实验。本轮已停止，不自动重试或进入R15。
