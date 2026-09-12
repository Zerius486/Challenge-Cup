# UDP 通信测试

烧录 `UDP_Communication_Test.hex`，电脑有线网卡设置为 `192.168.1.20/24`，连接开发板网口。在 `MainCode` 目录运行：

```powershell
python .\Projects\ModuleTests\udp_communication\tools\udp_echo_test.py --count 100 --size 64
```

也可以使用 `python .\tools\udp_client.py --host 192.168.1.10 hello` 做单次连通性检查。

通过标准：测试脚本报告 `lost=0`、`corrupt=0`，即每个 UDP 数据报都收到相同内容，长度和字节完全一致；开发板 LED0 心跳正常，LED1 在网线链路建立后点亮，LED2 不亮。需要压力测试时再将 `--count` 调大，结果以实际统计为准。
