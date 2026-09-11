# UDP 通信测试

烧录 `UDP_Communication_Test.hex`，电脑有线网卡设置为 `192.168.1.20/24`，连接开发板网口。运行 `python tools/udp_client.py --host 192.168.1.10 hello` 或 `Projects/ModuleTests/udp_communication/tools/udp_echo_test.py`。

通过标准：主机发送的每个 UDP 数据报都收到相同内容，长度和字节完全一致；开发板 LED0 心跳正常，LED1 随收包翻转，LED2 不亮。连续发送 1000 个数据报无异常丢包或死机。
