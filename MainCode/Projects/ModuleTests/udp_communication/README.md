# UDP 通讯独立测试工程

本工程不运行电机或力传感器协议，只验证启明欣欣开发板的 8 MHz MCU HSE、LAN8720A 的 25 MHz PHY 晶振、RMII、lwIP 和 UDP 收发链路。

默认地址为 `192.168.1.10/24`，UDP 端口 `20001`。板端原样回显 1--512 字节负载。

## 测试步骤

1. 烧录 `UDP_Communication_Test.hex`，用网线连接电脑与开发板。
2. 将电脑有线网卡设为例如 `192.168.1.20/24`，先确认没有相同 IP 的设备。
3. 在 `MainCode` 目录执行：

```powershell
python .\Projects\ModuleTests\udp_communication\tools\udp_echo_test.py --count 100 --size 64
```

脚本逐包验证来源和内容，并报告 received、lost、corrupt 与 RTT min/avg/p95/max。全部正确时退出码为 0。

## 调试变量与 LED

- `g_udp_link_up`：PHY 链路状态
- `g_udp_rx_packets/tx_packets`、`rx_bytes/tx_bytes`：流量计数
- `g_udp_error_count`：分配、复制、发送或超长报文错误
- `g_udp_last_peer_ipv4/port`、`g_udp_last_payload_length`：最近客户端

LED0 心跳闪烁，LED1 表示 PHY link up，LED2 表示 UDP 处理错误。若 link 不起，优先检查 PE2 PHY reset、RMII 50 MHz REF_CLK、网线和供电，不要先修改 UDP 代码。
