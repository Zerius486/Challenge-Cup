# UART 通讯独立测试工程

本工程只验证开发板 USART6 的 TX、RX、DMA 接收和连续收发，不解析力传感器协议。USART6 使用主工程相同的 `460800 8N1` 参数，连接为 PC6/TX、PC7/RX；PB0 仍作为外接 RS485 收发器的 DE 与 `/RE` 方向控制。

## 接线和运行

最简单的验证方式是在 PC6 与 PC7 之间做 TTL 回环，然后烧录 `UART_Communication_Test.hex`。若使用 RS485 收发器，则将 PC6/PC7 接 DI/RO、PB0 接 DE 与 `/RE`，在总线另一端接 USB-RS485 适配器或第二个节点，并让对端回送数据。逻辑侧必须使用 3.3 V；不要把 RS485 A/B 直接接到 MCU 引脚。

固件每 500 ms 发送一行 `UART_TEST,<counter>`。回环或对端回送后，LED1 翻转，接收行数和最后一行长度递增。LED0 是心跳；LED2 在没有有效接收、DMA 重启或发送/接收错误时点亮。

## 调试变量和通过标准

- `g_uart_tx_count`：成功发送的测试行数；
- `g_uart_rx_bytes`、`g_uart_rx_lines`：接收字节和完整行计数；
- `g_uart_last_rx_ms`、`g_uart_last_line_length`：最近一行的时间和长度；
- `g_uart_rx_errors`、`g_uart_stale`：错误和超时状态。

连续运行 10 s 后，发送计数应持续增长，回环/对端收到的完整行数也应持续增长，`g_uart_rx_errors=0`，LED2 熄灭。拔掉回环线后 LED2 应在约 2 s 内点亮；恢复连接后应继续接收。

构建产物为 `output/standalone_tests/uart_communication/UART_Communication_Test.hex`，构建和烧录命令见上级 [模块测试说明](../README.md)。
