# UART 通讯测试

烧录 `output/standalone_tests/uart_communication/UART_Communication_Test.hex`。USART6 使用 PC6/TX、PC7/RX、`460800 8N1`；直接测试时在 PC6 与 PC7 之间做 TTL 回环，或通过 3.3 V RS485 收发器连接一个会回送数据的 USB-RS485 适配器。RS485 方案还要将 PB0 接 DE 与 `/RE`。

通过标准：固件每 500 ms 发送 `UART_TEST,<counter>`，连续运行 10 s 后 TX 和 RX 行计数都持续增加、`g_uart_rx_errors=0`、LED1 持续翻转且 LED2 熄灭。断开回环后约 2 s 内 LED2 点亮，恢复连接后计数继续增加。
