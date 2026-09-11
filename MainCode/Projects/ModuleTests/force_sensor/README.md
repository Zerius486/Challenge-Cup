# 六维力传感器测试工程

本工程只验证中科米点六维力传感器的 RS485 通讯和数据解析。启动后先发送停止连续输出，再默认以 10 Hz 发送单次读取命令。

## 接线

- PC6 USART6_TX -> 外置 3.3 V RS485 收发器 DI
- PC7 USART6_RX <- 收发器 RO
- PB0 -> DE 与 `/RE` 并联
- 收发器 A/B -> 传感器 A/B，逻辑侧共地
- USART6 固定 460800 baud、8N1；传感器使用独立 9--24 V 电源

板载 USART2 RS485 与 Ethernet 的 PA2/MDIO 冲突，所以本工程继续使用 USART6 和外置收发器。

## 调试变量

- `g_force_fx/fy/fz/mx/my/mz`：最近六维数据
- `g_force_frame_count`、`g_force_last_sample_ms`：有效帧统计
- `g_force_non_sample_frame_count`：合法但不是六维测量的回复
- `g_force_parse_error_count`：协议或数值解析错误
- `g_force_io_error_count`：UART 发送/恢复错误
- `g_force_uart_restart_count`：USART 错误后 DMA 自动恢复次数
- `g_force_valid`、`g_force_stale`：当前有效性

LED0 心跳闪烁，LED1 每收到一帧有效测量翻转；超过 1 s 无有效帧或 UART 出错时 LED2 点亮。

## 配置

`Inc/test_config.h` 中：

- `FORCE_TEST_CONTINUOUS_MODE=0`：周期单次读取，便于首次排障。
- 改为 `1`：启动传感器 1 kHz 连续输出。
- 默认浮点顺序遵循手册线例 `DA 0F 49 40 -> 3.1415925`；若实物数值明显异常，再以已知载荷验证 `FORCE_TEST_FLOAT_ORDER=1`，不要仅凭表格文字切换。
