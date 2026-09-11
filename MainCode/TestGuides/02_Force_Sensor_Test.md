# 力传感器测试

烧录 `Force_Sensor_Test.hex`。PC6 接收发器 DI、PC7 接 RO、PB0 接 DE 和 `/RE`，使用 3.3 V TTL 逻辑；传感器独立供电并共地。

通过标准：每 100 ms 至少解析一帧，Fx/Fy/Fz/Mx/My/Mz 数值稳定且计数递增；人为断开再接回 RS485 后，DMA 在 1 s 内恢复接收，错误标志可观察且不死机。连续流模式还应保持 1 kHz 数据链路。
