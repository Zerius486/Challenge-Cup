#ifndef VOFA_FIREWATER_H
#define VOFA_FIREWATER_H

#include <stdint.h>

/**
 * @brief 周期发送一帧 VOFA+ FireWater CSV 调试数据。
 * @param now_ms 当前系统毫秒计数。
 */
void vofa_firewater_poll(uint32_t now_ms);

#endif
