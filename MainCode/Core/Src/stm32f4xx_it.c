#include "board.h"
#include "stm32f4xx_hal.h"

void NMI_Handler(void) {}

void HardFault_Handler(void) {
  while (1) {
  }
}

void MemManage_Handler(void) {
  while (1) {
  }
}

void BusFault_Handler(void) {
  while (1) {
  }
}

void UsageFault_Handler(void) {
  while (1) {
  }
}

void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void) {
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void CAN1_RX0_IRQHandler(void) {
  HAL_CAN_IRQHandler(&hcan1);
}

void CAN2_RX0_IRQHandler(void) {
  HAL_CAN_IRQHandler(&hcan2);
}

void CAN1_SCE_IRQHandler(void) {
  HAL_CAN_IRQHandler(&hcan1);
}

void CAN2_SCE_IRQHandler(void) {
  HAL_CAN_IRQHandler(&hcan2);
}

void DMA2_Stream1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
}

void USART6_IRQHandler(void) {
  HAL_UART_IRQHandler(&huart6);
}
