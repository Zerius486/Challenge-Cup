#include "board.h"

#include <string.h>

#include "app_bridge.h"
#include "app_config.h"
#include "main.h"
#include "udp_transport.h"

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
UART_HandleTypeDef huart6;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart6_rx;
static volatile bool force_uart_rx_faulted;

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

static void gpio_init(void) {
  GPIO_InitTypeDef init = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, ETH_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, LED0_Pin | LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOG, LED2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, FORCE_RS485_DE_Pin, GPIO_PIN_RESET);

  init.Pin = ETH_RESET_Pin | LED0_Pin | LED1_Pin;
  init.Mode = GPIO_MODE_OUTPUT_PP;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &init);

  init.Pin = LED2_Pin;
  HAL_GPIO_Init(GPIOG, &init);

  init.Pin = FORCE_RS485_DE_Pin;
  init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &init);
}

static bool can_init(CAN_HandleTypeDef *handle, CAN_TypeDef *instance) {
  CAN_FilterTypeDef filter = {0};

  handle->Instance = instance;
  handle->Init.Prescaler = 3U;
  handle->Init.Mode = CAN_MODE_NORMAL;
  handle->Init.SyncJumpWidth = CAN_SJW_1TQ;
  handle->Init.TimeSeg1 = CAN_BS1_9TQ;
  handle->Init.TimeSeg2 = CAN_BS2_4TQ;
  handle->Init.TimeTriggeredMode = DISABLE;
  handle->Init.AutoBusOff = ENABLE;
  handle->Init.AutoWakeUp = DISABLE;
  handle->Init.AutoRetransmission = ENABLE;
  handle->Init.ReceiveFifoLocked = DISABLE;
  handle->Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(handle) != HAL_OK) {
    return false;
  }

  filter.FilterBank = (instance == CAN1) ? 0U : 14U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0U;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0U;
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;
  return HAL_CAN_ConfigFilter(handle, &filter) == HAL_OK;
}

static bool uart_init(void) {
  huart6.Instance = USART6;
  huart6.Init.BaudRate = APP_FORCE_UART_BAUD;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  return HAL_UART_Init(&huart6) == HAL_OK;
}

static bool vofa_uart_init(void) {
  huart3.Instance = USART3;
  huart3.Init.BaudRate = APP_VOFA_UART_BAUD;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  return HAL_UART_Init(&huart3) == HAL_OK;
}

/** @brief 初始化主工程所需的全部 CAN/UART/GPIO 外设。 */
bool board_peripherals_init(void) {
  return board_peripherals_init_selected(BOARD_PERIPHERAL_ALL);
}

/** @brief 按测试工程选择的掩码初始化外设。 */
bool board_peripherals_init_selected(uint32_t peripherals) {
  bool ok = true;

  if ((peripherals & ~BOARD_PERIPHERAL_ALL) != 0U) {
    return false;
  }
  gpio_init();
  if ((peripherals & BOARD_PERIPHERAL_CAN1) != 0U) {
    ok = can_init(&hcan1, CAN1) && ok;
  }
  if ((peripherals & BOARD_PERIPHERAL_CAN2) != 0U) {
    ok = can_init(&hcan2, CAN2) && ok;
  }
  if ((peripherals & BOARD_PERIPHERAL_FORCE_UART) != 0U) {
    ok = uart_init() && ok;
  }
  if ((peripherals & BOARD_PERIPHERAL_VOFA_UART) != 0U) {
    ok = vofa_uart_init() && ok;
  }
  return ok;
}

/** @brief 启动全部已初始化的 CAN 接收中断。 */
bool board_peripherals_start(void) {
  return board_peripherals_start_selected(BOARD_PERIPHERAL_ALL);
}

/** @brief 启动指定掩码中的 CAN 外设和接收通知。 */
bool board_peripherals_start_selected(uint32_t peripherals) {
  uint32_t notifications = CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF |
                           CAN_IT_ERROR;
  bool ok = true;

  if ((peripherals & ~BOARD_PERIPHERAL_ALL) != 0U) {
    return false;
  }
  if ((peripherals & BOARD_PERIPHERAL_CAN1) != 0U) {
    ok = HAL_CAN_Start(&hcan1) == HAL_OK &&
         HAL_CAN_ActivateNotification(&hcan1, notifications) == HAL_OK && ok;
  }
  if ((peripherals & BOARD_PERIPHERAL_CAN2) != 0U) {
    ok = HAL_CAN_Start(&hcan2) == HAL_OK &&
         HAL_CAN_ActivateNotification(&hcan2, notifications) == HAL_OK && ok;
  }
  return ok;
}

void board_phy_reset_release(void) {
  HAL_GPIO_WritePin(ETH_RESET_GPIO_Port, ETH_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(2U);
  HAL_GPIO_WritePin(ETH_RESET_GPIO_Port, ETH_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(50U);
}

static void led_pin(BoardLed led, GPIO_TypeDef **port, uint16_t *pin) {
  if (led == BOARD_LED_STATUS) {
    *port = LED0_GPIO_Port;
    *pin = LED0_Pin;
  } else if (led == BOARD_LED_NETWORK) {
    *port = LED1_GPIO_Port;
    *pin = LED1_Pin;
  } else {
    *port = LED2_GPIO_Port;
    *pin = LED2_Pin;
  }
}

void board_led_set(BoardLed led, bool on) {
  GPIO_TypeDef *port;
  uint16_t pin;
  led_pin(led, &port, &pin);
  HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void board_led_toggle(BoardLed led) {
  GPIO_TypeDef *port;
  uint16_t pin;
  led_pin(led, &port, &pin);
  HAL_GPIO_TogglePin(port, pin);
}

uint32_t board_critical_enter(void) {
  uint32_t state = __get_PRIMASK();
  __disable_irq();
  __DMB();
  return state;
}

void board_critical_exit(uint32_t state) {
  __DMB();
  __set_PRIMASK(state);
}

void board_delay_ms(uint32_t delay_ms) {
  HAL_Delay(delay_ms);
}

/** @brief 将统一 CanFrame 转换为 HAL 帧并发送。 */
bool board_can_send(uint8_t bus, const CanFrame *frame, uint32_t timeout_ms) {
  CAN_HandleTypeDef *handle;
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox;
  uint32_t start;

  if (frame == NULL || frame->dlc > 8U || (bus != 1U && bus != 2U)) {
    return false;
  }
  handle = (bus == 1U) ? &hcan1 : &hcan2;
  header.IDE = frame->is_extended ? CAN_ID_EXT : CAN_ID_STD;
  header.StdId = frame->is_extended ? 0U : frame->id;
  header.ExtId = frame->is_extended ? frame->id : 0U;
  header.RTR = frame->is_remote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
  header.DLC = frame->dlc;
  header.TransmitGlobalTime = DISABLE;

  start = HAL_GetTick();
  while (HAL_CAN_GetTxMailboxesFreeLevel(handle) == 0U) {
    if ((HAL_GetTick() - start) >= timeout_ms ||
        HAL_CAN_GetState(handle) == HAL_CAN_STATE_ERROR) {
      return false;
    }
  }
  return HAL_CAN_AddTxMessage(handle, &header, (uint8_t *)frame->data,
                              &mailbox) == HAL_OK;
}

bool board_force_uart_start_rx(uint8_t *buffer, uint16_t length) {
  if (buffer == NULL || length == 0U) {
    return false;
  }
  force_uart_rx_faulted = false;
  if (HAL_UART_Receive_DMA(&huart6, buffer, length) != HAL_OK) {
    force_uart_rx_faulted = true;
    return false;
  }
  return true;
}

bool board_force_uart_maintain_rx(uint8_t *buffer, uint16_t length,
                                  bool *restarted) {
  if (buffer == NULL || length == 0U || restarted == NULL) {
    return false;
  }
  *restarted = false;
  if (!force_uart_rx_faulted) {
    return true;
  }

  force_uart_rx_faulted = false;
  if (HAL_UART_Receive_DMA(&huart6, buffer, length) != HAL_OK) {
    force_uart_rx_faulted = true;
    return false;
  }
  *restarted = true;
  return true;
}

uint16_t board_force_uart_rx_index(uint16_t length) {
  uint16_t remaining;

  if (length == 0U) {
    return 0U;
  }
  remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
  if (remaining == 0U || remaining >= length) {
    return 0U;
  }
  return (uint16_t)(length - remaining);
}

bool board_force_uart_send(const uint8_t *data, uint16_t length,
                           uint32_t timeout_ms) {
  HAL_StatusTypeDef status;
  if (data == NULL || length == 0U) {
    return false;
  }
  HAL_GPIO_WritePin(FORCE_RS485_DE_GPIO_Port, FORCE_RS485_DE_Pin,
                    GPIO_PIN_SET);
  status = HAL_UART_Transmit(&huart6, (uint8_t *)data, length, timeout_ms);
  HAL_GPIO_WritePin(FORCE_RS485_DE_GPIO_Port, FORCE_RS485_DE_Pin,
                    GPIO_PIN_RESET);
  return status == HAL_OK;
}

bool board_vofa_uart_send(const uint8_t *data, uint16_t length,
                          uint32_t timeout_ms) {
  if (data == NULL || length == 0U) {
    return false;
  }
  return HAL_UART_Transmit(&huart3, (uint8_t *)data, length, timeout_ms) ==
         HAL_OK;
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *handle) {
  GPIO_InitTypeDef init = {0};

  init.Mode = GPIO_MODE_AF_PP;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  init.Alternate = GPIO_AF9_CAN1;

  if (handle->Instance == CAN1) {
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOD, &init);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  } else if (handle->Instance == CAN2) {
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_CAN2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    init.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    init.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &init);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef *handle) {
  GPIO_InitTypeDef init = {0};
  if (handle->Instance == USART3) {
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    init.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    init.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &init);
    return;
  }
  if (handle->Instance != USART6) {
    return;
  }

  __HAL_RCC_USART6_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  init.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  init.Mode = GPIO_MODE_AF_PP;
  init.Pull = GPIO_PULLUP;
  init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  init.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &init);

  hdma_usart6_rx.Instance = DMA2_Stream1;
  hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
  hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
  hdma_usart6_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK) {
    Error_Handler();
  }
  __HAL_LINKDMA(handle, hdmarx, hdma_usart6_rx);

  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  HAL_NVIC_SetPriority(USART6_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *handle) {
  CAN_RxHeaderTypeDef header;
  CanFrame frame;
  uint8_t bus = (handle->Instance == CAN1) ? 1U : 2U;

  while (HAL_CAN_GetRxFifoFillLevel(handle, CAN_RX_FIFO0) > 0U) {
    memset(&frame, 0, sizeof(frame));
    if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &header, frame.data) !=
        HAL_OK) {
      return;
    }
    frame.is_extended = header.IDE == CAN_ID_EXT;
    frame.is_remote = header.RTR == CAN_RTR_REMOTE;
    frame.id = frame.is_extended ? header.ExtId : header.StdId;
    frame.dlc = (uint8_t)header.DLC;
    app_bridge_on_can_frame(bus, &frame, HAL_GetTick());
    udp_transport_queue_can_from_isr(bus, &frame);
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *handle) {
  app_bridge_on_can_error((handle->Instance == CAN1) ? 1U : 2U,
                          HAL_CAN_GetError(handle));
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle) {
  if (handle->Instance == USART6) {
    force_uart_rx_faulted = true;
  }
}
