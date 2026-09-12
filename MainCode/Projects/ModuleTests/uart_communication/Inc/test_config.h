#ifndef UART_TEST_CONFIG_H
#define UART_TEST_CONFIG_H

/* USART6 is fixed to the main project's 460800 8N1 setting. */
#define UART_TEST_PERIOD_MS 500U
#define UART_TEST_STALE_TIMEOUT_MS 2000U
#define UART_TEST_RX_BUFFER_SIZE 256U
#define UART_TEST_LINE_SIZE 96U

#endif
