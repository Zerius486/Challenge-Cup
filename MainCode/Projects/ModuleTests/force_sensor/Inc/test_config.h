#ifndef FORCE_TEST_CONFIG_H
#define FORCE_TEST_CONFIG_H

/* 0: request one sample periodically. 1: request the 1 kHz stream once. */
#ifndef FORCE_TEST_CONTINUOUS_MODE
#define FORCE_TEST_CONTINUOUS_MODE 0
#endif
#define FORCE_TEST_REQUEST_PERIOD_MS 100U
#define FORCE_TEST_STALE_TIMEOUT_MS 1000U

/* 0 follows the manual's DA 0F 49 40 example; 1 is literal big-endian. */
#define FORCE_TEST_FLOAT_ORDER 0U

#endif
