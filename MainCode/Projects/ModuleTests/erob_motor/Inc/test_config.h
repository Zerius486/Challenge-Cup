#ifndef EROB_TEST_CONFIG_H
#define EROB_TEST_CONFIG_H

#define EROB_TEST_CAN_BUS 1U
#define EROB_TEST_NODE_ID 11U
#define EROB_TEST_STATUS_PERIOD_MS 100U

/* Keep at 0 until CAN feedback and the independent emergency stop are proven. */
#ifndef EROB_TEST_ENABLE_MOTION
#define EROB_TEST_ENABLE_MOTION 0
#endif

/* Absolute encoder units used only when EROB_TEST_ENABLE_MOTION is 1. */
#define EROB_TEST_TARGET_POSITION_PLUS 1000
#define EROB_TEST_PROFILE_VELOCITY_PLUS_S 1000U
#define EROB_TEST_COMMAND_STEP_MS 50U
#define EROB_TEST_ENABLE_SETTLE_MS 500U
#define EROB_TEST_HOLD_MS 1000U

#endif
