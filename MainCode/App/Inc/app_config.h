#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Network settings inherited from UWV, with the third octet corrected to 1. */
#define APP_IP_ADDR0 192U
#define APP_IP_ADDR1 168U
#define APP_IP_ADDR2 1U
#define APP_IP_ADDR3 10U
#define APP_NETMASK_ADDR0 255U
#define APP_NETMASK_ADDR1 255U
#define APP_NETMASK_ADDR2 255U
#define APP_NETMASK_ADDR3 0U
#define APP_GATEWAY_ADDR0 192U
#define APP_GATEWAY_ADDR1 168U
#define APP_GATEWAY_ADDR2 1U
#define APP_GATEWAY_ADDR3 1U
#define APP_UDP_PORT 20001U
#define APP_TELEMETRY_PERIOD_MS 20U
#define APP_LEGACY_CAN_RX_PORT 19999U
#define APP_LEGACY_CAN_TX_PORT 20000U
#define APP_LEGACY_RAW_CAN_BUS 2U

#define APP_EROB_CAN_BUS 1U
#define APP_ROBSTRIDE_CAN_BUS 2U
#define APP_ROBSTRIDE_MASTER_ID 0xFDU
#define APP_EROB_STATUS_POLL_PERIOD_MS 20U
#define APP_EROB_STATUS_POLL_BUDGET 2U
#define APP_EROB_ENABLE_SETTLE_MS 500U
#define APP_EROB_STATUS_STALE_MS 250U
#define APP_ROBSTRIDE_FEEDBACK_STALE_MS 250U
#define APP_ROBSTRIDE_PARAMETER_GAP_MS 1U

/* Fixed arm wiring and commissioning values.  J1..J4 are eRob on CAN1;
 * J5/J6 are RobStride on CAN2.  All six motors use the same positive
 * rotation convention, so a complete arm reversal only changes this value. */
#define APP_ARM_J1_NODE_ID 11U
#define APP_ARM_J2_NODE_ID 12U
#define APP_ARM_J3_NODE_ID 13U
#define APP_ARM_J4_NODE_ID 14U
#define APP_ARM_J5_NODE_ID 15U
#define APP_ARM_J6_NODE_ID 16U
#define APP_ARM_DIRECTION 1.0F
#define APP_EROB_ENCODER_COUNTS_PER_REV 524288.0F
#define APP_EROB_ENCODER_MIDPOINT 262144.0F
#define APP_EROB_PROFILE_VELOCITY_COUNTS_S 10000.0F
#define APP_ROBSTRIDE_J5_MODEL 1U /* RS01; confirm against the installed unit. */
#define APP_ROBSTRIDE_J6_MODEL 1U
#define APP_ROBSTRIDE_PROFILE_SPEED_RAD_S 1.0F
#define APP_ARM_COMMAND_TIMEOUT_MS 3000U
#define APP_ARM_POSITION_SETTLE_MS 800U

/* JZH first field is interpreted as signed milliradians.  Positive position
 * closes the gripper; negative position opens it. */
#define APP_GRIPPER_MIN_POSITION_RAD (-1.5F)
#define APP_GRIPPER_MAX_POSITION_RAD 1.5F
#define APP_GRIPPER_CLOSE_DIRECTION_THRESHOLD_RAD 0.0F
#define APP_GRIPPER_FORCE_THRESHOLD_N 5.0F
#define APP_GRIPPER_MAX_CURRENT_MA 20000U

/* Submission build: commissioning is treated as complete.  Set this to 0
 * for a bench build that must reject remote position commands. */
#ifndef APP_REMOTE_MOTION_ALLOWED
#define APP_REMOTE_MOTION_ALLOWED 1
#endif
#ifndef APP_LEGACY_RAW_CAN_ALLOWED
#define APP_LEGACY_RAW_CAN_ALLOWED 0
#endif
#define APP_COMMAND_ARM_OPTION 0x8000U

#define APP_FORCE_UART_BAUD 460800U
#define APP_FORCE_RX_DMA_SIZE 4096U
#define APP_FORCE_STALE_TIMEOUT_MS 1000U
/* 0: follows the manual line example; 1: literal big-endian table wording. */
#define APP_FORCE_FLOAT_ORDER 0U

/* USART3/PB10 is the optional VOFA+ FireWater diagnostic output. */
#define APP_VOFA_UART_BAUD 460800U
#define APP_VOFA_PERIOD_MS 100U
#define APP_VOFA_TX_BUFFER_SIZE 768U

#define APP_MAX_TRACKED_MOTORS 16U
#define APP_DEFAULT_COMMAND_TIMEOUT_MS 200U
#define APP_STOP_RETRY_PERIOD_MS 20U
#define APP_CAN_MIRROR_TX_BUDGET 2U

#endif
