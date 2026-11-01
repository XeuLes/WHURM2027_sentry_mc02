#pragma once

/* Hardware placeholders: fill these before enabling either board.
 * Encoder midpoints are RAW counts [0,8191], NOT driver angle_rad (relative
 * to the first received frame). Directions map motor rotation to joint axes.
 * CAN pointers / MotorType names are expanded only in C++ application code.
 */
/* Use the same BOARD_ROLE supplied by board-chassis / board-gimbal as the
 * board-link module. Keep the two controllers in their independent files.
 * Explicit -D overrides remain available for disabling an axis/application. */
#ifndef GIMBAL_BIG_YAW_ENABLE
#define GIMBAL_BIG_YAW_ENABLE (BOARD_ROLE == 1)
#endif
#ifndef GIMBAL_SMALL_ENABLE
#define GIMBAL_SMALL_ENABLE (BOARD_ROLE == 2)
#endif

#if defined(TUNING_BENCH) && TUNING_BENCH
#define GIMBAL_CONTROL_ENABLED 0
#else
#define GIMBAL_CONTROL_ENABLED (GIMBAL_BIG_YAW_ENABLE || GIMBAL_SMALL_ENABLE)
#endif
#define GIMBAL_BIG_YAW_TYPE MotorType::DjiGm6020
#define GIMBAL_BIG_YAW_CAN (&hfdcan2)
#define GIMBAL_BIG_YAW_ID 1U
#define GIMBAL_BIG_YAW_DIRECTION 1
#define GIMBAL_BIG_YAW_RATIO 1.0f
#define GIMBAL_BIG_YAW_MID_ECD 0U

#define GIMBAL_YAW_TYPE MotorType::DjiGm6020
#define GIMBAL_YAW_CAN (&hfdcan2)
#define GIMBAL_YAW_ID 2U
#define GIMBAL_YAW_DIRECTION 1
#define GIMBAL_YAW_RATIO 1.0f
#define GIMBAL_YAW_MID_ECD 0U
#define GIMBAL_PITCH_TYPE MotorType::DjiGm6020
#define GIMBAL_PITCH_CAN (&hfdcan2)
#define GIMBAL_PITCH_ID 3U
#define GIMBAL_PITCH_DIRECTION (-1)
#define GIMBAL_PITCH_RATIO 1.0f
#define GIMBAL_PITCH_MID_ECD 0U

/* All angles rad, angular velocities rad/s; controller output is DJI raw
 * command (GM6020 voltage command, NOT amperes). Gains are placeholders.
 * Parameter order: J,K,c,c1,c2,p,q,beta,epsilon. */
#define GIMBAL_BIG_YAW_SMC_PARAMS {35.0f,1500.0f,6.0f,0,0,0,0,0,0.001f}
#define GIMBAL_YAW_SMC_PARAMS {60.0f,160.0f,20.0f,0,0,0,0,0,0.001f}
#define GIMBAL_PITCH_SMC_PARAMS {20.0f,750.0f,0,60.0f,0.05f,0,0,0,0.001f}
#define GIMBAL_BIG_YAW_OUTPUT_MAX 10000.0f
#define GIMBAL_YAW_OUTPUT_MAX 15000.0f
#define GIMBAL_PITCH_OUTPUT_MAX 15000.0f
#define GIMBAL_SMC_SAT_LIMIT 1.0f
#define GIMBAL_POSITION_EPS 0.0001f
#define GIMBAL_INTEGRAL_MAX 0.1f

#define GIMBAL_PERIOD_MS 1U
#define GIMBAL_IMU_TIMEOUT_MS 20U
#define GIMBAL_AIM_TIMEOUT_MS 200U
#define GIMBAL_INIT_TIMEOUT_MS 3000U
#define GIMBAL_INIT_TOLERANCE_RAD 0.03f
/* Mechanical limits must be measured; defaults are placeholders. */
#define GIMBAL_SMALL_YAW_MIN_RAD (-1.5f)
#define GIMBAL_SMALL_YAW_MAX_RAD 1.5f
#define GIMBAL_PITCH_MIN_RAD (-0.4f)
#define GIMBAL_PITCH_MAX_RAD 0.4f
#define GIMBAL_FOLLOW_ENTER_RAD 1.2f
#define GIMBAL_FOLLOW_EXIT_RAD 0.05f
#define GIMBAL_FOLLOW_GAIN 2.0f
#define GIMBAL_FOLLOW_MAX_RAD_S 1.0f
#define GIMBAL_MANUAL_YAW_RAD_S 1.0f
#define GIMBAL_MANUAL_PITCH_RAD_S 0.5f
#define GIMBAL_SCAN_YAW_RAD_S 0.3f
#define GIMBAL_SCAN_PITCH_AMPLITUDE_RAD 0.15f
#define GIMBAL_SCAN_PITCH_PERIOD_S 4.0f
#define GIMBAL_RC_DEADZONE 0.02f
/* Existing USB command has no target-valid flag. Enable only once the host
 * contract is confirmed: absolute local IMU yaw/pitch, radians, streamed
 * only while tracking a valid target. Equal consecutive targets stay valid. */
#ifndef GIMBAL_AUTO_AIM_ENABLE
#define GIMBAL_AUTO_AIM_ENABLE 0
#endif
#define GIMBAL_AIM_ANGLE_TO_RAD 1.0f
