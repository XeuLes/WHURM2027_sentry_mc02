#pragma once

/* 大 Yaw 电机参数供副板移植，主控只使用目标规划参数。 */
#define GIMBAL_BIG_YAW_CAN (&hfdcan2)
#define GIMBAL_BIG_YAW_ID 1U
#define GIMBAL_BIG_YAW_MASTER_ID 0x51U
#define GIMBAL_BIG_YAW_DIRECTION 1
#define GIMBAL_BIG_YAW_MID_RAD 0.0f

/* All three gimbal-board motors share this bus. GM6020 midpoint is raw ECD. */
#define GIMBAL_SMALL_CAN (&hfdcan1)
#define GIMBAL_YAW_ID 2U
#define GIMBAL_YAW_DIRECTION 1
#define GIMBAL_YAW_MID_ECD 0U
#define GIMBAL_PITCH_ID 3U
#define GIMBAL_PITCH_MASTER_ID 0x53U
#define GIMBAL_PITCH_DIRECTION (-1)
#define GIMBAL_PITCH_MID_RAD 0.0f

/* DM protocol positions are output-shaft radians. The motor handles its
 * internal gearbox; these gimbal axes have no additional reduction. */
#define GIMBAL_DM_POS_MAX_RAD 12.5f
#define GIMBAL_DM_VEL_MAX_RAD_S 30.0f
#define GIMBAL_DM_TORQUE_MAX_NM 10.0f
#define GIMBAL_DM_COMMAND_PERIOD_MS 20U

/* SMC order: J,K,c,c1,c2,p,q,beta,epsilon. DM output is N*m.
 * GM6020 output is a raw voltage command. DM gains/limits are placeholders. */
#define GIMBAL_BIG_YAW_SMC_PARAMS {0.03f,20.0f,6.0f,0,0,0,0,0,0.001f}
#define GIMBAL_YAW_SMC_PARAMS {60.0f,160.0f,20.0f,0,0,0,0,0,0.001f}
#define GIMBAL_PITCH_SMC_PARAMS {0.02f,20.0f,0,6.0f,0.05f,0,0,0,0.001f}
#define GIMBAL_BIG_YAW_OUTPUT_MAX_NM 1.0f
#define GIMBAL_YAW_OUTPUT_MAX 15000.0f
#define GIMBAL_PITCH_OUTPUT_MAX_NM 1.0f
#define GIMBAL_SMC_SAT_LIMIT 1.0f
#define GIMBAL_POSITION_EPS 0.0001f
#define GIMBAL_INTEGRAL_MAX 0.1f

#define GIMBAL_PERIOD_MS 1U
#define GIMBAL_IMU_TIMEOUT_MS 20U
#define GIMBAL_AIM_TIMEOUT_MS 200U
#define GIMBAL_AIM_LOST_MS 1000U
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
/* Bound the moving big-yaw reference if the mechanism falls behind. */
#define GIMBAL_BIG_YAW_MAX_LEAD_RAD 0.5f
#define GIMBAL_SCAN_PITCH_AMPLITUDE_RAD 0.15f
#define GIMBAL_SCAN_PITCH_PERIOD_S 4.0f
#define GIMBAL_RC_DEADZONE 0.02f
#define GIMBAL_ROLL_ID 4U
#define GIMBAL_ROLL_MASTER_ID 0x54U
#define GIMBAL_ROLL_DIRECTION 1
#define GIMBAL_ROLL_MID_RAD 0.0f
#define GIMBAL_ROLL_SMC_PARAMS {0.03f,20.0f,6.0f,0,0,0,0,0,0.001f}
#define GIMBAL_ROLL_OUTPUT_MAX_NM 1.0f
#define GIMBAL_ROLL_MIN_RAD (-1.7f)
#define GIMBAL_ROLL_MAX_RAD 1.7f
#define GIMBAL_ROLL_UPRIGHT_RAD 0.0f
#define GIMBAL_PASS_ROLL_RAD 1.5707963268f
#define GIMBAL_PASS_PITCH_RAD 0.0f
/* ET08A: rc.s[2] is channel 5, already forwarded by board-link.
 * Leave rc.s[0]/[1] (SB/SC, channels 7/8) to enable/mode selection. */
#ifndef GIMBAL_PASS_SWITCH_INDEX
#define GIMBAL_PASS_SWITCH_INDEX 2U
#endif
#ifndef GIMBAL_PASS_SWITCH_POSITION
#define GIMBAL_PASS_SWITCH_POSITION RC_SW_UP
#endif
#define GIMBAL_PASS_JOINT_SPEED_RAD_S 0.5f
#define GIMBAL_PASS_JOINT_ACCEL_RAD_S2 1.0f
#define GIMBAL_PASS_BIG_YAW_SPEED_RAD_S 3.0f
#define GIMBAL_PASS_BIG_YAW_ACCEL_RAD_S2 12.0f
#define GIMBAL_PASS_BIG_YAW_SETTLE_MS 80U
#define GIMBAL_ROLL_SPEED_RAD_S 0.5f
#define GIMBAL_ROLL_ACCEL_RAD_S2 1.0f
/* A 90-degree fold/unfold reference takes about 0.48 s from rest. */
#define GIMBAL_PASS_ROLL_SPEED_RAD_S 6.5f
#define GIMBAL_PASS_ROLL_ACCEL_RAD_S2 27.0f
#define GIMBAL_PASS_ROLL_SETTLE_MS 80U
#define GIMBAL_PASS_POSITION_TOL_RAD 0.03f
#define GIMBAL_PASS_SPEED_TOL_RAD_S 0.05f
#define GIMBAL_PASS_SETTLE_MS 200U
#define GIMBAL_PASS_PHASE_TIMEOUT_MS 10000U
