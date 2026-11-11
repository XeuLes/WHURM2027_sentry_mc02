#ifndef DM_IMU_HPP
#define DM_IMU_HPP

#include <stdbool.h>
#include "core_linkage.h"
#include "stm32h7xx_hal.h"


#define ACCEL_CAN_MAX (235.2f)
#define ACCEL_CAN_MIN	(-235.2f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

#define CMD_READ 0
#define CMD_WRITE 1

typedef enum
{
	COM_USB=0,
	COM_RS485,
	COM_CAN,
	COM_VOFA

}imu_com_port_e;

typedef enum
{
	CAN_BAUD_1M=0,
	CAN_BAUD_500K,
	CAN_BAUD_400K,
	CAN_BAUD_250K,
	CAN_BAUD_200K,
	CAN_BAUD_100K,
	CAN_BAUD_50K,
	CAN_BAUD_25K
	
}imu_baudrate_e;

typedef enum 
{
	REBOOT_IMU=0,
	ACCEL_DATA,
	GYRO_DATA,
	EULER_DATA,
	QUAT_DATA,
	SET_ZERO,
	ACCEL_CALI,
	GYRO_CALI,
	MAG_CALI,
	CHANGE_COM,
	SET_DELAY,
	CHANGE_ACTIVE,
	SET_BAUD,
	SET_CAN_ID,
	SET_MST_ID,
	DATA_OUTPUT_SELECTION,
	SAVE_PARAM=254,
	RESTORE_SETTING=255
}reg_id_e;



typedef struct
{
	uint16_t can_id;
	uint16_t mst_id;
	
	FDCAN_HandleTypeDef *can_handle;
	
	float pitch;
	float roll;
	float yaw;

	float gyro[3];
	float accel[3];
	
	float q[4];

	float cur_temp;

	// 与设备上报位图一致：bit0 四元数，bit1 欧拉角，bit2 角速度，bit3 加速度和温度。
	uint8_t received_mask;
	uint32_t last_rx_ms; // 最近一帧有效测量的主控时间
	uint32_t euler_rx_ms;
	uint32_t gyro_rx_ms;

}imu_t;

// BSP 启动后、任务启动前绑定总线并注册接收回调。
LF_EXTERN_C bool dm_imu_init(uint16_t can_id, uint16_t mst_id, FDCAN_HandleTypeDef* bus);
// 复制最近的测量值。收到任一有效测量帧后返回 true，各类型是否收到见 received_mask。
LF_EXTERN_C bool dm_imu_get_data(imu_t* result);

// 下列命令返回 true 表示 CAN 帧已进入发送 FIFO。
LF_EXTERN_C bool imu_write_reg(uint8_t reg_id, uint32_t data);
LF_EXTERN_C bool imu_read_reg(uint8_t reg_id);
LF_EXTERN_C bool imu_reboot(void);
LF_EXTERN_C bool imu_accel_calibration(void);
LF_EXTERN_C bool imu_gyro_calibration(void);
LF_EXTERN_C bool imu_change_com_port(imu_com_port_e port);
LF_EXTERN_C bool imu_set_active_mode_delay(uint32_t delay);
LF_EXTERN_C bool imu_change_to_active(void);
LF_EXTERN_C bool imu_change_to_request(void);
LF_EXTERN_C bool imu_set_baud(imu_baudrate_e baud);
LF_EXTERN_C bool imu_set_can_id(uint8_t can_id);
LF_EXTERN_C bool imu_set_mst_id(uint8_t mst_id);
LF_EXTERN_C bool imu_save_parameters(void);
LF_EXTERN_C bool imu_restore_settings(void);
LF_EXTERN_C bool imu_request_accel(void);
LF_EXTERN_C bool imu_request_gyro(void);
LF_EXTERN_C bool imu_request_euler(void);
// 保留按需四元数请求/解析接口。当前 CAN3 周期预算不包含四元数主动上报，
// 正常控制任务不调用此请求；设备端也须关闭 QUAT 的主动输出。
LF_EXTERN_C bool imu_request_quat(void);

#endif
