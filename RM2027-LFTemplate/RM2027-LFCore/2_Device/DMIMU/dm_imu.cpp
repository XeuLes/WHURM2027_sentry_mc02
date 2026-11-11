#include "dm_imu.hpp"
#include "bsp_can.h"

namespace {
constexpr uint16_t kMaxStandardId = 0x7FFU;
imu_t imu{};

uint16_t readLe16(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0] | (data[1] << 8));
}

float uintToFloat(uint16_t raw, float minimum, float maximum, unsigned bits)
{
    return static_cast<float>(raw) * (maximum - minimum)
        / static_cast<float>((1U << bits) - 1U) + minimum;
}

void updateQuaternion(const uint8_t* data)
{
    // Four consecutive 14-bit fields in W, X, Y, Z order.
    const uint16_t w = (data[1] << 6) | (data[2] >> 2);
    const uint16_t x = ((data[2] & 0x03U) << 12) | (data[3] << 4) | (data[4] >> 4);
    const uint16_t y = ((data[4] & 0x0FU) << 10) | (data[5] << 2) | (data[6] >> 6);
    const uint16_t z = ((data[6] & 0x3FU) << 8) | data[7];

    imu.q[0] = uintToFloat(w, Quaternion_MIN, Quaternion_MAX, 14);
    imu.q[1] = uintToFloat(x, Quaternion_MIN, Quaternion_MAX, 14);
    imu.q[2] = uintToFloat(y, Quaternion_MIN, Quaternion_MAX, 14);
    imu.q[3] = uintToFloat(z, Quaternion_MIN, Quaternion_MAX, 14);
}

void onCanFrame(FDCAN_HandleTypeDef* bus, const FDCAN_RxHeaderTypeDef* header,
                const uint8_t* data, uint8_t length)
{
    if (bus != imu.can_handle || header->Identifier != imu.mst_id
        || header->IdType != FDCAN_STANDARD_ID || header->RxFrameType != FDCAN_DATA_FRAME
        || header->FDFormat != FDCAN_CLASSIC_CAN || length != 8U) {
        return;
    }

    switch (data[0]) {
        case ACCEL_DATA:
            imu.cur_temp = data[1];
            for (unsigned axis = 0; axis < 3; ++axis) {
                imu.accel[axis] = uintToFloat(readLe16(data + 2 + 2 * axis),
                                              ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
            }
            break;
        case GYRO_DATA:
            for (unsigned axis = 0; axis < 3; ++axis) {
                imu.gyro[axis] = uintToFloat(readLe16(data + 2 + 2 * axis),
                                             GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
            }
            break;
        case EULER_DATA:
            imu.pitch = uintToFloat(readLe16(data + 2), PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
            imu.yaw = uintToFloat(readLe16(data + 4), YAW_CAN_MIN, YAW_CAN_MAX, 16);
            imu.roll = uintToFloat(readLe16(data + 6), ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
            break;
        case QUAT_DATA:
            updateQuaternion(data);
            break;
        default:
            return;
    }

    // Report types 4, 3, 2, 1 correspond to bits 0, 1, 2, 3.
    imu.received_mask |= 1U << (QUAT_DATA - data[0]);
    imu.last_rx_ms = HAL_GetTick();
    if (data[0] == EULER_DATA) { imu.euler_rx_ms = imu.last_rx_ms; }
    if (data[0] == GYRO_DATA) { imu.gyro_rx_ms = imu.last_rx_ms; }
}

bool sendCommand(uint8_t reg_id, uint8_t action, uint32_t value)
{
    const uint8_t data[8] = {
        0xCCU, reg_id, action, 0xDDU,
        static_cast<uint8_t>(value),
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value >> 16),
        static_cast<uint8_t>(value >> 24),
    };
    return bspFdcanSend(imu.can_handle, imu.can_id, data, sizeof(data));
}
}

bool dm_imu_init(uint16_t can_id, uint16_t mst_id, FDCAN_HandleTypeDef* bus)
{
    if (bus == nullptr || can_id > kMaxStandardId || mst_id > kMaxStandardId) {
        return false;
    }
    if (imu.can_handle != nullptr) {
        return bus == imu.can_handle && can_id == imu.can_id && mst_id == imu.mst_id;
    }

    imu.can_id = can_id;
    imu.mst_id = mst_id;
    imu.can_handle = bus;
    if (bspFdcanAddObserver(onCanFrame)) {
        return true;
    }

    imu = {};
    return false;
}

bool dm_imu_get_data(imu_t* result)
{
    if (result == nullptr) {
        return false;
    }

    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    *result = imu;
    __DMB();
    __set_PRIMASK(mask);
    return result->received_mask != 0U;
}

bool imu_write_reg(uint8_t reg_id, uint32_t data)
{
    return sendCommand(reg_id, CMD_WRITE, data);
}

bool imu_read_reg(uint8_t reg_id)
{
    return sendCommand(reg_id, CMD_READ, 0);
}

bool imu_reboot()
{
    return imu_write_reg(REBOOT_IMU, 0);
}

bool imu_accel_calibration()
{
    return imu_write_reg(ACCEL_CALI, 0);
}

bool imu_gyro_calibration()
{
    return imu_write_reg(GYRO_CALI, 0);
}

bool imu_change_com_port(imu_com_port_e port)
{
    return imu_write_reg(CHANGE_COM, static_cast<uint8_t>(port));
}

bool imu_set_active_mode_delay(uint32_t delay)
{
    return imu_write_reg(SET_DELAY, delay);
}

bool imu_change_to_active()
{
    return imu_write_reg(CHANGE_ACTIVE, 1);
}

bool imu_change_to_request()
{
    return imu_write_reg(CHANGE_ACTIVE, 0);
}

bool imu_set_baud(imu_baudrate_e baud)
{
    return imu_write_reg(SET_BAUD, static_cast<uint8_t>(baud));
}

bool imu_set_can_id(uint8_t can_id)
{
    return imu_write_reg(SET_CAN_ID, can_id);
}

bool imu_set_mst_id(uint8_t mst_id)
{
    return imu_write_reg(SET_MST_ID, mst_id);
}

bool imu_save_parameters()
{
    return imu_write_reg(SAVE_PARAM, 0);
}

bool imu_restore_settings()
{
    return imu_write_reg(RESTORE_SETTING, 0);
}

bool imu_request_accel()
{
    return imu_read_reg(ACCEL_DATA);
}

bool imu_request_gyro()
{
    return imu_read_reg(GYRO_DATA);
}

bool imu_request_euler()
{
    return imu_read_reg(EULER_DATA);
}

bool imu_request_quat()
{
    return imu_read_reg(QUAT_DATA);
}
