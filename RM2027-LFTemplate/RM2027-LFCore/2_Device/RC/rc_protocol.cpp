// Remote protocol decoding derived from Polarbear remote_control (2018-2025).
//s[0] ← 第 5 通道：`SB`
//s[1] ← 第 8 通道：`SC`


#include "rc_protocol.hpp"

#include <algorithm>

#include "alg_basic.hpp"

namespace {
static void sbusToRc(volatile const uint8_t* sbus_buf, RcControl* rc_ctrl) {
    if (sbus_buf == NULL || rc_ctrl == NULL) {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff; //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | //!< Channel 2
                         (sbus_buf[4] << 10))
        & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003); //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8); //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8); //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8); //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12]; //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13]; //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8); //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8); //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

#if RC_TYPE != RC_DT7
// SBUS通道解析
static void decodeSbus(volatile const uint8_t* buffer, SbusData& sbus) {
    // Each SBUS channel is an unsigned 11-bit little-endian bit field.
    for (uint8_t channel = 0; channel < 16; ++channel) {
        const uint16_t bit_offset = channel * 11;
        const uint8_t byte_offset = 1 + bit_offset / 8;
        const uint8_t shift = bit_offset % 8;
        uint32_t packed = buffer[byte_offset] | (uint32_t(buffer[byte_offset + 1]) << 8);
        if (shift > 5) {
            packed |= uint32_t(buffer[byte_offset + 2]) << 16;
        }
        sbus.ch[channel] = (packed >> shift) & 0x07ff;
    }
    sbus.connect_flag = buffer[23];
}

static void clearSpecialChannels(RcControl* control) {
    control->mouse = {};
    control->key = {};
    control->rc.ch[4] = 0;
}

static char decodeSwitch(uint16_t value, int offset, int span) {
    const int position = static_cast<int>(value) - offset;
    const int threshold = span / 3;
    if (position < -threshold) {
        return RC_SW_UP;
    }
    if (position > threshold) {
        return RC_SW_DOWN;
    }
    return RC_SW_MID;
}

static void sbusToControl(volatile const uint8_t* buffer, RcControl* control, SbusData& sbus) {
    if (buffer == nullptr || control == nullptr) {
        return;
    }
    decodeSbus(buffer, sbus);
    clearSpecialChannels(control);
    #if RC_TYPE == RC_AT9S_PRO
    constexpr int OFFSET = AT9S_PRO_RC_CH_VALUE_OFFSET;
    constexpr float SCALE = 660.0f / 800.0f;
    constexpr int SWITCH_SPAN = 800;
    constexpr uint8_t SWITCH_CHANNELS[2] = { 5, 4 };
    #elif RC_TYPE == RC_HT8A
    constexpr int OFFSET = HT8A_RC_CH013_VALUE_OFFSET;
    constexpr float SCALE = 660.0f / 560.0f;
    constexpr int SWITCH_SPAN = 800;
    constexpr uint8_t SWITCH_CHANNELS[2] = { 7, 4 };
    #elif RC_TYPE == RC_ET08A
    constexpr int OFFSET = ET08A_RC_CH_VALUE_OFFSET;
    constexpr float SCALE = 660.0f / 671.0f;
    constexpr int SWITCH_SPAN = 670;
    // 当前 ET08A 配置：第 7 通道为 SB，第 8 通道为 SC。
    constexpr uint8_t SWITCH_CHANNELS[4] = { 6, 7, 4, 5 };
    if (buffer[23] != ET08A_RC_CONNECTED_FLAG) {
        for (int index = 0; index < 4; ++index) {
            control->rc.ch[index] = 0;
        }
        control->rc.s[0] = RC_SW_UP;
        control->rc.s[1] = RC_SW_MID;
        return;
    }
    #endif
    constexpr uint8_t CHANNELS[4] = { 0, 1, 3, 2 };
    for (uint8_t index = 0; index < 4; ++index) {
        float value = (int(sbus.ch[CHANNELS[index]]) - OFFSET) * SCALE;
    #if RC_TYPE == RC_HT8A
        if (index == 3) {
            value = (int(sbus.ch[2]) - HT8A_RC_CH247_VALUE_OFFSET) * (660.0f / 800.0f);
        }
    #else
        if (index == 1) {
            value = -value;
        }
    #endif
        control->rc.ch[index] = static_cast<int16_t>(value);
    }
    for (uint8_t index = 0; index < sizeof(SWITCH_CHANNELS); ++index) {
        control->rc.s[index] = decodeSwitch(sbus.ch[SWITCH_CHANNELS[index]], OFFSET, SWITCH_SPAN);
    }
}
#endif // RC_TYPE != RC_DT7

} // namespace

bool rc_protocol::decodeFrame(
    const uint8_t* buffer,
    uint16_t length,
    RcControl* control,
    SbusData* sbus
) {
    if (buffer == nullptr || control == nullptr || sbus == nullptr) {
        return false;
    }
    if (length == RC_FRAME_LENGTH) {
        sbusToRc(buffer, control);
        return true;
    }
#if RC_TYPE != RC_DT7
    if (length == SBUS_RC_FRAME_LENGTH) {
        sbusToControl(buffer, control, *sbus);
        return true;
    }
#endif
    return false;
}
