/**
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *  V2.0.0     Feb-17-2025     Penguin         1. support RC AT9S PRO
  *                                             2. support RC HT8A
  *                                             3. support normal sbus RC in struct Sbus_t
  *  V2.0.1     Feb-25-2025     Penguin         1. support RC ET08A
  *
  @verbatim
  ==============================================================================
  使用At9sPro遥控器时请设置5通为SwE，6通为SwG

  注：使用非DT7遥控器时，需要先检查通道值数据是否正常（一般遥控器都带有通道值数据偏移功能，将通道值中值移动到正确数值后再使用）
      AT9S PRO 遥控器中值为 1000
      HT8A 遥控器中值为 992
      ET08A 遥控器中值为 1024
  
  ET08A 遥控器设置指南：
    1. 设置 主菜单->系统设置->摇杆模式 为模式2
    2. 设置 主菜单->通用功能->通道设置 7通道为 [辅助1 SB --] 8通道为 [辅助2 SC --]


  底盘模式切换：
  SC 上档 → CHASSIS_MODE_AUTO_VISION
  SC 中档 → CHASSIS_MODE_DIRECT_RC
  SC 下档 → CHASSIS_MODE_DISABLED
  SB 下档 → CHASSIS_MODE_DISABLED(强制禁用)
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  */

#include "remote_control.h"

#include "alg_basic.hpp"
#include "drv_uart.h"
#include "main.h"
#include "rc_protocol.hpp"
#include "stm32h7xx_it.h"
#include <algorithm>

#include "string.h"

//#include "detect_task.h"
//#include "robot_param.h"
//#include "communication.h"

// 遥控器掉线时间阈值
#define RC_LOST_TIME 100 // ms
// 非dt7遥控器连续断线上线次数（超过认为断连）
#define SBUS_MAX_LOST_NUN 10

//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

//取正函数

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */

//remote control data
//遥控器控制变量
RcControl rc_state;
SbusData sbus = { {}, 0xFF };

// DMA receives up to 36 bytes; each row occupies two isolated cache lines.
constexpr uint32_t SBUS_DMA_BUFFER_SIZE = 64;
alignas(32) static uint8_t sbus_rx_buf[2][SBUS_DMA_BUFFER_SIZE];

// 上一次接收数据的时间
static volatile uint32_t last_receive_time = 0;
// 记录连续接收数据的次数
static volatile uint32_t receive_count = 0;
// 记录非dt7的sbus遥控器连续断连次数
static volatile uint32_t sbus_lost_count = SBUS_MAX_LOST_NUN + 5;

#if (RC_TYPE != RC_DT7)
uint8_t connected_flag; // 遥控器连接标志位
#endif // __RC_TYPE != RC_DT7

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remoteControlInit(void) {
    SCB_CleanInvalidateDCache_by_Addr(
        reinterpret_cast<uint32_t*>(sbus_rx_buf),
        sizeof(sbus_rx_buf)
    );
    rcInit(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
#if (RC_TYPE == RC_AT9S_PRO)
    connected_flag = AT9S_PRO_RC_CONNECTED_FLAG;
#elif (RC_TYPE == RC_HT8A)
    connected_flag = HT8A_RC_CONNECTED_FLAG;
#elif (RC_TYPE == RC_ET08A)
    connected_flag = ET08A_RC_CONNECTED_FLAG;
#endif
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RcControl* getRemoteControlPoint(void) {
    return &rc_state;
}

/**
  * @brief          获取SBUS遥控器数据指针
  * @param[in]      none
  * @retval         SBUS遥控器数据指针
  */
const SbusData* getSbusPoint(void) {
    return &sbus;
}

//判断遥控器数据是否出错，
uint8_t rcDataIsError(void) {
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (basic_math::abs<int>(rc_state.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (basic_math::abs<int>(rc_state.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (basic_math::abs<int>(rc_state.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (basic_math::abs<int>(rc_state.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE) {
        goto error;
    }
    if (rc_state.rc.s[0] == 0) {
        goto error;
    }
    if (rc_state.rc.s[1] == 0) {
        goto error;
    }
    return 0;

error:
    rc_state.rc.ch[0] = 0;
    rc_state.rc.ch[1] = 0;
    rc_state.rc.ch[2] = 0;
    rc_state.rc.ch[3] = 0;
    rc_state.rc.ch[4] = 0;
    rc_state.rc.s[0] = RC_SW_DOWN;
    rc_state.rc.s[1] = RC_SW_DOWN;
    rc_state.mouse.x = 0;
    rc_state.mouse.y = 0;
    rc_state.mouse.z = 0;
    rc_state.mouse.press_l = 0;
    rc_state.mouse.press_r = 0;
    rc_state.key.v = 0;
    return 1;
}

void solveRcLost(void) {
    rcRestart(SBUS_RX_BUF_NUM);
}
void solveDataError(void) {
    rcRestart(SBUS_RX_BUF_NUM);
}

// 记录接收数据的次数
static void countReceived(uint32_t now) {
    if (now - last_receive_time > RC_LOST_TIME) {
        receive_count = 0;
    }
    ++receive_count;
}
// The vector table requires the exact C symbol and spelling.
extern "C" void UART5_IRQHandler() {
    if ((huart5.Instance->ISR & UART_FLAG_RXNE) != 0) {
        __HAL_UART_CLEAR_PEFLAG(&huart5);
        return;
    }
    if ((UART5->ISR & UART_FLAG_IDLE) == 0) {
        return;
    }
    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    auto* stream = static_cast<DMA_Stream_TypeDef*>(hdma_uart5_rx.Instance);
    const uint8_t completed = (stream->CR & DMA_SxCR_CT) != 0 ? 1 : 0;
    __HAL_DMA_DISABLE(&hdma_uart5_rx);
    while ((stream->CR & DMA_SxCR_EN) != 0) {}
    const uint16_t length = SBUS_RX_BUF_NUM - stream->NDTR;
    stream->NDTR = SBUS_RX_BUF_NUM;
    stream->CR ^= DMA_SxCR_CT;
    __HAL_DMA_ENABLE(&hdma_uart5_rx);

    // Each DMA row owns complete cache lines; invalidation cannot discard neighbours.
    auto* buffer = sbus_rx_buf[completed];
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(buffer), SBUS_DMA_BUFFER_SIZE);
    if (!rc_protocol::decodeFrame(buffer, length, &rc_state, &sbus)) {
        return;
    }
#if RC_TYPE != RC_DT7
    if (length == SBUS_RC_FRAME_LENGTH) {
        if (sbus.connect_flag == connected_flag) {
            sbus_lost_count = 0;
        } else if (sbus_lost_count <= SBUS_MAX_LOST_NUN) {
            ++sbus_lost_count;
        }
    }
#endif
    const uint32_t now = HAL_GetTick();
    countReceived(now);
    last_receive_time = now;
}

//取正函数

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
/******************************************************************/
/* API                                                            */
/*----------------------------------------------------------------*/
/* function:      GetRcOffline                                    */
/*                GetDt7RcCh                                      */
/*                GetDt7RcSw                                      */
/*                GetDt7MouseSpeed                                */
/*                GetDt7Mouse                                     */
/*                GetDt7Keyboard                                  */
/******************************************************************/

/**
  * @brief          获取遥控器是否离线。
  * @retval         true:离线，false:在线
  */
bool getRcOffline(void) {
#if RC_TYPE == RC_DT7
    #define USE_SBUS_LOST_COUNT 0
#else
    #define USE_SBUS_LOST_COUNT 1
#endif

#if CONTROL_LINK_RC == CL_RC_DIRECT
    return !((receive_count > 5) && (HAL_GetTick() - last_receive_time < RC_LOST_TIME))
        || ((sbus_lost_count > SBUS_MAX_LOST_NUN) && USE_SBUS_LOST_COUNT);
#elif CONTROL_LINK_RC == CL_RC_UART2
    return getUartRcOffline();
#else
    return true;
#endif

#undef USE_SBUS_LOST_COUNT
}

/**
  * @brief          获取DT7遥控器通道值。
  * @param[in]      ch 通道id，0-右平, 1-右竖, 2-左平, 3-左竖, 4-左滚轮，配合ch id宏进行使用
  * @retval         DT7遥控器通道值，范围为 [−1,1]
  */
float getDt7RcCh(uint8_t ch) {
    return ch < 5 ? rc_state.rc.ch[ch] * RC_TO_ONE : 0.0f;
}
/**
  * @brief          获取DT7遥控器拨杆值，可配合switch_is_xxx系列宏函数使用。
  * @param[in]      sw 通道id，0-右, 1-左，配合sw id宏进行使用
  * @retval         DT7遥控器拨杆值，范围为{1,2,3}
  */
char getDt7RcSw(uint8_t sw) {
    return sw < 4 ? rc_state.rc.s[sw] : 0;
}
/**
  * @brief          获取鼠标axis轴的移动速度
  * @param[in]      axis 轴id, 0-, 1-, 2-，配合轴id宏进行使用
  * @retval         鼠标axis轴移动速度，范围为[,]
  */
float getDt7MouseSpeed(uint8_t axis) {
    switch (axis) {
        case AX_X:
            return rc_state.mouse.x;
        case AX_Y:
            return rc_state.mouse.y;
        case AX_Z:
            return rc_state.mouse.z;
        default:
            return 0;
    }
}
/**
  * @brief          获取鼠标按键信息
  * @param[in]      key 按键id，配合按键id宏进行使用
  * @retval         鼠标按键是否被按下
  */
bool getDt7Mouse(uint8_t key) {
    switch (key) {
        case KEY_LEFT:
            return rc_state.mouse.press_l;
        case KEY_RIGHT:
            return rc_state.mouse.press_r;
        default:
            return 0;
    }
}
/**
  * @brief          获取键盘按键信息
  * @param[in]      key 按键id，配合按键id宏进行使用
  * @retval         键盘按键是否被按下
  */
bool getDt7Keyboard(uint8_t key) {
    return key < 16 && (rc_state.key.v & (uint16_t(1) << key)) != 0;
}
/*------------------------------ End of File ------------------------------*/
