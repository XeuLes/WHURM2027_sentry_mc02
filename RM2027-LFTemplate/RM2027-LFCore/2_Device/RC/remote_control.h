
#ifndef REMOTE_CONTROL_H
    #define REMOTE_CONTROL_H

    #include "core_linkage.h"

    #include <stdbool.h>

    #include <stdint.h>
//#include "struct_typedef.h"
//#include "attribute_typedef.h"
//#include "macro_typedef.h"

    #define RC_DT7 0 // DT7遥控器
    #define RC_AT9S_PRO 1 // AT9S PRO遥控器
    #define RC_HT8A 2 // HT8A遥控器
    #define RC_ET08A 3 // ET08A遥控器

    #ifndef RC_TYPE
        #define RC_TYPE RC_ET08A // 遥控器类型

    #endif
    #ifndef CONTROL_LINK_RC
        #define CONTROL_LINK_RC CL_RC_DIRECT // 遥控器数据传输方式
    #endif

    #define AX_X 0 // x轴
    #define AX_Y 1 // y轴
    #define AX_Z 2 // z轴

    #define AX_ROLL AX_X // roll轴
    #define AX_PITCH AX_Y // pitch轴
    #define AX_YAW AX_Z // yaw轴

/*遥控器相关宏定义*/

    #define DT7_SW_LEFT 1 // DT7遥控器左侧拨杆通道号
    #define DT7_SW_RIGHT 0 // DT7遥控器右侧拨杆通道号
    #define DT7_CH_LH 0 // DT7遥控器右侧水平摇杆通道号
    #define DT7_CH_LV 1 // DT7遥控器右侧竖直摇杆通道号
    #define DT7_CH_RH 2 // DT7遥控器左侧水平摇杆通道号
    #define DT7_CH_RV 3 // DT7遥控器左侧竖直摇杆通道号
    #define DT7_CH_ROLLER 4 // DT7遥控器左上角滚轮通道号

    #define SBUS_RX_BUF_NUM 36u

    #define RC_FRAME_LENGTH 18u
    #define SBUS_RC_FRAME_LENGTH 25u
    // DT7遥控器通道值范围
    #define RC_CH_VALUE_MIN ((uint16_t)364)
    #define RC_CH_VALUE_OFFSET ((uint16_t)1024)
    #define RC_CH_VALUE_MAX ((uint16_t)1684)

    // AT9S PRO 遥控器通道值范围
    #define AT9S_PRO_RC_CH_VALUE_MIN ((uint16_t)200)
    #define AT9S_PRO_RC_CH_VALUE_OFFSET ((uint16_t)1000)
    #define AT9S_PRO_RC_CH_VALUE_MAX ((uint16_t)1800)

    #define AT9S_PRO_RC_CONNECTED_FLAG ((uint8_t)12)

    // HT8A 遥控器通道值范围
    #define HT8A_RC_CH013_VALUE_MIN ((uint16_t)432)
    #define HT8A_RC_CH013_VALUE_OFFSET ((uint16_t)992)
    #define HT8A_RC_CH013_VALUE_MAX ((uint16_t)1552)

    #define HT8A_RC_CH247_VALUE_MIN ((uint16_t)192)
    #define HT8A_RC_CH247_VALUE_OFFSET ((uint16_t)992)
    #define HT8A_RC_CH247_VALUE_MAX ((uint16_t)1792)

    #define HT8A_RC_CH47_VALUE_MIN ((uint16_t)192)
    #define HT8A_RC_CH47_VALUE_OFFSET ((uint16_t)992)
    #define HT8A_RC_CH47_VALUE_MAX ((uint16_t)1792)

    #define HT8A_RC_CONNECTED_FLAG ((uint8_t)12)

    // ET08A 遥控器通道值范围
    #define ET08A_RC_CH_VALUE_MIN ((uint16_t)353)
    #define ET08A_RC_CH_VALUE_OFFSET ((uint16_t)1024)
    #define ET08A_RC_CH_VALUE_MAX ((uint16_t)1694)

    #define ET08A_RC_CONNECTED_FLAG ((uint8_t)3)

    #define RC_TO_ONE 0.0015151515151515f // (1/660)遥控器通道值归一化系数

    /* ----------------------- RC Switch Definition----------------------------- */
    #define RC_SW_UP ((uint16_t)1)
    #define RC_SW_MID ((uint16_t)3)
    #define RC_SW_DOWN ((uint16_t)2)
    #define SWITCH_IS_DOWN(s) (s == RC_SW_DOWN)
    #define SWITCH_IS_MID(s) (s == RC_SW_MID)
    #define SWITCH_IS_UP(s) (s == RC_SW_UP)

    #define RC_CH_LEFT_HORIZONTAL ((uint8_t)2)
    #define RC_CH_LEFT_VERTICAL ((uint8_t)3)
    #define RC_CH_RIGHT_HORIZONTAL ((uint8_t)0)
    #define RC_CH_RIGHT_VERTICAL ((uint8_t)1)
    #define RC_CH_LEFT_ROTATE ((uint8_t)4)

    #define RC_SW_LEFT ((uint8_t)1)
    #define RC_SW_RIGHT ((uint8_t)0)
    /* ----------------------- PC Key Definition-------------------------------- */
    #define KEY_W ((uint8_t)0)
    #define KEY_S ((uint8_t)1)
    #define KEY_A ((uint8_t)2)
    #define KEY_D ((uint8_t)3)
    #define KEY_SHIFT ((uint8_t)4)
    #define KEY_CTRL ((uint8_t)5)
    #define KEY_Q ((uint8_t)6)
    #define KEY_E ((uint8_t)7)
    #define KEY_R ((uint8_t)8)
    #define KEY_F ((uint8_t)9)
    #define KEY_G ((uint8_t)10)
    #define KEY_Z ((uint8_t)11)
    #define KEY_X ((uint8_t)12)
    #define KEY_C ((uint8_t)13)
    #define KEY_V ((uint8_t)14)
    #define KEY_B ((uint8_t)15)

    #define KEY_LEFT ((uint8_t)0)
    #define KEY_RIGHT ((uint8_t)1)

    #define KEY_PRESSED_OFFSET_W ((uint16_t)1 << KEY_W)
    #define KEY_PRESSED_OFFSET_S ((uint16_t)1 << KEY_S)
    #define KEY_PRESSED_OFFSET_A ((uint16_t)1 << KEY_A)
    #define KEY_PRESSED_OFFSET_D ((uint16_t)1 << KEY_D)
    #define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << KEY_SHIFT)
    #define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << KEY_CTRL)
    #define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << KEY_Q)
    #define KEY_PRESSED_OFFSET_E ((uint16_t)1 << KEY_E)
    #define KEY_PRESSED_OFFSET_R ((uint16_t)1 << KEY_R)
    #define KEY_PRESSED_OFFSET_F ((uint16_t)1 << KEY_F)
    #define KEY_PRESSED_OFFSET_G ((uint16_t)1 << KEY_G)
    #define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << KEY_Z)
    #define KEY_PRESSED_OFFSET_X ((uint16_t)1 << KEY_X)
    #define KEY_PRESSED_OFFSET_C ((uint16_t)1 << KEY_C)
    #define KEY_PRESSED_OFFSET_V ((uint16_t)1 << KEY_V)
    #define KEY_PRESSED_OFFSET_B ((uint16_t)1 << KEY_B)

/* ----------------------- Data Struct ------------------------------------- */
typedef struct RcControlData {
    struct RcChannels {
        int16_t ch[5];
        char s[4];
    } __attribute__((packed)) rc;
    struct RcMouse {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } __attribute__((packed)) mouse;
    struct RcKey {
        uint16_t v;
    } __attribute__((packed)) key;

} __attribute__((packed)) RcControl;

typedef struct {
    uint16_t ch[16];
    uint8_t connect_flag;
} __attribute__((packed)) SbusData;

/* ----------------------- Internal Data ----------------------------------- */

LF_EXTERN_C void remoteControlInit(void);
LF_EXTERN_C const RcControl* getRemoteControlPoint(void);
LF_EXTERN_C const SbusData* getSbusPoint(void);
LF_EXTERN_C uint8_t rcDataIsError(void);
LF_EXTERN_C void solveRcLost(void);
LF_EXTERN_C void solveDataError(void);
//LF_EXTERN_C void sbus_to_usart1(uint8_t * sbus);

/******************************************************************/
/* API                                                            */
/******************************************************************/

LF_EXTERN_C bool getRcOffline(void);

LF_EXTERN_C float getDt7RcCh(uint8_t ch);
LF_EXTERN_C char getDt7RcSw(uint8_t sw);
LF_EXTERN_C float getDt7MouseSpeed(uint8_t axis);
LF_EXTERN_C bool getDt7Mouse(uint8_t key);
LF_EXTERN_C bool getDt7Keyboard(uint8_t key);

#endif
/*------------------------------ End of File ------------------------------*/
