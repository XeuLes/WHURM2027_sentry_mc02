/**
 * @file dvc_dji_motor.hpp
 * @author LuojiaFox
 * @brief DJI 电机驱动（GM6020 / M3508 / M2006）
 * @version 3.0
 * @date 2026-10-20
 *
 * @copyright WHU-LuojiaFox (c) 2026
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor.hpp"

/* Exported macros -----------------------------------------------------------*/

/** @brief DJI 编码器一圈的计数 */
constexpr uint16_t kDjiEncoderRange = 8192U;

/** @brief 手册规定的原始指令上限；GM6020 为电压模式，M3508/M2006 为电流模式 */
constexpr int16_t DJI_GM6020_MAX_COMMAND = 25000;
constexpr int16_t DJI_M3508_MAX_COMMAND = 16384;
constexpr int16_t DJI_M2006_MAX_COMMAND = 10000;
/** @brief M3508 P19 原装减速箱精确减速比，裸电机或改装减速箱需显式覆盖 */
constexpr float DJI_M3508_DEFAULT_REDUCTION_RATIO = 3591.0f / 187.0f;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief DJI 电机反馈数据
 */
struct DjiMotorData {
    uint16_t encoder = 0;         /*!< 当前编码器值，0~8191 */
    uint16_t last_encoder = 0;    /*!< 上次编码器值 */
    int16_t velocity_rpm = 0;     /*!< 转子转速，rpm */
    int16_t current_raw = 0;      /*!< 反馈电流原始值，与输出电压/电流指令独立 */
    uint8_t temperature = 0;      /*!< 温度，摄氏度；temperature_valid 为 true 时有效 */
    bool temperature_valid = false; /*!< GM6020/C620 有温度，C610 的对应字节为保留位 */
    uint8_t error_code = 0;       /*!< C610/C620 的 DATA[7] 错误码；GM6020 不提供 */
    float velocity_rad_s = 0.0f;  /*!< 输出轴角速度，rad/s */
    float current_a = 0.0f;       /*!< 按 init() 的反馈电流比例换算为 A 并应用方向 */
    float angle_rad = 0.0f;       /*!< 相对首帧位置的输出轴角度，-pi~pi，rad */
    float total_angle_rad = 0.0f; /*!< 输出轴连续累计角度，rad */
    uint8_t raw[8] = {};          /*!< 最近一帧原始反馈数据 */
    bool initialized = false;     /*!< 是否收到过反馈 */
};

/**
 * @brief DJI 电机（GM6020 / M3508 / M2006）
 *
 * 标识符规则（标准帧）：
 * | 类型            | 接收标识符      | 发送标识符（ID 1~4 / 5~8） |
 * | --------------- | --------------- | -------------------------- |
 * | GM6020          | 0x205 + ID - 1  | 0x1FF / 0x2FF              |
 * | M3508、M2006    | 0x201 + ID - 1  | 0x200 / 0x1FF              |
 *
 * 注意：0x1FF 同时是 GM6020 的 ID 1~4 和 M3508/M2006 的 ID 5~8 的发送帧，
 *       同一条总线上不要混用这两组电机。
 */
class DjiMotor : public MotorBase {
public:
    DjiMotor();

    /**
     * @brief 配置一台 DJI 电机
     * @param type            电机类型
     * @param id              拨码 ID，GM6020 为 1~7，M3508/M2006 为 1~8
     * @param bus             FDCAN 句柄
     * @param direction       安装方向，1 或 -1
     * @param reduction_ratio 输出轴减速比，传 0 使用型号默认值
     * @param feedback_current_a_per_lsb 反馈电流 A/LSB，默认沿用 SENTRY 的 0.001；
     *                        本地手册未明确反馈电流比例，须按驱动固件/实测核实，不能套用指令比例
     * @return true 配置成功
     */
    bool init(MotorType type, uint8_t id, FDCAN_HandleTypeDef* bus, int8_t direction = 1,
              float reduction_ratio = 0.0f, float feedback_current_a_per_lsb = 0.001f);

    bool update(uint32_t identifier, const uint8_t* data, uint8_t length = 8U) override;
    void reset() override;

    /** @brief 本电机所在发送帧的标识符 */
    uint32_t txIdentifier() const override;

    /** @brief 本电机在发送帧中的槽位，0~3 */
    uint8_t txSlot() const override;
    bool sharesTxFrame() const override { return true; }

    /**
     * @brief 汇总注册表中同总线、同发送 ID 的全部 DJI 电机，发送一帧
     * @param bus        CAN 口
     * @param identifier 发送标识符，例如 0x200 / 0x1FF / 0x2FF
     * @return true 表示至少有一台匹配的电机并已写入 TX FIFO
     * @note 每个 (bus, identifier) 应由一个任务负责周期发送，其他模块只 setCommand()。
     *        典型用法：一个任务只负责自己的 0x1FF 或 0x200 帧。
     */
    static bool sendGroup(FDCAN_HandleTypeDef* bus, uint32_t identifier);

    /**
     * @brief 直接打包一帧 4 台 DJI 电机的控制量
     * @param bus        CAN 口
     * @param identifier 发送标识符（0x200 / 0x1FF / 0x2FF）
     * @param current    4 台电机的控制量，按槽位 0~3 排列
     * @return true 表示已写入 TX FIFO
     * @note  不做限幅，调用方需保证数值在对应电机的允许范围内。
     */
    static bool sendFrame(FDCAN_HandleTypeDef* bus, uint32_t identifier,
                          const int16_t current[4]);

    int16_t command() const
    {
        return command_;
    }

    /**
     * @brief 设置机构方向的控制量，发送时应用 direction；幅值仍为协议原始单位
     * @param value GM6020 为电压量（-25000~25000），M3508 为电流量（-16384~16384），
     *              M2006 为电流量（-10000~10000），超出范围会自动限幅
     */
    void setCommand(int16_t value);

    /** @brief 在短临界区内复制反馈；在线状态和接收计数由 MotorBase 查询。 */
    DjiMotorData data() const
    {
        MotorCriticalSection lock;
        return data_;
    }

private:
    DjiMotorData data_;
    int16_t command_;
    float feedback_current_a_per_lsb_;  //DJI 电机反馈电流的换算系数
};

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/
