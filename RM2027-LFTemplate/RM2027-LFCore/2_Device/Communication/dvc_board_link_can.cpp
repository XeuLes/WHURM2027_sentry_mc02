#include "dvc_board_link_can.hpp"

#include "bsp_can.h"
#include "fdcan.h"

#include "FreeRTOS.h"
#include "task.h"

#include <cstring>

namespace board_link {
TxDebugState g_tx_debug{};

namespace can3 {
LoadDebugState g_can3_load{};

namespace {
ReceivedYawFeedback g_feedback{};
ReceivedChassisFeedback g_chassis_feedback{};
Statistics g_statistics{};
BspFdcanStats g_previous_bus_stats{};
uint32_t g_last_load_sample_ms = 0U;
bool g_initialized = false;
uint32_t g_short_yaw_rx_frames = 0U;
uint32_t g_offset_tx_frames = 0U;

template<class Frame>
void receiveFrame(ReceivedFeedback<Frame>& slot, const uint8_t* data, uint32_t now_ms,
                  uint8_t allowed_flags, uint32_t timeout) {
    Frame incoming{};
    std::memcpy(&incoming, data, sizeof(incoming));
    const uint8_t delta = static_cast<uint8_t>(incoming.sequence - slot.data.sequence);
    if ((incoming.flags & kVersionMask) != kVersion ||
        (incoming.flags & ~(kVersionMask | allowed_flags)) != 0 ||
        (slot.seen && delta == 0U) || (slot.fresh(now_ms, timeout) && delta >= 128U)) {
        ++g_statistics.invalid_rx_frames;
        return;
    }
    if (slot.fresh(now_ms, timeout) && delta != 1U) {
        ++g_statistics.sequence_gaps;
    }
    slot.data = incoming;
    slot.rx_ms = now_ms;
    ++slot.revision;
    slot.seen = true;
    ++g_statistics.valid_rx_frames;
}

void receiveFromCan(FDCAN_HandleTypeDef* bus, const FDCAN_RxHeaderTypeDef* header,
                    const uint8_t* data, uint8_t length) {
    if (bus != &hfdcan3 || (header->Identifier != kBigYawFeedbackId &&
                           header->Identifier != kChassisFeedbackId)) {
        return;
    }
    const bool yaw = header->Identifier == kBigYawFeedbackId;
    if (header->IdType != FDCAN_STANDARD_ID || header->RxFrameType != FDCAN_DATA_FRAME ||
        header->FDFormat != FDCAN_CLASSIC_CAN ||
        length != (yaw ? sizeof(BigYawFeedbackFrame) : sizeof(ChassisFeedbackFrame))) {
        ++g_statistics.invalid_rx_frames;
        return;
    }
    if (yaw) {
        ++g_short_yaw_rx_frames;
        receiveFrame(g_feedback, data, HAL_GetTick(),
            kBigYawFeedbackValid | kBigYawFeedbackReady | kBigYawFeedbackSlow, kBigYawTimeoutMs);
    } else {
        receiveFrame(g_chassis_feedback, data, HAL_GetTick(),
            kChassisFeedbackValid, kChassisFeedbackTimeoutMs);
    }
}

template<typename Frame>
bool sendFrame(uint16_t id, const Frame& frame, Frame& debug_payload) {
    const bool sent = bspFdcanSend(&hfdcan3, id, reinterpret_cast<const uint8_t*>(&frame),
                                  sizeof(frame));
    taskENTER_CRITICAL();
    ++g_statistics.tx_attempts;
    if (!sent) { ++g_statistics.tx_failures; }
    if (sent && id == kGimbalOffsetId) { ++g_offset_tx_frames; }
    g_tx_debug.last_tx_ms = HAL_GetTick();
    debug_payload = frame;
    taskEXIT_CRITICAL();
    return sent;
}
} // namespace

bool init() {
    if (g_initialized) { return true; }
    if (!bspFdcanAddObserver(receiveFromCan)) { return false; }
    taskENTER_CRITICAL();
    bspFdcanGetStats(&hfdcan3, &g_previous_bus_stats);
    g_short_yaw_rx_frames = 0U;
    g_last_load_sample_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
    g_initialized = true;
    return true;
}

ReceivedYawFeedback readFeedback() {
    taskENTER_CRITICAL();
    const auto feedback = g_feedback;
    taskEXIT_CRITICAL();
    return feedback;
}

ReceivedChassisFeedback readChassisFeedback() {
    taskENTER_CRITICAL();
    const auto feedback = g_chassis_feedback;
    taskEXIT_CRITICAL();
    return feedback;
}

void updateBusLoad(uint32_t now_ms) {
    const uint32_t elapsed_ms = now_ms - g_last_load_sample_ms;
    if (elapsed_ms < 1000U) { return; }

    BspFdcanStats stats{};
    taskENTER_CRITICAL();
    const bool available = bspFdcanGetStats(&hfdcan3, &stats);
    const uint32_t yaw_rx = g_short_yaw_rx_frames;
    const uint32_t offset_tx = g_offset_tx_frames;
    if (available) {
        g_short_yaw_rx_frames = 0U;
        g_offset_tx_frames = 0U;
    }
    taskEXIT_CRITICAL();
    if (!available) { return; }
    const uint32_t rx_frames = stats.rx_frames - g_previous_bus_stats.rx_frames;
    const uint32_t tx_queued = stats.tx_queued - g_previous_bus_stats.tx_queued;
    const uint64_t bits = (static_cast<uint64_t>(rx_frames) + tx_queued) * kMaxFrameBits -
        static_cast<uint64_t>(yaw_rx) * (kMaxFrameBits - maxFrameBits(sizeof(BigYawFeedbackFrame))) -
        static_cast<uint64_t>(offset_tx) * (kMaxFrameBits - maxFrameBits(sizeof(GimbalOffsetFrame)));
    const uint32_t estimated_permille = static_cast<uint32_t>(
        bits * 1000000U / (static_cast<uint64_t>(kBitrate) * elapsed_ms));
    g_can3_load = {
        elapsed_ms, rx_frames, tx_queued,
        stats.rx_lost - g_previous_bus_stats.rx_lost,
        stats.tx_failed - g_previous_bus_stats.tx_failed,
        stats.errors - g_previous_bus_stats.errors,
        estimated_permille, stats.bus_off
    };
    g_previous_bus_stats = stats;
    g_last_load_sample_ms = now_ms;
}

bool send(const ChassisTargetFrame& frame) {
    return sendFrame(kChassisTargetId, frame, g_tx_debug.last_chassis_payload);
}

bool send(const BigYawTargetFrame& frame) {
    return sendFrame(kBigYawTargetId, frame, g_tx_debug.last_yaw_payload);
}

bool send(const GimbalOffsetFrame& frame) {
    return sendFrame(kGimbalOffsetId, frame, g_tx_debug.last_offset_payload);
}

bool send(const RefereePowerFrame& frame) {
    return sendFrame(kRefereePowerId, frame, g_tx_debug.last_referee_payload);
}

} // namespace can3

Statistics getStatistics() {
    taskENTER_CRITICAL();
    const auto result = can3::g_statistics;
    taskEXIT_CRITICAL();
    return result;
}

} // namespace board_link
