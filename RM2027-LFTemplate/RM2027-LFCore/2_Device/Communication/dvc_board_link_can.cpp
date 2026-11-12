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
ReceivedBoardFeedback g_feedback{};
Statistics g_statistics{};
BspFdcanStats g_previous_bus_stats{};
uint32_t g_last_load_sample_ms = 0U;
bool g_initialized = false;

void receiveFromCan(FDCAN_HandleTypeDef* bus, const FDCAN_RxHeaderTypeDef* header,
                    const uint8_t* data, uint8_t length) {
    if (bus != &hfdcan3 || header == nullptr || data == nullptr ||
        header->Identifier != kFeedbackId) { return; }
    if (header->IdType != FDCAN_STANDARD_ID || header->RxFrameType != FDCAN_DATA_FRAME ||
        header->FDFormat != FDCAN_CLASSIC_CAN || length != sizeof(FeedbackFrame)) {
        ++g_statistics.invalid_rx_frames;
        return;
    }
    FeedbackFrame wire{};
    std::memcpy(&wire, data, sizeof(wire));
    const auto incoming = decodeFeedback(wire);
    const uint32_t now = HAL_GetTick();
    const uint8_t delta = (incoming.sequence - g_feedback.data.sequence) & 0x0FU;
    const bool fresh = g_feedback.fresh(now, kBigYawTimeoutMs);
    // Four-bit sequence: accept steps 1..7 while fresh; after timeout accept a
    // new baseline (including the same sequence) so a reboot can recover.
    const bool yaw_valid = incoming.flags & kBigYawFeedbackValid;
    if ((fresh && (delta == 0U || delta >= 8U)) ||
        (yaw_valid && (incoming.joint < -7854 || incoming.joint > 7854)) ||
        (!yaw_valid && (incoming.flags & (kBigYawFeedbackReady | kBigYawFeedbackSlow)))) {
        ++g_statistics.invalid_rx_frames;
        return;
    }
    if (fresh && delta != 1U) { ++g_statistics.sequence_gaps; }
    g_feedback.data = incoming;
    g_feedback.rx_ms = now;
    ++g_feedback.revision;
    g_feedback.seen = true;
    ++g_statistics.valid_rx_frames;
}

template<typename Frame>
bool sendFrame(uint16_t id, const Frame& frame, Frame& debug_payload) {
    const bool sent = bspFdcanSend(&hfdcan3, id, reinterpret_cast<const uint8_t*>(&frame),
                                  sizeof(frame));
    taskENTER_CRITICAL();
    ++g_statistics.tx_attempts;
    if (!sent) { ++g_statistics.tx_failures; }
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
    g_last_load_sample_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
    g_initialized = true;
    return true;
}

ReceivedBoardFeedback readFeedback() {
    taskENTER_CRITICAL();
    const auto feedback = g_feedback;
    taskEXIT_CRITICAL();
    return feedback;
}

void updateBusLoad(uint32_t now_ms) {
    const uint32_t elapsed_ms = now_ms - g_last_load_sample_ms;
    if (elapsed_ms < 1000U) { return; }

    BspFdcanStats stats{};
    taskENTER_CRITICAL();
    const bool available = bspFdcanGetStats(&hfdcan3, &stats);
    taskEXIT_CRITICAL();
    if (!available) { return; }
    const uint32_t rx_frames = stats.rx_frames - g_previous_bus_stats.rx_frames;
    const uint32_t tx_queued = stats.tx_queued - g_previous_bus_stats.tx_queued;
    // All scheduled v3 frames have DLC 8. This estimates queued traffic,
    // not measured bus occupancy or successful on-wire delivery.
    const uint64_t bits = (static_cast<uint64_t>(rx_frames) + tx_queued) * kMaxFrameBits;
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
