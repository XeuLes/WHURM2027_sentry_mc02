#include "dvc_board_link_can.hpp"
#include "bsp_can.h"
#include "fdcan.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstring>

namespace board_link { namespace can3 {
namespace {
ReceivedTargets targets{};
Statistics statistics{};
bool initialized = false;
constexpr int16_t kPiEncoded = 31416;
bool angle(int16_t value) { return value >= -kPiEncoded && value <= kPiEncoded; }

bool valid(const ChassisTargetFrame& f) {
    return (f.flags & 0x08U) == 0U && (f.flags & kChassisModeMask) <= 2U;
}
bool valid(const BigYawTargetFrame& f) {
    return (f.flags & 0x08U) == 0U &&
        (!(f.flags & kBigYawTargetValid) ||
         ((f.flags & kBigYawEnable) && angle(f.target) &&
          ((f.flags & kBigYawJointFrame) || angle(f.position_or_acceleration))));
}
bool valid(const RefereePowerFrame& f) { return (f.flags & 0x08U) == 0U; }

template<class Frame>
void receive(Received<Frame>& slot, const uint8_t* data, uint8_t length,
             uint32_t now, uint32_t timeout) {
    if (length != sizeof(Frame)) { ++statistics.rejected; return; }
    Frame incoming{};
    std::memcpy(&incoming, data, sizeof(incoming));
    const uint8_t delta = static_cast<uint8_t>(incoming.sequence - slot.data.sequence);
    // Fresh streams accept forward sequence steps only. After timeout, allow
    // a restarted master to establish a new sequence baseline.
    if ((incoming.flags & kVersionMask) != kVersion || !valid(incoming) ||
        (slot.fresh(now, timeout) && (delta == 0U || delta >= 128U))) {
        ++statistics.rejected;
        return;
    }
    if (slot.fresh(now, timeout) && delta != 1U) { ++statistics.sequence_gaps; }
    slot.data = incoming;
    slot.rx_ms = now;
    slot.seen = true;
    ++slot.revision;
    ++statistics.accepted;
}
void onCan(FDCAN_HandleTypeDef* bus, const FDCAN_RxHeaderTypeDef* header,
           const uint8_t* data, uint8_t length) {
    if (bus != &hfdcan3 || header == nullptr || data == nullptr) { return; }
    const auto id = header->Identifier;
    if (id < kChassisTargetId || id > kRefereePowerId) { return; }
    if (header->IdType != FDCAN_STANDARD_ID || header->RxFrameType != FDCAN_DATA_FRAME ||
        header->FDFormat != FDCAN_CLASSIC_CAN) { ++statistics.rejected; return; }
    const uint32_t now = HAL_GetTick();
    switch (id) {
    case kChassisTargetId: receive(targets.chassis, data, length, now, kChassisTimeoutMs); break;
    case kBigYawTargetId: receive(targets.yaw, data, length, now, kBigYawTimeoutMs); break;
    case kRefereePowerId: receive(targets.referee, data, length, now, kChassisTimeoutMs); break;
    default: break;
    }
}
}
bool init() {
    if (initialized) { return true; }
    initialized = bspFdcanAddObserver(onCan);
    return initialized;
}
ReceivedTargets readTargets() {
    taskENTER_CRITICAL();
    const auto result = targets;
    taskEXIT_CRITICAL();
    return result;
}
Statistics getStatistics() {
    taskENTER_CRITICAL();
    const auto result = statistics;
    taskEXIT_CRITICAL();
    return result;
}
template<class Frame> bool sendFeedbackFrame(uint16_t id, const Frame& frame) {
    const bool ok = bspFdcanSend(&hfdcan3, id,
        reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
    taskENTER_CRITICAL();
    ++statistics.tx_attempts;
    if (!ok) { ++statistics.tx_failures; }
    taskEXIT_CRITICAL();
    return ok;
}
bool send(const FeedbackFrame& frame) { return sendFeedbackFrame(kFeedbackId, frame); }
} }

