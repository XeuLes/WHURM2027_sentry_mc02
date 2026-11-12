#pragma once

#include "board_link_can.hpp"

namespace board_link {

struct Statistics {
    uint32_t tx_attempts;
    uint32_t tx_failures;
    uint32_t valid_rx_frames;
    uint32_t invalid_rx_frames;
    uint32_t sequence_gaps;
};

struct TxDebugState {
    uint32_t last_tx_ms;
    can3::ChassisTargetFrame last_chassis_payload;
    can3::BigYawTargetFrame last_yaw_payload;
    can3::RefereePowerFrame last_referee_payload;
};

extern TxDebugState g_tx_debug;
Statistics getStatistics();

namespace can3 {

template<class Frame> struct ReceivedFeedback {
    Frame data{};
    uint32_t rx_ms = 0U;
    uint32_t revision = 0U;
    bool seen = false;

    bool fresh(uint32_t now, uint32_t timeout) const {
        return seen && now - rx_ms < timeout;
    }
};
using ReceivedBoardFeedback = ReceivedFeedback<FeedbackValues>;

struct LoadDebugState {
    uint32_t sample_ms;
    uint32_t rx_frames;
    uint32_t tx_queued;
    uint32_t rx_loss_events;
    uint32_t tx_failures;
    uint32_t errors;
    uint32_t estimated_permille;
    bool bus_off;
};

extern LoadDebugState g_can3_load;

bool init();
ReceivedBoardFeedback readFeedback();
void updateBusLoad(uint32_t now_ms);

bool send(const ChassisTargetFrame& frame);
bool send(const BigYawTargetFrame& frame);
bool send(const RefereePowerFrame& frame);

} // namespace can3
} // namespace board_link
