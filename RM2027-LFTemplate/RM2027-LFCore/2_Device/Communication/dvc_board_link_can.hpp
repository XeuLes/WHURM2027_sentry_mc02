#pragma once
#include "board_link_can.hpp"

namespace board_link { namespace can3 {
template<class Frame> struct Received {
    Frame data{};
    uint32_t rx_ms = 0;
    uint32_t revision = 0;
    bool seen = false;
    bool fresh(uint32_t now, uint32_t timeout) const {
        return seen && now - rx_ms < timeout;
    }
};
struct ReceivedTargets {
    Received<ChassisTargetFrame> chassis;
    Received<BigYawTargetFrame> yaw;
    Received<RefereePowerFrame> referee;
};
struct Statistics {
    uint32_t accepted = 0, rejected = 0, sequence_gaps = 0;
    uint32_t tx_attempts = 0, tx_failures = 0;
};
bool init();
ReceivedTargets readTargets();
Statistics getStatistics();
bool send(const FeedbackFrame& frame);
} }

