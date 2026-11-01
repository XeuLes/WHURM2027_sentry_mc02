#pragma once
#include "board_cfg.hpp"
//编译云台板：mingw32-make board-gimbal -j4
//查看底盘发来的消息：g_shared_state.chassis_data
//查看云台(本板)最近一次发送的消息：g_tx_debug
namespace board_cfg {

constexpr RoleConfig kConfig = {
    BoardRole::kAimGimbal, // role

    10U, // self_board_id
    9U, // peer_board_id

    false, // has_chassis
    true, // has_yaw
    true, // has_pitch
    true, // has_shooter

    false, // has_rc_receiver: remote control is forwarded by chassis board
    false, // has_referee: referee data is forwarded by chassis board
    true, // has_mini_pc

    RcSource::kBoardLink, // rc_source
    200U, // link_timeout_ms
};

} // namespace board_cfg
