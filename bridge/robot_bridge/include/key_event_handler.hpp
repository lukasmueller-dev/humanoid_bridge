// NOT BUILT. No target in CMakeLists.txt compiles this file; it is upstream
// code kept for reference. The live G1 path is bridge_core.cpp + G1_bridge.cpp.
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "bridge_interface/msg/low_cmd.hpp"
#include "bridge_interface/msg/low_state.hpp"
#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/low_state.hpp"
#include <functional>
#include <memory>


//=======================For H1 and G1, the key mapping is the same ======================
enum WirelessKey_H1_G1 : uint32_t
{
    KEY_R1     = 1 << 0,    // 0b00000000 00000001 = 1
    KEY_L1     = 1 << 1,    // 0b00000000 00000010 = 2
    KEY_START  = 1 << 2,    // 0b00000000 00000100 = 4
    KEY_SELECT = 1 << 3,    // 0b00000000 00001000 = 8
    KEY_R2     = 1 << 4,    // 0b00000000 00010000 = 16
    KEY_L2     = 1 << 5,    // 0b00000000 00100000 = 32

    KEY_A      = 1 << 8,    // 0b00000001 00000000 = 256
    KEY_B      = 1 << 9,    // 0b00000010 00000000 = 512
    KEY_X      = 1 << 10,   // 0b00000100 00000000 = 1024
    KEY_Y      = 1 << 11,   // 0b00001000 00000000 = 2048

    KEY_UP     = 1 << 12,   // 0b00010000 00000000 = 4096
    KEY_RIGHT  = 1 << 13,   // 0b00100000 00000000 = 8192
    KEY_DOWN   = 1 << 14,   // 0b01000000 00000000 = 16384
    KEY_LEFT   = 1 << 15    // 0b10000000 00000000 = 32768
};

void handle_key_event_unitree(
    int key,
    std::shared_ptr<rclcpp::Node> node,
    bool &control_started,
    unitree_go::msg::LowCmd &low_command,
    const unitree_go::msg::LowState &current_state,
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr publisher,
    int num_joint,
    double duration,
    std::function<bool()> init_control,
    std::function<void()> ready_position,
    std::function<void()> zero_position,
    std::function<void(double, int, bool)> calculate_interpolation,
    std::function<void(unitree_go::msg::LowCmd &)> compute_crc
);



