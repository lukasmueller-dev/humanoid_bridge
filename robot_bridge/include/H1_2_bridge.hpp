#ifndef SAIROL_BRIDGE_H1_2_BRIDGE_HPP_
#define SAIROL_BRIDGE_H1_2_BRIDGE_HPP_

#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>
#include "bridge_core.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"
#include "common/motor_crc_hg.h"
#include "unitree_go/msg/wireless_controller.hpp"

namespace sairol_bridge {

enum WirelessKey_H1_2 : uint32_t
{
    KEY_R1     = 1 << 0,
    KEY_L1     = 1 << 1,
    KEY_START  = 1 << 2,
    KEY_SELECT = 1 << 3,
    KEY_R2     = 1 << 4,
    KEY_L2     = 1 << 5,
    KEY_A      = 1 << 8,
    KEY_B      = 1 << 9,
    KEY_X      = 1 << 10,
    KEY_Y      = 1 << 11,
    KEY_UP     = 1 << 12,
    KEY_RIGHT  = 1 << 13,
    KEY_DOWN   = 1 << 14,
    KEY_LEFT   = 1 << 15
};

enum H1_2_PRorAB { H1_2_PR = 0, H1_2_AB = 1 };

class H1_2Bridge : public BridgeCore {
public:
    explicit H1_2Bridge(rclcpp::Node::SharedPtr node);
    void stop() override;

private:
    void wireless_callback(unitree_go::msg::WirelessController::SharedPtr data);
    void publishLowCommand_() override;
    void lowStateHandler_(unitree_hg::msg::LowState::SharedPtr message);
    bool initControl_(bridge_interface::msg::RobotCmd default_cmd) override;
    void finishControl_() override;
    bool checkExternalPublisher_(std::string topic_name);

    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowStateSubscriber_;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr remoteControlSubscriber_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowCommandPublisher_;
    int mode_machine_{0};
};

}

#endif // SAIROL_BRIDGE_H1_2_BRIDGE_HPP_
