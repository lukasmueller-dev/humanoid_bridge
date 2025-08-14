#ifndef SAIROL_BRIDGE_T1_BRIDGE_HPP_
#define SAIROL_BRIDGE_T1_BRIDGE_HPP_

#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>
#include "bridge_core.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "key_event_handler.hpp"
#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/remote_controller_state.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"

#include <booster/robot/common/robot_shared.hpp>


namespace sairol_bridge {
class T1Bridge : public BridgeCore {
public:
    explicit T1Bridge(rclcpp::Node::SharedPtr node);
    void stop() override;

private:

    void wireless_callback(sensor_msgs::msg::Joy::SharedPtr message);
    void publishLowCommand_();
    void lowStateHandler_(booster_interface::msg::LowState::SharedPtr message);
    bool initControl_(bridge_interface::msg::RobotCmd default_cmd) override;
    void switch_mode(booster::robot::RobotMode target_mode);
    void switch_to_damping_mode();
    void switch_to_prepare_mode();
    void stopControlServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) override;
    void finishControl_();

    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSubscriber_;  
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr remoteControlSubscriber_;
    rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr lowCommandPublisher_;
    rclcpp::Client<booster_interface::srv::RpcService>::SharedPtr client_;


};
}
#endif // SAIROL_BRIDGE_T1_BRIDGE_HPP_