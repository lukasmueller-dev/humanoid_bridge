// #pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "common/motor_crc.h"
#include "std_srvs/srv/set_bool.hpp" 
#include <vector>
#include <limits>
#include <iostream>
#include <chrono>
#include <array>


class SignalPublisher : public rclcpp::Node {
public:
    SignalPublisher();
    ~SignalPublisher();

private:
    void publish_signal();

    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr signalPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;


    unitree_go::msg::LowCmd lowCommand_;
};
