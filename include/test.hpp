// #pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class SignalPublisher : public rclcpp::Node {
public:
    SignalPublisher();

private:
    void publish_signal();

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
