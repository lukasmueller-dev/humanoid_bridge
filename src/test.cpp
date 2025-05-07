#include "test.hpp"
#include <chrono>

using namespace std::chrono_literals;

SignalPublisher::SignalPublisher()
: Node("signal_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("test_signal", 10);
    timer_ = this->create_wall_timer(20ms, std::bind(&SignalPublisher::publish_signal, this));
}

void SignalPublisher::publish_signal() {
    auto message = std_msgs::msg::Float64();
    message.data = 1.23;
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published: '%f'", message.data);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalPublisher>());
    rclcpp::shutdown();
    return 0;
}
