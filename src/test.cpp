#include "test.hpp"
#include <chrono>

using namespace std::chrono_literals;

SignalPublisher::SignalPublisher()
: Node("signal_publisher") {
    signalPublisher_ = this->create_publisher<unitree_go::msg::LowCmd>("/test_signal", 10);
    timer_ = this->create_wall_timer(20ms, std::bind(&SignalPublisher::publish_signal, this));//50hz
}

SignalPublisher::~SignalPublisher(){
    // Destructor implementation (if needed)
}

void SignalPublisher::publish_signal() {
    static double t = 0.0;
    double freq = 0.2;  // Hz，频率
    double amp = 0.2;   // 振幅
    double dt = 0.02;   // 每次调用时间间隔 = 20ms
    double omega = 2 * M_PI * freq;

    double q12 = amp * std::sin(omega * t);
    double q16 = amp * std::sin(omega * t + M_PI);

    lowCommand_.motor_cmd[12].q = q12;
    lowCommand_.motor_cmd[16].q = q16;

    // 发布消息
    signalPublisher_->publish(lowCommand_);

    // 时间推进
    t += dt;

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalPublisher>());
    rclcpp::shutdown();
    return 0;
}
