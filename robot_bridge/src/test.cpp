#include "test.hpp"
#include <chrono>

using namespace std::chrono_literals;

SignalPublisher::SignalPublisher()
: Node("signal_publisher") {
    signalPublisher_ = this->create_publisher<bridge_interface::msg::RobotCmd>("/robot_cmd", 10);
    timer_ = this->create_wall_timer(20ms, std::bind(&SignalPublisher::publish_signal, this));//50hz
    lowCommand_.motor_cmd.resize(20);

    // for (int i = 0; i < 20; ++i) {
    //     lowCommand_.motor_cmd[i].q = 0.0;
    //     lowCommand_.motor_cmd[i].dq = 0.0;
    //     lowCommand_.motor_cmd[i].tau = 0.0;
    //     lowCommand_.motor_cmd[i].kp = i<9 ? 100.0 : 50.0;
    //     lowCommand_.motor_cmd[i].kd = 1.0;
    // }
}

SignalPublisher::~SignalPublisher(){
    // Destructor implementation (if needed)
}

void SignalPublisher::publish_signal() {
    static double t = 0.0;
    double freq = 0.3;  // Hz，frequency
    double amp = 0.3;   // ampitude
    double dt = 0.02;   // call interval = 20ms
    double omega = 2 * M_PI * freq;

    double q12 = amp * std::sin(omega * t);
    double q16 = amp * std::sin(omega * t + M_PI);

    lowCommand_.motor_cmd[12].q = q12;
    lowCommand_.motor_cmd[16].q = q16;
    lowCommand_.interpolation_order = 1;
    lowCommand_.hold_position = false;
    lowCommand_.duration = 0.02;
    
    signalPublisher_->publish(lowCommand_);

    
    t += dt;

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalPublisher>());
    rclcpp::shutdown();
    return 0;
}
