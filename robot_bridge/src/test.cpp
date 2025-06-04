#include "test.hpp"
#include <chrono>
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float64.hpp" 
using namespace std::chrono_literals;

SignalPublisher::SignalPublisher()
: Node("signal_publisher") {
    signalPublisher_ = this->create_publisher<bridge_interface::msg::RobotCmd>("/robot_cmd", 10);
    timer_ = this->create_wall_timer(20ms, std::bind(&SignalPublisher::publish_signal, this));//50hz
    lowCommand_.motor_cmd.resize(20);

    ////test
    // testPublisher_ = this->create_publisher<bridge_interface::msg::TestSignal>("time_topic", 10);
    // testTimer_ = this->create_wall_timer(20ms, std::bind(&SignalPublisher::publish_signal, this));
    // relayscription_ = this->create_subscription<bridge_interface::msg::TestSignal>(
    // "time_topic_relay", 10,
    // std::bind(&SignalPublisher::topic_callback, this, std::placeholders::_1));

}

SignalPublisher::~SignalPublisher(){
    // Destructor implementation (if needed)
}

//test
// void SignalPublisher::topic_callback(const bridge_interface::msg::TestSignal::SharedPtr msg)

//     {   



//         auto now = this->get_clock()->now();
//         RCLCPP_INFO(this->get_logger(), "start: %.7f , return: %.7f , lag: %.7f", msg->time , now.seconds(), now.seconds() - msg->time);

//     }





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

    //test
    // auto now = rclcpp::Clock().now();
    // RCLCPP_INFO(rclcpp::get_logger("logger"), "current %d time: %.9f", i , now.seconds());
    // i++;
    // auto message = bridge_interface::msg::TestSignal();
    // for (int i = 0; i < 20; ++i) {
    //     info.motor_cmd[i].q = 0.0;
    //     info.motor_cmd[i].dq = 0.0;
    //     info.motor_cmd[i].tau = 0.0;
    //     info.motor_cmd[i].kp = i<9 ? 100.0 : 50.0;
    //     info.motor_cmd[i].kd = 1.0;
    // }
    
    // auto now = this->get_clock()->now();
    // message.time = now.seconds(); 
    // message.lowcmd = info;
    // testPublisher_->publish(message);

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalPublisher>());
    rclcpp::shutdown();
    return 0;
}
