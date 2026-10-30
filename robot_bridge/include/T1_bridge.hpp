#ifndef SAIROL_BRIDGE_T1_BRIDGE_HPP_
#define SAIROL_BRIDGE_T1_BRIDGE_HPP_

#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>
#include <array>
#include "bridge_core.hpp"
#include "t1_parallel_ankle.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/remote_controller_state.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"

#include <booster/robot/common/robot_shared.hpp>


namespace sairol_bridge {


enum WirelessKey_T1 : uint32_t
{
    Button_X      = 1 << 0,  // 0b00000001 = 1
    Button_A      = 1 << 1,  // 0b00000010 = 2
    Button_B      = 1 << 2,  // 0b00000100 = 4
    Button_Y      = 1 << 3,  // 0b00001000 = 8
    Button_LB     = 1 << 4,  // 0b00010000 = 16
    Button_RB     = 1 << 5,  // 0b00100000 = 32
    Button_LT     = 1 << 6,  // 0b01000000 = 64
    Button_RT     = 1 << 7,  // 0b10000000 = 128
    Button_BACK   = 1 << 8,  // 0b100000000 = 256
    Button_START  = 1 << 9   // 0b1000000000 = 512
};


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
    void finishControl_() override;
    void loadParallelAnkleParameters_();

    // Parallel ankle mapping (t1_parallel_ankle.hpp): when enabled, the bridge sends CMD_TYPE_PARALLEL and turns the
    // serial ankle commands (pitch, roll) into crank-motor commands itself instead of the Booster firmware.
    struct AnkleMapping
    {
        t1_parallel_ankle::Vec2 q_m{0.0, 0.0}, dq_m{0.0, 0.0};  // crank state, XML convention
        t1_parallel_ankle::Vec2 q_s_firmware{0.0, 0.0};         // motor_state_serial, for the consistency check
        t1_parallel_ankle::CrankCommand crank;
        double estimate_error{0.0};
        bool ok{false};
    };
    int crankSlot_(int side, int crank) const;
    bool mapAnklesToCranks_(booster_interface::msg::LowCmd &cmd, std::array<AnkleMapping, 2> &mapping) const;

    static constexpr int kAnkleSerialIdx[2][2] = {{15, 16}, {21, 22}}; // [side][pitch, roll]; the two crank motors in parallel mode
    bool parallelAnkle_{false};
    bool parallelAnkleSwapCranks_{false};          // the pitch slot of motor_state_parallel is crank b, not crank a
    std::vector<double> parallelAnkleCrankSign_{1.0, 1.0}; // XML crank angle = sign * motor angle, per crank (a, b)
    double parallelAnkleCrankTauLimit_{t1_parallel_ankle::kCrankTorqueMax};
    double parallelAnkleMaxEstimateError_{0.05};   // stop if |f^-1(q_m) - firmware serial q| exceeds this [rad]
    std::vector<booster_interface::msg::MotorState> parallelState_;

    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr remoteControlSubscriber_;
    rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr lowCommandPublisher_;
    rclcpp::Client<booster_interface::srv::RpcService>::SharedPtr client_;


};
}
#endif // SAIROL_BRIDGE_T1_BRIDGE_HPP_