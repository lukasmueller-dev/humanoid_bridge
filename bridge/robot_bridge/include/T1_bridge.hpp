#ifndef SAIROL_BRIDGE_T1_BRIDGE_HPP_
#define SAIROL_BRIDGE_T1_BRIDGE_HPP_

#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>
#include <array>
#include "bridge_core.hpp"
#include "t1_parallel_ankle.hpp"
#include "booster_mode.hpp"
#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/remote_controller_state.hpp"


namespace sairol_bridge {

// Booster T1 on booster_interface: /low_state in, /joint_ctrl out, its mode over booster_rpc_service.
// Custom mode on arming, damping when the bridge stops. The remote is the T1 gamepad on
// /remote_controller_state (estop.remote_keys: t1).
class T1Bridge : public BridgeCore {
public:
    explicit T1Bridge(rclcpp::Node::SharedPtr node);
    void stop() override;

private:
    void lowStateHandler_(booster_interface::msg::LowState::SharedPtr message);
    void remoteHandler_(booster_interface::msg::RemoteControllerState::SharedPtr message);
    void publishLowCommand_() override;
    void publishStopCommand_() override;
    bool initControl_(bridge_interface::msg::RobotCmd default_cmd) override;
    void finishControl_() override;
    bool robotAllowsArming_(std::string &why) override;
    estop::Cause robotFault_() override;
    void watchJointCtrl_();
    void dampBeforeShutdown_();
    // Asks for the mode; the reply is logged. False when the service is not there.
    bool switchMode_(booster::RobotMode mode);
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
    // mutex_ held.
    bool mapAnklesToCranks_(booster_interface::msg::LowCmd &cmd, std::array<AnkleMapping, 2> &mapping) const;

    static constexpr int kAnkleSerialIdx[2][2] = {{15, 16}, {21, 22}}; // [side][pitch, roll]; the two crank motors in parallel mode
    bool parallelAnkle_{false};
    bool parallelAnkleSwapCranks_{false};          // the pitch slot of motor_state_parallel is crank b, not crank a
    std::vector<double> parallelAnkleCrankSign_{1.0, 1.0}; // XML crank angle = sign * motor angle, per crank (a, b)
    double parallelAnkleCrankTauLimit_{t1_parallel_ankle::kCrankTorqueMax};
    double parallelAnkleMaxEstimateError_{0.05};   // stop if |f^-1(q_m) - firmware serial q| exceeds this [rad]
    std::vector<booster_interface::msg::MotorState> parallelState_;

    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSubscriber_;
    rclcpp::Subscription<booster_interface::msg::RemoteControllerState>::SharedPtr remoteSubscriber_;
    rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr lowCommandPublisher_;
    rclcpp::Client<booster_interface::srv::RpcService>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr jointCtrlWatchTimer_;
    estop::Cause lastRemote_{estop::Cause::None};          // executor thread only
    estop::Cause lastJointCtrlFault_{estop::Cause::None};  // executor thread only
    bool stateLogged_{false};
};
}
#endif // SAIROL_BRIDGE_T1_BRIDGE_HPP_
