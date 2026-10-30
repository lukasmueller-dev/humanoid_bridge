// controllers.cpp ------------------------------------------------------------
// The bridge's default low-level controllers, as pluginlib plugins (controllers.xml).
#include "low_level_controller/controller.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace sairol_bridge
{
    using low_level_controller::MotorCommand;
    using low_level_controller::State;

    // motor_pd: the target as is; the motor runs the PD.
    class MotorPd : public low_level_controller::Controller
    {
    public:
        void compute(const State &, const std::vector<MotorCommand> &target, std::vector<MotorCommand> &out) override
        {
            out = target;   // same size: no allocation
        }
    };

    // bridge_pd: the PD here at 1 kHz, sent as tau with kp = kd = 0.
    class BridgePd : public low_level_controller::Controller
    {
    public:
        void compute(const State &state, const std::vector<MotorCommand> &target,
                     std::vector<MotorCommand> &out) override
        {
            for (size_t i = 0; i < target.size(); ++i)
            {
                const auto &cmd = target[i];
                const auto &joint = state.joints[i];
                out[i] = cmd;
                out[i].tau = cmd.kp * (cmd.q - joint.q) + cmd.kd * (cmd.dq - joint.dq) + cmd.tau;
                out[i].kp = 0.0f;
                out[i].kd = 0.0f;
            }
        }
    };
} // namespace sairol_bridge

PLUGINLIB_EXPORT_CLASS(sairol_bridge::MotorPd, low_level_controller::Controller)
PLUGINLIB_EXPORT_CLASS(sairol_bridge::BridgePd, low_level_controller::Controller)
