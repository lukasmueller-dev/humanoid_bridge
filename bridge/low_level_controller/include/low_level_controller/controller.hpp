// controller.hpp -------------------------------------------------------------
// A low-level controller: the bridge's 1 kHz tick runs one, loaded by name with pluginlib.
// No ROS or vendor types, so a controller builds and tests without the bridge.
#pragma once

#include <array>
#include <vector>

namespace low_level_controller
{
    // One joint, measured.
    struct JointState
    {
        float q{0};
        float dq{0};
        float tau_est{0};
    };

    // The base IMU.
    struct ImuState
    {
        std::array<float, 4> quaternion{1, 0, 0, 0};   // w, x, y, z
        std::array<float, 3> gyroscope{0, 0, 0};       // rad/s
        std::array<float, 3> accelerometer{0, 0, 0};   // m/s^2
        std::array<float, 3> rpy{0, 0, 0};             // rad
    };

    struct State
    {
        std::vector<JointState> joints;   // config joint order
        ImuState imu;
    };

    // One motor's command: the motor applies kp·(q - q_meas) + kd·(dq - dq_meas) + tau.
    struct MotorCommand
    {
        float q{0};
        float dq{0};
        float kp{0};
        float kd{0};
        float tau{0};
    };

    class Controller
    {
    public:
        virtual ~Controller() = default;

        // Each tick, on the control thread: no allocation, no blocking, no throw.
        // target: the interpolated, range- and speed-gated command; out: what the motors get, same size.
        // After this the bridge clamps tau to tau_limit and q to the torque window.
        virtual void compute(const State &state, const std::vector<MotorCommand> &target,
                             std::vector<MotorCommand> &out) = 0;
    };
} // namespace low_level_controller
