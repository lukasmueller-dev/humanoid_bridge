// motor_pd and bridge_pd, loaded by name as BridgeCore does, against a fixed state.
// Run with the workspace sourced: build/robot_bridge/test_controllers
#include "low_level_controller/controller.hpp"

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>

#include <memory>
#include <string>
#include <vector>

using low_level_controller::Controller;
using low_level_controller::MotorCommand;
using low_level_controller::State;

namespace
{
    // Joint 0 lags its target; joint 1 is past it and moves the other way.
    State fixedState()
    {
        State state;
        state.joints = {{0.10f, 0.50f, 1.0f}, {-0.20f, -1.00f, -2.0f}};
        state.imu.rpy = {0.01f, -0.02f, 0.0f};
        return state;
    }

    // q, dq, kp, kd, tau
    const std::vector<MotorCommand> TARGET = {{0.30f, 0.00f, 50.0f, 2.0f, 0.5f},
                                              {-0.40f, 0.20f, 100.0f, 4.0f, -1.5f}};

    std::vector<MotorCommand> run(const std::string &name)
    {
        pluginlib::ClassLoader<Controller> loader("low_level_controller", "low_level_controller::Controller");
        std::vector<MotorCommand> out(TARGET.size());
        // Destroyed before the loader that holds its library.
        std::shared_ptr<Controller> controller = loader.createSharedInstance(name);
        controller->compute(fixedState(), TARGET, out);
        return out;
    }
} // namespace

TEST(MotorPd, SendsTheTargetAsIs)
{
    const auto out = run("motor_pd");
    ASSERT_EQ(out.size(), TARGET.size());
    for (size_t i = 0; i < TARGET.size(); ++i)
    {
        EXPECT_FLOAT_EQ(out[i].q, TARGET[i].q);
        EXPECT_FLOAT_EQ(out[i].dq, TARGET[i].dq);
        EXPECT_FLOAT_EQ(out[i].kp, TARGET[i].kp);
        EXPECT_FLOAT_EQ(out[i].kd, TARGET[i].kd);
        EXPECT_FLOAT_EQ(out[i].tau, TARGET[i].tau);
    }
}

TEST(BridgePd, SendsThePdAsTauWithZeroGains)
{
    const auto out = run("bridge_pd");
    ASSERT_EQ(out.size(), TARGET.size());
    // 50·(0.30 - 0.10) + 2·(0.00 - 0.50) + 0.5 and 100·(-0.40 + 0.20) + 4·(0.20 + 1.00) - 1.5
    const float tau[] = {9.5f, -16.7f};
    for (size_t i = 0; i < TARGET.size(); ++i)
    {
        EXPECT_NEAR(out[i].tau, tau[i], 1e-4f);
        EXPECT_FLOAT_EQ(out[i].kp, 0.0f);
        EXPECT_FLOAT_EQ(out[i].kd, 0.0f);
        EXPECT_FLOAT_EQ(out[i].q, TARGET[i].q);
        EXPECT_FLOAT_EQ(out[i].dq, TARGET[i].dq);
    }
}
