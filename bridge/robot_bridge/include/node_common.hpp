#pragma once

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

namespace sairol_bridge
{

// Refuse every parameter change. Keep the returned handle alive: dropping it removes the callback.
inline rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr refuseParameterChanges(rclcpp::Node &nh)
{
    return nh.add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "robot_bridge parameters are read-only: edit the config and restart the bridge";
        return result;
    });
}

// With the NIC down, Cyclone's dds_delete waits forever for its receive thread and the process
// never exits: end it if teardown takes over 3 s. Call after bridge->stop(), before main returns.
inline void exitIfTeardownHangs()
{
    std::thread([] {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        std::fflush(nullptr);
        std::_Exit(0);
    }).detach();
}

}  // namespace sairol_bridge
