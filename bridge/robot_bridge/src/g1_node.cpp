#include "rclcpp/rclcpp.hpp"
#include "G1_bridge.hpp"
#include "node_common.hpp"

#include <chrono>


int main(int argc, char **argv)
try
{
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("robot_bridge", options);

    rclcpp::sleep_for(std::chrono::milliseconds(100));

    std::string robot_name;
    nh->get_parameter("robot_name", robot_name);

    // The G1 reads its remote, so a config without the key still needs it to arm. Declared
    // before the bridge reads it, so the bridge logs the value it uses.
    if (!nh->has_parameter("estop.require_remote"))
    {
        RCLCPP_WARN(nh->get_logger(), "No estop.require_remote in the config: the G1 default is true, "
                                      "arming needs the remote");
        nh->declare_parameter("estop.require_remote", true);
    }

    std::shared_ptr<sairol_bridge::BridgeCore> bridge;

    try
    {
        auto g1_bridge = std::make_shared<sairol_bridge::G1Bridge>(nh);
        bridge = g1_bridge;
    }
    catch (const sairol_bridge::ConfigError &)   // logged: "Failed to load parameters."
    {
        rclcpp::shutdown();
        return 1;
    }

    // Read once above, while readers such as robot_web_commander read the live values: refuse every change.
    auto read_only = sairol_bridge::refuseParameterChanges(*nh);

    if (bridge && rclcpp::ok())   // not after Ctrl-C in the constructor's waits: spin would throw
    {
        bridge->start();
        rclcpp::spin(nh);
    }

    bridge->stop();
    sairol_bridge::exitIfTeardownHangs();
    return 0;
}
catch (const std::exception &)
{
    // Ctrl-C before spin (main's sleep, the parameter load, between an ok() check and the next
    // graph call): that call threw on the shut-down context. Exit as after spin; anything else aborts.
    if (!rclcpp::ok())
    {
        return 0;
    }
    throw;
}
