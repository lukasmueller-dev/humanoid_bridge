#include "rclcpp/rclcpp.hpp"
#include "H1_bridge.hpp"


int main(int argc, char **argv)
try
{
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("robot_bridge", options);

    rclcpp::sleep_for(std::chrono::milliseconds(100));

    std::string robot_name;
    nh->get_parameter("robot_name", robot_name);

    std::shared_ptr<sairol_bridge::BridgeCore> bridge;

    try
    {
        auto h1_bridge = std::make_shared<sairol_bridge::H1Bridge>(nh);
        bridge = h1_bridge;
    }
    catch (const sairol_bridge::ConfigError &)   // logged: "Failed to load parameters."
    {
        rclcpp::shutdown();
        return 1;
    }

    if (bridge && rclcpp::ok())   // not after Ctrl-C in the constructor's waits: spin would throw
    {   
        bridge->start();
        rclcpp::spin(nh);
    }

    bridge->stop();
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
