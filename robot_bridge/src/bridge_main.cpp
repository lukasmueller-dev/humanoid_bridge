#include "rclcpp/rclcpp.hpp"
#include "T1_bridge.hpp"
// #include "H1_bridge.hpp"
// #include "G1_bridge.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("robot_bridge", options);

    std::string robot_name;
    nh->get_parameter("robot_name", robot_name);

    std::shared_ptr<sairol_bridge::BridgeCore> bridge;

    if (robot_name == "T1")
    {
        bridge = std::make_shared<sairol_bridge::T1Bridge>(nh);
    }
    // else if (robot_name == "H1")
    // {
    //     bridge = std::make_shared<sairol_bridge::H1Bridge>(nh);
    // }
    // else if (robot_name == "G1")
    // {
    //     bridge = std::make_shared<sairol_bridge::G1Bridge>(nh);
    // }
    else
    {
        RCLCPP_ERROR(nh->get_logger(), "Unknown robot name: %s", robot_name.c_str());
        rclcpp::shutdown();
        return 1;
    }

    if (bridge)
    {
        bridge->start();
        rclcpp::spin(nh);
    }

    rclcpp::shutdown();
    return 0;
}
