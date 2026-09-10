// NOT BUILT. No target in CMakeLists.txt compiles this file; it is upstream
// code kept for reference. The live G1 path is bridge_core.cpp + G1_bridge.cpp.
#include "key_event_handler.hpp"

//=======================For H1 and G1, the key mapping is the same ======================
void handle_key_event_unitree(
    int key,
    std::shared_ptr<rclcpp::Node> node,
    bool &control_started,
    unitree_go::msg::LowCmd &low_command,
    const unitree_go::msg::LowState &current_state,
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr publisher,
    int num_joint,
    double duration,
    std::function<bool()> init_control,
    std::function<void()> ready_position,
    std::function<void()> zero_position,
    std::function<void(double, int, bool)> calculate_interpolation,
    std::function<void(unitree_go::msg::LowCmd &)> compute_crc
)
{
    if ((key & (KEY_L2 | KEY_START)) == (KEY_L2 | KEY_START))  // L2 + START : start control
    {
        if (!control_started)
        {
            RCLCPP_INFO(node->get_logger(), "Starting control...");
            init_control();
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Control already started.");
        }
    }
    else if ((key & (KEY_L2 | KEY_UP | KEY_LEFT)) == (KEY_L2 | KEY_UP | KEY_LEFT))  // L2 + UP + LEFT : stop control
    {
        if (control_started)
        {
            RCLCPP_INFO(node->get_logger(), "Stopping control...");
            control_started = false;
            for (int i = 0; i < num_joint; i++)
            {
                low_command.motor_cmd[i].q = current_state.motor_state[i].q;
                low_command.motor_cmd[i].dq = 0.0;
                low_command.motor_cmd[i].tau = 0.0;
                low_command.motor_cmd[i].kp = 0.0;
                low_command.motor_cmd[i].kd = 0.0;
            }
            compute_crc(low_command);
            publisher->publish(low_command);
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Control not started yet.");
        }
    }
    else if (key & KEY_L1)  //L1 : Ready position
    {
        if (!control_started)
            init_control();
        RCLCPP_INFO(node->get_logger(), "Setting ready position...");
        ready_position();
        calculate_interpolation(duration, 1, true);
    }
    else if (key & KEY_R1)  // R1 : Zero position
    {
        if (!control_started)
            init_control();
        RCLCPP_INFO(node->get_logger(), "Setting zero position...");
        zero_position();
        calculate_interpolation(duration, 1, true);
    }
}
