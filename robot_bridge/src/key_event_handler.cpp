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

// //=======================For T1, the key mapping is different from H1 and G1 ======================
// void handle_key_event_T1(
//     std::vector<int32_t> buttons,
//     std::shared_ptr<rclcpp::Node> node,
//     bool &control_started,
//     bridge_interface::msg::LowCmd &low_command,
//     const bridge_interface::msg::LowState &current_state,
//     rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr publisher,
//     int num_joint,
//     double duration,
//     std::function<void()> switch_to_damping_mode,
//     std::function<bool()> init_control,
//     std::function<void()> ready_position,
//     std::function<void()> zero_position,
//     std::function<void(double, int, bool)> calculate_interpolation
// )
// {
//     if (buttons[Button_LT] && buttons[Button_START])  // LT + START : start control
//     {
//         if (!control_started)
//         {
//             RCLCPP_INFO(node->get_logger(), "Starting control...");
//             init_control();
//         }
//         else
//         {
//             RCLCPP_WARN(node->get_logger(), "Control already started.");
//         }
//     }

//     else if (buttons[Button_LT] && buttons[Button_A] && buttons[Button_B])  // LT + A + B: stop control
//     {
//         if (control_started)
//         {
//             RCLCPP_INFO(node->get_logger(), "Stopping control...");
//             control_started = false;
//             switch_to_damping_mode();  
//         }
//         else
//         {
//             RCLCPP_WARN(node->get_logger(), "Control not started yet.");
//         }
//     }

//     else if (buttons[Button_LB])  // LB : Ready position
//     {
//         if (!control_started)
//             init_control();
//         RCLCPP_INFO(node->get_logger(), "Setting ready position...");
//         ready_position();
//         calculate_interpolation(duration, 1, true);
//     }

//     else if (buttons[Button_RB])  // RB : Zero position
//     {
//         if (!control_started)
//             init_control();
//         RCLCPP_INFO(node->get_logger(), "Setting zero position...");
//         zero_position();
//         calculate_interpolation(duration, 1, true);
//     }
// }