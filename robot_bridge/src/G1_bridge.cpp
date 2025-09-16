#include "G1_bridge.hpp"
// #include "unitree_hg/message_utils.hpp"

using namespace std::chrono_literals;

sairol_bridge::G1Bridge::G1Bridge(rclcpp::Node::SharedPtr node) : BridgeCore(node)
{

    lowCommandDesired_.motor_cmd.resize(numJoint_);
    lastCommand_.motor_cmd.resize(numJoint_);
    currentState_.motor_state.resize(numJoint_);
    cmdParams_.resize(numJoint_);

    lowStateSubscriber_ = nh->create_subscription<unitree_hg::msg::LowState>(
        "/lowstate", 1, std::bind(&sairol_bridge::G1Bridge::lowStateHandler_, this, std::placeholders::_1));



    lowCommandPublisher_ = nh->create_publisher<unitree_hg::msg::LowCmd>(
        "/lowcmd", 1); // /joint_ctrl

    // Waiting for publisher on topic lowstate
    RCLCPP_INFO(nh->get_logger(), "Waiting for publisher on topic /lowstate...");

    while (nh->count_publishers("/lowstate") == 0)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}


void sairol_bridge::G1Bridge::stop()
{
    // Stop the control thread if it's running
    if (controlThread_.joinable())
    {
        controlThread_.join();
    }

    RCLCPP_INFO(nh->get_logger(), "T1 Bridge stopped.");
}

// void sairol_bridge::G1Bridge::wireless_callback(sensor_msgs::msg::Joy::SharedPtr message)
// {
//     auto buttons = message->buttons;
//     uint32_t key = 0;
//     if (buttons[0]) key |= Button_X;
//     if (buttons[1]) key |= Button_A;
//     if (buttons[2]) key |= Button_B;
//     if (buttons[3]) key |= Button_Y;
//     if (buttons[4]) key |= Button_LB;
//     if (buttons[5]) key |= Button_RB;
//     if (buttons[6]) key |= Button_LT;
//     if (buttons[7]) key |= Button_RT;
//     if (buttons[8]) key |= Button_BACK;
//     if (buttons[9]) key |= Button_START;

//     if (key == (Button_LT | Button_START))  // start: LT + START
//     {
//         RCLCPP_INFO(nh->get_logger(), "Starting control...");
//         initControl_(bridge_interface::msg::RobotCmd());
//         return;
//     }
//     else if (key == Button_LB)  // ready position: LB
//     {
//         RCLCPP_INFO(nh->get_logger(), "Ready position control...");
//         if (!controlStarted_) initControl_(bridge_interface::msg::RobotCmd());
//         readyPositionControl_();
//         calculateInterpolationParams_(duration_, 1, true);
//         return;
//     }
//     else if (key == Button_RB)  // zero position RB
//     {
//         RCLCPP_INFO(nh->get_logger(), "Zero position control...");
//         if (!controlStarted_) initControl_(bridge_interface::msg::RobotCmd());
//         zeroPositionControl_();
//         calculateInterpolationParams_(duration_, 1, true);
//         return;
//     }
//     else if (key == (Button_BACK | Button_LT))  // shutdown: BACK + LT
//     {
//         RCLCPP_INFO(nh->get_logger(), "Shutting down...");
//         rclcpp::shutdown();
//         return;
//     }
//     else if (key == Button_BACK)  // stop: BACK
//     {
//         RCLCPP_INFO(nh->get_logger(), "Stopping control...");
//         switch_to_damping_mode();
//         return;
//     }

// }

void sairol_bridge::G1Bridge::lowStateHandler_(unitree_hg::msg::LowState::SharedPtr msg)
{
    // Update the last state time using the same clock source
    last_state_time_ = nh->get_clock()->now();
    for (size_t i = 0; i < msg->motor_state.size(); ++i)
    {
        currentState_.motor_state[i].q = msg->motor_state[i].q;
        currentState_.motor_state[i].dq = msg->motor_state[i].dq;
        currentState_.motor_state[i].ddq = msg->motor_state[i].ddq;
        currentState_.motor_state[i].tau_est = msg->motor_state[i].tau_est;
    }
}

void sairol_bridge::G1Bridge::publishLowCommand_()
{
    rclcpp::Time current = nh->get_clock()->now();

    // float_t phase = (current.seconds() - tStart_) / (tFinal_ - tStart_);

    float_t phase = 1.0f;
    if (tFinal_ - tStart_ > 1e-6) {
        phase = (current.seconds() - tStart_) / (tFinal_ - tStart_);
    }

    phase = std::clamp(phase, 0.0f, 1.0f); // Ensure phase is between 0 and 1

    for (int i = 0; i < numJoint_; ++i)
    {
        auto &cmd = lastCommand_.motor_cmd[i];
        auto &joint_info = joints_[i];

        // Set motor command mode and initial values
        // cmd.mode = (i < emptyJointIndex_) ? 0x0A : 0x01;

        if (1e-6 < cmdInterpOrder_ && cmdInterpOrder_ < 1.0 - 1e-6)
        {
            // Low pass filter for cmd
            cmd.q = cmd.q * cmdInterpOrder_ + cmdParams_[i].q_0 * (1 - cmdInterpOrder_);
            cmd.dq = cmd.dq * cmdInterpOrder_ + cmdParams_[i].dq_0 * (1 - cmdInterpOrder_);
        }
        else {
            // Interpolation
            cmd.q = cmdParams_[i].q_0 + cmdParams_[i].q_1 * phase;
            cmd.dq = cmdParams_[i].dq_0 + cmdParams_[i].dq_1 * phase;
        }

        cmd.kp = cmdParams_[i].kp_0 + cmdParams_[i].kp_1 * phase;
        cmd.kd = cmdParams_[i].kd_0 + cmdParams_[i].kd_1 * phase;

        cmd.dq = std::clamp(cmd.dq, -joint_info.dq_limit, joint_info.dq_limit);

        if (torqueControl_)
        {
            cmd.tau = cmd.kp * (cmd.q - currentState_.motor_state[i].q) + cmd.kd * (cmd.dq - currentState_.motor_state[i].dq) + cmdParams_[i].tau_0;
            cmd.tau = std::clamp(cmd.tau, -joint_info.tau_limit, joint_info.tau_limit);
            cmd.kp = 0.0;
            cmd.kd = 0.0;
        }
        else
        {
            cmd.tau = cmdParams_[i].tau_0 + cmdParams_[i].tau_1 * phase;
            if (!receivedCmd_)
            {
                if (joints_[i].if_parallel_joint)
                {
                    cmd.tau = std::clamp((cmd.q - currentState_.motor_state[i].q) * cmd.kp, -joint_info.tau_limit, joint_info.tau_limit);
                    cmd.kp = 0.0;
                }
            
            }
            // cmd.tau = std::clamp(cmd.tau, -joint_info.tau_limit, joint_info.tau_limit);
            cmd.q = std::clamp(cmd.q, (-cmd.kd * (currentState_.motor_state[i].q - cmd.dq) - joint_info.tau_limit) / cmd.kp + currentState_.motor_state[i].q, (-cmd.kd * (currentState_.motor_state[i].q - cmd.dq) + joint_info.tau_limit) / cmd.kp + currentState_.motor_state[i].q);



            // auto tau_predict = cmd.kp * (cmd.q - currentState_.motor_state[i].q) + cmd.kd * (cmd.dq - currentState_.motor_state[i].dq) + cmd.tau;

            // auto error_q = cmd.q - currentState_.motor_state[i].q;
            // auto error_dq = cmd.dq - currentState_.motor_state[i].dq;

            // if (std::abs(error_q) < 1e-6)
            //     error_q = 1e-6; // Avoid division by zero

            // if (std::abs(tau_predict) > joint_info.tau_limit)
            // {
            //     RCLCPP_WARN(nh->get_logger(), "Tau prediction exceeds limit: %f > %f", std::abs(tau_predict), joint_info.tau_limit);
            //     double adjusted_factor = 1.0;
            //     auto A = 1;
            //     double B = cmd.kd * error_dq / (cmd.kp * error_q);
            //     double sign = (tau_predict > 0) ? 1.0 : -1.0;
            //     double C = -1 * joint_info.tau_limit * sign / (cmd.kp * error_q);
            //     double discriminant = B * B - 4 * A * C;
            //     if (discriminant >= 0)
            //     {
            //         double sqrt_dis = std::sqrt(discriminant);
            //         double x1 = (-B + sqrt_dis) / (2 * A);
            //         double x2 = (-B - sqrt_dis) / (2 * A);
            //         if (x1 > 0)
            //             adjusted_factor = x1;
            //         else if (x2 > 0)
            //             adjusted_factor = x2;
            //         else
            //         {
            //             adjusted_factor = 0.0;
            //             RCLCPP_WARN(nh->get_logger(), "No positive root found, using kp = 0.0, kd = 0.0");
            //         }
            //     }
            //     else
            //     {
            //         adjusted_factor = 0.0;
            //         RCLCPP_WARN(nh->get_logger(), "Discriminant is negative, using kp = 0.0, kd = 0.0");
            //     }
            //     cmd.kp = cmd.kp * adjusted_factor;
            //     cmd.kd = cmd.kd * sqrt(adjusted_factor);
            // }
        }    
    }

    unitree_hg::msg::LowCmd unitree_cmd;
    // unitree_cmd.motor_cmd.resize(lowCommand_.motor_cmd.size());
    // unitree_cmd.cmd_type = unitree_hg::msg::LowCmd::CMD_TYPE_SERIAL;
    for (size_t i = 0; i < lastCommand_.motor_cmd.size(); ++i)
        {
            unitree_cmd.motor_cmd[i].mode = 0;
            unitree_cmd.motor_cmd[i].q = lastCommand_.motor_cmd[i].q;
            unitree_cmd.motor_cmd[i].dq = lastCommand_.motor_cmd[i].dq;
            unitree_cmd.motor_cmd[i].tau = lastCommand_.motor_cmd[i].tau;
            unitree_cmd.motor_cmd[i].kp = lastCommand_.motor_cmd[i].kp;
            unitree_cmd.motor_cmd[i].kd = lastCommand_.motor_cmd[i].kd;
        }
    lowCommandPublisher_->publish(unitree_cmd);
}

bool sairol_bridge::G1Bridge::initControl_(bridge_interface::msg::RobotCmd default_cmd)
{
    last_state_time_ = nh->get_clock()->now();

    for (int i = 0; i < numJoint_; ++i)
    {
        lowCommandDesired_.motor_cmd[i].q = currentState_.motor_state[i].q;
        lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
        lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
    }

    calculateInterpolationParams_(0.0, 1, true);

    controlStarted_ = true;
    receivedCmd_ = true;
    rclcpp::Rate rate(100);
    rate.sleep();
    RCLCPP_INFO(nh->get_logger(), "Control initialized successfully.");
    
    if (default_cmd.motor_cmd.size() == numJoint_)
    {
        for (size_t i = 0; i < numJoint_; ++i)
        {
            lowCommandDesired_.motor_cmd[i].q = default_cmd.motor_cmd[i].q;
            lowCommandDesired_.motor_cmd[i].kp = default_cmd.motor_cmd[i].kp;
            lowCommandDesired_.motor_cmd[i].kd = default_cmd.motor_cmd[i].kd;
        }
    }
    else 
    {
        for (size_t i = 0; i < numJoint_; ++i)
        {
            lowCommandDesired_.motor_cmd[i].q = ready_q_[i];
            lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
            lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
        }
    }

    calculateInterpolationParams_(duration_, 1, true);

    return true;
}

void sairol_bridge::G1Bridge::finishControl_() {
    RCLCPP_INFO(nh->get_logger(), "finishControl_ called from G1Bridge");
}

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);

//     auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
//     rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("robot_bridge", options);

//     rclcpp::sleep_for(std::chrono::milliseconds(100));

//     std::string robot_name;
//     nh->get_parameter("robot_name", robot_name);

//     std::shared_ptr<sairol_bridge::BridgeCore> bridge;

//     auto g1_bridge = std::make_shared<sairol_bridge::G1Bridge>(nh);
//     bridge = g1_bridge;


//     if (bridge)
//     {   
//         bridge->start();
//         rclcpp::spin(nh);
//     }

//     bridge->stop();
//     return 0;
// }

