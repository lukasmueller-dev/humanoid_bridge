#include "T1_bridge.hpp"
#include "booster_interface/message_utils.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

sairol_bridge::T1Bridge::T1Bridge(rclcpp::Node::SharedPtr node) : BridgeCore(node)
{
    client_ = nh->create_client<booster_interface::srv::RpcService>("booster_rpc_service");

    while (!client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                            "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(nh->get_logger(), "service not available, waiting again...");
    }

    // switch_to_prepare_mode();

    lowCommandDesired_.motor_cmd.resize(numJoint_);
    lowCommand_.motor_cmd.resize(numJoint_);
    currentState_.motor_state.resize(numJoint_);
    cmdParams_.resize(numJoint_);

    lowStateSubscriber_ = nh->create_subscription<booster_interface::msg::LowState>(
        "/low_state", 1, std::bind(&sairol_bridge::T1Bridge::lowStateHandler_, this, std::placeholders::_1));

    // remoteControlSubscriber_ = nh->create_subscription<sensor_msgs::msg::Joy>(
    //     "/joy", 1, std::bind(&sairol_bridge::T1Bridge::wireless_callback, this, std::placeholders::_1));

    lowCommandPublisher_ = nh->create_publisher<booster_interface::msg::LowCmd>(
        "/joint_ctrl", 1); // /joint_ctrl

    // Waiting for publisher on topic lowstate
    RCLCPP_INFO(nh->get_logger(), "Waiting for publisher on topic /lowstate...");

    while (nh->count_publishers("/low_state") == 0)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

void sairol_bridge::T1Bridge::stop()
{
    // Switch to damping mode before stopping
    switch_to_damping_mode();

    // Stop the control thread if it's running
    if (controlThread_.joinable())
    {
        controlThread_.join();
    }

    RCLCPP_INFO(nh->get_logger(), "T1 Bridge stopped.");
}

void sairol_bridge::T1Bridge::wireless_callback(sensor_msgs::msg::Joy::SharedPtr message)
{
    auto buttons = message->buttons;
    uint32_t key = 0;
    if (buttons[0]) key |= Button_X;
    if (buttons[1]) key |= Button_A;
    if (buttons[2]) key |= Button_B;
    if (buttons[3]) key |= Button_Y;
    if (buttons[4]) key |= Button_LB;
    if (buttons[5]) key |= Button_RB;
    if (buttons[6]) key |= Button_LT;
    if (buttons[7]) key |= Button_RT;
    if (buttons[8]) key |= Button_BACK;
    if (buttons[9]) key |= Button_START;

    if (key == (Button_LT | Button_START))  // start: LT + START
    {
        RCLCPP_INFO(nh->get_logger(), "Starting control...");
        initControl_(bridge_interface::msg::RobotCmd());
        return;
    }
    else if (key == Button_LB)  // ready position: LB
    {
        RCLCPP_INFO(nh->get_logger(), "Ready position control...");
        if (!controlStarted_) initControl_(bridge_interface::msg::RobotCmd());
        readyPositionControl_();
        calculateInterpolationParams_(duration_, 1, true);
        return;
    }
    else if (key == Button_RB)  // zero position RB
    {
        RCLCPP_INFO(nh->get_logger(), "Zero position control...");
        if (!controlStarted_) initControl_(bridge_interface::msg::RobotCmd());
        zeroPositionControl_();
        calculateInterpolationParams_(duration_, 1, true);
        return;
    }
    else if (key == (Button_BACK | Button_LT))  // shutdown: BACK + LT
    {
        RCLCPP_INFO(nh->get_logger(), "Shutting down...");
        rclcpp::shutdown();
        return;
    }
    else if (key == Button_BACK)  // stop: BACK
    {
        RCLCPP_INFO(nh->get_logger(), "Stopping control...");
        switch_to_damping_mode();
        return;
    }

}

void sairol_bridge::T1Bridge::lowStateHandler_(booster_interface::msg::LowState::SharedPtr msg)
{        
    if (std::abs(msg->imu_state.rpy[0]) > imu_rpy_threshold_ || std::abs(msg->imu_state.rpy[1]) > imu_rpy_threshold_)
    {
        RCLCPP_WARN(nh->get_logger(), "IMU base rpy values are too large: [%f, %f]", msg->imu_state.rpy[0], msg->imu_state.rpy[1]);
        controlStarted_ = false;
        switch_to_damping_mode();
    }
    // Update the last state time using the same clock source
    last_state_time_ = nh->get_clock()->now();
    for (size_t i = 0; i < msg->motor_state_serial.size(); ++i)
    {
        currentState_.motor_state[i].q = msg->motor_state_serial[i].q;
        currentState_.motor_state[i].dq = msg->motor_state_serial[i].dq;
        currentState_.motor_state[i].ddq = msg->motor_state_serial[i].ddq;
        currentState_.motor_state[i].tau_est = msg->motor_state_serial[i].tau_est;
    }
}

void sairol_bridge::T1Bridge::publishLowCommand_()
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
        auto &cmd = lowCommand_.motor_cmd[i];
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
            if (!if_init_)
            {
                if (i == 15 || i == 16 || i == 21 || i == 22) // Special case for parallel joints
                {
                    cmd.tau = std::clamp((cmd.q - currentState_.motor_state[i].q) * cmd.kp, -joint_info.tau_limit, joint_info.tau_limit);
                    cmd.kp = 0.0;
                }
            }
            // cmd.tau = std::clamp(cmd.tau, -joint_info.tau_limit, joint_info.tau_limit);
            cmd.q = std::clamp(cmd.q, (-cmd.kd * (currentState_.motor_state[i].dq - cmd.dq) - joint_info.tau_limit) / cmd.kp + currentState_.motor_state[i].q, (-cmd.kd * (currentState_.motor_state[i].dq - cmd.dq) + joint_info.tau_limit) / cmd.kp + currentState_.motor_state[i].q);



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

    booster_interface::msg::LowCmd booster_cmd;
    booster_cmd.motor_cmd.resize(lowCommand_.motor_cmd.size());
    booster_cmd.cmd_type = booster_interface::msg::LowCmd::CMD_TYPE_SERIAL;
    for (size_t i = 0; i < lowCommand_.motor_cmd.size(); ++i)
    {
        booster_cmd.motor_cmd[i].mode = 0;
        booster_cmd.motor_cmd[i].q = lowCommand_.motor_cmd[i].q;
        booster_cmd.motor_cmd[i].dq = lowCommand_.motor_cmd[i].dq;
        booster_cmd.motor_cmd[i].tau = lowCommand_.motor_cmd[i].tau;
        booster_cmd.motor_cmd[i].kp = lowCommand_.motor_cmd[i].kp;
        booster_cmd.motor_cmd[i].kd = lowCommand_.motor_cmd[i].kd;
        booster_cmd.motor_cmd[i].weight = 1.0; // Default weight, can be adjusted later
    }
    lowCommandPublisher_->publish(booster_cmd);
}

bool sairol_bridge::T1Bridge::initControl_(bridge_interface::msg::RobotCmd default_cmd)
{
    booster_interface::msg::LowCmd booster_cmd;
    booster_cmd.motor_cmd.resize(numJoint_);
    for (size_t i = 0; i < numJoint_; ++i)
    {
        booster_cmd.motor_cmd[i].q = currentState_.motor_state[i].q;
        booster_cmd.motor_cmd[i].dq = 0.0;
        booster_cmd.motor_cmd[i].tau = 0.0;
        booster_cmd.motor_cmd[i].kp = joints_[i].kp;
        booster_cmd.motor_cmd[i].kd = joints_[i].kd;
    }

    lowCommandPublisher_->publish(booster_cmd);

    switch_mode(booster::robot::RobotMode::kCustom);

    last_state_time_ = nh->get_clock()->now();

    for (int i = 0; i < numJoint_; ++i)
    {
        lowCommandDesired_.motor_cmd[i].q = currentState_.motor_state[i].q;
        lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
        lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
    }

    calculateInterpolationParams_(0.0, 1, true);

    controlStarted_ = true;
    if_init_ = true;
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

void sairol_bridge::T1Bridge::stopControlServiceCB_(
const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(nh->get_logger(), "Stopping control service...");
    switch_to_damping_mode();
    response->success = true;
    response->message = "Stop control service activated";
}

void sairol_bridge::T1Bridge::switch_mode(booster::robot::RobotMode target_mode)
{
    booster_interface::srv::RpcService::Request::SharedPtr req = std::make_shared<booster_interface::srv::RpcService::Request>();
    req->msg = booster_interface::CreateChangeModeMsg(target_mode);

    using ServiceResponseFuture = rclcpp::Client<booster_interface::srv::RpcService>::SharedFuture;

    auto result_future = client_->async_send_request(
        req,
        [this](ServiceResponseFuture future)
        {
            try
            {
                auto response = future.get();
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(nh->get_logger(), "Service call failed: %s", e.what());
            }
        });
    // RCLCPP_INFO(nh->get_logger(), "Waiting 2s to ensure robot is ready...");
    // std::this_thread::sleep_for(2s);
}

void sairol_bridge::T1Bridge::switch_to_damping_mode()
{
    if (controlStarted_) controlStarted_ = false;
    switch_mode(booster::robot::RobotMode::kDamping);
    RCLCPP_INFO(nh->get_logger(), "Switched to damping mode.");
}

void sairol_bridge::T1Bridge::switch_to_prepare_mode()
{
    if (controlStarted_) controlStarted_ = false;
    switch_mode(booster::robot::RobotMode::kPrepare);
    RCLCPP_INFO(nh->get_logger(), "Switched to prepare mode.");
}

void sairol_bridge::T1Bridge::finishControl_(){
    // Switch to damping mode when control is finished
    switch_to_damping_mode();
    RCLCPP_INFO(nh->get_logger(), "Control finished, switched to damping mode.");
}