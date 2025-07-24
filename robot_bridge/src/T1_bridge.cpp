#include "T1_bridge.hpp"
#include "booster_interface/message_utils.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

sairol_bridge::T1Bridge::T1Bridge(rclcpp::Node::SharedPtr node) : BridgeCore(node)  
{
    client_ = nh->create_client<booster_interface::srv::RpcService>("booster_rpc_service");
    
    lowCommandDesired_.motor_cmd.resize(numJoint_);
    lowCommand_.motor_cmd.resize(numJoint_);
    currentState_.motor_state.resize(numJoint_);
    cmdParams_.resize(numJoint_);

    lowStateSubscriber_ = nh->create_subscription<booster_interface::msg::LowState>(
        "/low_state", 10, std::bind(&sairol_bridge::T1Bridge::lowStateHandler_, this, std::placeholders::_1));
    
    remoteControlSubscriber_ = nh->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&sairol_bridge::T1Bridge::wireless_callback, this, std::placeholders::_1));
    
    lowCommandPublisher_ = nh->create_publisher<booster_interface::msg::LowCmd>(
        "/joint_ctrl", 10);  // /joint_ctrl
    
    // Waiting for publisher on topic lowstate
    RCLCPP_INFO(nh->get_logger(), "Waiting for publisher on topic /lowstate...");

    while (nh->count_publishers("/low_state") == 0)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(nh->get_logger(), "Publisher detected on topic /lowstate.");

}


sairol_bridge::T1Bridge::~T1Bridge()
{
    if (controlThread_.joinable())
    {
        controlThread_.join();
    }
} 


void sairol_bridge::T1Bridge::wireless_callback(sensor_msgs::msg::Joy::SharedPtr message)
{
    // Handle the wireless callback logic here
    // This function can be used to process remote controller inputs or other wireless events
    // RCLCPP_INFO(nh->get_logger(), "Wireless callback triggered.");
    
    // // 打印手柄数据
    // RCLCPP_INFO(nh->get_logger(), "Joy message received:");
    // RCLCPP_INFO(nh->get_logger(), "  Axes count: %zu", message->axes.size());
    // for (size_t i = 0; i < message->axes.size(); ++i)
    // {
    //     RCLCPP_INFO(nh->get_logger(), "  Axis[%zu]: %.3f", i, message->axes[i]);
    // }
    
    // RCLCPP_INFO(nh->get_logger(), "  Buttons count: %zu", message->buttons.size());
    // for (size_t i = 0; i < message->buttons.size(); ++i)
    // {
    //     RCLCPP_INFO(nh->get_logger(), "  Button[%zu]: %d", i, message->buttons[i]);
    // }


    handle_key_event_T1(
    message->buttons, nh, controlStarted_, lowCommand_, currentState_, lowCommandPublisher_,
    numJoint_, duration_,
    std::bind(&sairol_bridge::T1Bridge::switch_to_damping_mode, this),
    std::bind(&sairol_bridge::T1Bridge::initControl_, this),
    std::bind(&sairol_bridge::T1Bridge::readyPositionControl_, this),
    std::bind(&sairol_bridge::T1Bridge::zeroPositionControl_, this),
    std::bind(&sairol_bridge::T1Bridge::calculateInterpolationParams_, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    );
}


void sairol_bridge::T1Bridge::lowStateHandler_(booster_interface::msg::LowState::SharedPtr msg)
{
    // Update the last state time using the same clock source
    last_state_time_ = nh->get_clock()->now();
    
    // // 打印接收到的状态数据
    // RCLCPP_INFO(nh->get_logger(), "Low state message received:");
    // RCLCPP_INFO(nh->get_logger(), "  Serial motors count: %zu", msg->motor_state_serial.size());
  
    
    // // 打印IMU数据
    // RCLCPP_INFO(nh->get_logger(), "  IMU RPY: [%.3f, %.3f, %.3f]", 
    //              msg->imu_state.rpy[0], msg->imu_state.rpy[1], msg->imu_state.rpy[2]);
    // RCLCPP_INFO(nh->get_logger(), "  IMU ACC: [%.3f, %.3f, %.3f]", 
    //              msg->imu_state.acc[0], msg->imu_state.acc[1], msg->imu_state.acc[2]);
    // RCLCPP_INFO(nh->get_logger(), "  IMU GYRO: [%.3f, %.3f, %.3f]", 
    //              msg->imu_state.gyro[0], msg->imu_state.gyro[1], msg->imu_state.gyro[2]);
    
    // Copy motor states to the base class (combine parallel and serial motors)
    // currentState_.motor_state.clear();
    // currentState_.motor_state.resize(msg->motor_state_serial.size());
    
    // Add serial motors
    for (size_t i = 0; i < msg->motor_state_serial.size(); ++i)
    {
        currentState_.motor_state[i].q = msg->motor_state_serial[i].q;
        currentState_.motor_state[i].dq = msg->motor_state_serial[i].dq;
        currentState_.motor_state[i].ddq = msg->motor_state_serial[i].ddq;
        currentState_.motor_state[i].tau_est = msg->motor_state_serial[i].tau_est;

        // RCLCPP_INFO(nh->get_logger(), "  Motor[%zu]: q=%.3f, dq=%.3f, ddq=%.3f, tau=%.3f", 
        //       i, currentState_.motor_state[i].q, currentState_.motor_state[i].dq, currentState_.motor_state[i].ddq, currentState_.motor_state[i].tau_est);
  
    }
    
    // Copy IMU data to the base class
    // for (int i = 0; i < 3; ++i)
    // {
    //     imu_.rpy[i] = msg->imu_state.rpy[i];
    //     imu_.gyroscope[i] = msg->imu_state.gyro[i];
    //     imu_.accelerometer[i] = msg->imu_state.acc[i];
    // }
  
    // // 将读到的状态作为期望命令 - 保持当前位置
    // for (size_t i = 0; i < msg->motor_state_serial.size(); ++i)
    // {
    //     lowCommandDesired_.motor_cmd[i].q = 0;//msg->motor_state_serial[i].q;    // 期望位置 = 当前位置
    //     lowCommandDesired_.motor_cmd[i].dq = 0.0;                            // 期望速度 = 0 (保持静止)
    //     lowCommandDesired_.motor_cmd[i].tau = 0.0;                           // 期望扭矩 = 0
    //     lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;                  // 使用默认 kp
    //     lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;                  // 使用默认 kd
    // }
    
    // RCLCPP_INFO(nh->get_logger(), "Updated desired command from current state");
  
    // // 调用 publishLowCommand_() 来发布当前状态
    // publishLowCommand_();
}


void sairol_bridge::T1Bridge::publishLowCommand_()
{
    // RCLCPP_INFO(nh->get_logger(), "Publishing low command.");
    rclcpp::Time current = nh->get_clock()->now();

    double phase = (current.seconds() - tStart_) / (tFinal_ - tStart_);
    // RCLCPP_INFO(nh->get_logger(), "Phase: %.3f ,tStart: %.3f, tFinal: %.3f, current: %.3f", phase, tStart_, tFinal_, current.seconds());
    if (phase == -INFINITY)
    {
        RCLCPP_WARN(nh->get_logger(), "Phase is -INFINITY ");
    }

    phase = (phase < 0.0) ? 0.0 : (phase > 1.0) ? 1.0 : phase;
    
    
    for (int i = 0; i < numJoint_; ++i)
    {
        auto &cmd = lowCommand_.motor_cmd[i];
        auto &joint_info = joints_[i];

        // Set motor command mode and initial values
        cmd.mode = (i < emptyJointIndex_) ? 0x0A : 0x01;
        
        // Clamp q within limits
        float q_target = cmdParams_[i].q_0 + cmdParams_[i].q_1 * phase;
        cmd.q = (q_target < joint_info.q_min) ? joint_info.q_min : 
                (q_target > joint_info.q_max) ? joint_info.q_max : q_target;
   

        // Clamp dq within limits  
        float dq_target = cmdParams_[i].dq_0 + cmdParams_[i].dq_1 * phase;
        cmd.dq = (dq_target < -joint_info.dq_limit) ? -joint_info.dq_limit :
                 (dq_target > joint_info.dq_limit) ? joint_info.dq_limit : dq_target;
        
    
        if (torqueControl_)
        {
            auto kp = cmdParams_[i].kp_0 + cmdParams_[i].kp_1 * phase;
            auto kd = cmdParams_[i].kd_0 + cmdParams_[i].kd_1 * phase;
            auto tau_set = cmdParams_[i].tau_1; 

            cmd.tau = kp * (cmd.q - currentState_.motor_state[i].q) + kd * (cmd.dq - currentState_.motor_state[i].dq) + tau_set;
            cmd.kp = 0.0;
            cmd.kd = 0.0;
        }
        else
        {   
            cmd.tau = 0.0; 
            cmd.kp = cmdParams_[i].kp_0 + cmdParams_[i].kp_1 * phase;
            cmd.kd = cmdParams_[i].kd_0 + cmdParams_[i].kd_1 * phase;

            auto tau_predict =  cmd.kp * (cmd.q - currentState_.motor_state[i].q) + cmd.kd * (cmd.dq - currentState_.motor_state[i].dq) + cmd.tau;
            
            auto error_q = cmd.q - currentState_.motor_state[i].q;
            auto error_dq = cmd.dq - currentState_.motor_state[i].dq;
            
            if (std::abs(tau_predict) > joint_info.tau_limit){
                RCLCPP_WARN(nh->get_logger(), "Tau prediction exceeds limit: %f > %f", std::abs(tau_predict), joint_info.tau_limit);
                double adjusted_factor = 1.0;
                auto A = 1;
                auto B = cmd.kd * error_dq / (cmd.kp * error_q);
                double sign = (tau_predict > 0) ? 1.0 : -1.0;
                auto C = -1 * joint_info.tau_limit *  sign / (cmd.kp * error_q);
                auto discriminant = B * B - 4 * A * C;
                if (discriminant >= 0) {
                    double sqrt_dis = std::sqrt(discriminant);
                    double x1 = (-B + sqrt_dis) / (2 * A);
                    double x2 = (-B - sqrt_dis) / (2 * A);
                    if (x1 > 0) adjusted_factor = x1;
                    else if (x2 > 0) adjusted_factor = x2;
                    else {
                        adjusted_factor = 0.0;
                        RCLCPP_WARN(nh->get_logger(), "No positive root found, using kp = 0.0, kd = 0.0");
                    }
                } else {
                    adjusted_factor = 0.0;
                    RCLCPP_WARN(nh->get_logger(), "Discriminant is negative, using kp = 0.0, kd = 0.0");
                }
                cmd.kp = cmd.kp * adjusted_factor;
                cmd.kd = cmd.kd * sqrt(adjusted_factor);

            }

            // double max_delta_q = joint_info.dq_limit * controlDt_;
            // double delta_q_l = currentState_.motor_state[i].q - max_delta_q;
            // double delta_q_u = currentState_.motor_state[i].q + max_delta_q;
            // double q_min = std::max(joint_info.q_min, delta_q_l);
            // double q_max = std::min(joint_info.q_max, delta_q_u);
            // cmd.q = clamp(cmd.q, q_min, q_max);
        }

        if (i < emptyJointIndex_)
        {
            cmd.kp = (cmd.kp < lower_limbs_kp_min_) ? lower_limbs_kp_min_ : 
                     (cmd.kp > lower_limbs_kp_max_) ? lower_limbs_kp_max_ : cmd.kp;
            cmd.kd = (cmd.kd < lower_limbs_kd_min_) ? lower_limbs_kd_min_ :
                     (cmd.kd > lower_limbs_kd_max_) ? lower_limbs_kd_max_ : cmd.kd;
        }
        else
        {
            cmd.kp = (cmd.kp < upper_limbs_kp_min_) ? upper_limbs_kp_min_ :
                     (cmd.kp > upper_limbs_kp_max_) ? upper_limbs_kp_max_ : cmd.kp;
            cmd.kd = (cmd.kd < upper_limbs_kd_min_) ? upper_limbs_kd_min_ :
                     (cmd.kd > upper_limbs_kd_max_) ? upper_limbs_kd_max_ : cmd.kd;
        }

        cmd.tau = (cmd.tau < -joint_info.tau_limit) ? -joint_info.tau_limit :
                  (cmd.tau > joint_info.tau_limit) ? joint_info.tau_limit : cmd.tau;
    }

    // 创建 booster_interface::msg::LowCmd 消息来发布
    booster_interface::msg::LowCmd booster_cmd;
    booster_cmd.motor_cmd.resize(lowCommand_.motor_cmd.size());
    booster_cmd.cmd_type = booster_interface::msg::LowCmd::CMD_TYPE_SERIAL;  
    for (size_t i = 0; i < lowCommand_.motor_cmd.size(); ++i)
    {
        // 将 bridge_interface 转换为 booster_interface
        // RCLCPP_INFO(nh->get_logger(), "Publishing motor command for motor %zu: q=%.3f, dq=%.3f, tau=%.3f, kp=%.3f, kd=%.3f",
        //     i, lowCommand_.motor_cmd[i].q, lowCommand_.motor_cmd[i].dq, lowCommand_.motor_cmd[i].tau,
        //     lowCommand_.motor_cmd[i].kp, lowCommand_.motor_cmd[i].kd);
        booster_cmd.motor_cmd[i].mode = 0;  
        booster_cmd.motor_cmd[i].q = lowCommand_.motor_cmd[i].q;
        booster_cmd.motor_cmd[i].dq = lowCommand_.motor_cmd[i].dq;
        booster_cmd.motor_cmd[i].tau = lowCommand_.motor_cmd[i].tau;
        booster_cmd.motor_cmd[i].kp = lowCommand_.motor_cmd[i].kp;
        booster_cmd.motor_cmd[i].kd = lowCommand_.motor_cmd[i].kd;
        booster_cmd.motor_cmd[i].weight = 1.0; // 设置权重为1.0，表示所有关节的命令都将被执行
    }

    lowCommandPublisher_->publish(booster_cmd);
}


void sairol_bridge::T1Bridge::readyPositionControl_()
{
    for (int i = 0; i < numJoint_; ++i)
    {
        lowCommandDesired_.motor_cmd[i].q = 0.0;
        lowCommandDesired_.motor_cmd[i].dq = 0.0;
        lowCommandDesired_.motor_cmd[i].tau = 0.0;
        lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
        lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
    }
    // 简单示例：抬手
    lowCommandDesired_.motor_cmd[3].q = -1.3;
    lowCommandDesired_.motor_cmd[7].q = 1.3;
    lowCommandDesired_.motor_cmd[5].q = -1.5;
    lowCommandDesired_.motor_cmd[9].q = 1.5;
}


bool sairol_bridge::T1Bridge::initControl_()
{
    if (controlStarted_)
        return false;

    booster_interface::msg::LowCmd booster_cmd;
    booster_cmd.motor_cmd.resize(numJoint_);
    for (size_t i = 0; i < numJoint_; ++i)
    {
        booster_cmd.motor_cmd[i].q = currentState_.motor_state[i].q;
        booster_cmd.motor_cmd[i].dq = 0.0;  // 初始速度为0
        booster_cmd.motor_cmd[i].tau = 0.0;  // 初始扭矩为0
        booster_cmd.motor_cmd[i].kp = joints_[i].kp;
        booster_cmd.motor_cmd[i].kd = joints_[i].kd;
    }

    lowCommandPublisher_->publish(booster_cmd);

    switch_mode(booster::robot::RobotMode::kCustom); // 切换到自定义模式


    last_state_time_ = nh->get_clock()->now();

    for (int i = 0; i < numJoint_; ++i)
        {
            lowCommandDesired_.motor_cmd[i].q = currentState_.motor_state[i].q;
            lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
            lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
        }

    calculateInterpolationParams_(0.0, 1, true);
    controlStarted_ = true;
    rclcpp::Rate rate(100);
    rate.sleep();
    RCLCPP_INFO(nh->get_logger(), "Control initialized successfully.");
    return true;
}


void sairol_bridge::T1Bridge::stopControlServiceCB_(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!controlStarted_)
    {
        response->success = false;
        response->message = "Control service is not started";
        return;
    }
    else
    {
        controlStarted_ = false;
        switch_to_damping_mode();  // 切换到阻尼模式
        response->success = true;
        response->message = "Stop control service activated";
    }
}


void sairol_bridge::T1Bridge::switch_mode(auto target_mode) {
    
    while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                        "Interrupted while waiting for the service. Exiting.");
            
        }
        RCLCPP_INFO(nh->get_logger(), "service not available, waiting again...");
    }

    booster_interface::srv::RpcService::Request::SharedPtr req = std::make_shared<booster_interface::srv::RpcService::Request>();
    req->msg = booster_interface::CreateChangeModeMsg(target_mode);
    json j = json::parse(req->msg.body);
    int mode = j["mode"];
    RCLCPP_INFO(nh->get_logger(), "There are four modes! Damping: 0; Prepare: 1; Walking: 2; Custom: 3. Currently mode is: %d.", mode);

    using ServiceResponseFuture = rclcpp::Client<booster_interface::srv::RpcService>::SharedFuture;

    auto result_future = client_->async_send_request(
        req,
        [this](ServiceResponseFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(nh->get_logger(), "RPC response: %s", response->msg.body.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(nh->get_logger(), "Service call failed: %s", e.what());
            }
        }
    );
    // RCLCPP_INFO(nh->get_logger(), "Waiting 2s to ensure robot is ready...");
    // std::this_thread::sleep_for(2s);
}


void sairol_bridge::T1Bridge::switch_to_damping_mode() {
    switch_mode(booster::robot::RobotMode::kDamping);
    RCLCPP_INFO(nh->get_logger(), "Switched to damping mode.");
}   