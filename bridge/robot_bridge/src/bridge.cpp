#include "bridge.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace sairol_h1;

RobotBridge::RobotBridge(): prepareCmd_(&RobotBridge::prepareCmdInterpolation_)
{
    auto topic_name = "lowstate";

    auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);

    nh = std::make_shared<rclcpp::Node>("robot_bridge", options);

    rclcpp::sleep_for(std::chrono::milliseconds(100));
    // Check and close any redundant publishers that may exist
    checkExternalPublisher_();

    lowStateSubscriber_ = nh->create_subscription<unitree_go::msg::LowState>(
        topic_name, 10, std::bind(&RobotBridge::lowStateHandler_, this, _1));

    remoteControlSubscriber_ = nh->create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", 10, std::bind(&RobotBridge::wireless_callback, this, _1));

    desiredSubscriber_ = nh->create_subscription<bridge_interface::msg::RobotCmd>(
        "/robot_cmd", 10, std::bind(&RobotBridge::robotCmdCallBack_, this, _1));

    lowCmdPublisher_ = nh->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    startControlService_ = nh->create_service<std_srvs::srv::Trigger>(
        "start_control", std::bind(&RobotBridge::startControlServiceCB_, this, _1, _2));

    stopControlService_ = nh->create_service<std_srvs::srv::Trigger>(
        "stop_control", std::bind(&RobotBridge::stopControlServiceCB_, this, _1, _2));


    readyPositionService_ = nh->create_service<std_srvs::srv::Trigger>(
        "ready_position_control", std::bind(&RobotBridge::readyPositionControlServiceCB_, this, _1, _2));

    zeroPositionService_ = nh->create_service<std_srvs::srv::Trigger>(
        "zero_position_control", std::bind(&RobotBridge::zeroPositionControlServiceCB_, this, _1, _2));

    // Load parameters
    auto ret = loadParameters_();
    if (!ret)
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to load parameters");
        rclcpp::shutdown();
    }

    if (torqueControl_)
    {
        throw std::runtime_error("Torque control mode is not supported yet.");
        prepareCmd_ = &RobotBridge::prepareCmdTorque_;
        RCLCPP_INFO(nh->get_logger(), "Torque control mode enabled.");
    }
    else
    {
        prepareCmd_ = &RobotBridge::prepareCmdInterpolation_;
        RCLCPP_INFO(nh->get_logger(), "Interpolation control mode enabled.");
    }

    // Waiting for publisher on topic /lowstate
    RCLCPP_INFO(nh->get_logger(), "Waiting for publisher on topic /lowstate...");
    while (nh->count_publishers(topic_name) == 0)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(nh->get_logger(), "Publisher detected on topic /lowstate.");

    last_state_time_ = nh->get_clock()->now();

    lowCommandDesired_.motor_cmd.resize(numJoint_);

    // Init Thread
    controlThread_ = std::thread([this]()
                                 { this->update_(); });
}

sairol_h1::RobotBridge::~RobotBridge()
{
    if (controlThread_.joinable())
    {
        controlThread_.join();
    }

    for (int i = 0; i < numJoint_; i++)
    {
        lowCommand_.motor_cmd[i].q = currentState_.motor_state[i].q;
        lowCommand_.motor_cmd[i].dq = 0.0;
        lowCommand_.motor_cmd[i].tau = 0.0;
        lowCommand_.motor_cmd[i].kp = 0.0;
        lowCommand_.motor_cmd[i].kd = 0.0;
    }
    get_crc(lowCommand_);
    lowCmdPublisher_->publish(lowCommand_);
}

void RobotBridge::checkExternalPublisher_()
{
    auto publishers_info = nh->get_publishers_info_by_topic("/lowcmd");
    int publisher_count = publishers_info.size();
    if (publisher_count > 0)
    {
        RCLCPP_ERROR(nh->get_logger(),
                     "Detected %d publishers on /lowcmd. Assuming external publisher exists."
                     "Please run: `ros2 run robot_bridge release_node` to release external node.",
                     publisher_count);
        std::exit(EXIT_FAILURE);
    }
}

bool RobotBridge::loadParameters_()
{
    nh->get_parameter("num_joint", numJoint_);

    cmdParams_ = std::vector<CmdParams>(numJoint_);

    if (!nh->get_parameter("torque_control", torqueControl_))
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'torque_control' from parameters");
        return false;
    }
    if (!nh->get_parameter("duration", duration_))
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'duration' from parameters");
        return false;
    }
    if (!nh->get_parameter("control_dt", controlDt_))
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'control_dt' from parameters");
        return false;
    }
    if (!nh->get_parameter("upper_limbs_kp_min", upper_limbs_kp_min_))
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'upper_limbs_kp_min' from parameters");
        return false;
    }
    if (!nh->get_parameter("upper_limbs_kp_max", upper_limbs_kp_max_))
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'upper_limbs_kp_max' from parameters");
        return false;
    }
    if (!nh->get_parameter("upper_limbs_kd_min", upper_limbs_kd_min_))
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'upper_limbs_kd_min' from parameters");
        return false;
    }
    if (!nh->get_parameter("upper_limbs_kd_max", upper_limbs_kd_max_))
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'upper_limbs_kd_max' from parameters");
        return false;
    }
    if (!nh->get_parameter("lower_limbs_kp_min", lower_limbs_kp_min_))
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'lower_limbs_kp_min' from parameters");
        return false;
    }
    if (!nh->get_parameter("lower_limbs_kp_min", lower_limbs_kp_min_))
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'lower_limbs_kp_min' from parameters");
        return false;
    }
    if (!nh->get_parameter("lower_limbs_kp_max", lower_limbs_kp_max_))
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'lower_limbs_kp_max' from parameters");
        return false;
    }
    if (!nh->get_parameter("lower_limbs_kd_min", lower_limbs_kd_min_))
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'lower_limbs_kd_min' from parameters");
        return false;
    }
    if (!nh->get_parameter("lower_limbs_kd_max", lower_limbs_kd_max_))
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'lower_limbs_kd_max' from parameters");
        return false;
    }
    if (!nh->get_parameter("empty_joint_index", emptyJointIndex_))
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'empty_joint_index' from parameters");
        return false;
    }

    RCLCPP_INFO(nh->get_logger(), "Number of the joints  = %d", numJoint_);
    RCLCPP_INFO(nh->get_logger(), "duration = %.3f", duration_);
    RCLCPP_INFO(nh->get_logger(), "control_dt = %.3f", controlDt_);
    RCLCPP_INFO(nh->get_logger(), "Upper limbs kp range: [%.3f, %.3f]", upper_limbs_kp_min_, upper_limbs_kp_max_);
    RCLCPP_INFO(nh->get_logger(), "Upper limbs kd range: [%.3f, %.3f]", upper_limbs_kd_min_, upper_limbs_kd_max_);
    RCLCPP_INFO(nh->get_logger(), "Lower limbs kp range: [%.3f, %.3f]", lower_limbs_kp_min_, lower_limbs_kp_max_);
    RCLCPP_INFO(nh->get_logger(), "Lower limbs kd range: [%.3f, %.3f]", lower_limbs_kd_min_, lower_limbs_kd_max_);

    std::vector<std::string> joint_names;
    if (nh->get_parameter("joint_names", joint_names))
    {
        joints_.clear();
        bool ret = true;
        for (const auto &name : joint_names)
        {
            Joint joint_info;
            ret = ret && nh->get_parameter(name + ".idx", joint_info.idx);
            ret = ret && nh->get_parameter(name + ".q_min", joint_info.q_min);
            ret = ret && nh->get_parameter(name + ".q_max", joint_info.q_max);
            ret = ret && nh->get_parameter(name + ".dq_limit", joint_info.dq_limit);
            ret = ret && nh->get_parameter(name + ".tau_limit", joint_info.tau_limit);
            ret = ret && nh->get_parameter(name + ".kp", joint_info.kp);
            ret = ret && nh->get_parameter(name + ".kd", joint_info.kd);
            joints_.push_back(joint_info);
        }
        if (!ret)
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get joint parameters from parameters");
            return false;
        }
        RCLCPP_INFO(nh->get_logger(), "Loaded %ld joint limits", joints_.size());
    }
    else
    {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'joint_names' from parameters");
        return false;
    }
    return true;
}

void RobotBridge::wireless_callback(unitree_go::msg::WirelessController::SharedPtr data)
{
    // RCLCPP_INFO(nh->get_logger(), "Wireless controller -- lx: %f; ly: %f; rx: %f; ry: %f; key value: %d",
    //             data->lx, data->ly, data->rx, data->ry, data->keys);

    handle_key_event_unitree(
    data->keys, nh, controlStarted_, lowCommand_, currentState_, lowCmdPublisher_,
    numJoint_, duration_,
    std::bind(&RobotBridge::initControl_, this),
    std::bind(&RobotBridge::readyPositionControl_, this),
    std::bind(&RobotBridge::zeroPositionControl_, this),
    std::bind(&RobotBridge::calculateInterpolationParams_, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
    get_crc
    );
}

void RobotBridge::robotCmdCallBack_(bridge_interface::msg::RobotCmd::SharedPtr message)
{
    lowCommandDesired_.motor_cmd = message->motor_cmd;
    auto duration = message->duration;
    auto interpolation_order = message->interpolation_order;
    auto hold_position = message->hold_position;


    if (not controlStarted_)
    {
        RCLCPP_WARN_ONCE(nh->get_logger(), "Control not started. Please start control first.");
        return;
    }

    if (checkCommand_())
    {
        calculateInterpolationParams_(duration, interpolation_order, hold_position);
    }
    else
    {
        RCLCPP_ERROR(nh->get_logger(), "Command check failed. Please inspect the command message.");
        return;
    }
}

void RobotBridge::lowStateHandler_(unitree_go::msg::LowState::SharedPtr message)
{
    imu_ = message->imu_state;
    currentState_.motor_state = message->motor_state;
    last_state_time_ = nh->get_clock()->now();
}

void RobotBridge::publishLowCommand_()
{
    rclcpp::Time current = nh->get_clock()->now();

    double phase = clamp((current.seconds() - tStart) / (tFinal - tStart), 0.0, 1.0);
    for (int i = 0; i < numJoint_; ++i)
    {
        auto &cmd = lowCommand_.motor_cmd[i];
        auto &joint_info = joints_[i];

        // Set motor command mode and initial values
        cmd.mode = (i < emptyJointIndex_) ? 0x0A : 0x01;
        cmd.q = clamp(cmdParams_[i].q_0 + cmdParams_[i].q_1 * phase, joint_info.q_min, joint_info.q_max);
        cmd.dq = clamp(cmdParams_[i].dq_0 + cmdParams_[i].dq_1 * phase, -joint_info.dq_limit, joint_info.dq_limit);
        
        // (this->*prepareCmd_)(phase, i);
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
            cmd.kp = clamp(cmd.kp, lower_limbs_kp_min_, lower_limbs_kp_max_);
            cmd.kd = clamp(cmd.kd, lower_limbs_kd_min_, lower_limbs_kd_max_);
        }
        else
        {
            cmd.kp = clamp(cmd.kp, upper_limbs_kp_min_, upper_limbs_kp_max_);
            cmd.kd = clamp(cmd.kd, upper_limbs_kd_min_, upper_limbs_kd_max_);
        }

        cmd.tau = clamp(cmd.tau, -joint_info.tau_limit, joint_info.tau_limit);
    }

    get_crc(lowCommand_);
    lowCmdPublisher_->publish(lowCommand_);
}

void RobotBridge::prepareCmdInterpolation_(double phase, int i)
{
    auto &cmd = lowCommand_.motor_cmd[i];
    auto &joint_info = joints_[i];
    cmd.tau = cmdParams_[i].tau_0 + cmdParams_[i].tau_1 * phase;
    cmd.kp = cmdParams_[i].kp_0 + cmdParams_[i].kp_1 * phase;
    cmd.kd = cmdParams_[i].kd_0 + cmdParams_[i].kd_1 * phase;

    // double max_delta_q = joint_info.dq_limit * controlDt_;
    // double delta_q_l = currentState_.motor_state[i].q - max_delta_q;
    // double delta_q_u = currentState_.motor_state[i].q + max_delta_q;
    // double q_min = std::max(joint_info.q_min, delta_q_l);
    // double q_max = std::min(joint_info.q_max, delta_q_u);
    // cmd.q = clamp(cmd.q, q_min, q_max);
}

void RobotBridge::prepareCmdTorque_(double phase, int i)
{  
    auto &cmd = lowCommand_.motor_cmd[i];
    auto &joint_info = joints_[i];
    auto kp = cmdParams_[i].kp_0 + cmdParams_[i].kp_1 * phase;
    auto kd = cmdParams_[i].kd_0 + cmdParams_[i].kd_1 * phase;
    auto tau_set = cmdParams_[i].tau_1; 

    cmd.tau = kp * (cmd.q - currentState_.motor_state[i].q) + kd * (cmd.dq - currentState_.motor_state[i].dq) + tau_set;
    cmd.kp = 0.0;
    cmd.kd = 0.0;
}

double RobotBridge::clamp(double value, double low, double high)
{
    assert(low <= high);
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

void RobotBridge::zeroPositionControl_()
{
    for (int i = 0; i < numJoint_; ++i)
    {
        lowCommandDesired_.motor_cmd[i].q = 0.0;
        lowCommandDesired_.motor_cmd[i].dq = 0.0;
        lowCommandDesired_.motor_cmd[i].tau = 0.0;
        lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
        lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
    }
}

void RobotBridge::readyPositionControl_()
{
    for (int i = 0; i < numJoint_; ++i)
    {
        lowCommandDesired_.motor_cmd[i].tau = 0.0;
        lowCommandDesired_.motor_cmd[i].dq = 0.0;
        lowCommandDesired_.motor_cmd[i].q = 0.0;
        lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
        lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
    }
    lowCommandDesired_.motor_cmd[13].q = -1.0;
    lowCommandDesired_.motor_cmd[15].q = 1.6;
    lowCommandDesired_.motor_cmd[17].q = 1.0;
    lowCommandDesired_.motor_cmd[19].q = 1.6;
}

bool RobotBridge::initControl_()
{
    if (controlStarted_)
    {
        return false;
    }
    for (int i = 0; i < numJoint_; i++)
    {
        lowCommandDesired_.motor_cmd[i].q = currentState_.motor_state[i].q;
        lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
        lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
    }

    calculateInterpolationParams_(0.0, 1, true);
    controlStarted_ = true;
    rclcpp::Rate rate(100);
    rate.sleep();
    return true;
}

void RobotBridge::update_()
{
    auto rate = rclcpp::Rate(1.0 / controlDt_);
    while (rclcpp::ok())
    {
        // Safety check
        if (not checkState_())
        {
            RCLCPP_ERROR(nh->get_logger(), "Robot state check failed. Please inspect the robot carefully.");
            return;
        }
        if (controlStarted_)
        {
            std::unique_lock<std::mutex> lock(mutex_);
            // Update the low command
            publishLowCommand_();
            lock.unlock();
            controlStarted_ = (nh->get_clock()->now()).seconds() < tValid;
        }
        rate.sleep();
    }
}

bool RobotBridge::checkState_()
{
    double dt_state_ = (nh->get_clock()->now() - last_state_time_).seconds();

    // Check if the state message is received within the expected interval
    if (dt_state_ > 0.1)
    {
        RCLCPP_ERROR(nh->get_logger(),
                     "Robot signal lost! No LowState message received for %.2f seconds. "
                     "Expected interval ~0.002s (500Hz). Shutting down the node to prevent unsafe operation.",
                     dt_state_);
        rclcpp::shutdown();
    }
    // Check if the state message has valid data
    if (currentState_.motor_state.size() != numJoint_)
    {
        RCLCPP_ERROR(nh->get_logger(),
                     "Motor state length mismatch: expected %d, got %ld",
                     numJoint_, currentState_.motor_state.size());
        return false;
    }
    // Check if the motor state values are valid (not NaN or Inf) and within limits
    for (size_t i = 0; i < numJoint_; ++i)
    {
        const auto &motor = currentState_.motor_state[i];
        if (!std::isfinite(motor.q) || !std::isfinite(motor.dq) ||
            !std::isfinite(motor.ddq) || !std::isfinite(motor.tau_est))
        {
            RCLCPP_ERROR(nh->get_logger(),
                         "Motor state at index %lu contains invalid (NaN/Inf) values.", i);
            return false;
        }

        const auto &joint = joints_[i];

        if (std::abs(motor.dq) > joint.dq_limit)
        {
            RCLCPP_ERROR(nh->get_logger(),
                         "Joint [%lu] dq (%.3f) exceeds limit (%.3f). Shutting down for safety.",
                         i, motor.dq, joint.dq_limit);
            rclcpp::shutdown();  
            return false;
        }
    }

    // Check if the IMU state values are valid (not NaN or Inf)
    for (int i = 0; i < 3; ++i)
    {
        if (!std::isfinite(imu_.rpy[i]) ||
            !std::isfinite(imu_.gyroscope[i]) ||
            !std::isfinite(imu_.accelerometer[i]))
        {
            RCLCPP_ERROR(nh->get_logger(),
                         "IMU state contains invalid (NaN/Inf) values at index %d.", i);
            return false;
        }
    }
    return true;
}

bool RobotBridge::checkCommand_()
{
    // Check if the command message has valid data
    if (lowCommandDesired_.motor_cmd.size() != numJoint_)
    {
        RCLCPP_ERROR(nh->get_logger(),
                     "Command check failed: motor_cmd size mismatch. Expected %ld, got %ld.",
                     joints_.size(), lowCommand_.motor_cmd.size());
        return false;
    }
    int any_value_clipped = -1;
    for (size_t i = 0; i < numJoint_; ++i)
    {
        auto &cmd = lowCommandDesired_.motor_cmd[i];
        auto &joint_info = joints_[i];
        // Check for invalid numbers
        if (!std::isfinite(cmd.q) || !std::isfinite(cmd.dq) ||
            !std::isfinite(cmd.tau) || !std::isfinite(cmd.kp) ||
            !std::isfinite(cmd.kd))
        {
            RCLCPP_WARN_ONCE(nh->get_logger(),
                             "Command check failed: motor_cmd[%lu] contains invalid (NaN/Inf) values.", i);
            return false; // Can't clip NaN/Inf, so still return false
        }
        // Check gains
        if (i < emptyJointIndex_)
        {
            if (joint_info.kp < lower_limbs_kp_min_ || joint_info.kp > lower_limbs_kp_max_)
            {
                cmd.kp = clamp(joint_info.kp, lower_limbs_kp_min_, lower_limbs_kp_max_);
                any_value_clipped = i;
            }
            if (joint_info.kd < lower_limbs_kd_min_ || joint_info.kd > lower_limbs_kd_max_)
            {
                cmd.kd = clamp(joint_info.kd, lower_limbs_kd_min_, lower_limbs_kd_max_);
                any_value_clipped = i;
            }
        }
        else
        {
            if (joint_info.kp < upper_limbs_kp_min_ || joint_info.kp > upper_limbs_kp_max_)
            {
                cmd.kp = clamp(joint_info.kp, upper_limbs_kp_min_, upper_limbs_kp_max_);
                any_value_clipped = i;
            }
            if (joint_info.kd < upper_limbs_kd_min_ || joint_info.kd > upper_limbs_kd_max_)
            {
                cmd.kd = clamp(joint_info.kd, upper_limbs_kd_min_, upper_limbs_kd_max_);
                any_value_clipped = i;
            }
        }
        // Check position
        if (cmd.q < joint_info.q_min || cmd.q > joint_info.q_max)
        {
            RCLCPP_WARN_ONCE(nh->get_logger(), "Clipping motor_cmd[%lu] q: %.3f", i, cmd.q);
            cmd.q = clamp(cmd.q, joint_info.q_min, joint_info.q_max);
            any_value_clipped = i;
        }
        // Check velocity
        if (cmd.dq < -joint_info.dq_limit || cmd.dq > joint_info.dq_limit)
        {
            RCLCPP_WARN_ONCE(nh->get_logger(), "Clipping motor_cmd[%lu] dq: %.3f", i, cmd.dq);
            cmd.dq = clamp(cmd.dq, -joint_info.dq_limit, joint_info.dq_limit);
            any_value_clipped = i;
        }
        // Check torque
        if (cmd.tau < -joint_info.tau_limit || cmd.tau > joint_info.tau_limit)
        {
            RCLCPP_WARN_ONCE(nh->get_logger(), "Clipping motor_cmd[%lu] tau: %.3f", i, cmd.tau);
            cmd.tau = clamp(cmd.tau, -joint_info.tau_limit, joint_info.tau_limit);
            any_value_clipped = i;
        }
        // Check gains
        auto kp_max_ = (i < emptyJointIndex_) ? lower_limbs_kp_max_ : upper_limbs_kp_max_;
        auto kp_min_ = (i < emptyJointIndex_) ? lower_limbs_kp_min_ : upper_limbs_kp_min_;
        auto kd_max_ = (i < emptyJointIndex_) ? lower_limbs_kd_max_ : upper_limbs_kd_max_;
        auto kd_min_ = (i < emptyJointIndex_) ? lower_limbs_kd_min_ : upper_limbs_kd_min_;
        if (cmd.kp < kp_min_ || cmd.kp > kp_max_)
        {
            RCLCPP_WARN_ONCE(nh->get_logger(), "Clipping motor_cmd[%lu] kp: %.3f", i, cmd.kp);
            cmd.kp = clamp(cmd.kp, kp_min_, kp_max_);
            any_value_clipped = i;
        }
        if (cmd.kd < kd_min_ || cmd.kd > kd_max_)
        {
            RCLCPP_WARN_ONCE(nh->get_logger(), "Clipping motor_cmd[%lu] kd: %.3f", i, cmd.kd);
            cmd.kd = clamp(cmd.kd, kd_min_, kd_max_);
            any_value_clipped = i;
        }
        if (cmd.kp == 0.0 and cmd.kd == 0.0)
        {
            cmd.kp = joint_info.kp;
            cmd.kd = joint_info.kd;
            RCLCPP_WARN_ONCE(nh->get_logger(), "motor_cmd[%lu] kp and kd are both zero."
                                               "Using default gains: [%.3f, %.3f].",
                             i, joint_info.kp, joint_info.kd);
        }

        if (any_value_clipped > 0)
        {
            RCLCPP_WARN(nh->get_logger(), "Joint %d were clipped to stay within safety limits", any_value_clipped);
        }
    }

    return true;
}

void RobotBridge::startControlServiceCB_(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (initControl_())
    {
        response->success = true;
        response->message = "Start control service activated";
    }
    else
    {
        response->success = false;
        response->message = "Failed to start control service";
    }
}

void RobotBridge::stopControlServiceCB_(
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
        for (int i = 0; i < numJoint_; i++)
        {
            lowCommand_.motor_cmd[i].q = currentState_.motor_state[i].q;
            lowCommand_.motor_cmd[i].dq = 0.0;
            lowCommand_.motor_cmd[i].tau = 0.0;
            lowCommand_.motor_cmd[i].kp = 0.0;
            lowCommand_.motor_cmd[i].kd = 0.0;
        }
        get_crc(lowCommand_);
        lowCmdPublisher_->publish(lowCommand_);

        response->success = true;
        response->message = "Stop control service activated";
    }
}

void RobotBridge::readyPositionControlServiceCB_(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!controlStarted_)
        initControl_();
    readyPositionControl_();
    calculateInterpolationParams_(duration_, 1, true);
    response->success = true;
    response->message = "Ready position control activated";
}

void RobotBridge::zeroPositionControlServiceCB_(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!controlStarted_)
        initControl_();
    zeroPositionControl_();
    calculateInterpolationParams_(duration_, 1, true);
    response->success = true;
    response->message = "Zero position control activated";
}

void RobotBridge::calculateInterpolationParams_(double duration, int interpolation_order, bool hold_position)
{
    checkCommand_();
    std::unique_lock<std::mutex> lock(mutex_);

    if (interpolation_order == 0)
    {
        for (int i = 0; i < numJoint_; i++)
        {
            cmdParams_[i].q_0 = lowCommandDesired_.motor_cmd[i].q;
            cmdParams_[i].q_1 = 0.0;
            cmdParams_[i].tau_0 = lowCommandDesired_.motor_cmd[i].tau;
            cmdParams_[i].tau_1 = 0.0;
            cmdParams_[i].dq_0 = lowCommandDesired_.motor_cmd[i].dq;
            cmdParams_[i].dq_1 = 0.0;
            cmdParams_[i].kp_0 = lowCommandDesired_.motor_cmd[i].kp;
            cmdParams_[i].kp_1 = 0.0;
            cmdParams_[i].kd_0 = lowCommandDesired_.motor_cmd[i].kd;
            cmdParams_[i].kd_1 = 0.0;
        }
    }
    else if (interpolation_order == 1)
    {
        for (int i = 0; i < numJoint_; i++)
        {
            cmdParams_[i].q_0 = lowCommand_.motor_cmd[i].q; // currentState_.motor_state[i].q;
            cmdParams_[i].q_1 = lowCommandDesired_.motor_cmd[i].q - cmdParams_[i].q_0;
            cmdParams_[i].tau_0 = lowCommandDesired_.motor_cmd[i].tau; // currentState_.motor_state[i].tau;
            cmdParams_[i].tau_1 = 0.0;
            cmdParams_[i].dq_0 = lowCommand_.motor_cmd[i].dq; // currentState_.motor_state[i].dq;
            cmdParams_[i].dq_1 = lowCommandDesired_.motor_cmd[i].dq - cmdParams_[i].dq_0;
            cmdParams_[i].kp_0 = lowCommand_.motor_cmd[i].kp; // currentState_.motor_state[i].kp;
            cmdParams_[i].kp_1 = lowCommandDesired_.motor_cmd[i].kp - cmdParams_[i].kp_0;
            cmdParams_[i].kd_0 = lowCommand_.motor_cmd[i].kd; // currentState_.motor_state[i].kd;
            cmdParams_[i].kd_1 = lowCommandDesired_.motor_cmd[i].kd - cmdParams_[i].kd_0;
        }
    }

    lock.unlock();

    tStart = nh->get_clock()->now().seconds();
    tFinal = tStart + duration;
    if (hold_position)
    {
        tValid = INF;
    }
    else
    {
        tValid = tFinal + 1.0;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::TimerBase::SharedPtr timer_;
    auto bridge = std::make_shared<RobotBridge>();
    rclcpp::spin(bridge->nh);
    rclcpp::shutdown();
    return 0;
}
