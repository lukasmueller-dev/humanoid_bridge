#include "T1_bridge.hpp"
#include "booster_interface/message_utils.hpp"

#include <array>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

using json = nlohmann::json;
using namespace std::chrono_literals;

namespace
{
constexpr std::array<int, 4> kT1LowCmdDebugJointIndices = {15, 16, 21, 22};
constexpr std::array<const char *, 4> kT1LowCmdDebugJointNames = {
    "Left_Ankle_Pitch",
    "Left_Ankle_Roll",
    "Right_Ankle_Pitch",
    "Right_Ankle_Roll",
};

bool isDebugJoint(int joint_index)
{
    return std::find(
               kT1LowCmdDebugJointIndices.begin(),
               kT1LowCmdDebugJointIndices.end(),
               joint_index) != kT1LowCmdDebugJointIndices.end();
}

std::string debugJointName(int joint_index)
{
    for (size_t i = 0; i < kT1LowCmdDebugJointIndices.size(); ++i)
    {
        if (kT1LowCmdDebugJointIndices[i] == joint_index)
        {
            return kT1LowCmdDebugJointNames[i];
        }
    }
    return "joint_" + std::to_string(joint_index);
}

std::string timestampForFilename()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm tm_snapshot{};
#if defined(_WIN32)
    localtime_s(&tm_snapshot, &time);
#else
    localtime_r(&time, &tm_snapshot);
#endif
    std::ostringstream stream;
    stream << std::put_time(&tm_snapshot, "%Y%m%d_%H%M%S");
    return stream.str();
}

std::filesystem::path resolveDebugLogDirectory()
{
    const auto cwd = std::filesystem::current_path();
    const std::array<std::filesystem::path, 3> candidates = {
        cwd / "src" / "humanoid_bridge" / "robot_bridge" / "example" / "T1_lmj_walking" / "debug_logs",
        cwd / "robot_bridge" / "example" / "T1_lmj_walking" / "debug_logs",
        cwd / "t1_bridge_debug_logs",
    };

    for (const auto &candidate : candidates)
    {
        if (std::filesystem::exists(candidate.parent_path()))
        {
            std::filesystem::create_directories(candidate);
            return candidate;
        }
    }

    std::filesystem::create_directories(candidates.back());
    return candidates.back();
}

class T1LowCmdDebugLogger
{
public:
    void ensureInitialized(const rclcpp::Node::SharedPtr &node)
    {
        if (initialized_)
        {
            return;
        }
        initialized_ = true;

        try
        {
            const auto log_dir = resolveDebugLogDirectory();
            path_ = log_dir / ("t1_bridge_lowcmd_" + timestampForFilename() + ".jsonl");
            stream_.open(path_, std::ios::out);
            if (!stream_.is_open())
            {
                RCLCPP_WARN(node->get_logger(), "Failed to open T1 low command debug log: %s", path_.string().c_str());
                return;
            }

            json metadata;
            metadata["event"] = "metadata";
            metadata["log_path"] = path_.string();
            metadata["debug_joint_indices"] = kT1LowCmdDebugJointIndices;
            metadata["debug_joint_names"] = kT1LowCmdDebugJointNames;
            metadata["note"] = "T1 bridge final low-level ankle command debug log.";
            stream_ << metadata.dump() << "\n";
            stream_.flush();
            RCLCPP_INFO(node->get_logger(), "Writing T1 low command debug log to %s", path_.string().c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(node->get_logger(), "Failed to initialize T1 low command debug log: %s", e.what());
        }
    }

    bool isOpen() const
    {
        return stream_.is_open();
    }

    uint64_t nextPublishSequence()
    {
        return ++publish_sequence_;
    }

    void write(const json &row)
    {
        if (stream_.is_open())
        {
            stream_ << row.dump() << "\n";
            ++lines_since_flush_;
        }
    }

    void flush()
    {
        if (stream_.is_open() && lines_since_flush_ > 0)
        {
            stream_.flush();
            lines_since_flush_ = 0;
        }
    }

private:
    bool initialized_{false};
    uint64_t publish_sequence_{0};
    uint64_t lines_since_flush_{0};
    std::filesystem::path path_;
    std::ofstream stream_;
};

T1LowCmdDebugLogger &lowCmdDebugLogger()
{
    static T1LowCmdDebugLogger logger;
    return logger;
}
} // namespace

sairol_bridge::T1Bridge::T1Bridge(rclcpp::Node::SharedPtr node) : BridgeCore(node)
{
    client_ = nh->create_client<booster_interface::srv::RpcService>("booster_rpc_service");

    while (!client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(nh->get_logger(), "service not available, waiting again...");
    }

    // switch_to_prepare_mode();

    lowCommandDesired_.motor_cmd.resize(numJoint_);
    lastCommand_.motor_cmd.resize(numJoint_);
    currentState_.motor_state.resize(numJoint_);
    cmdParams_.resize(numJoint_);

    lowStateSubscriber_ = nh->create_subscription<booster_interface::msg::LowState>(
        "/low_state", 1, std::bind(&sairol_bridge::T1Bridge::lowStateHandler_, this, std::placeholders::_1));

    // remoteControlSubscriber_ = nh->create_subscription<sensor_msgs::msg::Joy>(
    //     "/joy", 1, std::bind(&sairol_bridge::T1Bridge::wireless_callback, this, std::placeholders::_1));

    lowCommandPublisher_ = nh->create_publisher<booster_interface::msg::LowCmd>("/joint_ctrl", 1); // /joint_ctrl

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

    if (key == (Button_LT | Button_RT | Button_START))  // start: LT + RT + START
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
    // Update the last state time using the same clock source
    last_state_time_ = nh->get_clock()->now();
    for (size_t i = 0; i < msg->motor_state_serial.size(); ++i)
    {
        currentState_.motor_state[i].q = msg->motor_state_serial[i].q;
        currentState_.motor_state[i].dq = msg->motor_state_serial[i].dq;
        currentState_.motor_state[i].ddq = msg->motor_state_serial[i].ddq;
        currentState_.motor_state[i].tau_est = msg->motor_state_serial[i].tau_est;
    }
    imu_.rpy = msg->imu_state.rpy;
    imu_.gyroscope = msg->imu_state.gyro;
    imu_.accelerometer = msg->imu_state.acc;
}

void sairol_bridge::T1Bridge::publishLowCommand_()
{
    rclcpp::Time current = nh->get_clock()->now();
    auto &debug_logger = lowCmdDebugLogger();
    debug_logger.ensureInitialized(nh);
    const uint64_t debug_publish_sequence = debug_logger.nextPublishSequence();

    float_t phase = 1.0f;
    if (tFinal_ - tStart_ > 1e-6) {
        phase = (current.seconds() - tStart_) / (tFinal_ - tStart_);
    }

    phase = std::clamp(phase, 0.0f, 1.0f); // Ensure phase is between 0 and 1

    booster_interface::msg::LowCmd booster_cmd;
    booster_cmd.motor_cmd.resize(lastCommand_.motor_cmd.size());
    booster_cmd.cmd_type = booster_interface::msg::LowCmd::CMD_TYPE_SERIAL;

    for (int i = 0; i < numJoint_; ++i)
    {
        auto &cmd = booster_cmd.motor_cmd[i];
        auto &last_cmd = lastCommand_.motor_cmd[i];
        auto &joint_info = joints_[i];
        const bool debug_joint = isDebugJoint(i);
        const auto state_q = currentState_.motor_state[i].q;
        const auto state_dq = currentState_.motor_state[i].dq;
        const auto state_tau_est = currentState_.motor_state[i].tau_est;
        const auto cmd_param_q_0 = cmdParams_[i].q_0;
        const auto cmd_param_q_1 = cmdParams_[i].q_1;
        const auto cmd_param_dq_0 = cmdParams_[i].dq_0;
        const auto cmd_param_dq_1 = cmdParams_[i].dq_1;
        const auto cmd_param_tau_0 = cmdParams_[i].tau_0;
        const auto cmd_param_tau_1 = cmdParams_[i].tau_1;
        const auto cmd_param_kp_0 = cmdParams_[i].kp_0;
        const auto cmd_param_kp_1 = cmdParams_[i].kp_1;
        const auto cmd_param_kd_0 = cmdParams_[i].kd_0;
        const auto cmd_param_kd_1 = cmdParams_[i].kd_1;

        booster_cmd.motor_cmd[i].mode = 0;
        booster_cmd.motor_cmd[i].weight = 1.0;

        if (1e-6 < cmdInterpOrder_ && cmdInterpOrder_ < 1.0 - 1e-6)
        {
            // Low pass filter for cmd
            cmd.q = last_cmd.q * cmdInterpOrder_ + cmdParams_[i].q_0 * (1 - cmdInterpOrder_);
            cmd.dq = last_cmd.dq * cmdInterpOrder_ + cmdParams_[i].dq_0 * (1 - cmdInterpOrder_);
        }
        else {
            // Interpolation
            cmd.q = cmdParams_[i].q_0 + cmdParams_[i].q_1 * phase;
            cmd.dq = cmdParams_[i].dq_0 + cmdParams_[i].dq_1 * phase;
        }
        const auto interp_q = cmd.q;
        const auto interp_dq = cmd.dq;

        cmd.kp = cmdParams_[i].kp_0 + cmdParams_[i].kp_1 * phase;
        cmd.kd = cmdParams_[i].kd_0 + cmdParams_[i].kd_1 * phase;
        const auto interp_kp = cmd.kp;
        const auto interp_kd = cmd.kd;
        const auto dq_before_clamp = cmd.dq;
        cmd.dq = std::clamp(cmd.dq, -joint_info.dq_limit, joint_info.dq_limit);
        const bool dq_clipped = cmd.dq != dq_before_clamp;

        float_t torque_control_raw_tau = cmdParams_[i].tau_0 + cmdParams_[i].tau_1 * phase;
        bool torque_control_tau_clipped = false;
        if (torqueControl_)
        {
            torque_control_raw_tau = cmd.kp * (cmd.q - currentState_.motor_state[i].q) + cmd.kd * (cmd.dq - currentState_.motor_state[i].dq) + cmdParams_[i].tau_0;
            cmd.tau = torque_control_raw_tau;
            cmd.tau = std::clamp(cmd.tau, -joint_info.tau_limit, joint_info.tau_limit);
            torque_control_tau_clipped = cmd.tau != torque_control_raw_tau;
            cmd.kp = 0.0;
            cmd.kd = 0.0;
        } else {
            cmd.tau = cmdParams_[i].tau_0 + cmdParams_[i].tau_1 * phase;
        }

        const auto q_before_torque_limit_clamp = cmd.q;
        const auto q_torque_limit_lower = (cmd.kd * (currentState_.motor_state[i].dq - cmd.dq) - joint_info.tau_limit) / cmd.kp + currentState_.motor_state[i].q;
        const auto q_torque_limit_upper = (cmd.kd * (currentState_.motor_state[i].dq - cmd.dq) + joint_info.tau_limit) / cmd.kp + currentState_.motor_state[i].q;
        cmd.q = std::clamp(cmd.q, q_torque_limit_lower, q_torque_limit_upper);
        const bool q_torque_limit_clipped = cmd.q != q_before_torque_limit_clamp;

        const auto pre_parallel_q = cmd.q;
        const auto pre_parallel_dq = cmd.dq;
        const auto pre_parallel_kp = cmd.kp;
        const auto pre_parallel_kd = cmd.kd;
        const auto pre_parallel_tau = cmd.tau;

        last_cmd.q = cmd.q;
        last_cmd.dq = cmd.dq;
        last_cmd.kp = cmd.kp;
        last_cmd.kd = cmd.kd;
        last_cmd.tau = cmd.tau;

        bool parallel_applied = false;
        float_t parallel_raw_tau = cmd.tau;
        bool parallel_tau_clipped = false;
        if (receivedCmd_ && joints_[i].if_parallel_joint)
        {
            parallel_applied = true;
            parallel_raw_tau = (last_cmd.q - currentState_.motor_state[i].q) * cmd.kp;
            cmd.tau = std::clamp(parallel_raw_tau, -joint_info.tau_limit, joint_info.tau_limit);
            parallel_tau_clipped = cmd.tau != parallel_raw_tau;
            cmd.q = currentState_.motor_state[i].q;
            cmd.kp = 0.0;
        }

        if (debug_joint && debug_logger.isOpen())
        {
            json row;
            row["event"] = "lowcmd_publish";
            row["publish_sequence"] = debug_publish_sequence;
            row["ros_time"] = current.seconds();
            row["joint_index"] = i;
            row["joint_name"] = debugJointName(i);
            row["phase"] = phase;
            row["cmd_interp_order"] = cmdInterpOrder_;
            row["torque_control"] = torqueControl_;
            row["control_started"] = controlStarted_;
            row["received_cmd"] = receivedCmd_;
            row["if_parallel_joint"] = joint_info.if_parallel_joint;
            row["if_strong_joint"] = joint_info.if_strong_joint;
            row["joint_limits"] = {
                {"q_min", joint_info.q_min},
                {"q_max", joint_info.q_max},
                {"dq_limit", joint_info.dq_limit},
                {"tau_limit", joint_info.tau_limit},
                {"configured_kp", joint_info.kp},
                {"configured_kd", joint_info.kd},
            };
            row["motor_state"] = {
                {"q", state_q},
                {"dq", state_dq},
                {"tau_est", state_tau_est},
            };
            row["cmd_params"] = {
                {"q_0", cmd_param_q_0},
                {"q_1", cmd_param_q_1},
                {"dq_0", cmd_param_dq_0},
                {"dq_1", cmd_param_dq_1},
                {"tau_0", cmd_param_tau_0},
                {"tau_1", cmd_param_tau_1},
                {"kp_0", cmd_param_kp_0},
                {"kp_1", cmd_param_kp_1},
                {"kd_0", cmd_param_kd_0},
                {"kd_1", cmd_param_kd_1},
            };
            row["after_interpolation"] = {
                {"q", interp_q},
                {"dq", interp_dq},
                {"kp", interp_kp},
                {"kd", interp_kd},
            };
            row["after_dq_clamp"] = {
                {"dq_before_clamp", dq_before_clamp},
                {"dq", cmd.dq},
                {"dq_clipped", dq_clipped},
            };
            row["torque_control_stage"] = {
                {"raw_tau", torque_control_raw_tau},
                {"tau", pre_parallel_tau},
                {"tau_clipped", torque_control_tau_clipped},
            };
            row["q_torque_limit_clamp"] = {
                {"q_before_clamp", q_before_torque_limit_clamp},
                {"q_lower", q_torque_limit_lower},
                {"q_upper", q_torque_limit_upper},
                {"q_after_clamp", pre_parallel_q},
                {"q_clipped", q_torque_limit_clipped},
            };
            row["pre_parallel_command"] = {
                {"q", pre_parallel_q},
                {"dq", pre_parallel_dq},
                {"kp", pre_parallel_kp},
                {"kd", pre_parallel_kd},
                {"tau", pre_parallel_tau},
                {"q_error", pre_parallel_q - state_q},
            };
            row["parallel_stage"] = {
                {"applied", parallel_applied},
                {"raw_tau", parallel_raw_tau},
                {"tau", cmd.tau},
                {"tau_clipped", parallel_tau_clipped},
                {"position_error_used_for_tau", pre_parallel_q - state_q},
            };
            row["final_published_command"] = {
                {"q", cmd.q},
                {"dq", cmd.dq},
                {"kp", cmd.kp},
                {"kd", cmd.kd},
                {"tau", cmd.tau},
                {"q_error", cmd.q - state_q},
            };
            debug_logger.write(row);
        }
    }

    debug_logger.flush();
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

    
    for (int i = 0; i < numJoint_; ++i)
    {
        lowCommandDesired_.motor_cmd[i].q = currentState_.motor_state[i].q;
        lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
        lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
    }

    calculateInterpolationParams_(0.0, 1, true);

    controlStarted_ = true;
    receivedCmd_ = false;
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

    last_state_time_ = nh->get_clock()->now();

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
