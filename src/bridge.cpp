#include "bridge.hpp"


using std::placeholders::_1;
using std::placeholders::_2;  



RobotBridge::RobotBridge(){
    auto topic_name = "lf/lowstate";
    if (HIGH_FREQ) {
        topic_name = "lowstate";
    }
    
    auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true);
        
    nh = std::make_shared<rclcpp::Node>("robot_bridge", options);


    lowStateSubscriber_ = nh->create_subscription<unitree_go::msg::LowState>(
        topic_name, 10, std::bind(&RobotBridge::lowStateHandler_, this, _1));
    

    lowCmdPublisher_ = nh->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);


    startControlService_ = nh->create_service<std_srvs::srv::SetBool>(
        "start_control", std::bind(&RobotBridge::startControlServiceCB_, this, _1, _2));

    stopControlService_ = nh->create_service<std_srvs::srv::SetBool>(
        "stop_control", std::bind(&RobotBridge::stopControlServiceCB_, this, _1, _2));


    readyPositionService_ = nh->create_service<std_srvs::srv::SetBool>(
        "ready_position_control", std::bind(&RobotBridge::readyPositionControlServiceCB_, this, _1, _2));
    
    zeroPositionService_ = nh->create_service<std_srvs::srv::SetBool>(
        "zero_position_control", std::bind(&RobotBridge::zeroPositionControlServiceCB_, this, _1, _2));
    

    // Check and close any redundant publishers that may exist
    if(!releaseOtherNode_) checkForExternalPublisherAndRelease_();

    // Load parameters
    load_parameters_();

    //Waiting for publisher on topic /lowstate
    RCLCPP_INFO(nh->get_logger(), "Waiting for publisher on topic /lowstate...");
    while (nh->count_publishers(topic_name) == 0) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(nh->get_logger(), "Publisher detected on topic /lowstate.");

    last_state_time_ = nh->get_clock()->now();
        
    // Init Thread 
    controlThread_ = std::thread([this]() {
        this->update_();
    }); 
}


RobotBridge::~RobotBridge() {
    if (controlThread_.joinable()) {
        controlThread_.join();
    }
}


void RobotBridge::load_parameters_() {
    // nh->declare_parameter<int>("num_joint", 20);
    // nh->declare_parameter<double>("duration", 3.0);
    // nh->declare_parameter<double>("control_dt", 0.002);
    // nh->declare_parameter<int>("empty_joint_index", 9);
    // nh->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>());
    
    nh->get_parameter("num_joint", numJoint_);
    nh->get_parameter("duration", duration_);
    
    nh->get_parameter("control_dt", controlDt_);
    timerDt_ = static_cast<int>(controlDt_ * 1000);
    nh->get_parameter("upper_limbs_kp_min", upper_limbs_kp_min_);
    nh->get_parameter("upper_limbs_kp_max", upper_limbs_kp_max_);
    nh->get_parameter("upper_limbs_kd_min", upper_limbs_kd_min_);
    nh->get_parameter("upper_limbs_kd_max", upper_limbs_kd_max_);
    nh->get_parameter("lower_limbs_kp_min", lower_limbs_kp_min_);
    nh->get_parameter("lower_limbs_kp_max", lower_limbs_kp_max_);
    nh->get_parameter("lower_limbs_kd_min", lower_limbs_kd_min_);
    nh->get_parameter("lower_limbs_kd_max", lower_limbs_kd_max_);

    if(nh->get_parameter("empty_joint_index", emptyJointIndex_)) RCLCPP_INFO(nh->get_logger(), "empty_joint_index = %d", emptyJointIndex_);

    RCLCPP_INFO(nh->get_logger(), "Number of the joints  = %d", numJoint_);
    RCLCPP_INFO(nh->get_logger(), "duration = %.3f", duration_);
    RCLCPP_INFO(nh->get_logger(), "control_dt = %.3f", controlDt_);

    RCLCPP_INFO(nh->get_logger(), "Upper limbs kp range: [%.3f, %.3f]", upper_limbs_kp_min_, upper_limbs_kp_max_);
    RCLCPP_INFO(nh->get_logger(), "Upper limbs kd range: [%.3f, %.3f]", upper_limbs_kd_min_, upper_limbs_kd_max_);
    RCLCPP_INFO(nh->get_logger(), "Lower limbs kp range: [%.3f, %.3f]", lower_limbs_kp_min_, lower_limbs_kp_max_);
    RCLCPP_INFO(nh->get_logger(), "Lower limbs kd range: [%.3f, %.3f]", lower_limbs_kd_min_, lower_limbs_kd_max_);
    
    std::vector<std::string> joint_names;
    if(nh->get_parameter("joint_names", joint_names)) {
        joints_.clear();
        
        for(const auto& name : joint_names) {
            Joint limit;
            nh->get_parameter(name + ".idx", limit.idx);
            nh->get_parameter(name + ".q_min", limit.q_min);
            nh->get_parameter(name + ".q_max", limit.q_max);
            nh->get_parameter(name + ".dq_min", limit.dq_min);
            nh->get_parameter(name + ".dq_max", limit.dq_max);
            nh->get_parameter(name + ".tau_min", limit.tau_min);
            nh->get_parameter(name + ".tau_max", limit.tau_max);
            nh->get_parameter(name + ".kp", limit.kp);
            nh->get_parameter(name + ".kd", limit.kd);

            joints_.push_back(limit);
        }
        
        RCLCPP_INFO(nh->get_logger(), "Loaded %ld joint limits", joints_.size());
    } else {
        RCLCPP_ERROR(nh->get_logger(), "Failed to get 'joint_names' from parameters");
    }
}


void RobotBridge::lowStateHandler_(unitree_go::msg::LowState::SharedPtr message){
    imu_ = message->imu_state;
    currentState_.motor_state = message->motor_state;

    last_state_time_ = nh->get_clock()->now();
    
    if (INFO_IMU)
    {
        // Info imu_ states
        // RPY euler angle(ZYX order respected to body frame)
        // Quaternion
        // Gyroscope (raw data)
        // Accelerometer (raw data)
        // RCLCPP_INFO(nh->get_logger(), "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu_.rpy[0], imu_.rpy[1], imu_.rpy[2]);
        // RCLCPP_INFO(nh->get_logger(), "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f",
        //             imu_.quaternion[0], imu_.quaternion[1], imu_.quaternion[2], imu_.quaternion[3]);
        // RCLCPP_INFO(nh->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f", imu_.gyroscope[0], imu_.gyroscope[1], imu_.gyroscope[2]);
        // RCLCPP_INFO(nh->get_logger(), "Accelerometer -- ax: %f; ay: %f; az: %f",
        //             imu_.accelerometer[0], imu_.accelerometer[1], imu_.accelerometer[2]);
    }
    if (INFO_MOTOR)
    {
        // Info motor states
        // q: angluar (rad)
        // dq: angluar velocity (rad/s)
        // ddq: angluar acceleration (rad/(s^2))
        // tau_est: Estimated external torque
    }

}


void RobotBridge::publishLowCommand_(){

    rclcpp::Time current = nh->get_clock()->now();
    double phase = clamp((current.seconds() - tStart)/(tFinal-tStart), 0.0, 1.0);
    for (int i = 0; i < numJoint_; ++i) {
        lowCommand_.motor_cmd[i].mode = (i < emptyJointIndex_) ? 0x0A : 0x01;
        lowCommand_.motor_cmd[i].q = a0[i] + a1[i] * phase;
        lowCommand_.motor_cmd[i].tau = 0.0;
        lowCommand_.motor_cmd[i].dq = 0.0;
    }
    get_crc(lowCommand_);
    lowCmdPublisher_->publish(lowCommand_);
}


double RobotBridge::clamp(double value, double low, double high) 
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}


void RobotBridge::zeroPositionControl_(){

    for (int i = 0; i < numJoint_; ++i) 
    {   
        lowCommandDesired_.motor_cmd[i].q = 0.0;
        lowCommandDesired_.motor_cmd[i].tau = 0.0;
        lowCommandDesired_.motor_cmd[i].dq = 0.0;
        lowCommand_.motor_cmd[i].kp = joints_[i].kp;
        lowCommand_.motor_cmd[i].kd = joints_[i].kd;
    }

}


void RobotBridge::readyPositionControl_(){

    for (int i = 0; i < numJoint_; ++i) 
    {   
        lowCommandDesired_.motor_cmd[i].tau = 0.0;
        lowCommandDesired_.motor_cmd[i].dq = 0.0;

        double target_q = 0.0;
        if (i == 17) {
            target_q = 1.0;
        } 
        else if (i == 13) {
            target_q = -1.0;
        }
        else if (i == 15) {
            target_q = 1.6;
        }
        else if (i == 19) {
            target_q = 1.6;
        }
        lowCommandDesired_.motor_cmd[i].q = target_q;
        lowCommand_.motor_cmd[i].kp = joints_[i].kp;
        lowCommand_.motor_cmd[i].kd = joints_[i].kd;
    }
}


bool RobotBridge::initControl_() {

    for (int i = 0; i < numJoint_; i++)
    {
         a0[i] = currentState_.motor_state[i].q;
         a1[i] = 0.0;
         b0[i] = 0.0; //currentState_.motor_state[i].dq;
         b1[i] = 0.0;
    }
    rclcpp::Time now = nh->get_clock()->now();
    tStart = now.seconds();  
    tFinal = inf;
    controlStarted_ = true;
    
    rclcpp::Rate rate(10);
    rate.sleep();

    return true;
}


void RobotBridge::update_() {
    auto rate = rclcpp::Rate(1.0 / controlDt_);
    while (rclcpp::ok() ){
        // Safety check
        if (checkState_()){
            // pass
        }
        else {
            RCLCPP_ERROR(nh->get_logger(), "Robot state check failed. Please inspect the robot carefully.");
            return; 
        }

        if (controlStarted_  && checkCommand_()) {
            std::unique_lock<std::mutex> lock(mutex_);
            // Update the low command
            publishLowCommand_();
            lock.unlock();
        }
        rate.sleep();
    }
}


void RobotBridge::checkForExternalPublisherAndRelease_()
{
    
    auto publishers_info = nh->get_publishers_info_by_topic("/lowcmd");

    int publisher_count = publishers_info.size();

    if (publisher_count > 1) {
        RCLCPP_WARN(nh->get_logger(), 
            "Detected %d publishers on /lowcmd. Assuming external publisher exists. Executing release_node.", 
            publisher_count);
    
        int result = std::system("ros2 run robot_bridge release_node");
    
        if (result != 0) {
            RCLCPP_ERROR(nh->get_logger(), "Failed to execute release_node.");
        } else {
            RCLCPP_INFO(nh->get_logger(), "Successfully executed release_node.");
        }
    } else {
        RCLCPP_INFO(nh->get_logger(), "No external publishers detected on /lowcmd.");
    }
    releaseOtherNode_ = true;
    
}


bool RobotBridge::checkState_() {

    double dt_state_ = (nh->get_clock()->now() - last_state_time_).seconds();

    // Check if the state message is received within the expected interval
    if (dt_state_ > 1.0) {
        RCLCPP_ERROR(nh->get_logger(), 
            "Robot signal lost! No LowState message received for %.2f seconds. "
            "Expected interval ~0.002s (500Hz). Shutting down the node to prevent unsafe operation.",
            dt_state_);
        rclcpp::shutdown(); 
    }

    // Check if the state message has valid data
    if (currentState_.motor_state.size() != numJoint_) {
        RCLCPP_ERROR(nh->get_logger(), 
                     "Motor state length mismatch: expected %d, got %ld", 
                     numJoint_, currentState_.motor_state.size());
        return false;
    }

    // Check if the motor state values are valid (not NaN or Inf)
    for (size_t i = 0; i < numJoint_; ++i) {
        const auto& motor = currentState_.motor_state[i];
        if (!std::isfinite(motor.q) || !std::isfinite(motor.dq) || 
            !std::isfinite(motor.ddq) || !std::isfinite(motor.tau_est)) {
            RCLCPP_ERROR(nh->get_logger(), 
                         "Motor state at index %lu contains invalid (NaN/Inf) values.", i);
            return false;
        }
    }

    // Check if the IMU state values are valid (not NaN or Inf)
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(imu_.rpy[i]) || 
            !std::isfinite(imu_.gyroscope[i]) || 
            !std::isfinite(imu_.accelerometer[i])) {
            RCLCPP_ERROR(nh->get_logger(), 
                         "IMU state contains invalid (NaN/Inf) values at index %d.", i);
            return false;
        }
    }

    return true;
}


bool RobotBridge::checkCommand_() {
    // Check if the command message has valid data
    if (lowCommand_.motor_cmd.size() != numJoint_) {
        RCLCPP_ERROR(nh->get_logger(),
                    "Command check failed: motor_cmd size mismatch. Expected %ld, got %ld.",
                    joints_.size(), lowCommand_.motor_cmd.size());
        return false;
    }

    int any_value_clipped = -1;

    for (size_t i = 0; i < numJoint_; ++i) {
        auto& cmd = lowCommand_.motor_cmd[i];
        auto& limit = joints_[i];

        // Check for invalid numbers
        if (!std::isfinite(cmd.q) || !std::isfinite(cmd.dq) ||
            !std::isfinite(cmd.tau) || !std::isfinite(cmd.kp) ||
            !std::isfinite(cmd.kd)) {
            RCLCPP_WARN_ONCE(nh->get_logger(),
                            "Command check failed: motor_cmd[%lu] contains invalid (NaN/Inf) values.", i);
            return false;  // Can't clip NaN/Inf, so still return false
        }

        // Check gains
        if(i < emptyJointIndex_){

            if (limit.kp < lower_limbs_kp_min_ || limit.kp > lower_limbs_kp_max_) {
                cmd.kp = clamp(limit.kp, lower_limbs_kp_min_, lower_limbs_kp_max_);
                any_value_clipped = i;
            }
    
            if (limit.kd < lower_limbs_kd_min_ || limit.kd > lower_limbs_kd_max_) {
                cmd.kd = clamp(limit.kd, lower_limbs_kd_min_, lower_limbs_kd_max_);
                any_value_clipped = i;
            }
        }
        else{

            if (limit.kp < upper_limbs_kp_min_ || limit.kp > upper_limbs_kp_max_) {
                cmd.kp = clamp(limit.kp, upper_limbs_kp_min_, upper_limbs_kp_max_);
                any_value_clipped = i;
            }
    
            if (limit.kd < upper_limbs_kd_min_ || limit.kd > upper_limbs_kd_max_) {
                cmd.kd = clamp(limit.kd, upper_limbs_kd_min_, upper_limbs_kd_max_);
                any_value_clipped = i;
            }

        }
        

        // Check position
        if (cmd.q < limit.q_min || cmd.q > limit.q_max) {
            RCLCPP_WARN_ONCE(nh->get_logger(),
                       "Clipping motor_cmd[%lu] q: %.3f -> %.3f",
                       i, cmd.q, clamp(cmd.q, limit.q_min, limit.q_max));
            cmd.q = clamp(cmd.q, limit.q_min, limit.q_max);
            any_value_clipped = i;
        }

        // Check velocity
        if (cmd.dq < limit.dq_min || cmd.dq > limit.dq_max) {
            RCLCPP_WARN_ONCE(nh->get_logger(),
                       "Clipping motor_cmd[%lu] dq: %.3f -> %.3f",
                       i, cmd.dq, clamp(cmd.dq, limit.dq_min, limit.dq_max));
            cmd.dq = clamp(cmd.dq, limit.dq_min, limit.dq_max);
            any_value_clipped = i;
        }

        // Check torque
        if (cmd.tau < limit.tau_min || cmd.tau > limit.tau_max) {
            RCLCPP_WARN_ONCE(nh->get_logger(),
                       "Clipping motor_cmd[%lu] tau: %.3f -> %.3f",
                       i, cmd.tau, clamp(cmd.tau, limit.tau_min, limit.tau_max));
            cmd.tau = clamp(cmd.tau, limit.tau_min, limit.tau_max);
            any_value_clipped = i;
        }
        if (any_value_clipped > 0){
            RCLCPP_WARN(nh->get_logger(), "Joint %d were clipped to stay within safety limits", any_value_clipped);
        }
    }


    return true;
}


void RobotBridge::startControlServiceCB_(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
        initControl_();
        response->success = true;
        response->message = "Start control service activated";
    } else {
        controlStarted_ = false;
        response->success = false;
        response->message = request->data ? 
            "Failed to start control service" : 
            "Request data was false";
    }
        
}


void RobotBridge::stopControlServiceCB_(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
        controlStarted_ = false;
        response->success = true;
        response->message = "Stop control service activated";
    } else {
        response->success = false;
        response->message = "Request data was false";
    }
}


void RobotBridge::readyPositionControlServiceCB_(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

    if (request->data) {

        if (!controlStarted_) initControl_();
        
        readyPositionControl_();

        calculateInterpolationParams_();

        response->success = true;
        response->message = "Ready position control activated";
    } else {
        response->success = false;
        response->message = "Request data was false";
    }
}



void RobotBridge::zeroPositionControlServiceCB_(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

    if (request->data) {

        if (!controlStarted_) initControl_();
        
        zeroPositionControl_();

        calculateInterpolationParams_();

        response->success = true;
        response->message = "Ready position control activated";
    } else {
        response->success = false;
        response->message = "Request data was false";
    }
}

void RobotBridge::calculateInterpolationParams_() {
    
    RCLCPP_INFO(nh->get_logger(), "lowCommand_1= %.3f", a0[0]);
    RCLCPP_INFO(nh->get_logger(), "lowCommand_= %.3f", lowCommand_.motor_cmd[0].q);
    std::unique_lock<std::mutex> lock(mutex_);
    for (int i = 0; i < numJoint_; i++)
    {
        a0[i] = lowCommand_.motor_cmd[i].q; // currentState_.motor_state[i].q;
        a1[i] = lowCommandDesired_.motor_cmd[i].q - a0[i];
        b0[i] = 0.0; //currentState_.motor_state[i].dq;
        b1[i] = 0.0;
    }
    lock.unlock();

    rclcpp::Time now = nh->get_clock()->now();
    tStart = now.seconds();  
    tFinal = tStart + duration_; //duration_ = 3.0s
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

