#include "bridge.hpp"


using std::placeholders::_1;
using std::placeholders::_2;  



RobotBridge::RobotBridge(){
    controlDt_ = 0.002;
    timerDt_ = controlDt_ * 1000;
    auto topic_name = "lf/lowstate";
    if (HIGH_FREQ) {
        topic_name = "lowstate";
    }

    nh = std::make_shared<rclcpp::Node>("robot_bridge");

    
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

    //Waiting for publisher on topic /lowstate
    RCLCPP_INFO(nh->get_logger(), "Waiting for publisher on topic /lowstate...");
    while (nh->count_publishers(topic_name) == 0) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(nh->get_logger(), "Publisher detected on topic /lowstate.");
        
    // Init Thread 
    timer_ = nh->create_wall_timer(
        std::chrono::milliseconds(timerDt_),
        std::bind(&RobotBridge::update_, this)
    );
  
}


RobotBridge::~RobotBridge() {
    //nothing
}



void RobotBridge::lowStateHandler_(unitree_go::msg::LowState::SharedPtr message){


    imu_ = message->imu_state;
    currentState_.motor_state = message->motor_state;

    static rclcpp::Time last_time = nh->get_clock()->now();
    rclcpp::Time now = nh->get_clock()->now();
    dt_ = (now - last_time).seconds();
    last_time = now;
    
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
    for (int i = 0; i < NumJoint; ++i) {
        lowCommand_.motor_cmd[i].mode = (i < H1_JointIndex::EmptyJoint) ? 0x0A : 0x01;
        lowCommand_.motor_cmd[i].q = a0[i] + a1[i] * phase;
        lowCommand_.motor_cmd[i].tau = 0.0;
        lowCommand_.motor_cmd[i].dq = 0.0;
        lowCommand_.motor_cmd[i].kp = (i < H1_JointIndex::EmptyJoint) ? 200.0 : 25.0;
        lowCommand_.motor_cmd[i].kd = 1.0;
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

    for (int i = 0; i < NumJoint; ++i) 
    {   
        lowCommandDesired_.motor_cmd[i].q = 0.0;
        lowCommandDesired_.motor_cmd[i].tau = 0.0;
        lowCommandDesired_.motor_cmd[i].dq = 0.0;

    }

}


void RobotBridge::readyPositionControl_(){

    for (int i = 0; i < NumJoint; ++i) 
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
    }
}


bool RobotBridge::initControl_() {
    for (int i = 0; i < NumJoint; i++)
    {
         a0[i] = currentState_.motor_state[i].q;
         a1[i] = 0.0;
         b0[i] = 0.0; //currentState_.motor_state[i].dq;
         b1[i] = 0.0;
  
    }
    rclcpp::Time now = nh->get_clock()->now();
    tStart = now.seconds();  
    tFinal = inf;

    return true;
}


void RobotBridge::update_() {
    // Check and close any redundant publishers that may exist
    if(!releaseOtherNode_) checkForExternalPublisherAndRelease_();
    
    // Safety check
    if (checkState_()){
        // pass
    }
    else {
        RCLCPP_ERROR(nh->get_logger(), "Robot state check failed. Please inspect the robot carefully.");
        return; 
    }

    if (controlStarted_ ) { // && checkCommand_()
        // Update the low command
        publishLowCommand_();
    }
}


bool RobotBridge::startControl_() {
    if(initControl_()) {
        return true;
    } else {
        return false;
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

    double expected_freq = 500.0;
    double freq = 1.0 / dt_;

    if (dt_ > 1.0) {
        RCLCPP_ERROR(nh->get_logger(), 
            "Robot signal lost! No LowState message received for %.2f seconds. "
            "Expected interval ~0.002s (500Hz). Shutting down the node to prevent unsafe operation.",
            dt_);
        rclcpp::shutdown(); 
    }
    

    // if (std::abs(freq - expected_freq) > 200.0) {  
    //     RCLCPP_WARN(nh->get_logger(), 
    //                 "LowState frequency deviation: expected ~500Hz, got %.2fHz", freq);
    //     return false;
    // }

    if (currentState_.motor_state.size() != NumJoint) {
        RCLCPP_ERROR(nh->get_logger(), 
                     "Motor state length mismatch: expected %d, got %ld", 
                     NumJoint, currentState_.motor_state.size());
        return false;
    }

    for (size_t i = 0; i < currentState_.motor_state.size(); ++i) {
        const auto& motor = currentState_.motor_state[i];
        if (!std::isfinite(motor.q) || !std::isfinite(motor.dq) || 
            !std::isfinite(motor.ddq) || !std::isfinite(motor.tau_est)) {
            RCLCPP_ERROR(nh->get_logger(), 
                         "Motor state at index %lu contains invalid (NaN/Inf) values.", i);
            return false;
        }
    }

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
    if (lowCommand_.motor_cmd.size() != NumJoint) {
        RCLCPP_ERROR(nh->get_logger(),
                     "Command check failed: motor_cmd size mismatch. Expected %d, got %ld.",
                     NumJoint, lowCommand_.motor_cmd.size());
        return false;
    }

    for (size_t i = 0; i < lowCommand_.motor_cmd.size(); ++i) {
        const auto& cmd = lowCommand_.motor_cmd[i];

        if (!std::isfinite(cmd.q) || !std::isfinite(cmd.dq) ||
            !std::isfinite(cmd.tau) || !std::isfinite(cmd.kp) ||
            !std::isfinite(cmd.kd)) {
            RCLCPP_ERROR(nh->get_logger(),
                         "Command check failed: motor_cmd[%lu] contains invalid (NaN/Inf) values.", i);
            return false;
        }

        if (cmd.kp < 0.0 || cmd.kd < 0.0) {
            RCLCPP_ERROR(nh->get_logger(),
                         "Command check failed: motor_cmd[%lu] has negative gains (kp or kd).", i);
            return false;
        }

        // if (std::abs(cmd.q) > 10.0 || std::abs(cmd.dq) > 100.0 || std::abs(cmd.tau) > 200.0) {
        //     RCLCPP_WARN(nh->get_logger(),
        //                 "Command check warning: motor_cmd[%lu] has extreme values. q=%.2f, dq=%.2f, tau=%.2f",
        //                 i, cmd.q, cmd.dq, cmd.tau);
        // }
    }

    return true;
}


void RobotBridge::startControlServiceCB_(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
        initControl_();
        controlStarted_ = true;
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

        if (!controlStarted_) startControl_();
        
        readyPositionControl_();

        for (int i = 0; i < NumJoint; i++)
        {
             a0[i] = currentState_.motor_state[i].q;
             a1[i] = lowCommandDesired_.motor_cmd[i].q - currentState_.motor_state[i].q;
            //  RCLCPP_INFO(nh->get_logger(), "a0[%d]: %f", i, a0[i]);
            //  RCLCPP_INFO(nh->get_logger(), "a1[%d]: %f", i, a1[i]);
             b0[i] = 0.0; //currentState_.motor_state[i].dq;
             b1[i] = 0.0;
      
        }
        rclcpp::Time now = nh->get_clock()->now();
        tStart = now.seconds();  
        tFinal = tStart + 3.0; //duration_ = 3.0s

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

        if (!controlStarted_) startControl_();
        
        zeroPositionControl_();


        for (int i = 0; i < NumJoint; i++)
        {
                a0[i] = currentState_.motor_state[i].q;
                a1[i] = lowCommandDesired_.motor_cmd[i].q - currentState_.motor_state[i].q;
            //  RCLCPP_INFO(nh->get_logger(), "a0[%d]: %f", i, a0[i]);
            //  RCLCPP_INFO(nh->get_logger(), "a1[%d]: %f", i, a1[i]);
                b0[i] = 0.0; //currentState_.motor_state[i].dq;
                b1[i] = 0.0;
        
        }
        rclcpp::Time now = nh->get_clock()->now();
        tStart = now.seconds();  
        tFinal = tStart + 3.0; //duration_ = 3.0s

        response->success = true;
        response->message = "Ready position control activated";
    } else {
        response->success = false;
        response->message = "Request data was false";
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

