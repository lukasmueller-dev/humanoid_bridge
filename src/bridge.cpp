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

    
    lowstateSubscriber_ = nh->create_subscription<unitree_go::msg::LowState>(
        topic_name, 10, std::bind(&RobotBridge::lowStateHandler_, this, _1));
    

    lowcmdPublisher_ = nh->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);


    start_control_service_ = nh->create_service<std_srvs::srv::SetBool>(
        "start_control", std::bind(&RobotBridge::startControlService_, this, _1, _2));

    stop_control_service_ = nh->create_service<std_srvs::srv::SetBool>(
        "stop_control", std::bind(&RobotBridge::stopControlService_, this, _1, _2));


    ready_position_service_ = nh->create_service<std_srvs::srv::SetBool>(
        "ready_position_control", std::bind(&RobotBridge::readyPositionControlService_, this, _1, _2));
    
    zero_position_service_ = nh->create_service<std_srvs::srv::SetBool>(
        "zero_position_control", std::bind(&RobotBridge::zeroPositionControlService_, this, _1, _2));



    duration_= 3 ;
  
}


RobotBridge::~RobotBridge() {
    //nothing
}



void RobotBridge::lowStateHandler_(unitree_go::msg::LowState::SharedPtr message){
    //check frequency
    // rclcpp::Time now = this->get_clock()->now();

    // if (last_time_.nanoseconds() > 0) {
    //     double dt = (now - last_time_).seconds();
    //     double freq = 1.0 / dt;
    //     RCLCPP_INFO(this->get_logger(), "LowState frequency: %.2f Hz", freq);
    // }

    // last_time_ = now;

    imu_ = message->imu_state;
    for (int i = 0; i < NumJoint; i++)
    {
        currentState_.motor_state[i] = message->motor_state[i];
    }

    if (INFO_IMU)
    {
        // Info imu_ states
        // RPY euler angle(ZYX order respected to body frame)
        // Quaternion
        // Gyroscope (raw data)
        // Accelerometer (raw data)
        RCLCPP_INFO(nh->get_logger(), "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu_.rpy[0], imu_.rpy[1], imu_.rpy[2]);
        RCLCPP_INFO(nh->get_logger(), "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f",
                    imu_.quaternion[0], imu_.quaternion[1], imu_.quaternion[2], imu_.quaternion[3]);
        RCLCPP_INFO(nh->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f", imu_.gyroscope[0], imu_.gyroscope[1], imu_.gyroscope[2]);
        RCLCPP_INFO(nh->get_logger(), "Accelerometer -- ax: %f; ay: %f; az: %f",
                    imu_.accelerometer[0], imu_.accelerometer[1], imu_.accelerometer[2]);
    }
    if (INFO_MOTOR)
    {
        // Info motor states
        // q: angluar (rad)
        // dq: angluar velocity (rad/s)
        // ddq: angluar acceleration (rad/(s^2))
        // tau_est: Estimated external torque
        for (int i = 0; i < NumJoint; i++)
        {
            RCLCPP_INFO(nh->get_logger(), "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
                        i, currentState_.motor_state[i].q, currentState_.motor_state[i].dq, currentState_.motor_state[i].ddq, currentState_.motor_state[i].tau_est);
        }
    }

}


void RobotBridge::publishLowCommand_() {

    rclcpp::Time current = nh->get_clock()->now();
    double t = clamp((current.seconds() - t_start)/(t_final-t_start), 0.0, 1.0);
    for (int i = 0; i < NumJoint; ++i) {
        lowCommand_.motor_cmd[i].mode = (i < H1_JointIndex::EmptyJoint) ? 0x0A : 0x01;
        lowCommand_.motor_cmd[i].q = a0[i] + a1[i] * t;
        lowCommand_.motor_cmd[i].tau = 0.0;
        lowCommand_.motor_cmd[i].dq = 0.0;
        lowCommand_.motor_cmd[i].kp = (i < H1_JointIndex::EmptyJoint) ? 200.0 : 25.0;
        lowCommand_.motor_cmd[i].kd = 1.0;
    }

    get_crc(lowCommand_);
    lowcmdPublisher_->publish(lowCommand_);
}


double RobotBridge::clamp(double value, double low, double high) 
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}



void RobotBridge::readyPositionControl_(){

    for (int i = 0; i < NumJoint; ++i) 
    {   
        lowCommanddesired_.motor_cmd[i].q = 0.0;
        lowCommanddesired_.motor_cmd[i].tau = 0.0;
        lowCommanddesired_.motor_cmd[i].dq = 0.0;

    }

}


void RobotBridge::zeroPositionControl_(){

    for (int i = 0; i < NumJoint; ++i) 
    {   
        lowCommanddesired_.motor_cmd[i].tau = 0.0;
        lowCommanddesired_.motor_cmd[i].dq = 0.0;

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
        lowCommanddesired_.motor_cmd[i].q = target_q;
    }
}


bool RobotBridge::initControl() {
    for (int i = 0; i < NumJoint; i++)
    {
         a0[i] = currentState_.motor_state[i].q;
         a1[i] = 0.0;
         b0[i] = 0.0; //currentState_.motor_state[i].dq;
         b1[i] = 0.0;
  
    }
    rclcpp::Time now = nh->get_clock()->now();
    t_start = now.seconds();  
    t_final = inf; 

    return true;
}


bool RobotBridge::update() {

    if (timer_) timer_->cancel();  
    timer_ = nh->create_wall_timer(
        std::chrono::milliseconds(timerDt_),
        std::bind(&RobotBridge::publishLowCommand_, this)
    );

    return true;
}


bool RobotBridge::startControl() {

    if(initControl() && update()) {
        return true;
    } else {
        return false;
    }
}


void RobotBridge::startControlService_(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data && startControl()) {
        response->success = true;
        response->message = "Start control service activated";
    } else {
        response->success = false;
        response->message = request->data ? 
            "Failed to start control service" : 
            "Request data was false";
    }
        
}


void RobotBridge::stopControlService_(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  
}


void RobotBridge::readyPositionControlService_(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

    if (request->data) {

        if (!startControl()) {
            startControl();
        }
        
        readyPositionControl_();


        for (int i = 0; i < NumJoint; i++)
        {
             a0[i] = currentState_.motor_state[i].q;
             a1[i] = lowCommanddesired_.motor_cmd[i].q - currentState_.motor_state[i].q;
            //  RCLCPP_INFO(nh->get_logger(), "a0[%d]: %f", i, a0[i]);
            //  RCLCPP_INFO(nh->get_logger(), "a1[%d]: %f", i, a1[i]);
             b0[i] = 0.0; //currentState_.motor_state[i].dq;
             b1[i] = 0.0;
      
        }
        rclcpp::Time now = nh->get_clock()->now();
        t_start = now.seconds();  
        t_final = t_start + 5.0; //duration_ = 5.0s


        update();

        response->success = true;
        response->message = "Ready position control activated";
    } else {
        response->success = false;
        response->message = "Request data was false";
    }
}



void RobotBridge::zeroPositionControlService_(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

    if (request->data) {

        if (!startControl()) {
            startControl();
        }
        
        zeroPositionControl_();


        for (int i = 0; i < NumJoint; i++)
        {
                a0[i] = currentState_.motor_state[i].q;
                a1[i] = lowCommanddesired_.motor_cmd[i].q - currentState_.motor_state[i].q;
            //  RCLCPP_INFO(nh->get_logger(), "a0[%d]: %f", i, a0[i]);
            //  RCLCPP_INFO(nh->get_logger(), "a1[%d]: %f", i, a1[i]);
                b0[i] = 0.0; //currentState_.motor_state[i].dq;
                b1[i] = 0.0;
        
        }
        rclcpp::Time now = nh->get_clock()->now();
        t_start = now.seconds();  
        t_final = t_start + 5.0; //duration_ = 5.0s


        update();

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

