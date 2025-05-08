#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "common/motor_crc.h"
#include "unitree/robot/b2/motion_switcher/motion_switcher_client.hpp"
#include "std_srvs/srv/set_bool.hpp" 
#include <vector>
#include <limits>
#include <iostream>
#include <chrono>

#define INFO_IMU 1        // Set 1 to print IMU states
#define INFO_MOTOR 1      // Set 1 to print motor states
#define HIGH_FREQ 1       // Set 1 to subscribe at 500Hz

enum PRorAB { PR = 0, AB = 1 };

enum H1_JointIndex {
    // legs
    RightHipRoll = 0,
    RightHipPitch = 1,
    RightKnee = 2,
    LeftHipRoll = 3,
    LeftHipPitch = 4,
    LeftKnee = 5,
    Torso = 6,
    LeftHipYaw = 7,
    RightHipYaw = 8,
    EmptyJoint = 9,
    LeftAnkle = 10,
    RightAnkle = 11,
    RightShoulderPitch = 12,
    RightShoulderRoll = 13,
    RightShoulderYaw = 14,
    RightElbow = 15,
    LeftShoulderPitch = 16,
    LeftShoulderRoll = 17,
    LeftShoulderYaw = 18,
    LeftElbow = 19,
    NumJoint = 20,
  };



class RobotBridge{ 
public:
    RobotBridge();
    ~RobotBridge();

    std::shared_ptr<rclcpp::Node> nh;

private:

    void lowStateHandler_(unitree_go::msg::LowState::SharedPtr message);
    void readyPositionControl_();
    void zeroPositionControl_();
    double clamp(double value, double low, double high);
    bool startControl_();
    bool initControl_();
    void update_();
    void publishLowCommand_();
    void checkForExternalPublisherAndRelease_();
    bool checkState_();
    bool checkCommand_();

    // Service callback functions
    void startControlServiceCB_(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void stopControlServiceCB_(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void readyPositionControlServiceCB_(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void zeroPositionControlServiceCB_(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);



    // Service objects
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr startControlService_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stopControlService_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr readyPositionService_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr zeroPositionService_;



    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowStateSubscriber_;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowCmdPublisher_; 
    rclcpp::TimerBase::SharedPtr timer_;
 
    
    unitree_go::msg::LowCmd lowCommandDesired_;
    unitree_go::msg::LowCmd lowCommand_;
    unitree_go::msg::LowState currentState_;
    unitree_go::msg::IMUState imu_;
    
    //Interpolation parameters
    std::vector<double> a0 = std::vector<double>(20, 0);
    std::vector<double> a1 = std::vector<double>(20, 0);
    std::vector<double> b0 = std::vector<double>(20, 0);
    std::vector<double> b1 = std::vector<double>(20, 0);
    double inf = std::numeric_limits<double>::infinity();
    double tStart = 0;
    double tFinal = 0;
    double dt_ = 0;

    bool controlStarted_ = false;
    bool releaseOtherNode_ = false;


    double controlDt_;                                                      // 2ms
    int timerDt_;
    double time_;                                                                    // Running time count
    PRorAB mode_ = PRorAB::PR;
    
};