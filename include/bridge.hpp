#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include <chrono>
#include "common/motor_crc.h"
#include "unitree/robot/b2/motion_switcher/motion_switcher_client.hpp"

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
    void A_PositionControl_();
    double clamp(double value, double low, double high);


    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstateSubscriber_;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmdPublisher_; 
    rclcpp::TimerBase::SharedPtr timer_;

    unitree_go::msg::LowCmd lowCommand_;
    unitree_go::msg::LowState currentState_;
    unitree_go::msg::IMUState imu_;
    
    double controlDt_;                                                      // 2ms
    int timerDt_;
    double time_;                                                                    // Running time count
    double duration_;
    PRorAB mode_ = PRorAB::PR;
    
};