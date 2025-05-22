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
#include <array>
#include "bridge_interface/msg/test_signal.hpp"





#define INFO_IMU 1        // Set 1 to print IMU states
#define INFO_MOTOR 1      // Set 1 to print motor states
#define HIGH_FREQ 1       // Set 1 to subscribe at 500Hz

enum PRorAB { PR = 0, AB = 1 };

// enum H1_JointIndex {            //  angle range       velocity range         torque range
//     RightHipRoll = 0,           //-0.43~+0.43 rad     -23 ~ 23 rad/s        -200 ~ 200 N/m        
//     RightHipPitch = 1,          //-3.14~+2.53 rad     -23 ~ 23 rad/s        -200 ~ 200 N/m
//     RightKnee = 2,              //-0.26~+2.05 rad     -14 ~ 14 rad/s        -300 ~ 300 N/m
//     LeftHipRoll = 3,            //-0.43~+0.43 rad     -23 ~ 23 rad/s        -200 ~ 200 N/m
//     LeftHipPitch = 4,           //-3.14~+2.53 rad     -23 ~ 23 rad/s        -200 ~ 200 N/m
//     LeftKnee = 5,               //-0.26~+2.05 rad     -14 ~ 14 rad/s        -300 ~ 300 N/m
//     Torso = 6,                  //-2.35~+2.35 rad     -23 ~ 23 rad/s        -200 ~ 200 N/m
//     LeftHipYaw = 7,             //-0.43~+0.43 rad     -23 ~ 23 rad/s        -200 ~ 200 N/m
//     RightHipYaw = 8,            //-0.43~+0.43 rad     -23 ~ 23 rad/s        -200 ~ 200 N/m
//     EmptyJoint = 9,
//     LeftAnkle = 10,             //-0.87~+0.52 rad     -9  ~ 9  rad/s        -40  ~ 40  N/m
//     RightAnkle = 11,            //-0.87~+0.52 rad     -9  ~ 9  rad/s        -40  ~ 40  N/m
//     RightShoulderPitch = 12,    //-2.87~+2.87 rad     -9  ~ 9  rad/s        -40  ~ 40  N/m
//     RightShoulderRoll = 13,     //-3.11~+0.34 rad     -9  ~ 9  rad/s        -40  ~ 40  N/m
//     RightShoulderYaw = 14,      //-4.45~+1.3 rad      -20 ~ 20 rad/s        -18  ~ 18  N/m
//     RightElbow = 15,            //-1.25~+2.61 rad     -20 ~ 20 rad/s        -18  ~ 18  N/m  
//     LeftShoulderPitch = 16,     //-2.87~+2.87 rad     -9  ~ 9  rad/s        -40  ~ 40  N/m
//     LeftShoulderRoll = 17,      //-0.34~+3.11 rad     -9  ~ 9  rad/s        -40  ~ 40  N/m
//     LeftShoulderYaw = 18,       //-1.3~+4.45 rad      -20 ~ 20 rad/s        -18  ~ 18  N/m
//     LeftElbow = 19,             //-1.25~+2.61 rad     -20 ~ 20 rad/s        -18  ~ 18  N/m
//     NumJoint = 20,
//   };

//   namespace CommandLimits {

//     constexpr double KP_MIN = 25.0;
//     constexpr double KD_MIN = 1.0;
//     constexpr double KP_MAX = 200.0;
//     constexpr double KD_MAX = 1.0;
//     struct Limit {
//         double q_min;
//         double q_max;
//         double dq_min;
//         double dq_max;
//         double tau_min;
//         double tau_max;
//     };

//     constexpr std::array<Limit, NumJoint> JOINT_LIMITS = {{
//         {-0.43, 0.43,   -23.0, 23.0,   -200.0, 200.0}, // RightHipRoll
//         {-3.14, 2.53,   -23.0, 23.0,   -200.0, 200.0}, // RightHipPitch
//         {-0.26, 2.05,   -14.0, 14.0,   -300.0, 300.0}, // RightKnee
//         {-0.43, 0.43,   -23.0, 23.0,   -200.0, 200.0}, // LeftHipRoll
//         {-3.14, 2.53,   -23.0, 23.0,   -200.0, 200.0}, // LeftHipPitch
//         {-0.26, 2.05,   -14.0, 14.0,   -300.0, 300.0}, // LeftKnee
//         {-2.35, 2.35,   -23.0, 23.0,   -200.0, 200.0}, // Torso
//         {-0.43, 0.43,   -23.0, 23.0,   -200.0, 200.0}, // LeftHipYaw
//         {-0.43, 0.43,   -23.0, 23.0,   -200.0, 200.0}, // RightHipYaw
//         {-1e6, 1e6,     -1e6, 1e6,     -1e6, 1e6},     // EmptyJoint (disabled or placeholder)
//         {-0.87, 0.52,   -9.0,  9.0,    -40.0,  40.0},  // LeftAnkle
//         {-0.87, 0.52,   -9.0,  9.0,    -40.0,  40.0},  // RightAnkle
//         {-2.87, 2.87,   -9.0,  9.0,    -40.0,  40.0},  // RightShoulderPitch
//         {-3.11, 0.34,   -9.0,  9.0,    -40.0,  40.0},  // RightShoulderRoll
//         {-4.45, 1.3,    -20.0, 20.0,   -18.0,  18.0},  // RightShoulderYaw
//         {-1.25, 2.61,   -20.0, 20.0,   -18.0,  18.0},  // RightElbow
//         {-2.87, 2.87,   -9.0,  9.0,    -40.0,  40.0},  // LeftShoulderPitch
//         {-0.34, 3.11,   -9.0,  9.0,    -40.0,  40.0},  // LeftShoulderRoll
//         {-1.3, 4.45,    -20.0, 20.0,   -18.0,  18.0},  // LeftShoulderYaw
//         {-1.25, 2.61,   -20.0, 20.0,   -18.0,  18.0},  // LeftElbow
//     }};
// }


struct Joint {
    int idx;
    double q_min;
    double q_max;
    double dq_min;
    double dq_max;
    double tau_min;
    double tau_max;
    double kp;  
    double kd;  
};

class RobotBridge{ 
public:
    RobotBridge();
    ~RobotBridge();

    std::shared_ptr<rclcpp::Node> nh;

private:

    void lowStateHandler_(unitree_go::msg::LowState::SharedPtr message);

    void lowcmdCallBack_(bridge_interface::msg::TestSignal::SharedPtr message);

    void readyPositionControl_();
    void zeroPositionControl_();
    void calculateInterpolationParams_(double process_duration, int interpolation_order, bool hold_position =false);
    double clamp(double value, double low, double high);
    bool initControl_();
    void update_();
    void publishLowCommand_();
    void checkForExternalPublisherAndRelease_();
    bool checkState_();
    bool checkCommand_();
    void clipCommand_();
    void load_parameters_();

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

    void recievedMessageControlServiceCB_(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);



    // Service objects
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr startControlService_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stopControlService_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr readyPositionService_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr zeroPositionService_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr recievedMessageService_;

    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowStateSubscriber_;

    rclcpp::Subscription<bridge_interface::msg::TestSignal>::SharedPtr desiredSubscriber_;

    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowCmdPublisher_; 

    rclcpp::Time last_state_time_;

    std::thread controlThread_;
    std::mutex mutex_;
    
    bridge_interface::msg::TestSignal lowCommandDesired_;
    // unitree_go::msg::LowCmd lowCommandDesired_;
    

    unitree_go::msg::LowCmd lowCommand_;
 
    unitree_go::msg::LowState currentState_;
    unitree_go::msg::IMUState imu_;
    
    //Interpolation parameters
    std::vector<double> a0 ;
    std::vector<double> a1 ;
    std::vector<double> b0 ;
    std::vector<double> b1 ;
    std::vector<double> c0 ;
    std::vector<double> c1 ;

    double inf = std::numeric_limits<double>::infinity();
    double tStart = 0;
    double tFinal = 0;
    double tValid = 0;

    double upper_limbs_kp_min_;
    double upper_limbs_kp_max_;
    double upper_limbs_kd_min_;
    double upper_limbs_kd_max_;

    double lower_limbs_kp_min_;
    double lower_limbs_kp_max_;
    double lower_limbs_kd_min_;
    double lower_limbs_kd_max_;

    bool controlStarted_ = false;
    bool releaseOtherNode_ = false;
    bool if_recieve_message_ = false;
    bool if_ready_position_ = false;
    bool if_zero_position_ = false;
    
    int numJoint_; // Number of joints
    int emptyJointIndex_; // Index of empty joint
    double duration_; // Duration for ready position control
    double controlDt_ ;                                                      // 2ms
    int timerDt_;
    std::vector<Joint> joints_;                                                            // Running time count
    PRorAB mode_ = PRorAB::PR;


    
};