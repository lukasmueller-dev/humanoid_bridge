#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "common/motor_crc.h"
#include "unitree/robot/b2/motion_switcher/motion_switcher_client.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <vector>
#include <limits>
#include <iostream>
#include <chrono>
#include <array>
#include <deque>
#include "bridge_interface/msg/robot_cmd.hpp"
#include "unitree_go/msg//wireless_controller.hpp"
#include "key_event_handler.hpp"
namespace sairol_h1
{

#define INFO_IMU 1   // Set 1 to print IMU states
#define INFO_MOTOR 1 // Set 1 to print motor states
#define HIGH_FREQ 1  // Set 1 to subscribe at 500Hz

double INF = std::numeric_limits<double>::infinity();

    // enum PRorAB { PR = 0, AB = 1 };
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

    struct Joint
    {
        int idx;
        double q_min;
        double q_max;
        // double dq_min;
        // double dq_max;
        // double tau_min;
        double dq_limit;
        double tau_limit;
        // double tau_max;
        double kp;
        double kd;
    };

    struct CmdParams
    {
        double q_0 = 0.0;
        double q_1 = 0.0;
        double dq_0 = 0.0;
        double dq_1 = 0.0;
        double tau_0 = 0.0;
        double tau_1 = 0.0;
        double kp_0 = 0.0;
        double kp_1 = 0.0;
        double kd_0 = 0.0;
        double kd_1 = 0.0;
    };

    class RobotBridge
    {
    public:
        RobotBridge();
        ~RobotBridge();

        std::shared_ptr<rclcpp::Node> nh;

    private:
        void lowStateHandler_(unitree_go::msg::LowState::SharedPtr message);
        void robotCmdCallBack_(bridge_interface::msg::RobotCmd::SharedPtr message);
        void wireless_callback(unitree_go::msg::WirelessController::SharedPtr data);
        void readyPositionControl_();
        void zeroPositionControl_();
        void calculateInterpolationParams_(double process_duration, int interpolation_order, bool hold_position = false);
        double clamp(double value, double low, double high);
        bool initControl_();
        void update_();
        void publishLowCommand_();
        void checkExternalPublisher_();
        bool checkState_();
        bool checkCommand_();
        bool loadParameters_();

        using prepareCmdOp = void (RobotBridge::*)(double phase, int idx);

        void prepareCmdInterpolation_(double phase, int idx);
        void prepareCmdTorque_(double phase, int idx);

        // Service callback functions
        void startControlServiceCB_(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void stopControlServiceCB_(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        // gotoConfiguration: int -> bool, 0-zero, 1-ready, other - false
        void readyPositionControlServiceCB_(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void zeroPositionControlServiceCB_(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        // Service objects
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startControlService_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopControlService_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr readyPositionService_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zeroPositionService_;
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowStateSubscriber_;
        rclcpp::Subscription<bridge_interface::msg::RobotCmd>::SharedPtr desiredSubscriber_;
        rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr remoteControlSubscriber_;
        rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowCmdPublisher_;
        rclcpp::Time last_state_time_;

        std::thread controlThread_;
        std::mutex mutex_;
        prepareCmdOp  prepareCmd_;

        bridge_interface::msg::RobotCmd lowCommandDesired_;

        unitree_go::msg::LowCmd lowCommand_;
        unitree_go::msg::LowState currentState_;
        unitree_go::msg::IMUState imu_;

        // Interpolation parameters
        std::vector<CmdParams> cmdParams_;

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
        bool torqueControl_;

        int i = 0;               // Running time count
        int numJoint_;              // Number of joints
        int emptyJointIndex_;       // Index of empty joint
        double duration_;           // Duration for ready position control
        double controlDt_;          // 2ms
        std::vector<Joint> joints_; // Running time count
    };
}