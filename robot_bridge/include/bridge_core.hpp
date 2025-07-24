// bridge_core.hpp ------------------------------------------------------------
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "bridge_interface/msg/robot_cmd.hpp"
#include "bridge_interface/msg/low_cmd.hpp"
#include "bridge_interface/msg/low_state.hpp"
#include "bridge_interface/msg/imu_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <vector>
#include <limits>
#include <mutex>
#include <thread>
#include <cmath>
#include <algorithm>
#include <cassert>

namespace sairol_bridge
{
    struct Joint
    {
        int idx;
        float q_min;
        float q_max;
        float dq_limit;
        float tau_limit;
        float kp;
        float kd;
    };

    struct CmdParams
    {
        float q_0{0}, q_1{0};
        float dq_0{0}, dq_1{0};
        float tau_0{0}, tau_1{0};
        float kp_0{0}, kp_1{0};
        float kd_0{0}, kd_1{0};
    };

    class BridgeCore
    {
    public:
        explicit BridgeCore(rclcpp::Node::SharedPtr node);
        virtual ~BridgeCore();

        std::shared_ptr<rclcpp::Node> nh;

        void start();

    protected:
        bool loadParameters_();
        void calculateInterpolationParams_(float duration,
                                           int interpolation_order,
                                           bool hold_position = false);

        void robotCmdCallBack_(bridge_interface::msg::RobotCmd::SharedPtr message);

        void readyPositionControl_();
        void zeroPositionControl_();
        void update_();
        virtual void publishLowCommand_() = 0;
        bool initControl_();
        bool checkCommand_();
        bool checkState_();

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startControlService_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopControlService_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr readyPositionService_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zeroPositionService_;
        rclcpp::Subscription<bridge_interface::msg::RobotCmd>::SharedPtr desiredSubscriber_;

        std::vector<Joint> joints_;
        std::vector<CmdParams> cmdParams_;

        bridge_interface::msg::RobotCmd lowCommandDesired_;
        bridge_interface::msg::LowCmd lowCommand_;
        bridge_interface::msg::LowState currentState_;
        bridge_interface::msg::ImuState imu_;

        rclcpp::Time last_state_time_;

        std::thread controlThread_;
        std::mutex mutex_;
        float tStart_{0}, tFinal_{0}, tValid_{0};
        const float INF_{std::numeric_limits<float>::infinity()};

        int numJoint_{0};
        int emptyJointIndex_{0};
        float duration_{1.0};
        float controlDt_{0.002};

        float upper_limbs_kp_min_{0}, upper_limbs_kp_max_{0};
        float upper_limbs_kd_min_{0}, upper_limbs_kd_max_{0};
        float lower_limbs_kp_min_{0}, lower_limbs_kp_max_{0};
        float lower_limbs_kd_min_{0}, lower_limbs_kd_max_{0};

        bool torqueControl_{false};
        bool controlStarted_{false};

        void startControlServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void stopControlServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void readyPositionControlServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void zeroPositionControlServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    };

} // namespace sairol_bridge