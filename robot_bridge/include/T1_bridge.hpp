#include "bridge_core.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/remote_controller_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "key_event_handler.hpp"
#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"
#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>


namespace sairol_bridge {
class T1Bridge : public BridgeCore {
public:
    explicit T1Bridge(rclcpp::Node::SharedPtr node);
    virtual ~T1Bridge();

private:

    void wireless_callback(sensor_msgs::msg::Joy::SharedPtr message);
    void publishLowCommand_();
    void lowStateHandler_(booster_interface::msg::LowState::SharedPtr message);
    void readyPositionControl_();
    bool initControl_();
    void switch_mode(auto target_mode);
    void switch_to_damping_mode();
    void stopControlServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSubscriber_;  
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr remoteControlSubscriber_;
    rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr lowCommandPublisher_;
    rclcpp::Client<booster_interface::srv::RpcService>::SharedPtr client_;


};
}