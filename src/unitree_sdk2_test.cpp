#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>


int main(int argc, char **argv) {
    std::string networkInterface = "enp4s0";
    unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface);
    std::cout << "Initialize channel factory." << std::endl;

    unitree::robot::b2::MotionSwitcherClient motion_switch_client;
    motion_switch_client.SetTimeout(5.0F);
    motion_switch_client.Init();

    std::string robotForm,motionName;
    int motionStatus;
    int32_t ret = motion_switch_client.CheckMode(robotForm,motionName);

    ret = motion_switch_client.ReleaseMode(); 
    if (ret == 0) {
        std::cout << "ReleaseMode succeeded." << std::endl;
    } else {
        std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
    }
    // std::string form, name;
    // motion_switch_client.CheckMode(form, name);
    std::cout << "form: " << robotForm << std::endl;
    std::cout << "name: " << motionName << std::endl;
    return 0;
}