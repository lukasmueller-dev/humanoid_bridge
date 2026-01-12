cmake_minimum_required(VERSION 3.8)
project(booster_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoosterApiReqMsg.msg"
  "msg/BoosterApiRespMsg.msg"
  "msg/ButtonEventMsg.msg"
  "msg/MotorState.msg"
  "msg/MotorCmd.msg"
  "msg/LowCmd.msg"
  "msg/ImuState.msg"
  "msg/LowState.msg"
  "msg/RawBytesMsg.msg"
  "msg/Odometer.msg"
  "msg/FallDownState.msg"
  "msg/RemoteControllerState.msg"
  "msg/HandCommand.msg"
  "msg/HandDdsMsg.msg"
  "msg/HandParam.msg"
  "msg/ProneBodyControlStatus.msg"
  "msg/RobotStatesMsg.msg"
  "msg/RobotReplayTrajID.msg"
  "msg/Subtitle.msg"
  "srv/RpcService.srv"
  "srv/AgentService.srv"
)


install(DIRECTORY include/
  DESTINATION include/
)


ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()