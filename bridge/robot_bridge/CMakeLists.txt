cmake_minimum_required(VERSION 3.15)
project(robot_bridge)


## Set compiler to use c++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Choose build type
set(default_build_type "Release")

include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(std_srvs REQUIRED)
find_package(bridge_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(pluginlib REQUIRED)
find_package(low_level_controller REQUIRED)


include_directories(include include/common)

# motor_pd and bridge_pd: the default low-level controllers, plugins as a colleague's are (controllers.xml).
add_library(robot_bridge_controllers SHARED src/controllers.cpp)
ament_target_dependencies(robot_bridge_controllers pluginlib low_level_controller)
pluginlib_export_plugin_description_file(low_level_controller controllers.xml)
install(TARGETS robot_bridge_controllers LIBRARY DESTINATION lib)

# ==============================================================Unitree H1 and G1=============================================================

add_executable(G1_bridge
  src/bridge_core.cpp
  src/estop.cpp
  src/gates.cpp
  src/G1_bridge.cpp
  src/g1_node.cpp
  src/common/motor_crc_hg.cpp
)

ament_target_dependencies(G1_bridge
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  pluginlib
  low_level_controller
  std_msgs
  sensor_msgs
  std_srvs
  bridge_interface
  rosbag2_cpp
  rosidl_default_runtime
)


install(
  TARGETS G1_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



add_executable(H1_bridge
  src/bridge_core.cpp
  src/estop.cpp
  src/gates.cpp
  src/H1_bridge.cpp
  src/h1_node.cpp
  src/common/motor_crc.cpp
)

ament_target_dependencies(H1_bridge
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  pluginlib
  low_level_controller
  std_msgs
  sensor_msgs
  std_srvs
  bridge_interface
  rosbag2_cpp
  rosidl_default_runtime
)

ament_python_install_package(robot_bridge)


install(
  TARGETS H1_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Unitree Go2: unitree_go, 12 motors in the 20-slot frame (go2_frame.hpp).
add_executable(Go2_bridge
  src/bridge_core.cpp
  src/estop.cpp
  src/gates.cpp
  src/go2_frame.cpp
  src/Go2_bridge.cpp
  src/go2_node.cpp
  src/common/motor_crc.cpp
)

ament_target_dependencies(Go2_bridge
  unitree_go
  rclcpp
  pluginlib
  low_level_controller
  std_msgs
  std_srvs
  bridge_interface
  rosidl_default_runtime
)

install(
  TARGETS Go2_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


add_executable(H1_2_bridge
  src/bridge_core.cpp
  src/estop.cpp
  src/gates.cpp
  src/H1_2_bridge.cpp
  src/h1_2_node.cpp
  src/common/motor_crc_hg.cpp
)

  
ament_target_dependencies(H1_2_bridge
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  pluginlib
  low_level_controller
  std_msgs
  sensor_msgs
  std_srvs
  bridge_interface
  rosbag2_cpp
  rosidl_default_runtime
)

install(
  TARGETS H1_2_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================Booster T1===================================================================================

# booster_mode.hpp stands in for the Booster SDK: T1_bridge builds on any host.
option(BUILD_BOOSTER_T1 "Build Booster T1 bridge" ON)

if(BUILD_BOOSTER_T1)
  find_package(booster_interface REQUIRED)

  add_executable(T1_bridge
    src/t1_node.cpp
    src/bridge_core.cpp
    src/estop.cpp
    src/gates.cpp
    src/T1_bridge.cpp
  )

  target_include_directories(T1_bridge
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_target_dependencies(T1_bridge
    rclcpp
    pluginlib
    low_level_controller
    std_msgs
    std_srvs
    bridge_interface
    booster_interface
    rosidl_default_runtime
  )

  install(
    TARGETS T1_bridge
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

# ============================================================== AgiBot X2 =================================================================================

option(BUILD_AGIBOT_X2 "Build AgiBot X2 bridge" OFF)

if(BUILD_AGIBOT_X2)
  find_package(aimdk_msgs REQUIRED)

  add_executable(X2_bridge
    src/bridge_core.cpp
    src/estop.cpp
    src/gates.cpp
    src/X2_bridge.cpp
    src/x2_node.cpp
  )

  ament_target_dependencies(X2_bridge
    rclcpp
    pluginlib
    low_level_controller
    std_msgs
    sensor_msgs
    std_srvs
    bridge_interface
    aimdk_msgs
    rosidl_default_runtime
  )

  install(
    TARGETS X2_bridge
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(
    FILES params/X2_config.yaml
    DESTINATION share/${PROJECT_NAME}/params
  )
  
endif()


# ==============================================================test node cmake==============================================================================
# OFF by default: test_node streams a 23-motor (T1) /robot_cmd at 500 Hz; started by mistake
# beside a bridge, it is an outside /robot_cmd publisher.
option(BUILD_TEST_NODE "Build and install test_node" OFF)

if(BUILD_TEST_NODE)
  add_executable(test_node src/test.cpp)
  target_include_directories(test_node PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_node
   rclcpp 
   std_msgs
   std_srvs
   unitree_go
   bridge_interface
   )

  install(TARGETS
    test_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Pure e-stop logic, no ROS. Run alone: build/robot_bridge/test_estop
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_estop tests/test_estop.cpp src/estop.cpp)
  target_include_directories(test_estop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_gates tests/test_gates.cpp src/gates.cpp)
  target_include_directories(test_gates PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  # The Booster mode switch without the SDK; tools/check-booster-sdk.py checks the values against it.
  ament_add_gtest(test_booster tests/test_booster.cpp)
  target_include_directories(test_booster PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  # The T1 parallel ankle kinematics against loco-mujoco-t1's values; its own main, no gtest.
  add_executable(test_t1_parallel_ankle tests/test_t1_parallel_ankle.cpp)
  add_test(NAME test_t1_parallel_ankle COMMAND test_t1_parallel_ankle)
  # The Go2 frame (header, level flag, stopped slots, CRC against Unitree's SDK) and its stop frames.
  ament_add_gtest(test_go2 tests/test_go2.cpp src/go2_frame.cpp src/estop.cpp src/common/motor_crc.cpp)
  target_include_directories(test_go2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include/common)
  ament_target_dependencies(test_go2 rclcpp unitree_go)
  # The /lowcmd CRC against Unitree's SDK; run it in the build type the robot gets.
  ament_add_gtest(test_crc tests/test_crc.cpp src/common/motor_crc_hg.cpp)
  target_include_directories(test_crc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include/common)
  ament_target_dependencies(test_crc rclcpp unitree_hg)
  # motor_pd and bridge_pd loaded by name, as the bridge does: needs the installed workspace sourced.
  ament_add_gtest(test_controllers tests/test_controllers.cpp)
  ament_target_dependencies(test_controllers pluginlib low_level_controller)
endif()

ament_package()
