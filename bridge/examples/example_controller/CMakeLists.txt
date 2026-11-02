cmake_minimum_required(VERSION 3.15)
project(example_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(low_level_controller REQUIRED)
find_package(pluginlib REQUIRED)

# The bridge loads it by its name in plugins.xml, once the config lists it in controllers_allowed.
add_library(example_controller SHARED src/example_controller.cpp)
ament_target_dependencies(example_controller low_level_controller pluginlib)
pluginlib_export_plugin_description_file(low_level_controller plugins.xml)
install(TARGETS example_controller LIBRARY DESTINATION lib)

ament_package()
