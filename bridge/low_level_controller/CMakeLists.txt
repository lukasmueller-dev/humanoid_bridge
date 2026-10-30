cmake_minimum_required(VERSION 3.15)
project(low_level_controller)

# Header only: the plugin interface of the bridge's low-level controllers.
find_package(ament_cmake REQUIRED)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

ament_package()
