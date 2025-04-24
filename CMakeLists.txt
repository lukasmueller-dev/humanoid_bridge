cmake_minimum_required(VERSION 3.5)
project(robot_bridge)


## Set compiler to use c++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Chosse build type
set(default_build_type "Release")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()

include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(Threads REQUIRED)

set(UNITREE_SDK_PATH $ENV{HOME}/library)

# Import Libraries for unitree_sdk2
find_library(DDSC_LIB ddsc PATHS ${UNITREE_SDK_PATH}/lib NO_DEFAULT_PATH)
add_library(ddsc SHARED IMPORTED GLOBAL)
set_target_properties(ddsc PROPERTIES IMPORTED_LOCATION ${DDSC_LIB})
set_target_properties(ddsc PROPERTIES IMPORTED_NO_SONAME TRUE)
target_link_libraries(ddsc INTERFACE Threads::Threads)
target_link_directories(ddsc INTERFACE ${UNITREE_SDK_PATH}/lib)
target_include_directories(ddsc INTERFACE
        $<BUILD_INTERFACE:${UNITREE_SDK_PATH}/include>
        $<INSTALL_INTERFACE:include>)

find_library(DDSCXX_LIB ddscxx PATHS ${UNITREE_SDK_PATH}/lib NO_DEFAULT_PATH)
add_library(ddscxx SHARED IMPORTED GLOBAL)
set_target_properties(ddscxx PROPERTIES IMPORTED_LOCATION ${DDSCXX_LIB})
set_target_properties(ddscxx PROPERTIES IMPORTED_NO_SONAME TRUE)
target_link_libraries(ddscxx INTERFACE Threads::Threads)
target_link_directories(ddscxx INTERFACE ${UNITREE_SDK_PATH}/lib)
target_include_directories(ddscxx INTERFACE
        $<BUILD_INTERFACE:${UNITREE_SDK_PATH}/include>
        $<BUILD_INTERFACE:${UNITREE_SDK_PATH}/include/ddscxx>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/ddscxx>)

## Import Unitree SDK2 library
find_library(UNITREE_SDK_LIB unitree_sdk2 PATHS ${UNITREE_SDK_PATH}/lib NO_DEFAULT_PATH)

if (NOT UNITREE_SDK_LIB)
    message(FATAL_ERROR "Unitree SDK library for the architecture is not found")
else ()
    message(STATUS "Unitree SDK library found at: ${UNITREE_SDK_LIB}")
endif ()

message(STATUS "Importing: ${UNITREE_SDK_LIB}")

add_library(unitree_sdk2 STATIC IMPORTED GLOBAL)
set_target_properties(unitree_sdk2 PROPERTIES IMPORTED_LOCATION ${UNITREE_SDK_LIB})
target_link_libraries(unitree_sdk2 INTERFACE ddsc ddscxx Threads::Threads)
target_include_directories(unitree_sdk2 INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(unitree_ros2_test
#   ${DEPENDENCY_LIST}
# )

add_executable(unitree_sdk2_test src/unitree_sdk2_test.cpp)
target_link_libraries(unitree_sdk2_test unitree_sdk2)
target_include_directories(unitree_sdk2_test PRIVATE ${UNITREE_SDK_PATH}/include)

# ament_target_dependencies(robot_bridge
#   rclcpp
#   unitree_go
#   unitree_sdk2
#   cyclone_dds
# )

# target_include_directories(robot_bridge PRIVATE ${UNITREE_SDK_PATH}/include)

# 安装规则
install(TARGETS
  # unitree_ros2_test 
  unitree_sdk2_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()