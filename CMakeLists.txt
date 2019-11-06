cmake_minimum_required(VERSION 2.8.3)
project(ap_mavlink_statemachine)
add_compile_options(-std=c++14)


# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        mav_msgs
)

find_package(MAVSDK REQUIRED)

catkin_package(
        INCLUDE_DIRS src
        CATKIN_DEPENDS roscpp
        LIBRARIES ${PROJECT_NAME}
)

# Includes headers in include folder
include_directories(
        src
        ${catkin_INCLUDE_DIRS}
)

add_executable(ap_mavlink_statemachine src/ap_mavlink_statemachine.cpp)
add_executable(test_path src/test_path_node.cpp)

target_link_libraries(ap_mavlink_statemachine
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk
    ${catkin_LIBRARIES})


target_link_libraries(test_path
    ${catkin_LIBRARIES})

