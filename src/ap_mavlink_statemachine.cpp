#include <ros/ros.h>
#include <mavsdk/mavsdk.h>
#include <iostream>
#include <ap_mav_controller.hpp>
#include <geometry_msgs/Pose.h>
#include <mav_msgs/common.h>
#include <mav_msgs/AttitudeThrust.h>

#include <std_msgs/String.h>

void usage(std::string bin_name)
{
    std::cout << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

struct CallbackHandler {
    ApMavController *controller;

    void position_callback(const geometry_msgs::Pose pose) {
        if (controller->set_position(pose.position.x, pose.position.y, pose.position.z, 0)) {
            ROS_WARN_STREAM("Will not go to position: " << pose.position);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ap_mavlink_statemachine");
    ros::NodeHandle n;
    
    ApMavController controller;
    std::string connection_url;

    n.getParam("connection_url", connection_url);

    if (controller.connect(connection_url)) {
        ROS_ERROR_STREAM("Could not connect with URL: " << connection_url);
        ros::shutdown();
        return 1;
    }

    controller.arm();

    // Enter positin follow mode:
    if (controller.position_follow(0, 0, 1, 0)) {
        ROS_ERROR_STREAM("Cannot enter position follow mode");
        ros::shutdown();
        return 1;
    }

    CallbackHandler cb = { &controller };

    ros::Subscriber sub = n.subscribe("/uav/control/position", 1, &CallbackHandler::position_callback, &cb);

    ros::spin();
    return 0;
}
