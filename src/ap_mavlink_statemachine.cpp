#include <ros/ros.h>
#include <mavsdk/mavsdk.h>
#include <iostream>
#include <ap_mav_controller.hpp>
#include <geometry_msgs/Pose.h>
#include <mav_msgs/common.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

#include <std_msgs/Bool.h>

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
    double last_roll;
    double last_pitch;

    void arm_callback(const std_msgs::Bool to_arm) {
        if (to_arm.data && controller->arm()) {
            ROS_ERROR_STREAM("Arming failed");
        }
    }

    void offboard_callback(const std_msgs::Bool to_arm) {
        if (to_arm.data && controller->start_offboard()) {
            ROS_ERROR_STREAM("Starting offboard failed");
        }
    }

    void land_callback(const std_msgs::Bool land) {
        if (land.data && controller->land()) {
            ROS_ERROR_STREAM("Landing failed");
        }
    }

    void position_callback(const geometry_msgs::Pose pose) {
        if (controller->position_follow(pose.position.x, pose.position.y, pose.position.z, 0)) {
            ROS_WARN_STREAM("Will not go to position: " << pose.position);
        }
    }

    void attitude_callback(const mav_msgs::RollPitchYawrateThrust attitude) {
        float roll_rate  = attitude.roll - last_roll;
        float pitch_rate = attitude.pitch - last_pitch;

        last_roll = attitude.roll;
        last_pitch = attitude.pitch;

        if (controller->attitude_rate_follow(roll_rate, pitch_rate, attitude.yaw_rate, attitude.thrust.z)) {
            ROS_WARN_STREAM("Will not follow attitude");
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

    //controller.arm();
    //controller.start_offboard();

    CallbackHandler cb = { &controller };

    ros::Subscriber sub_arm      = n.subscribe("/uav/control/arm", 1, &CallbackHandler::arm_callback, &cb);
    ros::Subscriber sub_offboard = n.subscribe("/uav/control/offboard", 1, &CallbackHandler::offboard_callback, &cb);
    ros::Subscriber sub_land     = n.subscribe("/uav/control/land", 1, &CallbackHandler::land_callback, &cb);
    ros::Subscriber sub_position = n.subscribe("/uav/control/position", 1, &CallbackHandler::position_callback, &cb);
    ros::Subscriber sub_attitude = n.subscribe("/uav/control/attitude", 1, &CallbackHandler::attitude_callback, &cb);

    ros::spin();
    return 0;
}
