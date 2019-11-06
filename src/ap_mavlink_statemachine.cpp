#include <ros/ros.h>
#include <mavsdk/mavsdk.h>
#include <iostream>
#include <ap_mav_controller.hpp>
#include <geometry_msgs/Pose.h>
#include <mav_msgs/common.h>
#include <mav_msgs/RateThrust.h>
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

    void attitude_rate_callback(const mav_msgs::RateThrust attitude_rates) {
        double roll_rate = attitude_rates.angular_rates.x; 
        double pitch_rate = attitude_rates.angular_rates.y; 
        double yaw_rate = attitude_rates.angular_rates.z; 
        if (controller->attitude_rate_follow(roll_rate, pitch_rate, yaw_rate, attitude_rates.thrust.z)) {
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

    ros::Publisher pub_ready = n.advertise<std_msgs::Bool>("/uav/control/ready", 100, true);

    controller.on_ready([pub_ready](bool ready) {
        std_msgs::Bool msg;
        msg.data = ready;
        pub_ready.publish(msg);
    });

    CallbackHandler cb = { &controller };

    ros::Subscriber sub_arm      = n.subscribe("/uav/control/arm", 100, &CallbackHandler::arm_callback, &cb);
    ros::Subscriber sub_offboard = n.subscribe("/uav/control/offboard", 100, &CallbackHandler::offboard_callback, &cb);
    ros::Subscriber sub_land     = n.subscribe("/uav/control/land", 100, &CallbackHandler::land_callback, &cb);
    ros::Subscriber sub_position = n.subscribe("/uav/control/position", 1, &CallbackHandler::position_callback, &cb);
    ros::Subscriber sub_attitude = n.subscribe("/uav/control/attitude_rate", 1, &CallbackHandler::attitude_rate_callback, &cb);

    ros::spin();
    return 0;
}
