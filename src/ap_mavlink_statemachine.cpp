#include <ros/ros.h>
#include <mavsdk/mavsdk.h>
#include <iostream>
#include <ap_mav_controller.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <mav_msgs/common.h>
#include <mav_msgs/RateThrust.h>
#include <std_msgs/Bool.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

    void pose_callback(const geometry_msgs::PoseStamped pose) {
        uint64_t usecs = pose.header.stamp.sec * 1000000 + pose.header.stamp.nsec / 1000;
        double x = pose.pose.position.y;
        double y = pose.pose.position.x;
        double z = -pose.pose.position.z;

        double roll, pitch, yaw;
        tf2::Quaternion enu_to_ned;
        enu_to_ned.setRPY(M_PI_2, 0.0, M_PI_2);
        tf2::Quaternion rotation_q;
        tf2::convert(pose.pose.orientation, rotation_q);
        tf2::Matrix3x3 rotation(enu_to_ned*rotation_q);
        rotation.getRPY(roll, pitch, yaw);

        if (controller->send_odometery(usecs, x, y, z, roll, pitch, yaw)) {
            ROS_WARN_STREAM("Cannot send odometry");
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ap_mavlink_statemachine");
    ros::NodeHandle n;
    
    ApMavController controller;
    std::string connection_url;

    n.getParam("/uav/control/mavlink_url", connection_url);

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

    ros::Publisher pub_pose = n.advertise<geometry_msgs::Pose>("/uav/state/pose", 100);

    geometry_msgs::Pose pose_msg;

    controller.position_update(10, [&pose_msg, pub_pose](float x, float y, float z) {
        pose_msg.position.x = x;
        pose_msg.position.y = y;
        pose_msg.position.z = z;
        pub_pose.publish(pose_msg);
    });

    controller.attitude_update(10, [&pose_msg, pub_pose](float x, float y, float z, float w) {
        pose_msg.orientation.x = x;
        pose_msg.orientation.y = y;
        pose_msg.orientation.z = z;
        pose_msg.orientation.w = w;
        pub_pose.publish(pose_msg);
    });

    CallbackHandler cb = { &controller };

    ros::Subscriber sub_arm      = n.subscribe("/uav/control/arm", 100, &CallbackHandler::arm_callback, &cb);
    ros::Subscriber sub_offboard = n.subscribe("/uav/control/offboard", 100, &CallbackHandler::offboard_callback, &cb);
    ros::Subscriber sub_land     = n.subscribe("/uav/control/land", 100, &CallbackHandler::land_callback, &cb);
    ros::Subscriber sub_position = n.subscribe("/uav/control/position", 1, &CallbackHandler::position_callback, &cb);
    ros::Subscriber sub_attitude = n.subscribe("/uav/control/attitude_rate", 1, &CallbackHandler::attitude_rate_callback, &cb);
    ros::Subscriber sub_mocap    = n.subscribe("/uav/control/pose", 1, &CallbackHandler::pose_callback, &cb);

    ros::spin();
    return 0;
}
