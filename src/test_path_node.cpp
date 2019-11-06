#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_path_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Publisher pub_arm      = n.advertise<std_msgs::Bool>("/uav/control/arm", 1);
    ros::Publisher pub_land     = n.advertise<std_msgs::Bool>("/uav/control/land", 1);
    ros::Publisher pub_position = n.advertise<geometry_msgs::Pose>("/uav/control/position", 1 );

    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0.5;

    // Wait for ready
    while (ros::ok()) {
        auto msg = ros::topic::waitForMessage<std_msgs::Bool>("/uav/control/ready", n, ros::Duration(1));
        if (msg && msg->data) break;

        // Send setpoints
        pub_position.publish(pose);
        loop_rate.sleep();
    }

    // Wait to reach first setpoint
    for (int i = 0; i < 20; i++) {
        loop_rate.sleep();
    }

    // Drive in circle
    pose.position.x = 0.5;
    for (int i = 0; i < 10; i++) {
        pub_position.publish(pose);
        loop_rate.sleep();
    }

    pose.position.y = 0.5;
    for (int i = 0; i < 10; i++) {
        pub_position.publish(pose);
        loop_rate.sleep();
    }

    pose.position.x = -0.5;
    for (int i = 0; i < 10; i++) {
        pub_position.publish(pose);
        loop_rate.sleep();
    }

    pose.position.y = -0.5;
    for (int i = 0; i < 10; i++) {
        pub_position.publish(pose);
        loop_rate.sleep();
    }

    pose.position.x = 0;
    pose.position.x = 0;
    for (int i = 0; i < 10; i++) {
        pub_position.publish(pose);
        loop_rate.sleep();
    }

    // Land
    std_msgs::Bool land_msg;
    land_msg.data = true;
    pub_land.publish(land_msg);

    return 0;
}
