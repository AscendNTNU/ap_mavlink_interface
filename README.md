# AlphaPilot MAVLink interface

The main node here is `ap_mavlink_statemachine`, it is supposed
to handle *all* interactions with the pixhawk hardware.

## Params
| Name | About |
| ---- | ----- |
| `/uav/control/mavlink_url` | The url for mavlink to connect too. For simulator use `udp://:14540`. For serial use `serial:///path/to/serial/dev[:baudrate]`. |


## Subscribed topics 
| Topic | Message Type | About |
| ----- | ------------ | ----- |
| `/uav/control/arm` | `std_msgs/Bool` | Send value `true` to arm the drone | 
| `/uav/control/offboard` | `std_msgs/Bool` | Send value `true` to enter offboard mode | 
| `/uav/control/land` | `std_msgs/Bool` | Send value `true` to land the drone | 
| `/uav/control/position` | `geometry_msgs/Pose` | Postion for the drone to go to | 
| `/uav/control/attitude_rate` | `mav_msgs/Rates` | Rates for the drone to follow | 
| `/uav/control/pose` | `geometry_msgs/PoseStamped` | Odometry information | 

## Published topics 
| Topic | Message Type | About |
| ----- | ------------ | ----- |
| `/uav/control/ready` | `std_msgs/Bool` | Is the drone ready to fly (armed and in offboard mode) |
| (not impelemeted) `/uav/control/mav_pose` | `geometry_msgs/Pose` | The pose that the px4 thinks the drone is in |
