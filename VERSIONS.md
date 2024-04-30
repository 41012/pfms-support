

Version Changes (Pipes and ROS2_WS packages)
=========================

## 3.0.5 ##

- Audi library has updated computation of distance and end pose using EIgen for Matrix manipulation. Has been tested through series of unit tests as well as against running a TSP.

## 3.0.4 ##

- `PfmsHog::teleop` changed from using `gazebo_msgs::srv::DeleteEntity` and `gazebo_msgs::srv::SpawnEntity` instead of `gazebo_msgs::srv::SetEntityState`.   50 fold increase in speed, no delay, tested for `Quadcopter` `Ackerman` and `SkidSteer`. 
- Implemented a mechanism to pass `CTRL+C` to `PfmsConnector` and `PfmsHog` and then execute `rclcpp::shutdown();` to terminate thread of execution of the wrapped ROS node. Therefore the `pfmsconnector` library does not hang when `CTRL+C` is executed and terminates properly.
- Reduced buffer size for `drone/gt_odom` to 1 in `DroneSimpleController::Load`. Had issues of incomplete messages  
- Added `tower` as another object in `models` of `gazebo_tf`
- `reach` in `gazebo_tf` updates distance to goal even when within tolerance to truly report the minimum distance a goal has been passed.

## 3.0.3 ##

- For `Quadcopter` and `Ackerman`  `PfmsHog::teleop` changed from using `gazebo_msgs::srv::DeleteEntity` and `gazebo_msgs::srv::SpawnEntity` to `gazebo_msgs::srv::SetEntityState`.  `SkidSteer` still requires relaunch and therefore not ported yet.
- Changes to `DroneSimpleController` to broadcast `tf` of the platform 
- Made `gazebo_tf` distribute the `gazebo_model_path` in order to load simulation with objects in background