

Version Changes
=========================

## 3.0.4 ##

- PfmsHog::teleop changed from using `gazebo_msgs::srv::DeleteEntity` and `gazebo_msgs::srv::SpawnEntity` to `gazebo_msgs::srv::SetEntityState`.   50 fold increase in speed, no delay, tested for `Quadcopter` `Ackerman` and `SkidSteer`. previous issue was `SkidSteer` requiring relaunch.
- Reduced buffer size for `drone/gt_odom` to 1 in `DroneSimpleController::Load`. Had issues of incomplete messages  
- Added `tower` as another object in `models` of `gazebo_tf`
