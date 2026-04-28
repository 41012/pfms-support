// Copyright 2023 Georg Novotny
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLUGIN_DRONE_H
#define PLUGIN_DRONE_H

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/common/Time.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/callback_group.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "pid_controller.h"

#define LANDED_MODEL 0
#define FLYING_MODEL 1
#define TAKINGOFF_MODEL 2
#define LANDING_MODEL 3

using namespace std::placeholders;

namespace gz
{
namespace sim
{
namespace systems
{
class DroneSimpleController : public System,
                                public ISystemConfigure,
                                public ISystemPreUpdate
{
public:
  DroneSimpleController();
  virtual ~DroneSimpleController();

public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override;

protected:
  virtual void LoadControllerSettings(const std::shared_ptr<const sdf::Element> &_sdf);
  void UpdateDynamics(double dt, const UpdateInfo &_info, EntityComponentManager &_ecm);
  void UpdateState(double dt);
  virtual void Reset();

private:
  void tfTimerCallback();
  double m_timeAfterCmd;
  bool m_posCtrl;
  bool m_velMode;
  unsigned int navi_state;

  /// \brief Model entity
  Model model;

  /// \brief Link entity
  Entity link;

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> node_handle_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr posctrl_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  // extra robot control command
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr takeoff_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr land_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr switch_mode_subscriber_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_gt_odometry_; //for publishing ground truth pose
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_gt_vec_; //ground truth velocity in the body frame
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_gt_acc_; //ground truth acceleration in the body frame

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  int tf_timer_count_;
  int tf_timer_thres_;
  std::string robot_name_;
  std::string frame_id_;  
  
  geometry_msgs::msg::Twist cmd_val;
  // callback functions for subscribers
  void CmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void PosCtrlCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void TakeoffCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void LandCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void ResetCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void SwitchModeCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Time state_stamp_;
  gz::math::Pose3d pose;
  gz::math::Vector3d euler;
  gz::math::Vector3d velocity, acceleration, angular_velocity, position;

  std::string link_name_;
  std::string model_name_;
  std::string cmd_normal_topic_;
  std::string switch_mode_topic_;
  std::string posctrl_topic_;
  std::string imu_topic_;
  std::string takeoff_topic_;
  std::string land_topic_;
  std::string reset_topic_;
  std::string gt_topic_;
  std::string gt_vel_topic_;
  std::string gt_acc_topic_;
  
  double max_force_;
  double motion_small_noise_;
  double motion_drift_noise_;
  double motion_drift_noise_time_;

  struct Controllers {
    PIDController roll;
    PIDController pitch;
    PIDController yaw;
    PIDController velocity_x;
    PIDController velocity_y;
    PIDController velocity_z;
    PIDController pos_x;
    PIDController pos_y;
    PIDController pos_z;
  } controllers_;

  gz::math::Vector3d inertia;
  double mass;

  /// \brief save last_time
  std::chrono::steady_clock::duration last_sim_time;
};

}
}
}

#endif // PLUGIN_DRONE_HPP