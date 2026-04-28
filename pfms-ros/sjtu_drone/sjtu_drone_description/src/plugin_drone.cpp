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

#include "plugin_drone.h"


#include <cmath>
#include <stdlib.h>
#include <iostream>
#include <gz/sim/components/Name.hh>

namespace gz {
namespace sim {
namespace systems {

DroneSimpleController::DroneSimpleController()
{ 
  navi_state = LANDED_MODEL;
  m_posCtrl = false;
  m_velMode = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DroneSimpleController::~DroneSimpleController()
{
  // Cleanup handled by System lifecycle
}

////////////////////////////////////////////////////////////////////////////////
// Configure the controller
void DroneSimpleController::Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &/*_eventMgr*/)
{
  // Initialize ROS if not already initialized
  if(!rclcpp::ok()){
    RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "ROS was not initialized, initializing now...");
    rclcpp::init(0, nullptr);
  }

  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm)) {
    ignerr << "DroneSimpleController plugin should be attached to a model entity\n";
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "The drone plugin is loading!");
  
  //default parameters
  model_name_ = this->model.Name(_ecm);
  cmd_normal_topic_ = "cmd_vel";
  imu_topic_ = "imu";
  takeoff_topic_ = "takeoff";
  land_topic_ = "land";
  reset_topic_ = "reset";
  posctrl_topic_ = "posctrl";
  switch_mode_topic_ = "dronevel_mode";
  gt_topic_ = "odom";
  gt_vel_topic_ = "vel";
  gt_acc_topic_ = "acc";
  
  
  if (!_sdf->HasElement("bodyName"))
  {
    // Get canonical link
    this->link = this->model.CanonicalLink(_ecm);
    auto linkName = _ecm.Component<components::Name>(this->link);
    if (linkName) {
      link_name_ = linkName->Data();
    }
  }
  else {
    link_name_ = _sdf->Get<std::string>("bodyName");
    this->link = this->model.LinkByName(_ecm, link_name_);
  }

  if (!this->link || this->link == kNullEntity)
  {
    RCLCPP_FATAL(rclcpp::get_logger("DroneSimpleController"), "DroneSimpleController plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  max_force_ = _sdf->Get<double>("maxForce", -1.0).first;
  motion_small_noise_ = _sdf->Get<double>("motionSmallNoise", 0.0).first;
  motion_drift_noise_ = _sdf->Get<double>("motionDriftNoise", 0.0).first;
  motion_drift_noise_time_ = _sdf->Get<double>("motionDriftNoiseTime", 1.0).first;

  RCLCPP_INFO_STREAM(rclcpp::get_logger("DroneSimpleController"), "Using following parameters: \n" <<
                      "\t\tlink_name: "<<  link_name_.c_str() << ",\n" <<
                      "\t\tmax_force: "<<  max_force_ << ",\n" <<
                      "\t\tmotion_small_noise: "<<  motion_small_noise_ << ",\n" <<
                      "\t\tmotion_drift_noise: "<<  motion_drift_noise_ << ",\n" <<
                      "\t\tmotion_drift_noise_time: "<<  motion_drift_noise_time_
                    );

  // get inertia and mass of quadrotor body
  auto inertialComp = _ecm.Component<components::Inertial>(this->link);
  if (!inertialComp) {
    _ecm.CreateComponent(this->link, components::Inertial());
    return;
  }
  auto inertial_data = inertialComp->Data();
  // Get principal moments (diagonal elements of the inertia matrix)
  auto moi = inertial_data.Moi();
  inertia = gz::math::Vector3d(moi(0, 0), moi(1, 1), moi(2, 2));
  mass = inertial_data.MassMatrix().Mass();

  node_handle_ = std::make_shared<rclcpp::Node>("control", model_name_);
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_);
  tf_timer_thres_ = (int)(1e3 / 100.0);
  tf_timer_count_ = 0;

  // physics::LinkPtr footprint_link;
  // footprint_link = _model->GetLink("base_footprint");
  robot_name_ = _sdf->Get<std::string>("robot_name", "").first;
  if (robot_name_.empty()) {
    frame_id_ = link_name_;
  } else {
    frame_id_ = robot_name_ + "/" + link_name_;
  }

  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "The robot name is" << robot_name_ << " and the frame id is " << frame_id_);

  ////////////////////////////////////////////////////////////////////////////////
  // Subscribers
  // subscribe command: control command
  if (!cmd_normal_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    
    sub_opt.callback_group = node_handle_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(cmd_normal_topic_ + "/statistics");

    cmd_subscriber_ = node_handle_->create_subscription<geometry_msgs::msg::Twist>(cmd_normal_topic_, rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&DroneSimpleController::CmdCallback, this, std::placeholders::_1),sub_opt);
    if (cmd_subscriber_->get_topic_name()[0] != '\0') 
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using cmd_topic %s", cmd_normal_topic_.c_str());
    else 
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Cannot resolve the cmd_topic: %s !", cmd_normal_topic_.c_str());
  } else
    RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No cmd_topic defined!");
  
  if (!posctrl_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group( rclcpp::CallbackGroupType::Reentrant);

    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(posctrl_topic_ + "/statistics");

    posctrl_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Bool>(
      posctrl_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::PosCtrlCallback, this, std::placeholders::_1),
        sub_opt);

    if (posctrl_subscriber_->get_topic_name()[0] != '\0')
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using position control topic: %s!", posctrl_subscriber_->get_topic_name());
    else
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Cannot resolve the position control topic: %s !", posctrl_topic_.c_str());
  } else
      RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No position control defined!");

  // subscribe imu
  if (!imu_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(imu_topic_ + "/statistics");
    imu_subscriber_ = node_handle_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::ImuCallback, this, std::placeholders::_1),
      sub_opt);

    if (imu_subscriber_->get_topic_name()[0] != '\0')
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using imu on topic %s as source of orientation and angular velocity.", imu_topic_.c_str());
    else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Cannot resolve the IMU topic: %s !", imu_topic_.c_str());
  } else
      RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No imu topic defined!");


  // subscribe command: take off command
  if (!takeoff_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(takeoff_topic_ + "/statistics");

    takeoff_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Empty>(
      takeoff_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::TakeoffCallback, this, std::placeholders::_1),
      sub_opt
    );

    if (takeoff_subscriber_->get_topic_name()[0] != '\0')
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using the takeoff topic: %s", takeoff_subscriber_->get_topic_name() );
    else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Cannot resolve the takeoff topic: %s !", takeoff_topic_.c_str());
  }else
      RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No takeoff topic defined!");

  // subscribe command: land command
  if (!land_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(land_topic_ + "/statistics");
    land_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Empty>(
      land_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::LandCallback, this, std::placeholders::_1),
      sub_opt);

      if (land_subscriber_->get_topic_name()[0] != '\0') 
        RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using the land topic: %s", land_subscriber_->get_topic_name() );
      else
        RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Cannot resolve the land topic: %s !", land_topic_.c_str());
    }else
        RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No land topic defined!");

  // subscribe command: reset command
  if (!reset_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(reset_topic_ + "/statistics");
    reset_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Empty>(
      reset_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::ResetCallback, this, std::placeholders::_1),
      sub_opt);

    if (reset_subscriber_->get_topic_name()[0] != '\0')
        RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using the reset topic: %s", reset_subscriber_->get_topic_name() );
      else 
        RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Cannot resolve the reset topic: %s !", reset_topic_.c_str());
    }else
        RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No reset topic defined!");
  
  if (!switch_mode_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(switch_mode_topic_ + "/statistics");
    switch_mode_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Bool>(
      switch_mode_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::SwitchModeCallback, this, std::placeholders::_1),
      sub_opt);

    if (switch_mode_subscriber_->get_topic_name()[0] != '\0')
        RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using the switch mode topic: %s", switch_mode_subscriber_->get_topic_name() );
      else 
        RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Cannot resolve the switch mode topic: %s !", switch_mode_topic_.c_str());
    }else
        RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No switch mode topic defined!");


  ////////////////////////////////////////////////////////////////////////////////
  // Publishers
  if (!gt_topic_.empty()){
    pub_gt_odometry_ = node_handle_->create_publisher<nav_msgs::msg::Odometry>(gt_topic_,1);  
    if (pub_gt_odometry_->get_topic_name()[0] != '\0')
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Advertising the ground truth odometry topic on: %s !", pub_gt_odometry_->get_topic_name());  
    else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Could not resolve the ground odometry truth topic: %s !", gt_topic_.c_str());  
  } else
      RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No ground truth odometry topic defined!");

  if (!gt_vel_topic_.empty())
  {
    pub_gt_vec_ = node_handle_->create_publisher<geometry_msgs::msg::Twist>("gt_vel", 1024);
  if (pub_gt_vec_->get_topic_name()[0] != '\0')
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Advertising the ground truth velocity topic on: %s !", pub_gt_vec_->get_topic_name());  
    else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Could not resolve the ground truth velocity topic: %s !", gt_vel_topic_.c_str());  
  } else
      RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No ground truth velocity topic defined!");

  if (!gt_acc_topic_.empty())
  {
    pub_gt_acc_ = node_handle_->create_publisher<geometry_msgs::msg::Twist>("gt_acc", 1024);
    if (pub_gt_acc_->get_topic_name()[0] != '\0')
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Advertising the ground truth acceleration topic on: %s !", pub_gt_acc_->get_topic_name());  
    else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "Could not resolve the ground truth acceleration topic: %s !", gt_acc_topic_.c_str());  
  } else
      RCLCPP_ERROR(rclcpp::get_logger("DroneSimpleController"), "No ground truth acceleration topic defined!");

  

  LoadControllerSettings(_sdf);
  
  Reset();

  executor_->add_node(node_handle_);

  // Create components needed for updates
  _ecm.CreateComponent(this->link, components::WorldPose());
  _ecm.CreateComponent(this->link, components::WorldLinearVelocity());
  _ecm.CreateComponent(this->link, components::WorldAngularVelocity());
  _ecm.CreateComponent(this->link, components::WorldLinearAcceleration());
  _ecm.CreateComponent(this->link, components::ExternalWorldWrenchCmd());

  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "The drone plugin finished loading!");
}

/**
 * @brief Initialize the PID params
 * 
 * @param _sdf shared pointer to the sdf object
 */
void DroneSimpleController::LoadControllerSettings(const std::shared_ptr<const sdf::Element> &_sdf){
    controllers_.roll.Load(_sdf, "rollpitch");
    controllers_.pitch.Load(_sdf, "rollpitch");
    controllers_.yaw.Load(_sdf, "yaw");
    controllers_.velocity_x.Load(_sdf, "velocityXY");
    controllers_.velocity_y.Load(_sdf, "velocityXY");
    controllers_.velocity_z.Load(_sdf, "velocityZ");
    
    controllers_.pos_x.Load(_sdf, "positionXY");
    controllers_.pos_y.Load(_sdf, "positionXY");
    controllers_.pos_z.Load(_sdf, "positionZ");
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("DroneSimpleController"), "Using the PID parameters: \n" <<
                        "\tRoll Pitch:\n" << "\t\tkP: " << controllers_.roll.gain_p << ", kI: " << controllers_.roll.gain_i << ",kD: " << controllers_.roll.gain_d << ", Limit: " << controllers_.roll.limit << ", Time Constant: " << controllers_.roll.time_constant << "\n" << 
                        "\tYaw:\n" << "\t\tkP: " << controllers_.yaw.gain_p << ", kI: " << controllers_.yaw.gain_i << ",kD: " << controllers_.yaw.gain_d << ", Limit: " << controllers_.yaw.limit << ", Time Constant: " << controllers_.yaw.time_constant << "\n" << 
                        "\tVelocity X:\n" << "\t\tkP: " << controllers_.velocity_x.gain_p << ", kI: " << controllers_.velocity_x.gain_i << ",kD: " << controllers_.velocity_x.gain_d << ", Limit: " << controllers_.velocity_x.limit << ", Time Constant: " << controllers_.velocity_x.time_constant << "\n" << 
                        "\tVelocity Y:\n" << "\t\tkP: " << controllers_.velocity_y.gain_p << ", kI: " << controllers_.velocity_y.gain_i << ",kD: " << controllers_.velocity_y.gain_d << ", Limit: " << controllers_.velocity_y.limit << ", Time Constant: " << controllers_.velocity_y.time_constant << "\n" << 
                        "\tVelocity Z:\n" << "\t\tkP: " << controllers_.velocity_z.gain_p << ", kI: " << controllers_.velocity_z.gain_i << ",kD: " << controllers_.velocity_z.gain_d << ", Limit: " << controllers_.velocity_z.limit << ", Time Constant: " << controllers_.velocity_z.time_constant << "\n" << 
                        "\tPosition XY:\n" << "\t\tkP: " << controllers_.pos_x.gain_p << ", kI: " << controllers_.pos_x.gain_i << ",kD: " << controllers_.pos_x.gain_d << ", Limit: " << controllers_.pos_x.limit << ", Time Constant: " << controllers_.pos_x.time_constant << "\n" << 
                        "\tPosition Z:\n" << "\t\tkP: " << controllers_.pos_z.gain_p << ", kI: " << controllers_.pos_z.gain_i << ",kD: " << controllers_.pos_z.gain_d << ", Limit: " << controllers_.pos_z.limit << ", Time Constant: " << controllers_.pos_z.time_constant
    );
}

////////////////////////////////////////////////////////////////////////////////
// PreUpdate - called every simulation iteration
void DroneSimpleController::PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm)
{
  // Skip if paused
  if (_info.paused)
    return;

  // Skip if not properly initialized
  if (!executor_ || !node_handle_) {
    return;
  }

  // Get time
  std::chrono::steady_clock::duration current_sim_time = _info.simTime;
  double dt = std::chrono::duration<double>(current_sim_time - last_sim_time).count();
  if (dt == 0.0) {
    last_sim_time = current_sim_time;
    return;
  }
    
  executor_->spin_some(std::chrono::milliseconds(10));
  UpdateState(dt);
  UpdateDynamics(dt, _info, _ecm);

  if (tf_timer_count_++ >= tf_timer_thres_) {
    tf_timer_count_ = 0;
    tfTimerCallback();
  }
    
  // save last time stamp
  last_sim_time = current_sim_time;   
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
/**
* @brief Callback function for the drone command topic.
* This function is called whenever a new message is received on the drone command topic. It updates
* the cmd_val member variable with the new command message. It also generates motion noise for the
* drone's angular and linear velocities by adding drift and small noise values to the command message.
* The amount of noise added is determined by the motion_drift_noise_time_ and motion_small_noise_
* member variables of the DroneSimpleController class.
* The function uses the world->SimTime() function to get the current simulator time and calculate the
* time difference between the current and last simulation time. The time difference is used to update
* the drift noise values if the time_counter_for_drift_noise is greater than motion_drift_noise_time_.
* The updated command message is then used to update the cmd_val member variable.
* 
* @param cmd Pointer to the command message containing linear and angular velocities.
*/
void DroneSimpleController::CmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
{
  cmd_val = *cmd;

  static auto last_cmd_time = std::chrono::steady_clock::now();
  static double time_counter_for_drift_noise = 0;
  static double drift_noise[4] = {0.0, 0.0, 0.0, 0.0};
  
  // Get current time
  auto cur_cmd_time = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(cur_cmd_time - last_cmd_time).count();
  // save last time stamp
  last_cmd_time = cur_cmd_time;

  // generate noise
  if(time_counter_for_drift_noise > motion_drift_noise_time_)
  {
    drift_noise[0] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[1] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[2] = 2*motion_drift_noise_*(drand48()-0.5);
    drift_noise[3] = 2*motion_drift_noise_*(drand48()-0.5);
    time_counter_for_drift_noise = 0.0;
  }
  time_counter_for_drift_noise += dt;

  cmd_val.angular.x += drift_noise[0] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.angular.y += drift_noise[1] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.angular.z += drift_noise[3] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.linear.z += drift_noise[2] + 2*motion_small_noise_*(drand48()-0.5);

}

/**
* @brief Callback function for position control command.
* This function is called when a new position control command is received.
* It sets the m_posCtrl flag to the value of the command data.
* @param cmd The position control command message.
*/
void DroneSimpleController::PosCtrlCallback(const std_msgs::msg::Bool::SharedPtr cmd)
{
    m_posCtrl = cmd->data;
}

/**
* @brief Callback function to handle IMU sensor data.
* @param imu Shared pointer to IMU sensor data.
* The function reads the quaternion data from the IMU sensor and updates the orientation and angular velocity of the drone.
*/
void DroneSimpleController::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu)
{
  //directly read the quaternion from the IMU data
  pose.Rot().Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler = pose.Rot().Euler();
  angular_velocity = pose.Rot().RotateVector(gz::math::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
}

/**
* @brief Callback function to initiate taking off of the drone.
* @param msg Empty message.
*/
void DroneSimpleController::TakeoffCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  if(navi_state == LANDED_MODEL)
  {
    navi_state = TAKINGOFF_MODEL;
    m_timeAfterCmd = 0;
    RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Quadrotor takes off!!");
  }
}

/**
* @brief Callback function to initiate landing of the drone.
* @param msg Empty message.
*/
void DroneSimpleController::LandCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  if(navi_state == FLYING_MODEL||navi_state == TAKINGOFF_MODEL)
  {
    navi_state = LANDING_MODEL;
    m_timeAfterCmd = 0;
    RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Quadrotor lands!!");
  }
}

/**
* @brief Callback function for reset command
* This function resets the controller and the drone's state.
* @param msg Empty message
*/
void DroneSimpleController::ResetCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Reset quadrotor!!");
  Reset();
}

/**
* @brief Callback function for receiving a message to switch between velocity and position control modes
* @param msg Shared pointer to the message containing the boolean value for switching the mode
* The function switches between velocity and position control modes based on the boolean value in the message.
* If the boolean value is true, the control mode is switched to velocity control and if it's false, the control
* mode is switched to position control. It also resets the integral term of the controllers for the new mode.
*/
void DroneSimpleController::SwitchModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    m_velMode = msg->data;
}

/**
* @brief Updates the current state of the drone.
* This method is responsible for updating the current state of the drone based on the navigation state. If the drone is taking off, it checks if the time after the command is greater than 0.5 seconds. If it is, it sets the navigation state to flying. If the drone is landing, it checks if the time after the command is greater than 1 second. If it is, it sets the navigation state to landed. If the drone is neither taking off nor landing, it resets the time after the command to zero.
* 
* @param dt The time elapsed since the last update, in seconds.
*/
void DroneSimpleController::UpdateState(double dt){
    if(navi_state == TAKINGOFF_MODEL){
        
        m_timeAfterCmd += dt;
        if (m_timeAfterCmd > 0.5){
            navi_state = FLYING_MODEL;
            std::cout << "Entering flying model!" << std::endl;
        }
    }else if(navi_state == LANDING_MODEL){
        m_timeAfterCmd += dt;
        if(m_timeAfterCmd > 1.0){
            navi_state = LANDED_MODEL;
            std::cout << "Landed!" <<std::endl;
        }
    }else
        m_timeAfterCmd = 0;
}


/**
* @brief Update the dynamics of the drone.
* This method updates the dynamics of the drone based on its current state and the current
* commands being received. It computes the force and torque to be applied to the drone, and
* updates its position, velocity, and orientation accordingly. It also publishes the ground
* truth pose, velocity, and acceleration of the drone to ROS topics.
* 
* @param dt The time step to use for the update.
* @param _info The update info containing simulation time.
*/
void DroneSimpleController::UpdateDynamics(double dt, const UpdateInfo &_info, EntityComponentManager &_ecm){
  gz::math::Vector3d force, torque;
   
  // Get Pose/Orientation from Gazebo using ECS components
  auto poseComp = _ecm.Component<components::WorldPose>(this->link);
  if (poseComp) {
    pose = poseComp->Data();
    euler = pose.Rot().Euler();
  }
  
  auto angVelComp = _ecm.Component<components::WorldAngularVelocity>(this->link);
  if (angVelComp) {
    angular_velocity = angVelComp->Data();
  }
  
  auto linVelComp = _ecm.Component<components::WorldLinearVelocity>(this->link);
  if (linVelComp) {
    auto new_velocity = linVelComp->Data();
    acceleration = (new_velocity - velocity) / dt;
    velocity = new_velocity;
  }
    
  // Publish ground truth pose, velocity, and acceleration at 100Hz
  static auto last_publish_time = std::chrono::steady_clock::now();
  auto current_time = std::chrono::steady_clock::now();
  double time_since_last_publish = std::chrono::duration<double>(current_time - last_publish_time).count();

    if (time_since_last_publish >= 0.01) { // 100Hz = 1/100 seconds = 0.01 seconds
      last_publish_time = current_time;

      //publish the ground truth pose of the drone to the ROS topic
      geometry_msgs::msg::Pose gt_pose;
      gt_pose.position.x = pose.Pos().X();
      gt_pose.position.y = pose.Pos().Y();
      gt_pose.position.z = pose.Pos().Z();
      
      gt_pose.orientation.w = pose.Rot().W();
      gt_pose.orientation.x = pose.Rot().X();
      gt_pose.orientation.y = pose.Rot().Y();
      gt_pose.orientation.z = pose.Rot().Z();

      nav_msgs::msg::Odometry odom;
      // Use simulation time instead of wall time
      auto sim_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();
      odom.header.stamp.sec = static_cast<int32_t>(sim_time_ns / 1000000000);
      odom.header.stamp.nanosec = static_cast<uint32_t>(sim_time_ns % 1000000000);
      odom.header.frame_id = "world"; 
      odom.pose.pose = gt_pose;
      odom.twist.twist.linear.x = velocity.X();
      odom.twist.twist.linear.y = velocity.Y();
      odom.twist.twist.linear.z = velocity.Z();
      odom.twist.twist.angular.x = angular_velocity.X();
      odom.twist.twist.angular.y = angular_velocity.Y();
      odom.twist.twist.angular.z = angular_velocity.Z();

      //AA: look at limiting this to 100Hz
      pub_gt_odometry_->publish(odom);
      
      //convert the acceleration and velocity into the body frame
      gz::math::Vector3d body_vel = pose.Rot().RotateVector(velocity);
      gz::math::Vector3d body_acc = pose.Rot().RotateVector(acceleration);
      
      //publish the velocity
      geometry_msgs::msg::Twist tw;
      tw.linear.x = body_vel.X();
      tw.linear.y = body_vel.Y();
      tw.linear.z = body_vel.Z();
      pub_gt_vec_->publish(tw);
      
      //publish the acceleration
      tw.linear.x = body_acc.X();
      tw.linear.y = body_acc.Y();
      tw.linear.z = body_acc.Z();
      pub_gt_acc_->publish(tw);
      
    } 
               
    gz::math::Vector3d poschange = pose.Pos() - position;
    position = pose.Pos();
    
  
    // Get gravity (hardcoded for Ignition)
    gz::math::Vector3d world_gravity(0.0, 0.0, -9.81);
    gz::math::Vector3d gravity_body = pose.Rot().RotateVector(world_gravity);
    double gravity = gravity_body.Length();
    double load_factor = gravity * gravity / world_gravity.Dot(gravity_body);
  
    // Rotate vectors to coordinate frames relevant for control
    gz::math::Quaterniond heading_quaternion(cos(euler[2]/2), 0.0, 0.0, sin(euler[2]/2));
    gz::math::Vector3d velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
    gz::math::Vector3d acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
    gz::math::Vector3d angular_velocity_body = pose.Rot().RotateVectorReverse(angular_velocity);
  
    // update controllers
    force.Set(0.0, 0.0, 0.0);
    torque.Set(0.0, 0.0, 0.0);
    
    if( m_posCtrl){
        //position control
        if(navi_state == FLYING_MODEL){
            double vx = controllers_.pos_x.update(cmd_val.linear.x, position[0], poschange[0], dt);
            double vy = controllers_.pos_y.update(cmd_val.linear.y, position[1], poschange[1], dt);
            double vz = controllers_.pos_z.update(cmd_val.linear.z, position[2], poschange[2], dt);

            gz::math::Vector3d vb = heading_quaternion.RotateVectorReverse(gz::math::Vector3d(vx,vy,vz));
            
            double pitch_command =  controllers_.velocity_x.update(vb[0], velocity_xy[0], acceleration_xy[0], dt) / gravity;
            double roll_command  = -controllers_.velocity_y.update(vb[1], velocity_xy[1], acceleration_xy[1], dt) / gravity;
            torque[0] = inertia[0] *  controllers_.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
            torque[1] = inertia[1] *  controllers_.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);            
            force[2]  = mass      * (controllers_.velocity_z.update(vz,  velocity[2], acceleration[2], dt) + load_factor * gravity);
        }
    }else{
        //normal control
        if( navi_state == FLYING_MODEL )
        {
          //hovering
          double pitch_command =  controllers_.velocity_x.update(cmd_val.linear.x, velocity_xy[0], acceleration_xy[0], dt) / gravity;
          double roll_command  = -controllers_.velocity_y.update(cmd_val.linear.y, velocity_xy[1], acceleration_xy[1], dt) / gravity;
          torque[0] = inertia[0] *  controllers_.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
          torque[1] = inertia[1] *  controllers_.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);
        }else{
          //control by velocity
          if( m_velMode){
              double pitch_command =  controllers_.velocity_x.update(cmd_val.angular.x, velocity_xy[0], acceleration_xy[0], dt) / gravity;
              double roll_command  = -controllers_.velocity_y.update(cmd_val.angular.y, velocity_xy[1], acceleration_xy[1], dt) / gravity;
              torque[0] = inertia[0] *  controllers_.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
              torque[1] = inertia[1] *  controllers_.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);              
          }else{
            //control by tilting
            torque[0] = inertia[0] *  controllers_.roll.update(cmd_val.angular.x, euler[0], angular_velocity_body[0], dt);
            torque[1] = inertia[1] *  controllers_.pitch.update(cmd_val.angular.y, euler[1], angular_velocity_body[1], dt);
          }
        }
        torque[2] = inertia[2] *  controllers_.yaw.update(cmd_val.angular.z, angular_velocity[2], 0, dt);
        force[2]  = mass      * (controllers_.velocity_z.update(cmd_val.linear.z,  velocity[2], acceleration[2], dt) + load_factor * gravity);
    }

    
    if (max_force_ > 0.0 && force[2] > max_force_) force[2] = max_force_;
    if (force[2] < 0.0) force[2] = 0.0;
    
    
  
    // Apply forces and torques using ECS components
    gz::msgs::Wrench wrenchMsg;
    
    if(navi_state == LANDED_MODEL)
    {
      // No forces when landed
      gz::msgs::Set(wrenchMsg.mutable_force(), gz::math::Vector3d::Zero);
      gz::msgs::Set(wrenchMsg.mutable_torque(), gz::math::Vector3d::Zero);
    }
    else if(navi_state == FLYING_MODEL)
    {
      // Convert from relative to world frame
      gz::msgs::Set(wrenchMsg.mutable_force(), pose.Rot().RotateVector(force));
      gz::msgs::Set(wrenchMsg.mutable_torque(), pose.Rot().RotateVector(torque));
    }
    else if(navi_state == TAKINGOFF_MODEL)
    {
      gz::msgs::Set(wrenchMsg.mutable_force(), pose.Rot().RotateVector(force * 1.5));
      gz::msgs::Set(wrenchMsg.mutable_torque(), pose.Rot().RotateVector(torque * 1.5));
    }
    else if(navi_state == LANDING_MODEL)
    {
      gz::msgs::Set(wrenchMsg.mutable_force(), pose.Rot().RotateVector(force * 0.8));
      gz::msgs::Set(wrenchMsg.mutable_torque(), pose.Rot().RotateVector(torque * 0.8));
    }
    
    _ecm.SetComponentData<components::ExternalWorldWrenchCmd>(this->link, wrenchMsg);
   
}
////////////////////////////////////////////////////////////////////////////////
// Reset the controller
/**
 * @brief Reset the state of the drone controller and its associated objects to their initial values.
 * This method is called when the simulation is reset.
 */
void DroneSimpleController::Reset()
{
  // Reset the values of the controllers
  controllers_.roll.reset();
  controllers_.pitch.reset();
  controllers_.yaw.reset();
  controllers_.velocity_x.reset();
  controllers_.velocity_y.reset();
  controllers_.velocity_z.reset();

  // Reset the state of the drone
  pose.Reset();
  velocity = gz::math::Vector3d::Zero;
  angular_velocity = gz::math::Vector3d::Zero;
  acceleration = gz::math::Vector3d::Zero;
  euler = gz::math::Vector3d::Zero;
  state_stamp_ = rclcpp::Time();
  navi_state = LANDED_MODEL;
  m_timeAfterCmd = 0;
}

void DroneSimpleController::tfTimerCallback() {
  // TF publishing disabled - using pose_to_tf node instead to ensure sim time sync
  // if (!tf_broadcaster_ || !node_handle_) {
  //   return;
  // }
  // geometry_msgs::msg::TransformStamped t;
  // t.header.frame_id = "world";
  // t.child_frame_id = frame_id_;
  // t.header.stamp = node_handle_->now();
  // t.transform.translation.x = pose.Pos().X();
  // t.transform.translation.y = pose.Pos().Y();
  // t.transform.translation.z = pose.Pos().Z();
  // t.transform.rotation.w = pose.Rot().W();
  // t.transform.rotation.x = pose.Rot().X();
  // t.transform.rotation.y = pose.Rot().Y();
  // t.transform.rotation.z = pose.Rot().Z();
  // tf_broadcaster_->sendTransform(t);
}


// Register this plugin with the simulator
IGNITION_ADD_PLUGIN(
    gz::sim::systems::DroneSimpleController,
    gz::sim::System,
    gz::sim::systems::DroneSimpleController::ISystemConfigure,
    gz::sim::systems::DroneSimpleController::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(gz::sim::systems::DroneSimpleController,
                          "ignition::gazebo::systems::DroneSimpleController")

} // namespace systems
} // namespace sim
} // namespace gz