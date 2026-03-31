/**
 * @brief Convert Gazebo pose to TF with frame prefix
 * 
 * This is a C++ implementation that is more CPU-efficient than the Python version.
 * It subscribes to either PoseArray or Odometry and publishes TF transforms
 * at a configurable fixed rate.
 */

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class PoseToTF : public rclcpp::Node
{
public:
  PoseToTF()
      : Node("pose_to_tf"),
        latest_pose_received_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("frame_prefix", "");
    this->declare_parameter<std::string>("parent_frame", "world");
    this->declare_parameter<std::string>("child_frame", "base_footprint");
    this->declare_parameter<double>("publish_rate_hz", 20.0);
    this->declare_parameter<std::string>("input_mode", "pose_array");

    // Get parameters
    frame_prefix_ = this->get_parameter("frame_prefix").as_string();
    parent_frame_ = this->get_parameter("parent_frame").as_string();
    std::string child_frame_base = this->get_parameter("child_frame").as_string();
    child_frame_ = frame_prefix_ + child_frame_base;
    double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    input_mode_ = this->get_parameter("input_mode").as_string();

    // Protect against invalid rates
    if (publish_rate_hz <= 0.0) {
      publish_rate_hz = 20.0;
    }

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Set up QoS profile for sensor data (best effort, volatile)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                   .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                   .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Create subscription based on input mode
    if (input_mode_ == "odom") {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "odom", qos,
          std::bind(&PoseToTF::odomCallback, this, std::placeholders::_1));
    } else {
      pose_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
          "audibot/pose", qos,
          std::bind(&PoseToTF::poseArrayCallback, this, std::placeholders::_1));
    }

    // Create timer for fixed-rate TF publishing (decouples from message rate)
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&PoseToTF::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "Publishing TF: %s -> %s at %.1f Hz (mode=%s)",
                parent_frame_.c_str(), child_frame_.c_str(),
                publish_rate_hz, input_mode_.c_str());
  }

private:
  void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (!msg->poses.empty()) {
      // Store latest pose without processing - callback is lightweight
      latest_pose_ = msg->poses[0];
      latest_pose_received_ = true;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Store latest pose from odometry
    latest_pose_ = msg->pose.pose;
    latest_pose_received_ = true;
  }

  void timerCallback()
  {
    if (!latest_pose_received_) {
      return;
    }

    // Create and publish transform
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = parent_frame_;
    t.child_frame_id = child_frame_;

    t.transform.translation.x = latest_pose_.position.x;
    t.transform.translation.y = latest_pose_.position.y;
    t.transform.translation.z = latest_pose_.position.z;

    t.transform.rotation.x = latest_pose_.orientation.x;
    t.transform.rotation.y = latest_pose_.orientation.y;
    t.transform.rotation.z = latest_pose_.orientation.z;
    t.transform.rotation.w = latest_pose_.orientation.w;

    tf_broadcaster_->sendTransform(t);
  }

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Timer for fixed-rate publishing
  rclcpp::TimerBase::SharedPtr timer_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Latest pose storage
  geometry_msgs::msg::Pose latest_pose_;
  bool latest_pose_received_;

  // Parameters
  std::string frame_prefix_;
  std::string parent_frame_;
  std::string child_frame_;
  std::string input_mode_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToTF>());
  rclcpp::shutdown();
  return 0;
}
