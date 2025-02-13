#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <string>
#include <cmath>

#include "tf2/utils.h" //To use getYaw function from the quaternion of orientation
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "gazebo_msgs/msg/link_states.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
//#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

/**
 * @brief Connecte to gazebo and publishes the tf
 *
 */

/*!
 *  \brief     GazeboConnect Class
 *  \details
 *  Specifies the required interface for your Fusion class your ranger fusion
 * class must inherit from it. <b> You MUST NOT edit this file </b>.
 *  \author    Alen Alempijevic
 *  \version   1.02-0
 *  \date      2022-04-10
 *  \pre       none
 *  \bug       none (thought links should be in STL of pairs / string + time message sent)
 */

// //As we can not switch based on string in C++11 we define enum's 
// namespace gazebo_connect{
//   enum string_code {
//     eOrange,
//     eDrone,
//     eAudiobot,
//     ePole1,
//     ePole2
//   };
// }

class GazeboConnect : public rclcpp::Node
{
public:
  GazeboConnect() :
      Node("gazebo_connect"), 
      seq_(0)
  {

      //sub2 = nh_.subscribe("/orange/steering_cmd", 1000, &GazeboConnect::publishMarkers,this);

      sub1_ = this->create_subscription<gazebo_msgs::msg::LinkStates>(
        "/demo/link_states_demo", 1000, std::bind(&GazeboConnect::gazeboLinkStatesCallback,this,_1));

      odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>("husky/odom", 1000);


      // Initialize the transform broadcaster
      tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);
      // tfTimeStamp_ = ros::Time::now();
      // tfTimeStampPole1_ = ros::Time::now();
      // tfTimeStampPole2_ = ros::Time::now();

  }


  // void publishMarkers(const std_msgs::Float64::ConstPtr& msg){
  //     double steering=msg->data;

  //     if(fabs(steering)<0.03){
  //         ROS_INFO_STREAM_THROTTLE(1.0,"Straight line steering");
  //         return;
  //     }

  //     if(isnan(steering)){
  //         ROS_DEBUG_STREAM("Steering angle is nan");
  //         return;
  //     }

  //       if(fabs(steering)<0.03){
  //           ROS_DEBUG_STREAM("Straight line steering");
  //           return;
  //       }
  //     double radius = 2.65/(tan(steering/17.3));

  //     int marker_counter=0;
  //     visualization_msgs::MarkerArray marker_array;
  //     visualization_msgs::Marker marker;


  //     //We need to set the frame
  //     // Set the frame ID and time stamp.
  //     marker.header.frame_id = "orange/base_footprint";
  //     marker.header.seq = seq_;
  //     marker.header.stamp = ros::Time::now();


  //     //We set lifetime (it will dissapear in this many seconds)
  //     marker.lifetime = ros::Duration(2000.0);
  //     // Set the namespace and id for this marker.  This serves to create a unique ID
  //     // Any marker sent with the same namespace and id will overwrite the old one
  //     marker.ns = "pose";
  //     marker.id = marker_counter++;

  //     // The marker type, we use a cylinder in this example
  //     marker.type = visualization_msgs::Marker::CUBE_LIST;

  //     // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
  //     marker.action = visualization_msgs::Marker::ADD;

  //     //As an example, we are setting it
  //     marker.pose.position.x = 0;
  //     marker.pose.position.y = 0;
  //     marker.pose.position.z = 0;

  //     //Orientation, can we orientate it?
  //     marker.pose.orientation.x = 0.0;
  //     marker.pose.orientation.y = 0.0;
  //     marker.pose.orientation.z = 0.0;
  //     marker.pose.orientation.w = 1.0;


  //     // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m side
  //     marker.scale.x = 0.3;
  //     marker.scale.y = 0.3;
  //     marker.scale.z = 0.3;

  //     //Alpha is stransparency (50% transparent)
  //     marker.color.a = 0.5f;

  //     //Colour is r,g,b where each channel of colour is 0-1. Bellow will make it orange
  //     marker.color.r = 1.0;
  //     marker.color.g = 1.0;//static_cast<float>(177.0/255.0);
  //     marker.color.b = 0.0;

  //     double dr = (2*radius*M_PI)/0.5;
  //     double dtheta = 0.5/fabs(radius);
  //     ROS_INFO_STREAM("dr:" << dr << " dtheta:" << dtheta );

  //     for (double ang = 0; ang < (2*M_PI); ang +=dtheta){

  //       double dr = (2*radius*M_PI)/0.5;
  //       double dtheta = 0.5/fabs(radius);
  //       //ROS_INFO_STREAM("dr:" << dr << " dtheta:" << dtheta );
  //         geometry_msgs::Point p;
  //         //As an example, we are setting it
  //         p.x = (radius*cos(ang));
  //         p.y = (radius*sin(ang)) + radius;
  //         p.z = 0;
  //         marker.points.push_back(p);

  //     }
  //     //We push the marker back on our array of markers
  //     marker_array.markers.push_back(marker);

  //     //We publish the marker array
  //     viz_pub_.publish(marker_array);

  // }


  void sendTfBroadcast(geometry_msgs::msg::Pose pose,std::string link_name ){

    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = link_name;

    // tf::Transform transform;
    // transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    // tf::Quaternion q;
    // q.setValue(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
    // transform.setRotation(q);
    // tf::StampedTransform trStamped(transform, ros::Time::now(), "world", link_name);

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = pose.position.x;
    t.transform.translation.y = pose.position.y;
    t.transform.translation.z = pose.position.z;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setValue(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    // Instead of sending the tf, we check there has been an update in time
    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", link_name));

    // if(link_name == "pole1/base_footprint"){
    //     ros::Duration dt = trStamped.stamp_ - tfTimeStampPole1_;
    //     if(dt.toNSec()>0){
    //       br.sendTransform(trStamped);
    //       tfTimeStampPole1_=trStamped.stamp_;
    //     }
    // }
    // else if (link_name == "pole2/base_footprint") {
    //     ros::Duration dt = trStamped.stamp_ - tfTimeStampPole2_;
    //     if(dt.toNSec()>0){
    //       br.sendTransform(trStamped);
    //       tfTimeStampPole2_=trStamped.stamp_;
    //     }
    // }
    // else {
    //     ros::Duration dt = trStamped.stamp_ - tfTimeStamp_;
    //     if(dt.toNSec()>0){
    //       br.sendTransform(trStamped);
    //       tfTimeStamp_=trStamped.stamp_;
    //     }
    // }
  }

// - husky::base_link
// - husky::front_left_wheel
// - husky::front_right_wheel
// - husky::rear_left_wheel
// - husky::rear_right_wheel
  void gazeboLinkStatesCallback(const gazebo_msgs::msg::LinkStates& msg)
  {

    for (unsigned int i=0;i<msg.name.size();i++){
        if ((msg.name[i]).compare("husky::base_link") == 0) 
        {
          geometry_msgs::msg::Pose pose(msg.pose[i]);
          geometry_msgs::msg::Twist twist(msg.twist[i]);
          nav_msgs::msg::Odometry odom;
          odom.header.stamp = this->get_clock()->now();
          odom.header.frame_id="world";
          odom.pose.pose=pose;
          odom.twist.twist=twist;
          // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),
          //       *this->get_clock(),
          //       1000,
          //       "HUSKY x,y,yaw,vx,omega:" <<
          //         msg.pose[i].position.x << " " <<
          //         msg.pose[i].position.y << " " <<
          //         // tf::getYaw(msg->pose[i].orientation)*180/M_PI << " " <<
          //         msg.twist[i].linear.x << " "  <<
          //         msg.twist[i].angular.z );
          odomPub_->publish(odom);
          // vx_.push_back(msg.twist[i].linear.x);
          // omega_.push_back(msg.twist[i].angular.z);
          // if(vx_.size()>20){
          //   double vx = std::accumulate(vx_.begin(), vx_.end(), 0.0)/vx_.size();
          //   double omega = std::accumulate(omega_.begin(), omega_.end(), 0.0)/omega_.size();
          //   RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),
          //       *this->get_clock(),
          //       3000,
          //       "HUSKY: " <<
          //         vx << " " << omega );
          //   vx_.erase(vx_.begin());
          //   omega_.erase(omega_.begin());
          // }
          sendTfBroadcast(pose,"base_link" );
          // break;
        }
        // if ((msg.name[i]).compare("husky::laser_link") == 0) 
        // {
        //   geometry_msgs::msg::Pose pose(msg.pose[i]);
        //   geometry_msgs::msg::Twist twist(msg.twist[i]);
        //   nav_msgs::msg::Odometry odom;
        //   odom.header.stamp = this->get_clock()->now();
        //   odom.header.frame_id="world";
        //   odom.pose.pose=pose;
        //   odom.twist.twist=twist;    
        //   sendTfBroadcast(pose,"laser_link" );
        // }
        if ((msg.name[i]).compare("husky::front_left_wheel") == 0) 
        {
          geometry_msgs::msg::Pose pose(msg.pose[i]);
          geometry_msgs::msg::Twist twist(msg.twist[i]);
          nav_msgs::msg::Odometry odom;
          odom.header.stamp = this->get_clock()->now();
          odom.header.frame_id="world";
          odom.pose.pose=pose;
          odom.twist.twist=twist;    
          sendTfBroadcast(pose,"front_left_wheel" );
        }
        if ((msg.name[i]).compare("husky::front_right_wheel") == 0) 
        {
          geometry_msgs::msg::Pose pose(msg.pose[i]);
          geometry_msgs::msg::Twist twist(msg.twist[i]);
          nav_msgs::msg::Odometry odom;
          odom.header.stamp = this->get_clock()->now();
          odom.header.frame_id="world";
          odom.pose.pose=pose;
          odom.twist.twist=twist;    
          sendTfBroadcast(pose,"front_right_wheel" );
        }               
        if ((msg.name[i]).compare("husky::rear_left_wheel") == 0) 
        {
          geometry_msgs::msg::Pose pose(msg.pose[i]);
          geometry_msgs::msg::Twist twist(msg.twist[i]);
          nav_msgs::msg::Odometry odom;
          odom.header.stamp = this->get_clock()->now();
          odom.header.frame_id="world";
          odom.pose.pose=pose;
          odom.twist.twist=twist;    
          sendTfBroadcast(pose,"rear_left_wheel" );
        }
        if ((msg.name[i]).compare("husky::rear_right_wheel") == 0) 
        {
          geometry_msgs::msg::Pose pose(msg.pose[i]);
          geometry_msgs::msg::Twist twist(msg.twist[i]);
          nav_msgs::msg::Odometry odom;
          odom.header.stamp = this->get_clock()->now();
          odom.header.frame_id="world";
          odom.pose.pose=pose;
          odom.twist.twist=twist;    
          sendTfBroadcast(pose,"rear_right_wheel" );
          break;
        }        
      }
    }

        // else if ((msg->name[i]).compare("audibot::base_footprint") == 0) {
        //     geometry_msgs::Pose pose(msg->pose[i]);
        //     geometry_msgs::Twist twist(msg->twist[i]);
        //     nav_msgs::Odometry odom;
        //     odom.header.seq=seq_++;
        //     odom.header.stamp = ros::Time::now();
        //     odom.header.frame_id="world";
        //     odom.pose.pose=pose;
        //     odom.twist.twist=twist;
        //     ROS_INFO_STREAM_THROTTLE(60.0,"AUDI x,y,yaw,vx,vy:" <<
        //                           msg->pose[i].position.x << " " <<
        //                           msg->pose[i].position.y << " " <<
        //                           tf::getYaw(msg->pose[i].orientation)*180/M_PI << " " <<
        //                           msg->twist[i].linear.x << " "  <<
        //                           msg->twist[i].linear.y );

        //   ugv_odom_pub.publish(odom);
        //   sendTfBroadcast(pose,"base_footprint" );
        // }
        // else if ((msg->name[i]).compare("blue::base_footprint") == 0) {
        //     geometry_msgs::Pose pose(msg->pose[i]);
        //     geometry_msgs::Twist twist(msg->twist[i]);
        //     nav_msgs::Odometry odom;
        //     odom.header.seq=seq_++;
        //     odom.header.stamp = ros::Time::now();
        //     odom.header.frame_id="world";
        //     odom.pose.pose=pose;
        //     odom.twist.twist=twist;
        //     ROS_INFO_STREAM_THROTTLE(60.0,"BLUE x,y,yaw,vx,vy:" <<
        //                           msg->pose[i].position.x << " " <<
        //                           msg->pose[i].position.y << " " <<
        //                           tf::getYaw(msg->pose[i].orientation)*180/M_PI << " " <<
        //                           msg->twist[i].linear.x << " "  <<
        //                           msg->twist[i].linear.y );

        //   ugv_odom_pub2.publish(odom);
        //   // sendTfBroadcast(pose,"base_footprint" );
        // }        
//         else if ((msg->name[i]).compare("orange::base_footprint") == 0) {
//             geometry_msgs::Pose pose(msg->pose[i]);
//             geometry_msgs::Twist twist(msg->twist[i]);
//             nav_msgs::Odometry odom;
//             odom.header.seq=seq_++;
//             odom.header.stamp = ros::Time::now();
//             odom.header.frame_id="world";
//             odom.pose.pose=pose;
//             odom.twist.twist=twist;
//             ROS_INFO_STREAM_THROTTLE(60.0,"ORANGE x,y,yaw,vx,vy:" <<
//                                   msg->pose[i].position.x << " " <<
//                                   msg->pose[i].position.y << " " <<
//                                   tf::getYaw(msg->pose[i].orientation)*180/M_PI << " " <<
//                                   msg->twist[i].linear.x << " "  <<
//                                   msg->twist[i].linear.y );

//           ugv_odom_pub.publish(odom);
// //           sendTfBroadcast(pose,"base_footprint" );
//         }
//         else if ((msg->name[i]).compare("pole1::base_footprint") == 0) {
//             geometry_msgs::Pose pose(msg->pose[i]);
//             sendTfBroadcast(pose,"pole1/base_footprint" );
//         }
//         else if ((msg->name[i]).compare("pole2::base_footprint") == 0) {
//             geometry_msgs::Pose pose(msg->pose[i]);
//             sendTfBroadcast(pose,"pole2/base_footprint" );
//         }
  //   }
  // }

  // gazebo_connect::string_code hashit (std::string const& inString) {
  //   if (inString == "pole2::base_footprint") return gazebo_connect::ePole2;
  //   if (inString == "pole1::base_footprint") return gazebo_connect::ePole1;
  // }

private:
    //ros::NodeHandle nh_;
    rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr sub1_;
    // ros::Publisher uav_odom_pub,ugv_odom_pub,viz_pub_,ugv_odom_pub2;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // ros::Subscriber sub,sub2;
    unsigned int seq_;
    std::vector<double> vx_;
    std::vector<double> omega_;
    // ros::Time tfTimeStamp_;
    // ros::Time tfTimeStampPole1_;
    // ros::Time tfTimeStampPole2_;
};




int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboConnect>());
  rclcpp::shutdown();

  return 0;
}
