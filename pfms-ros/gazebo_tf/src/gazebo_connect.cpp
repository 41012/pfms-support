#include "ros/ros.h"

#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Pose.h"
#include <string>
#include <cmath>
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"

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


class GazeboConnect
{
public:
    GazeboConnect() :
        seq_(0)
    {
        ugv_odom_pub = nh_.advertise<nav_msgs::Odometry>("ugv_odom", 1000);
        uav_odom_pub = nh_.advertise<nav_msgs::Odometry>("uav_odom", 1000);
        viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);
        sub = nh_.subscribe("/gazebo/link_states", 1000, &GazeboConnect::gazeboLinkStatesCallback,this);
        sub2 = nh_.subscribe("/orange/steering_cmd", 1000, &GazeboConnect::publishMarkers,this);
        tfTimeStamp_ = ros::Time::now();
        tfTimeStampPole1_ = ros::Time::now();
        tfTimeStampPole2_ = ros::Time::now();
    }


    void publishMarkers(const std_msgs::Float64::ConstPtr& msg){
        double steering=msg->data;

        if(fabs(steering)<0.03){
            ROS_DEBUG_STREAM("Straight line steering");
            return;
        }

        if(isnan(steering)){
            ROS_DEBUG_STREAM("Steering angle is nan");
            return;
        }

        double radius = 2.65/(tan(steering/17.3));

        int marker_counter=0;
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;


         //We need to set the frame
         // Set the frame ID and time stamp.
         marker.header.frame_id = "orange/base_footprint";
         marker.header.seq = seq_;
         marker.header.stamp = ros::Time::now();


         //We set lifetime (it will dissapear in this many seconds)
         marker.lifetime = ros::Duration(2000.0);
         // Set the namespace and id for this marker.  This serves to create a unique ID
         // Any marker sent with the same namespace and id will overwrite the old one
         marker.ns = "pose";
         marker.id = marker_counter++;

         // The marker type, we use a cylinder in this example
         marker.type = visualization_msgs::Marker::CUBE_LIST;

         // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
         marker.action = visualization_msgs::Marker::ADD;

         //As an example, we are setting it
         marker.pose.position.x = 0;
         marker.pose.position.y = 0;
         marker.pose.position.z = 0;

         //Orientation, can we orientate it?
         marker.pose.orientation.x = 0.0;
         marker.pose.orientation.y = 0.0;
         marker.pose.orientation.z = 0.0;
         marker.pose.orientation.w = 1.0;


         // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m side
         marker.scale.x = 0.3;
         marker.scale.y = 0.3;
         marker.scale.z = 0.3;

         //Alpha is stransparency (50% transparent)
         marker.color.a = 0.5f;

         //Colour is r,g,b where each channel of colour is 0-1. Bellow will make it orange
         marker.color.r = 1.0;
         marker.color.g = 1.0;//static_cast<float>(177.0/255.0);
         marker.color.b = 0.0;

        double dr = (2*radius*M_PI)/0.5;
        double dtheta = 0.5/fabs(radius);
        //ROS_INFO_STREAM("dr:" << dr << " dtheta:" << dtheta );

        for (double ang = 0; ang < (2*M_PI); ang +=dtheta){

            geometry_msgs::Point p;
            //As an example, we are setting it
            p.x = (radius*cos(ang));
            p.y = (radius*sin(ang)) + radius;
            p.z = 0;
            marker.points.push_back(p);

        }
        //We push the marker back on our array of markers
        marker_array.markers.push_back(marker);

        //We publish the marker array
        viz_pub_.publish(marker_array);

    }


    void sendTfBroadcast(geometry_msgs::Pose pose,std::string link_name ){

      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
      tf::Quaternion q;
      q.setValue(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
      transform.setRotation(q);
      tf::StampedTransform trStamped(transform, ros::Time::now(), "world", link_name);
      // Instead of sending the tf, we check there has been an update in time
      //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", link_name));

     if(link_name == "pole1/base_footprint"){
         ros::Duration dt = trStamped.stamp_ - tfTimeStampPole1_;
         if(dt.toNSec()>0){
           br.sendTransform(trStamped);
           tfTimeStampPole1_=trStamped.stamp_;
         }
     }
     else if (link_name == "pole2/base_footprint") {
         ros::Duration dt = trStamped.stamp_ - tfTimeStampPole2_;
         if(dt.toNSec()>0){
           br.sendTransform(trStamped);
           tfTimeStampPole2_=trStamped.stamp_;
         }
     }
     else {
         ros::Duration dt = trStamped.stamp_ - tfTimeStamp_;
         if(dt.toNSec()>0){
           br.sendTransform(trStamped);
           tfTimeStamp_=trStamped.stamp_;
         }
     }
    }

    void gazeboLinkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
    {

      for (unsigned int i=0;i<msg->name.size();i++){
         if((msg->name[i]).compare("depth_1::depth_1") == 0){
           geometry_msgs::Pose pose(msg->pose[i]);
//           ROS_INFO_STREAM("msg" <<  i << "=" << msg->name[i]);
//           ROS_INFO_STREAM("pose" <<  pose);
           sendTfBroadcast(pose,"depth_1_optical" );
         }
         else if ((msg->name[i]).compare("sjtu_drone::base_link") == 0) {
           geometry_msgs::Pose pose(msg->pose[i]);
           geometry_msgs::Twist twist(msg->twist[i]);
           nav_msgs::Odometry odom;
           odom.header.seq=seq_++;
           odom.header.stamp = ros::Time::now();
           odom.header.frame_id="world";
           odom.pose.pose=pose;
           odom.twist.twist=twist;
           ROS_INFO_STREAM_THROTTLE(60.0,"QUAD x,y,yaw,vx,vy:" <<
                                    msg->pose[i].position.x << " " <<
                                    msg->pose[i].position.y << " " <<
                                    tf::getYaw(msg->pose[i].orientation)*180/M_PI << " " <<
                                    msg->twist[i].linear.x << " "  <<
                                    msg->twist[i].linear.y );
           uav_odom_pub.publish(odom);
           //sendTfBroadcast(pose,"base_link" );
           sendTfBroadcast(pose,"drone/base_link" );
         }
         else if ((msg->name[i]).compare("audibot::base_footprint") == 0) {
             geometry_msgs::Pose pose(msg->pose[i]);
             geometry_msgs::Twist twist(msg->twist[i]);
             nav_msgs::Odometry odom;
             odom.header.seq=seq_++;
             odom.header.stamp = ros::Time::now();
             odom.header.frame_id="world";
             odom.pose.pose=pose;
             odom.twist.twist=twist;
             ROS_INFO_STREAM_THROTTLE(60.0,"AUDI x,y,yaw,vx,vy:" <<
                                    msg->pose[i].position.x << " " <<
                                    msg->pose[i].position.y << " " <<
                                    tf::getYaw(msg->pose[i].orientation)*180/M_PI << " " <<
                                    msg->twist[i].linear.x << " "  <<
                                    msg->twist[i].linear.y );

           ugv_odom_pub.publish(odom);
           sendTfBroadcast(pose,"base_footprint" );
         }
         else if ((msg->name[i]).compare("orange::base_footprint") == 0) {
             geometry_msgs::Pose pose(msg->pose[i]);
             geometry_msgs::Twist twist(msg->twist[i]);
             nav_msgs::Odometry odom;
             odom.header.seq=seq_++;
             odom.header.stamp = ros::Time::now();
             odom.header.frame_id="world";
             odom.pose.pose=pose;
             odom.twist.twist=twist;
             ROS_INFO_STREAM_THROTTLE(60.0,"AUDI x,y,yaw,vx,vy:" <<
                                    msg->pose[i].position.x << " " <<
                                    msg->pose[i].position.y << " " <<
                                    tf::getYaw(msg->pose[i].orientation)*180/M_PI << " " <<
                                    msg->twist[i].linear.x << " "  <<
                                    msg->twist[i].linear.y );

           ugv_odom_pub.publish(odom);
//           sendTfBroadcast(pose,"base_footprint" );
         }
         else if ((msg->name[i]).compare("pole1::base_footprint") == 0) {
             geometry_msgs::Pose pose(msg->pose[i]);
             sendTfBroadcast(pose,"pole1/base_footprint" );
         }
         else if ((msg->name[i]).compare("pole2::base_footprint") == 0) {
             geometry_msgs::Pose pose(msg->pose[i]);
             sendTfBroadcast(pose,"pole2/base_footprint" );
         }
      }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher uav_odom_pub,ugv_odom_pub,viz_pub_;
    ros::Subscriber sub,sub2;
    unsigned int seq_;
    ros::Time tfTimeStamp_;
    ros::Time tfTimeStampPole1_;
    ros::Time tfTimeStampPole2_;
};




int main(int argc, char **argv)
{

  ros::init(argc, argv, "gazebo_connector");

  GazeboConnect gazeboConnect;

  ros::spin();

  return 0;
}
